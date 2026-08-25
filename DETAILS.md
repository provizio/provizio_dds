# provizio_dds details

Behaviour, configuration and tuning that the [README](README.md) summarises but does not spell
out. None of it is needed to publish or receive data: the defaults are the recommended settings,
and a participant created with `make_domain_participant()` already has them. Reach for this file
when you need to change one, or to understand what the library is doing on your behalf.

- [Network Auto-Recovery](#network-auto-recovery) — surviving interface and address changes
  - [Interfaces the filters skip](#interfaces-the-filters-skip)
  - [VPN and tunnel interfaces](#vpn-and-tunnel-interfaces)
- [Transport Selection](#transport-selection) — shared memory vs UDP, socket sizing, message caps
  - [Shared-Memory Cleanup](#shared-memory-cleanup)
- [Discovery Tuning](#discovery-tuning) — announcement and lease intervals
- [Logging](#logging) — routing the library's diagnostics into your own logger
- [XML Profiles](#xml-profiles) — configuring Fast-DDS directly
- [Discovering Endpoints and Known Types](#discovering-endpoints-and-known-types) — runtime introspection
- [Cloning only what you need](#cloning-only-what-you-need) — keeping a consumer's clone small


## Network Auto-Recovery

DDS participants bind their UDP transports to the set of network interfaces present at participant-creation time. If the host's network changes afterwards — the primary interface comes up after the application started, a DHCP lease arrives, a USB Ethernet adapter is plugged in, the host roams to a new network — Fast-DDS does not refresh those bindings, and affected participants stop discovering off-host peers until recreated.

provizio_dds handles this transparently. A process-wide background monitor watches the OS for interface **address and link-state** changes (netlink `RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR | RTMGRP_LINK` on Linux, `PF_ROUTE` incl. `RTM_IFINFO` on macOS, `NotifyUnicastIpAddressChange` + `NotifyIpInterfaceChange` on Windows), coalesces bursts of events (3 s of quiescence or up to 60 s of debounce), snapshot-diffs to filter out irrelevant churn (Docker / veth bridges, virtual interfaces, link-local IPv6), and on a confirmed change tears down and rebuilds the underlying Fast-DDS participant for every participant that opted in. Existing publisher and subscriber handles survive the rebuild — their internal Fast-DDS objects are swapped under the caller-held `shared_ptr` and the user-supplied callbacks are re-attached automatically.

An interface only counts as present while it is **operationally** up — carrier present, not merely administratively up (`IFF_RUNNING` on Linux/macOS, `OperStatus` on Windows). That deliberately matches how Fast-DDS itself enumerates interfaces, so the snapshot models exactly the set it will bind locators to. It is also why link-state events are subscribed to: powering on a network switch, or replugging a cable, moves an interface in and out of that set while often emitting no address event at all.

**A rebuild is triggered by what a rebuild can fix.** An address the host has *gained* is one: the new participant binds it and announces it. An address it has *lost* is not — nothing can be bound to what is gone, and tearing down endpoints that still work over the interfaces that remain would drop every sample and request in flight for no gain. So a loss alone is recorded and left at that; the rebuild happens when addresses return, which includes the case that looks like nothing changed:

- **Losing and regaining the same address still rebuilds.** A cable pulled and replugged, or a link that drops and comes back on the same DHCP lease, ends where it started — but the Fast-DDS sockets bound to that address were torn down while it was gone, so the participant is stale until rebuilt. On the event-driven backends this is caught inside a single burst (the burst-start snapshot is compared, not just the end state); a polling backend sees the two halves as separate observations and rebuilds on the second.
- **An unreadable interface list is not an interface change.** `getifaddrs` and `GetAdaptersAddresses` can fail — on macOS the former is a `sysctl(NET_RT_IFLIST)` size-then-fetch pair that can lose a race with a routing-table change. Since an empty interface set is a legitimate reading (a container whose only device is a filtered-out veth reports exactly that), a failure is reported as *no reading at all* rather than as an empty one: the last known set is kept, no rebuild decision is made, and a warning is logged once per run of failures. Reporting it as empty would rebuild every participant twice for a syscall that merely failed — once when the addresses "vanished" and once when they "returned".

Two further safety properties matter in the field:

- **Periodic re-verification.** Whenever no event burst is pending, the monitor re-checks the interface set directly every 30 s (`PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC`, `0` disables). This is the backstop for what a kernel event channel cannot report: an event the kernel dropped under load (netlink `ENOBUFS`), a notification channel that failed unrecoverably (the same check reopens it), and a change that raced the monitor's own startup. Purely event-driven recovery has no way back from any of those — the process would stay unable to communicate for the rest of its life.
- **Retry of failed rebuilds.** A rebuild that fails part-way (the OS refuses to create the replacement participant, or an endpoint cannot be re-created) leaves that participant inert. The same periodic check retries it — and only it — rather than waiting for a network change that may never come.

Note: IPv6 RFC 4941 temporary / privacy addresses are not filtered by `IFA_F_*` flags today, so on Linux/macOS/Windows hosts with privacy addresses enabled (the default on desktop installs of Ubuntu/Fedora/Mint, macOS, and Windows) the periodic rotation produces a snapshot delta — typically not more than once per 24 h with default kernel settings. In practice this is negligible and doesn't require any changes; if you ever hit a host where it matters, disable `use_tempaddr` on the DDS-carrying interface or opt out per-process via `PROVIZIO_DDS_NETWORK_RECOVERY=off`.

Auto-recovery is **on by default**. To override the mode per participant, pass a `network_recovery_mode` to `make_domain_participant`:

```C++
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/network_recovery.h"

// Default — honour the PROVIZIO_DDS_NETWORK_RECOVERY env var (on by default):
auto participant_default = provizio::dds::make_domain_participant();

// Always on, regardless of env var:
auto participant_on = provizio::dds::make_domain_participant(
    0, provizio::dds::network_recovery_mode::on);

// Always off:
auto participant_off = provizio::dds::make_domain_participant(
    0, provizio::dds::network_recovery_mode::off);
```

The Python binding exposes the same surface via `provizio_dds.make_domain_participant`:

```Python
import provizio_dds

# Default — honour the PROVIZIO_DDS_NETWORK_RECOVERY env var:
participant = provizio_dds.make_domain_participant()

# Always on:
participant = provizio_dds.make_domain_participant(
    0, provizio_dds.NetworkRecoveryMode.ON)

# Always off:
participant = provizio_dds.make_domain_participant(
    0, provizio_dds.NetworkRecoveryMode.OFF)
```

The Python implementation is event-driven on Linux (its own netlink subscription, same groups as the C++ side) and falls back to polling on macOS / Windows, or on Linux if the netlink socket cannot be opened — Python's stdlib has no portable kernel-event subscription API. Polling cannot observe a sub-interval transient (an address removed and re-added between two polls), which is the one blind spot the event-driven backend does not have. It applies the same operationally-up / loopback / link-local / per-OS adapter-name exclusions as the C++ side, AND on Linux the same `IFLA_INFO_KIND` exclusions (bridge / veth / dummy / vxlan / macvlan / ipvlan) via a small `RTM_GETLINK` netlink dump on each snapshot, plus the same periodic re-verification and failed-rebuild retry. The polling cadence is configurable via `PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC`, default 3 s. See `python/network_recovery.py` for details.

### Interfaces the filters skip

The exclusions above are heuristics aimed at container and virtualization churn, and they are applied to the *change-detection* snapshot only — Fast-DDS still binds to whatever the OS offers, with one deliberate exception: VPN / tunnel interfaces, which provizio_dds keeps out of the transports entirely (see [VPN and tunnel interfaces](#vpn-and-tunnel-interfaces) below) and therefore out of change detection too. Bridges are excluded from change detection, because `docker0` and `virbr0` are bridges — so if the interface your DDS traffic actually uses *is* a bridge (`br0` on a vehicle PC, `br-lan` on a router-like unit), name it explicitly so changes on it trigger a recovery:

```Bash
PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES=br0,br-lan my_app
```

Comma-separated, whitespace around entries ignored. A named interface bypasses the name / kind / adapter-type exclusions but still has to be operationally up, non-loopback, and carrying a non-link-local address. On Windows, match either the adapter's friendly name or its GUID-style name. It does **not** re-admit a VPN interface — `PROVIZIO_DDS_ALLOW_VPN_INTERFACES` is the knob for that, and re-admitting one here would put an interface back into change detection whose addresses the transports still refuse to bind.

### VPN and tunnel interfaces

**VPN and tunnel interfaces are excluded from the DDS transports by default.** A participant neither binds a locator to one nor announces its address to peers.

> **Breaking change.** A deployment that carried DDS *through* a tunnel — unicast discovery over a discovery server, or initial peers reachable only via the VPN — loses that path on upgrade, silently: the tunnel's address is simply never bound or announced, and nothing else about the participant changes. Set `PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1` (or name the tunnel, see below) to keep it. Everything discovered over multicast on a real network is unaffected.

The reason is a Fast-DDS behaviour that is easy to miss and expensive to hit. A participant announces an address on *every* interface it can bind, as both a discovery and a user-data locator, and a writer sends every sample to **all** of a matched reader's announced unicast locators — it does not pick the best path. So two hosts that share a LAN and are *both* on a VPN exchange every sample twice: once over the LAN, and once through the tunnel. Measured on a 1.6 MB stream between two hosts one switch apart, the tunnel carried 1,684,868 bytes of exact duplicate — and where the tunnel's egress is a metered cellular uplink, that duplicate is paid for and can saturate the link.

A VPN cannot be the path that establishes *multicast* discovery, which is what makes exclusion safe as a default: no VPN carries multicast, so for every peer found that way a tunnel only ever adds a duplicate path to a host already reached on a real network. The exception is unicast discovery that runs *through* the tunnel — a discovery server, or initial peers set programmatically rather than through an XML profile of your own, neither of which this library can detect — and such a deployment must opt out with `PROVIZIO_DDS_ALLOW_VPN_INTERFACES`.

What counts as a VPN: interfaces whose name *starts with* a tunnel convention (`tun*`, `tap*`, `utun*`, `ipsec*`, `wg*`, ZeroTier's `zt`+8-character devices) or *contains* a VPN vendor's name (`tailscale`, `wireguard`, `openvpn`, `zerotier`). The short conventions are anchored deliberately: as free substrings they appear inside unrelated product names — "NETGEAR WG111v3 Wireless USB Adapter" contains `wg` — and misclassifying a host's only NIC would drop **all** of its DDS traffic, which is far worse than the duplicate traffic this removes. The vendor names are long enough to match anywhere, which is what catches Windows descriptions like "OpenVPN TAP-Windows6". Additionally, on Linux, any interface whose rtnetlink `IFLA_INFO_KIND` is a tunnel kind (`tun`, `wireguard`, `xfrm`, `vti`, `vti6`, `ipip`, `ip6tnl`, `gre`, `gretap`, `ip6gre`, `sit`), which catches renamed devices too — where that kind lookup fails, classification falls back to the names alone and says so once, in a warning, since a renamed device is then indistinguishable from ordinary hardware. On Windows the adapter's friendly name and description are matched, plus `IF_TYPE_TUNNEL`. Deliberately *not* matched: `ppp*` (a PPP link is a real WAN uplink, not an overlay), bridges, and container plumbing. Classification never keys on address ranges — 100.64.0.0/10 would catch Tailscale, but it is the RFC 6598 carrier-NAT range that a cellular uplink, and therefore a peer's real LAN interface behind such a router, legitimately holds.

To carry DDS over a tunnel anyway — which also needs unicast discovery configured, since multicast will not cross it:

```Bash
# Every VPN interface, as before this default existed:
PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1 my_app

# Only the named ones; every other tunnel stays excluded:
PROVIZIO_DDS_ALLOW_VPN_INTERFACES=tailscale0,wg0 my_app
```

| Variable | Default | Effect |
|----------|---------|--------|
| `PROVIZIO_DDS_ALLOW_VPN_INTERFACES` | *(empty)* | `1` / `true` / `yes` / `on` / `all` re-admits every VPN interface; a comma-separated list re-admits only those names, matched case-insensitively (the classifier lower-cases too). On Windows, name it by whichever identity you have: the adapter's friendly name, its GUID-style device name, or its driver-supplied description — an adapter classified by its description alone ("TAP-Windows Adapter V9" on a NIC called "Ethernet 3") has no other name to give. Any other value (including `0` / `false` / `off`) leaves the default in place. Read once per process, so the transports, change detection and every rebuilt participant agree. A name that matches no tunnel on the host is reported once, as a warning naming it: a typo (`tailscale` for a device called `tailscale0`), or the wrong identity for the platform, otherwise leaves the tunnel excluded exactly as if the variable had never been set. Nothing is said where the exclusion found no tunnel to apply to, so one setting given fleet-wide to hosts whose tunnels differ stays quiet. |

**While a tunnel is excluded, two Fast-DDS behaviours change with it.** Any non-empty interface list puts the UDP transport into *whitelist* mode, and whitelist mode is per-interface where the default is not:

- **Loopback is filtered, your LAN interface is not.** Whitelist mode replaces the single any-address output socket with one socket per remaining interface, and every one of them would otherwise send every unicast datagram. Two of those sockets need opposite treatment, so provizio_dds gives each interface its own netmask filter rather than one setting for the transport. **Loopback** is always filtered: it stays in the allowed set (it is how same-host participants reach each other where shared memory is off) but can carry nothing to another host, so without a filter every unicast datagram costs it a failed `sendto` and a Fast-DDS warning — per datagram. **Real interfaces** are filtered only where two or more of them remain, which is the case where each would send its own copy of every sample; with a single LAN interface — the common vehicle host — nothing duplicates and it is left unfiltered, so peers reachable only through a gateway keep receiving unicast. Where filtering is on, a datagram is sent by the one socket whose subnet contains the destination, and a peer outside every remaining subnet is not sent to; if you route DDS across subnets on a multi-interface host (which needs your own XML profile with unicast initial peers anyway), set `PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1` or configure the transports yourself.
- **The usable interface set is fixed when the participant is created.** Fast-DDS builds the whitelist once in the transport's constructor, so an interface that comes up *later* (Wi-Fi associating, a link recovering) gets no output socket, where the default any-address socket would have covered it. Auto-recovery is what closes this: it rebuilds the participant on a network change, and the rebuild re-enumerates. If you set `PROVIZIO_DDS_NETWORK_RECOVERY=0`, expect a VPN-carrying host to keep the interface set it started with until the process restarts.

Neither applies on a host with no tunnel up: the blocklist is empty there, and the transports are left exactly as Fast-DDS built them.

Transports you configured yourself fall outside this feature entirely. A participant whose transports come from an XML profile you wrote (`FASTDDS_DEFAULT_PROFILES_FILE`, or a `DEFAULT_FASTDDS_PROFILES.xml` that Fast-DDS auto-loads from the working directory), from `FASTDDS_BUILTIN_TRANSPORTS`, or from a profile you loaded through the factory yourself (`load_XML_profiles_file` / `load_XML_profiles_string` with a default participant profile, or `set_default_participant_qos`), keeps exactly the transports that configuration asked for — descriptors, `interface_blocklist` and `netmask_filter` alike. provizio_dds only ever configures the descriptors it created itself, because rewriting yours would overwrite a blocklist or a filter setting you declared deliberately, process-wide, and nothing else you configured (initial peers, a discovery server, lease durations) is disturbed either. Use `interfaces`/`blocklist` in the profile if you need the same exclusion there. Note also that this stops *this* host from announcing tunnel locators; a peer running an older provizio_dds still announces its own, and this host will still send to them. The exclusion pays off fully once both ends have it.

Two consequences of owning the transports yourself are worth stating outright, because neither is visible from the outside:

- **Change detection follows the transports.** The snapshot that decides whether auto-recovery rebuilds a participant drops tunnel interfaces only where the exclusion actually reached the transports — a tunnel DDS never binds cannot move a locator, so its churn must not rebuild anything. Where the exclusion could *not* be applied (the transports are yours, a participant-level `netmask_filter` of `OFF` rules a blocklist out, or this host's interfaces could not be read at the moment the participant was configured), tunnels stay in the snapshot and their churn rebuilds participants like any other interface's, because DDS does bind and announce them there. The decision is process-wide and one-way: one participant that could not exclude makes the whole process watch tunnels, which costs at most an unnecessary rebuild. `PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1` re-admits tunnels to the transports and to change detection together, which is the whole point of it being one variable.
- **provizio_dds says so, once, when it matters.** A participant that finds a tunnel up but cannot apply the exclusion — because the transports are yours, or because your participant-level `netmask_filter` is `OFF`, which Fast-DDS refuses to combine with the `ON` the exclusion's own interface entries carry (it would drop those interfaces from the transport's allowed set, leaving it with none) — logs one line saying which and why. A participant that could not read this host's interfaces at all logs one too, because a failed reading and "no tunnel here" are the same empty answer and only the first leaves a tunnel possibly bound. Silence means that the exclusion applied, that the host has no tunnel up, or that the participant is confined to this host anyway (`transport_mode::localhost_only`, where no tunnel of the host's can carry traffic that never leaves loopback) -- provizio_dds says nothing about interfaces it had no reason to exclude. In Python, the `OFF` case cannot be detected: the bindings expose no `NetmaskFilterKind` values to compare against, so a profile of yours that sets participant-level `OFF` without configuring transports should exclude the tunnel in its own descriptors.

Two limits of the exclusion itself, which apply whoever owns the transports:

- **Netmask filtering comes with it, decided per interface.** An interface blocklist puts Fast-DDS' UDP transport into whitelist mode, which replaces the single any-address output socket with one socket per allowed interface — and `sendSync` sends through every one of them. The interfaces that leaves behind need opposite answers, so each one carries its own filter rather than the transport carrying one for all of them. **Loopback** is always filtered: it is in the allowed set (nothing blocks it, and it is how same-host participants reach each other where shared memory is off) but can carry nothing to another host, so unfiltered it costs a failed send and a Fast-DDS warning for every unicast datagram. **Real interfaces** are filtered only where two or more remain, which is where each would otherwise emit its own copy of every sample; a single LAN interface plus a tunnel — the common vehicle host — leaves nothing to duplicate, so it stays unfiltered and its peers behind a gateway keep receiving unicast. Where filtering is on, a peer outside every local subnet receives no unicast though multicast discovery still finds it, and the line reporting the exclusion says so. On a routed network with several local interfaces, exclude the tunnel in your own transport descriptors instead, or set `PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1`.
- **Whitelist mode fixes the interface set at participant creation.** Fast-DDS re-enumerates interfaces when told to, which is what an auto-recovery rebuild does; with auto-recovery off (`network_recovery_mode::off` or `PROVIZIO_DDS_NETWORK_RECOVERY=off`) nothing re-enumerates, so an interface that appears after a participant was created on a tunnel-carrying host is never bound for the life of that participant. Auto-recovery is on by default for this reason; turning it off on a host that has a tunnel up means restarting a process to pick up a new interface.

A last note on the working-directory profile: a `DEFAULT_FASTDDS_PROFILES.xml` sitting in the process' working directory counts as your transport configuration even if it configures no transports at all, because Fast-DDS auto-loads it and provizio_dds cannot tell what it contains. Services whose working directory is not fully under your control should set `SKIP_DEFAULT_XML_FILE=1` (Fast-DDS' own variable) so an unexpected file cannot quietly change how DDS is configured.

To disable auto-recovery process-wide, set the env var before launching:

```Bash
PROVIZIO_DDS_NETWORK_RECOVERY=off my_app
```

Recognised values (case-insensitive): `on` / `1` / `true` / `yes` to enable, `off` / `0` / `false` / `no` to disable. Unset or empty defaults to enabled. An unrecognised value is treated as enabled and logged as a warning.

All auto-recovery environment variables, each read once per process:

| Variable | Default | Effect |
|----------|---------|--------|
| `PROVIZIO_DDS_NETWORK_RECOVERY` | `on` | Enable / disable auto-recovery process-wide. |
| `PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC` | `30` | Cadence of the periodic re-verification — and of everything that rides on it: notification-channel revival and the bounded failed-rebuild retry. `0` disables **all three**, leaving recovery purely event-driven. Values above one day are clamped. Read by C++ everywhere and by Python on its event-driven (Linux) backend. The Python **polling** backends (macOS, Windows, Linux without netlink) ignore it entirely — for a poller the poll *is* the periodic check, so `0` cannot disable the backstop there, and the failed-rebuild retry runs at `..._POLL_INTERVAL_SEC` cadence instead. |
| `PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES` | *(empty)* | Comma-separated interfaces to include in change detection regardless of the name / kind exclusions. |
| `PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC` | `3` | Python only: quiet period, and the polling cadence on the non-event-driven backends. |

**Cost when not in use:** if no participant ever enables auto-recovery, the background monitor is never started — no threads, no kernel channels, no per-participant memory beyond a single boolean flag.

**Cost during a reset:** typical end-to-end recovery time is a few seconds — about 3 s of event coalescing plus the time Fast-DDS needs for rediscovery and TypeLookup against the new participant. Each reset is logged, once, when the change is detected (see [Logging](#logging) below); a network event that turns out to change nothing is silent.

For details see [include/provizio/dds/network_recovery.h](include/provizio/dds/network_recovery.h).

## Transport Selection

By default a participant uses the platform's standard transports: shared memory plus UDPv4 on Linux, and UDPv4 only on Windows/macOS (where shared memory is disabled to avoid a Boost.Interprocess cleanup bug). On every platform the UDP transport is tuned with enlarged (16 MiB) socket buffers so reliable delivery of large samples — camera frames, point clouds — works across hosts without extra configuration.

Pass `transport_mode::udp_only` (C++) / `TransportMode.UDP_ONLY` (Python) to `make_domain_participant` to disable shared memory on every platform. This helps when a participant bridges mismatched Fast-DDS major versions (e.g. a recorder relaying 2.x publishers), where cross-major shared-memory negotiation can degrade large-sample throughput.

Pass `transport_mode::localhost_only` (C++ only) for a domain whose peers are all processes on this host — a relay feeding a local recorder, say. Shared memory carries the samples wherever the platform allows it, and the UDP transport is confined to the loopback interface, so the participant binds, joins multicast on and announces over `127.0.0.1` alone: nothing it sends reaches the LAN. Discovery is also given `127.0.0.1` as a unicast initial peer, so it does not depend on loopback multicast, and participants on another host are refused at the discovery layer (Fast-DDS' `FILTER_DIFFERENT_HOST`), so an announcement that reaches the host anyway cannot become a peer.

Two limits are worth stating plainly:

- **The boundary is the host, not privilege.** On the machine itself the domain is deliberately open: any local process that joins it is a peer, whether or not it asks for this mode, and Fast-DDS' shared-memory segments are world-readable (`0644` under `/dev/shm`), so any local user can read the samples. Use this to keep a domain off the wire, not to keep it from other users of the machine.
- **It governs only the transports this library configures.** Where you own them — an XML profile of yours, `FASTDDS_BUILTIN_TRANSPORTS`, or descriptors set through `DomainParticipantFactory` — there is nothing here to confine, and confining them is yours to do. A participant that ends up *not* confined says so: it reads its own transport configuration back after creation and logs a warning naming the requested mode, so a run that believes it is contained and is not says so in its log. The test is the configuration, not who wrote it — transports of yours that are already restricted to `127.0.0.1` deliver exactly what the mode asks for and are left in silence, and the same holds for `transport_mode::udp_only`, which warns only where shared memory really is in the set.

UDP is kept *alongside* shared memory rather than replaced by it, on Linux as much as anywhere else. Shared-memory registration fails once the host's shared-memory space is exhausted, and a participant confined to this host with no second transport would then have none at all; loopback UDP is the fallback that keeps it talking.

Two things follow from a locator set that holds nothing but loopback, and both are automatic:

- the participant takes no part in [network auto-recovery](#network-auto-recovery), whatever `network_recovery_mode` it is given — no interface change can invalidate a loopback locator, and a rebuild would cost its peers a rediscovery for nothing;
- the VPN interface exclusion does not apply to it, because there is no tunnel in such a locator set to exclude. (Where the transports turn out to be yours, the exclusion's own "not excluding VPN / tunnel interface(s)" line is still emitted — that case does reach the tunnel.)

There is no Python counterpart: the Python layer selects transports through the process-global `FASTDDS_BUILTIN_TRANSPORTS` variable, which can name a transport stack but cannot express an interface allowlist.

```C++
#include "provizio/dds/domain_participant.h"

// Platform default — SHM + UDP on Linux, UDP-only on Windows/macOS:
auto participant_default = provizio::dds::make_domain_participant();

// UDP-only on every platform:
auto participant_udp_only = provizio::dds::make_domain_participant(
    0, provizio::dds::network_recovery_mode::env_var_controlled, {},
    provizio::dds::endpoint_kind::data_writer, provizio::dds::transport_mode::udp_only);

// Same-host only — SHM where the platform allows it, plus loopback-confined UDP:
auto participant_localhost_only = provizio::dds::make_domain_participant(
    0, provizio::dds::network_recovery_mode::env_var_controlled, {},
    provizio::dds::endpoint_kind::data_writer, provizio::dds::transport_mode::localhost_only);
```

```Python
import provizio_dds

# Platform default:
participant_default = provizio_dds.make_domain_participant()

# UDP-only on every platform:
participant_udp_only = provizio_dds.make_domain_participant(transport=provizio_dds.TransportMode.UDP_ONLY)
```

The large-sample message types listed under [Publishing Data](README.md#publishing-data) additionally default to asynchronous publishing with a small KEEP_LAST history, which complements the enlarged socket buffers for high-throughput data.

Three transport-level environment options are read once at participant creation. In C++ the first two are skipped when transports are configured through Fast-DDS's own `FASTDDS_BUILTIN_TRANSPORTS` variable or an [XML profile](#xml-profiles); in Python the socket-buffer request is applied *through* `FASTDDS_BUILTIN_TRANSPORTS` itself (an externally-set value always wins, and the first participant created in the process fixes it for the rest — the XML-profile skip does not apply). `PROVIZIO_DDS_MAX_MESSAGE_SIZE` caps the RTPS output path whichever transports carry it (XML-profile participants excepted):

| Environment variable | Default | Meaning |
|---|---|---|
| `PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE` | `16777216` | UDP send/receive socket buffer ceiling (bytes). The OS clamps it to `net.core.rmem_max` / `wmem_max` — raise those sysctls for the request to take full effect (C++ participants log a warning when they cap it). |
| `PROVIZIO_DDS_SHM_SEGMENT_SIZE` | derived by Fast-DDS | Shared-memory segment size (bytes) per participant, decoupled from the UDP socket buffer request Fast-DDS otherwise derives it from (16 MiB of socket buffer produces a ~33.5 MiB segment). Lower it on `/dev/shm`-constrained hosts. C++ participants only: the Python bindings configure transports through `FASTDDS_BUILTIN_TRANSPORTS`, which exposes no segment-size option — including on the VPN-excluding profile path, where they reproduce the size Fast-DDS itself would have derived (twice the socket buffer request). |
| `PROVIZIO_DDS_MAX_MESSAGE_SIZE` | `1400` | Send-side cap (bytes) for RTPS messages, mapping to Fast-DDS's `fastdds.max_message_size` property (output path only — reception from differently-configured peers is unaffected, so mixed fleets and default-configured ROS 2 peers stay compatible). The default keeps every UDP datagram within a single ~1500-byte-MTU link frame, so samples above it travel as individually-retransmittable RTPS fragments instead of one large UDP datagram that the IP layer splits into many link frames: with IP fragmentation, sustained frame loss makes large-sample delivery all-or-nothing and can exhaust the receiving kernel's reassembly cache (`net.ipv4.ipfrag_high_thresh` / `ipfrag_time`), blacking out every fragmented topic in ~30 s cycles while single-frame topics keep flowing — the dominant failure mode for radar point clouds on lossy links (Wi-Fi, embedded routers). Measured at 10% injected frame loss with 28 KB point clouds: ~36% delivery in blackout cycles at `65500` vs 99%+ at `1400`, p90 latency 160 ms. CycloneDDS keeps its fragments MTU-sized by default the same way (its `General/FragmentSize` default is 1344 bytes). Values below `576` are rejected (with a logged error): a complete discovery announcement must fit in one RTPS message, and Fast-DDS itself does not diagnose a property that breaks that. Values above Fast-DDS's `65500` maximum are clamped to it (with a logged warning). See the CPU trade-off note below. |

**Trade-off of the `1400` default — CPU when publishing very large samples.** The cap sets the size of every outgoing datagram, so per-datagram costs are paid per ~1.4 KB instead of per ~64 KB. Most messages published by Provizio components are far smaller than 64 KB, where the difference is negligible — radar point clouds and freespace polygons gain dramatically better deliverability on lossy networks at near-zero cost, which is why `1400` is the default. But bulk multi-MB streams pay for it: a 6 MB raw camera frame becomes ~4500 datagrams instead of ~100, costing roughly 10x the sender/receiver CPU. The cap is participant-wide, so same-host **shared-memory** traffic fragments at the same ~1.4 KB granularity even though no MTU is involved — an SHM-only multi-MB pipeline pays the same overhead and may equally prefer raising the cap. Hosts publishing such streams over clean, loss-free links (e.g. a wired lab bench) can set `PROVIZIO_DDS_MAX_MESSAGE_SIZE=65500` (Fast-DDS's maximum) to restore the single-datagram-per-64-KB behaviour and reclaim that CPU at the cost of loss resilience. Conversely, the cap can be lowered on paths with a smaller MTU — the cap is the UDP payload size, so it plus 28 bytes of UDP/IPv4 headers must fit the path MTU (e.g. `1350` keeps strict single-frame delivery through a WireGuard tunnel with its default 1420 MTU; exceeding a path's MTU is benign but splits each datagram into two IP fragments). One more sizing constraint: the participant's own discovery announcement grows with its addressed-interface count (~56 bytes per extra interface) and must fit the cap in one message, so the `1400` default accommodates roughly 15 addressed interfaces beyond a typical baseline — hosts with unusually many (dense container/VM networking) should raise the cap accordingly.

### Shared-Memory Cleanup

Fast-DDS never garbage-collects the shared-memory files of a participant that died without destroying itself. Every unclean process death — `SIGKILL`, a bare `exit()`, an uncaught exception — leaks that participant's data segment (~33.5 MiB at the default transport configuration), its lock file, and often its port files, **forever**. A service that exits and is restarted in a loop therefore fills `/dev/shm`, and once it is full every new participant *on the host* fails to register the shared-memory transport and silently falls back to UDP — a host-wide degradation with no symptom other than an obscure `Failed to create segment` line on the dying process's own stderr. (Measured on a deployed unit: 41,642 orphaned files, 3.87 GB, at ~850 files/hour.) eProsima's answer is to run `fastdds shm clean` by hand; provizio_dds runs the same algorithm automatically instead.

Every participant that may use shared memory sweeps the shared-memory directory **once per process, immediately before creating its first participant** — so a service restarted in a loop buries its own predecessor's corpse and the steady state is at most one dead generation, not unbounded growth — and again, rate-limited to once per 30 s, whenever it finds the filesystem nearly full, so a long-running process heals its host rather than only complaining about it. It is silent whatever it reclaims: this is housekeeping you neither asked for nor can act on, and what it removes is by definition unreachable by any live process.

Fast-DDS keeps a companion lock file beside every segment and port and holds an `flock()` on it for the owner's whole lifetime; the kernel releases flocks on process death, `SIGKILL` included. So a lock file that *can* be locked provably has no live owner. For segments that settles it. For **ports** it is Fast-DDS's own contract rather than a proof — a port opened for writing takes no lock at all, so the sweep inherits exactly the verdict `SharedMemGlobal::Port::is_zombie` reaches from the same evidence. On top of that:

- only the exact Fast-DDS lock-file name shapes are considered — `fastdds_<16 chars>_el` for a segment and `fastdds_port<N>_el|_sl` for a port, plus the Fast-DDS 2.x `fastrtps_` equivalents. Every other file in the directory, `fast_datasharing_*` segments in particular, is left strictly alone. (Narrower than `fastdds shm clean`, which also accepts `_sl` for segments: Fast-DDS never gives a segment a shared lock, so that name is free for the taking *while the segment is alive* — precisely what someone would need to aim a sweep at a live participant.);
- a lock file younger than `PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC` is skipped, which closes the microsecond window in which a participant has created its segment but not yet taken its lock — a participant caught there would lose its segment and silently spend the rest of its life unreachable over shared memory. The guard is short (5 s) so a corpse is reclaimed by the very next incarnation of an exit-looping service rather than lingering for several;
- the lock file must be one Fast-DDS could have written — a regular file, empty, and not hardlinked onto something else — and each companion is removed only if it belongs to the same user as its lock file, so nobody can steer the sweep by choosing a *name*;
- the directory is opened once and every lookup is made relative to that descriptor, so the sweep cannot be redirected mid-run; the lock is held across the unlink; and concurrent sweeps in any number of processes are harmless;
- Linux and macOS only — Windows uses different paths and locking semantics. In practice it runs on Linux, the only platform where this library selects shared memory; the macOS path exists for a participant that opts back into it through `FASTDDS_BUILTIN_TRANSPORTS` or an XML profile.

One deliberate limitation: a port file whose lock file is already gone is never reclaimed. A participant that opens a port for *writing* creates no lock file, so such a port may well be in use — and unlike segment names (random, so they accumulate without bound), port names are derived from the domain and are reused rather than multiplied.

| Environment variable | Default | Meaning |
|---|---|---|
| `PROVIZIO_DDS_SHM_CLEANUP` | `on` | Enable / disable the automatic sweep process-wide. `off` / `0` / `false` / `no` disable it; an unrecognised value leaves it enabled (with a logged warning), since a typo must not silently reintroduce the leak. |
| `PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC` | `5` | How long (seconds) a lock file must have been untouched before an unlocked one counts as a corpse — the safety margin over the microsecond window in which a starting participant has created its segment but not yet locked it. Rarely worth changing. `0` removes the guard entirely, which re-opens that window; values above one day are clamped to it (a guard that large would wrap `time_t` on 32-bit targets and invert the check). |

## Discovery Tuning

Participants discover each other over best-effort multicast (SPDP), so some announcements are lost on a busy or lossy network. Fast-DDS counters this with an **initial burst** of announcements sent once at participant creation, plus a **periodic re-announcement** thereafter. Both are levers: set too high, the discovery traffic itself becomes a primary source of UDP congestion once many participants (sensors + clients) run at once — the initial burst is paid on every participant creation (including each [network-recovery](#network-auto-recovery) reset, which recreates the participant), and the periodic re-announcement is paid by every participant forever, so its multicast rate scales with the participant count.

provizio_dds therefore uses de-escalated defaults — a modest initial burst of **15 announcements 100 ms apart** (still well above Fast-DDS's own default of 5, so a lossy link gets several shots through) and a relaxed periodic re-announcement of **3 s** (the Fast-DDS default). The participant lease duration is also raised from the Fast-DDS default of 20 s to **30 s**, giving the relaxed cadence more margin before a peer is wrongly declared lost when announcements are dropped (at the cost of detecting a genuinely-dead peer ~10 s later). For most deployments these need no tuning. When they do — an unusually large fleet, or a particularly lossy link — all four are overridable at runtime via environment variables (read once at participant creation; ignored, with a logged warning, when unset or malformed):

| Environment variable | Default | Meaning |
|---|---|---|
| `PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT` | `15` | Number of announcements in the one-time initial burst. Raise on very lossy links; lower to reduce the startup/reset burst. |
| `PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS` | `100` | Spacing (milliseconds) between the initial-burst announcements. |
| `PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS` | `3000` | Period (milliseconds) of the steady-state periodic re-announcement. Raise to cut steady-state congestion in large fleets; lower to speed up late-joiner discovery. |
| `PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS` | `30000` | Lease duration (milliseconds) — how long a peer is considered alive without a fresh announcement. Raise for more tolerance of dropped announcements; must stay longer than the re-announcement period. |

Keep the periodic re-announcement period shorter than the participant lease duration (30 s by default), or peers can be declared lost in the gap between announcements; a warning is logged if an override crosses that threshold. These settings are skipped entirely when discovery is configured through an [XML profile](#xml-profiles) instead.

```bash
# Example: a large fleet trading slightly slower late-joiner discovery for less steady-state traffic
PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS=6000 my_app
```

## Logging

provizio_dds logs sparingly and on purpose: **a healthy process says nothing about its own internals.** Nothing is emitted for start-up state, successful internal operations, or events the library handled by itself — those are its internals, and they are not your concern while it is working. What you do get is limited to things that either need your attention or change what the library can do for you:

- **your configuration was rejected** — an unparseable or out-of-range `PROVIZIO_DDS_*` value, naming the variable and the default used instead;
- **the host is limiting the library** — the kernel capping the requested socket buffers (with the `sysctl` to raise), or a shared-memory filesystem too full to register the transport;
- **something you gave us threw** — an exception out of any of your callbacks, caught at the library boundary (see below);
- **functionality was lost** — a participant that could not be created or rebuilt, a network monitor that could not start, auto-recovery unavailable for the process;
- **the network changed and participants were rebuilt** — one line per actual reset, since communication is briefly interrupted by it;
- **the transports leave an interface out** — one info line per distinct set of excluded [VPN and tunnel interfaces](#vpn-and-tunnel-interfaces), naming them and saying whether netmask filtering came with the exclusion, at participant creation on every host with a tunnel up; and a warning where a requested transport mode could not be applied because the transports are yours. These are the two lines a healthy process can emit, so a callback that forwards everything to an alerting channel should expect them.

By default, info and warning messages go to `std::cout` and errors go to `std::cerr`, all prefixed with `[provizio_dds]`. To route the output into your application's logging system, install a callback:

```C++
#include "provizio/dds/logging.h"

provizio::dds::set_log_callback(
    [](provizio::dds::log_level level, std::string_view message) {
        switch (level) {
            case provizio::dds::log_level::info:    my_logger.info(message);    break;
            case provizio::dds::log_level::warning: my_logger.warning(message); break;
            case provizio::dds::log_level::error:   my_logger.error(message);   break;
        }
    });
```

`set_log_callback` returns the previously installed callback; passing an empty callback restores the default stdout/stderr emitter. The callback may be invoked from any thread and must be reentrant; do any heavy work in your own background thread.

The Python binding mirrors this:

```Python
import provizio_dds

def on_log(level, message):
    if level == provizio_dds.LogLevel.INFO:    my_logger.info(message)
    elif level == provizio_dds.LogLevel.WARNING: my_logger.warning(message)
    elif level == provizio_dds.LogLevel.ERROR:   my_logger.error(message)

provizio_dds.set_log_callback(on_log)
```

Exceptions thrown from any user-supplied callback — data handlers, publisher/subscriber match notifications, the endpoint-discovery callback — are caught at the library boundary and reported through this log callback (at error level) instead of propagating into the Fast-DDS background threads, where an uncaught exception would terminate the process. Your callbacks can therefore throw without crashing the application, though handling errors within them is still preferable.

For details see [include/provizio/dds/logging.h](include/provizio/dds/logging.h) and [python/network_recovery.py](python/network_recovery.py).

## XML Profiles

As DDS allows for configuring many of its parameters, you can optionally define a custom XML profile and enable it via setting its path to environment variable `FASTDDS_DEFAULT_PROFILES_FILE` (renamed from `FASTRTPS_DEFAULT_PROFILES_FILE` in Fast-DDS 3.x). Make sure to set the `participant` tag's attribute `is_default_profile` to `true` to activate the profile automatically, f.e.:

```XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="my_custom_profile" is_default_profile="true">
        <rtps>
        ...
        </rtps>
    </participant>
</profiles>
```

For more details, please refer to [Fast-DDS documentation](https://fast-dds.docs.eprosima.com/en/v3.6.2/fastdds/xml_configuration/xml_configuration.html).
Please note that some of the changes may break compatibility between publishers and subscribers, unless applied on both sides.

You may see how this functionality is used to activate Discovery-Server-based participants discovery in provizio_dds tests:

- [test/congested_network_test/fast_dds_server_config.xml](test/congested_network_test/fast_dds_server_config.xml)
- [test/congested_network_test/fast_dds_client_config.xml](test/congested_network_test/fast_dds_client_config.xml)

## Discovering Endpoints and Known Types

For tools that don't know their topic list up front — recorders, bridges, monitors — the domain participant can report remote DDS endpoints as they appear and disappear on the network, and tell you which message types the process is able to handle.

Register an `on_discovered_endpoint` callback and it fires whenever a remote endpoint of the requested kind(s) is discovered or removed. The callback receives the topic name, the wire-format type name, the endpoint kind (data writer / data reader), whether it appeared or disappeared, and the endpoint's reliability and durability QoS — enough for a recording bridge to create a matching reader/writer per topic. `is_known_type` / `known_types` let the callback filter the stream down to the types it can actually deserialise.

**C++ Example:**

```C++
#include "provizio/dds/domain_participant.h"
#include <iostream>

int main()
{
    // Installing the callback at construction (rather than calling
    // on_discovered_endpoint() afterwards) guarantees no endpoint already on
    // the network is missed — the listener is attached before discovery starts.
    auto participant = provizio::dds::make_domain_participant(
        0,                                                        // DDS domain id
        provizio::dds::network_recovery_mode::env_var_controlled, // Network auto-recovery
        [](provizio::dds::domain_participant &participant, const std::string &topic_name,
           const std::string &type_name, provizio::dds::endpoint_kind /*kind*/, bool discovered,
           eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability,
           eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
            // Only react to types this process can actually deserialise
            if (discovered && participant.is_known_type(type_name))
            {
                const bool reliable = reliability == eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
                std::cout << (reliable ? "reliable" : "best-effort") << " publisher on " << topic_name
                          << " (" << type_name << ")\n";
            }
        });

    std::cin.get(); // Wait for any user input
    return 0;
}
```

By default the callback fires for remote DataWriters only (the right choice for a recorder). Pass `provizio::dds::endpoint_kind::data_writer | provizio::dds::endpoint_kind::data_reader` as the following argument to receive both.

**Python Example:**

```Python
import provizio_dds

def on_endpoint(participant, topic_name, type_name, kind, discovered, reliability, durability):
    # Only react to types this process can deserialise
    if discovered and participant.is_known_type(type_name):
        reliable = reliability == provizio_dds.RELIABLE_RELIABILITY_QOS
        print(f"{'reliable' if reliable else 'best-effort'} publisher on {topic_name} ({type_name})")

# Install at construction to avoid missing endpoints already on the network
participant = provizio_dds.make_domain_participant(initial_discovery_callback=on_endpoint)

input("Press Enter to continue...")
```

You can also register, replace, or unregister (with an empty / `None` callback) the handler after construction via `participant.on_discovered_endpoint(callback, kinds)` — at the cost of a tiny race window for endpoints discovered before the call.

**Notes:**

- The callback runs on the Fast-DDS discovery thread. Keep it short, don't block, and don't create endpoints or re-register the callback from inside it — filter and enqueue, then do the real work on your own thread.
- It survives network-recovery resets: the listener is re-attached to the recreated participant automatically.
- For C++ callers that want to construct a typed publisher/subscriber for a discovered type without hard-coding it, the build generates a `provizio::dds::visit_known_type<Visitor>(type_name, visitor)` dispatcher that routes a runtime type-name string to a templated visitor instantiated for every type shipped by `provizio_dds_idls`. See [cmake/known_types_dispatcher.h.in](cmake/known_types_dispatcher.h.in).

For details see [include/provizio/dds/domain_participant.h](include/provizio/dds/domain_participant.h).

## Cloning only what you need

Nothing in provizio_dds is derived from its git history — no `git describe`, no revision counting — so the history is pure transfer cost, and the transfer is where a build breaks: a network that stalls or a DNS lookup that fails partway through a large pack takes the whole build with it, and CMake's built-in clone retries have no backoff between attempts.

`GIT_SHALLOW TRUE` is the obvious answer and it is not enough here. CMake implements it as `--depth 1 --no-single-branch` (see `ExternalProject/shared_internal_commands.cmake`), and that second flag fetches the tip of **every** branch and **every** tag — 38 refs at the time of writing. That is free for a repository whose refs are all small. It is not free for this one: every ref carries its own committed binary cache (the prebuilt archives under `cache/`, around 160 MB of them), and the tips only share objects where those archives happen to be identical.

Measured against this repository, cloning `develop`:

| clone | transferred | time |
| ----- | ----------- | ---- |
| `--depth 1 --single-branch --branch develop` | 112 MB | 5 s |
| `--depth 1 --no-single-branch` (what `GIT_SHALLOW` runs) | 1006 MB | 124 s |

Nine times the bytes and twenty times the wait, for refs no build reads. Neither `ExternalProject_Add` nor `FetchContent_Declare` can be told to drop `--no-single-branch`, so the [Importing example](README.md#importing) does the clone itself from the download step — two commands, no extra file to keep:

```CMake
DOWNLOAD_COMMAND "${CMAKE_COMMAND}" -E remove_directory "${PROVIZIO_DDS_SOURCE_DIR}"
         COMMAND "${GIT_EXECUTABLE}" clone --depth 1 --single-branch
                 --branch "${PROVIZIO_DDS_GITHUB_BRANCH}"
                 "https://github.com/${PROVIZIO_DDS_GITHUB_PROJECT}.git"
                 "${PROVIZIO_DDS_SOURCE_DIR}"
```

The `remove_directory` is what makes the step re-runnable: git refuses to clone into a non-empty directory, so without it a second run of the download step would fail on the tree the first one left behind.

That second run is rarer than it looks, and this does **not** re-clone on every build. `DOWNLOAD_COMMAND` is a build step rather than a configure step, so `cmake` configure clones nothing at all; the clone happens on the first `cmake --build` and is then stamped, so later builds — and reconfigures that change nothing — skip it. It re-runs when the command itself changes, which is what editing `PROVIZIO_DDS_GITHUB_BRANCH` does, and that is exactly when a fresh clone is wanted. Stock `GIT_REPOSITORY`/`GIT_TAG` behaves the same way, for the same reason: CMake tracks its git info as a dependency of the same step.

What this gives up is `ExternalProject`'s own clone retry, which a hand-rolled `DOWNLOAD_COMMAND` does not inherit. That is a fair trade at a ninth of the bytes, since most of what makes a clone fail is how long it stays open, but if you want both, move those two commands into a `cmake -P` script and wrap the clone in a retry loop.

The one constraint is the same one `GIT_SHALLOW` has, for the same reason: a depth-1 clone fetches ref tips only, so `PROVIZIO_DDS_GITHUB_BRANCH` must name a **branch or tag, never a commit hash** — a hash is reachable only for as long as it happens to be a tip. To pin a raw SHA, clone normally (`GIT_REPOSITORY`/`GIT_TAG` with no `GIT_SHALLOW`) and pay for the history.

Leave the binary cache alone while you are at it. The `CMAKE_ARGS` in that example deliberately do **not** pass `IGNORE_BIN_CACHE`: the cache is what makes this dependency cheap to build, and the clone you just trimmed is what delivers it. Only a consumer that needs provizio_dds's from-source Fast-DDS layout — its `fastdds`/`fastcdr` CMake package configs, which the prebuilt archives do not ship — has any reason to force a source build, and that is a rare case rather than a default worth documenting here.

provizio_dds resolves its own dependencies (Fast-DDS, Fast-DDS-python, foonathan_memory, provizio_dds_idls) shallowly for the same transfer-cost reasons, and those repositories carry no per-ref payload, so plain `GIT_SHALLOW` is the right tool for them.

For Python there is nothing to add: pip already clones without the history's file contents (`git clone --filter=blob:none`, for git 2.17 and newer), so the bulk of a full clone is never transferred. pip has no `--depth` equivalent to pass, so if you want a genuinely shallow tree — a slow or metered link, say — clone it yourself and install the directory:

```Bash
git clone --depth 1 --branch TAG_or_BRANCH https://github.com/provizio/provizio_dds.git
python3 -m pip install -v ./provizio_dds
```
