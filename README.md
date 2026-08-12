# provizio_dds

C++ and Python library for [DDS communication](https://www.dds-foundation.org/what-is-dds-3/) in Provizio customer-facing APIs and internal Provizio software components. Built using
[eProsima Fast-DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds) DDS implementation (Apache
License 2.0).

Although based directly on DDS, it's compatible with [ROS2](https://docs.ros.org/en/rolling/) and provides all ROS2
built-in data types.

| CI Status: master | CI Status: develop |
| ----------------- | ----------------- |
| ![CI Status: master](https://github.com/provizio/provizio_dds/actions/workflows/ci.yml/badge.svg?branch=master) | ![CI Status: develop](https://github.com/provizio/provizio_dds/actions/workflows/ci.yml/badge.svg?branch=develop) |

## Provizio DDS API

[Table of DDS topics and their respective data types used in Provizio API](https://github.com/provizio/provizio_dds_idls/blob/master/TOPICS.md)

## Build dependencies

**C++ (Linux / macOS):**

- CMake (>= 3.15)
- Git
- C++ 17 compiler (gcc, clang, or MSVC)
- libssl-dev (OpenSSL development headers)
- eProsima Fast-DDS 3.x (provizio_dds builds against the Fast-DDS 3.x API): when a Fast-DDS installation is present it will be used, otherwise it's downloaded and built automatically. Note that the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable was renamed to `FASTDDS_DEFAULT_PROFILES_FILE` in Fast-DDS 3.x (see [XML Profiles](#xml-profiles)).

**C++ (Windows):**

- Visual Studio 2019+ (MSVC) with C++ 17 support
- CMake (>= 3.15)
- Git
- Ninja build system
- OpenSSL (install via `install_dependencies.ps1` or manually provide headers/libs)
- SWIG 4.0+ (SWIG 4.4+ required when using Python 3.14 or newer; only if building Python bindings)

**Python (Linux / macOS):**

- Git
- Python 3
- Pip 3
- unzip

**Python (Windows):**

- Git
- Python 3
- Pip 3
- All C++ (Windows) dependencies above (pip install builds from source)

When using non-binary-prebuilt configurations (i.e. any macOS, Linux Debug, non-x64/non-aarch64, non-default provizio_dds_idls) all the C++ dependencies will also be required.

There is a convenience Bash script to install all dependencies in *apt*-featuring Linux and macOS. In Linux it's to be executed with root privileges, f.e. using `sudo`.

**Windows:** Use the PowerShell script from an elevated (Administrator) prompt:

```PowerShell
.\install_dependencies.ps1 [-WithPython ON|OFF]
```

This installs OpenSSL (if not already available), SWIG (when Python is enabled), and other required tools. When static analysis is enabled, it also installs LLVM and cppcheck.

Use as:

```Bash
<PATH_TO_PROVIZIO_DDS>/install_dependencies.sh [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON] [INSTALL_ROS=OFF|ON] [FAST_DDS_INSTALL=OFF|ON|install_path]
```

Or download and execute (assuming `curl` is present):

```Bash
curl -s https://raw.githubusercontent.com/provizio/provizio_dds/<BRANCH_OR_TAG>/install_dependencies.sh | [sudo] bash -s [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON] [INSTALL_ROS=OFF|ON] [FAST_DDS_INSTALL=OFF|ON|install_path]
```

For example, for Python-enabled builds from non-root Linux user:

```Bash
curl -s https://raw.githubusercontent.com/provizio/provizio_dds/master/install_dependencies.sh | sudo bash -s ON
```

## Importing

**C++ (CMake):**

```CMake
# Resolve provizio_dds (https://github.com/provizio/provizio_dds)
set(PROVIZIO_DDS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_build")
set(PROVIZIO_DDS_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds")
set(PROVIZIO_DDS_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
set(PROVIZIO_DDS_GITHUB_PROJECT "provizio/provizio_dds")
set(PROVIZIO_DDS_GITHUB_BRANCH "master") # Or a specific tag or branch you prefer
set(PROVIZIO_DDS_INSTALL_DIR "${PROVIZIO_DDS_BINARY_DIR}/install")
ExternalProject_Add(libprovizio_dds
    GIT_REPOSITORY "https://github.com/${PROVIZIO_DDS_GITHUB_PROJECT}.git"
    GIT_TAG "${PROVIZIO_DDS_GITHUB_BRANCH}"
    UPDATE_COMMAND ""
    PREFIX "${PROVIZIO_DDS_PREFIX}"
    SOURCE_DIR "${PROVIZIO_DDS_SOURCE_DIR}"
    BINARY_DIR "${PROVIZIO_DDS_BINARY_DIR}"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_DDS_INSTALL_DIR}" "-DENABLE_TESTS=OFF" "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON"
)
add_dependencies(<YOUR_CMAKE_TARGET> libprovizio_dds)
target_include_directories(<YOUR_CMAKE_TARGET> SYSTEM PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/include")
target_link_directories(<YOUR_CMAKE_TARGET> PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/lib")
if(WIN32)
    # On Windows the installed import libraries are versioned (e.g.
    # fastdds-3.6.lib, fastcdr-2.3.lib). The ExternalProject above is a
    # standalone CMake build, so its internal unversioned imported targets
    # don't propagate here — link the versioned names directly. Replace
    # the trailing numbers if you pinned a different Fast-DDS version.
    target_link_libraries(<YOUR_CMAKE_TARGET> PUBLIC provizio_dds provizio_dds_types fastdds-3.6 fastcdr-2.3)
else()
    target_link_libraries(<YOUR_CMAKE_TARGET> PUBLIC provizio_dds provizio_dds_types fastdds fastcdr)
endif()
```

**Python (pip):**

```Bash
python3 -m pip install -v git+https://github.com/provizio/provizio_dds.git
```

or

```Bash
python3 -m pip install -v git+https://github.com/provizio/provizio_dds.git@TAG_or_BRANCH
```

On Windows, use `python` instead of `python3`:

```PowerShell
python -m pip install -v git+https://github.com/provizio/provizio_dds.git
```

## Publishing Data

**C++ Example:**

```C++
#include "provizio/dds/publisher.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>

int main()
{
    // Make a DDS Publisher
    auto publisher = provizio::dds::make_publisher<
        std_msgs::msg::StringPubSubType>(           // DDS Pub/Sub Type
        provizio::dds::make_domain_participant(),   // DDS Domain Participant
        "rt/chatter"                                // DDS Topic Name
    );

    // Create a message
    std_msgs::msg::String str;
    str.data("Hello World!");

    // Publish the message
    publisher->publish(str);

    return 0;
}
```

For more details see [provizio/dds/publisher.h](include/provizio/dds/publisher.h).

**Python Example:**

```Python
import provizio_dds

# Make a DDS Publisher
publisher = provizio_dds.Publisher(
    provizio_dds.make_domain_participant(), # DDS Domain Participant
    "rt/chatter",                           # DDS Topic Name
    provizio_dds.StringPubSubType)          # DDS Pub/Sub Type

# Create a message
message = provizio_dds.String()
message.data("Hello World!")

# Publish the message
publisher.publish(message)
```

For more details see [python/provizio_dds.py](python/provizio_dds.py) and [test/python/python_publisher.py](test/python/python_publisher.py).

**Notes:**

- Publishers support configurable reliability QoS (BEST_EFFORT or RELIABLE; RELIABLE by default). See `provizio::dds::make_publisher` (C++) or `provizio_dds.Publisher` (Python) for parameters.
- Durability and history depth are now configured **independently** (they used to be a single combined parameter):
  - `durability_kind` selects the DDS durability QoS — pass `TRANSIENT_LOCAL_DURABILITY_QOS` for late-joiner delivery or `VOLATILE_DURABILITY_QOS` to force volatile. Leave it unset (`std::nullopt` in C++, omit / `None` in Python) to keep the Fast-DDS / XML-profile default.
  - `history_depth` controls the KEEP_LAST history depth only. Pass `use_default_history_depth` (`-1`; `USE_DEFAULT_HISTORY_DEPTH` in Python), or any non-positive value, to keep the default depth (the per-type default where one exists — see the large-data note below — otherwise Fast-DDS's default); pass a positive value for an explicit KEEP_LAST depth.
- Large-sample types (`sensor_msgs::msg::Image`, `CompressedImage`, `MultiEchoLaserScan`, `PointCloud2`, `nav_msgs::msg::OccupancyGrid`, and the `geometry_msgs::msg::PolygonStamped` / `PolygonInstanceStamped` freespace polygons) default to **ASYNCHRONOUS** publishing with a small **KEEP_LAST(4)** history, so a multi-megabyte write hands off to the participant's async sender thread and a momentarily slow consumer doesn't drop frames. These are writer-local defaults and do not affect reader/writer matching or ROS 2 interoperability.
- You can optionally receive subscriber match/unmatch notifications. See `provizio::dds::make_publisher` (C++) or `provizio_dds.Publisher` (Python) for parameters.

## Receiving Data

**C++ Example:**

```C++
#include "provizio/dds/subscriber.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <iostream>

int main()
{
    // Make a DDS Subscriber
    const auto subscriber = provizio::dds::make_subscriber<
        std_msgs::msg::StringPubSubType>(           // DDS Pub/Sub Type
        provizio::dds::make_domain_participant(),   // DDS Domain Participant
        "rt/chatter",                               // DDS Topic Name
        [&](const std_msgs::msg::String &message) { // Message handler (takes DDS Data Type as a const reference)
            // Print the received message
            std::cout << message.data() << std::endl;
        });
    std::cin.get(); // Wait for any user input

    return 0;
}
```

For more details see [provizio/dds/subscriber.h](include/provizio/dds/subscriber.h).

**Python Example:**

```Python
import provizio_dds

subscriber = provizio_dds.Subscriber(
    provizio_dds.make_domain_participant(), # DDS Domain Participant
    "rt/chatter",                           # DDS Topic Name
    provizio_dds.StringPubSubType,          # DDS Pub/Sub Type
    provizio_dds.String,                    # DDS Data Type
    lambda message: print(message.data()))  # Message handler (takes a DDS Data Type object), prints the received message
input("Press Enter to continue...") # Wait for any user input
```

For more details see [python/provizio_dds.py](python/provizio_dds.py) and [test/python/python_subscriber.py](test/python/python_subscriber.py).

**Notes:**

- The data callback can take either one argument (the data) or two (data and `SampleInfo`). Both are supported in C++ and Python bindings.
- **By default a subscriber adopts the discovered publisher's reliability.** Rather than creating its DataReader eagerly, it defers creation until a matching remote DataWriter is discovered on its topic, then builds the reader with that writer's offered reliability (the first-discovered writer's reliability is adopted while it stays live; if it later leaves and only a differently-configured writer remains, a newly-created default subscriber re-derives the reliability from a still-live writer so it still matches — while an already-built reader keeps its own reliability for its lifetime). A default subscriber is therefore automatically reliable against a RELIABLE publisher and best-effort against a best-effort one, with no configuration and without the reliability mismatch that would otherwise prevent matching. Pass an explicit `reliability_kind` (`BEST_EFFORT_RELIABILITY_QOS` / `RELIABLE_RELIABILITY_QOS`) to opt out and get an eagerly-created reader with that fixed reliability (the previous behaviour).
- Durability and history depth are configured **independently** (as for publishers above): `durability_kind` selects the durability QoS (unset keeps the default), and `max_history_depth` controls only the KEEP_LAST history depth (`-1` / non-positive keeps the default depth; a positive value sets an explicit depth).
- You can optionally receive publisher match/unmatch notifications. See `provizio::dds::make_subscriber` (C++) or `provizio_dds.Subscriber` (Python) for parameters.

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

## Network Auto-Recovery

DDS participants bind their UDP transports to the set of network interfaces present at participant-creation time. If the host's network changes afterwards — the primary interface comes up after the application started, a DHCP lease arrives, a USB Ethernet adapter is plugged in, the host roams to a new network — Fast-DDS does not refresh those bindings, and affected participants stop discovering off-host peers until recreated.

provizio_dds handles this transparently. A process-wide background monitor watches the OS for interface **address and link-state** changes (netlink `RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR | RTMGRP_LINK` on Linux, `PF_ROUTE` incl. `RTM_IFINFO` on macOS, `NotifyUnicastIpAddressChange` + `NotifyIpInterfaceChange` on Windows), coalesces bursts of events (3 s of quiescence or up to 60 s of debounce), snapshot-diffs to filter out irrelevant churn (Docker / veth bridges, virtual interfaces, link-local IPv6), and on a confirmed change tears down and rebuilds the underlying Fast-DDS participant for every participant that opted in. Existing publisher and subscriber handles survive the rebuild — their internal Fast-DDS objects are swapped under the caller-held `shared_ptr` and the user-supplied callbacks are re-attached automatically.

An interface only counts as present while it is **operationally** up — carrier present, not merely administratively up (`IFF_RUNNING` on Linux/macOS, `OperStatus` on Windows). That deliberately matches how Fast-DDS itself enumerates interfaces, so the snapshot models exactly the set it will bind locators to. It is also why link-state events are subscribed to: powering on a network switch, or replugging a cable, moves an interface in and out of that set while often emitting no address event at all.

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

The exclusions above are heuristics aimed at container and virtualization churn, and they are applied to the *change-detection* snapshot only — Fast-DDS still binds to whatever the OS offers. Tunnel interfaces (`tun`, `ip6tnl`) are deliberately **not** excluded, since a VPN endpoint routinely carries real DDS traffic. Bridges are, because `docker0` and `virbr0` are bridges — so if the interface your DDS traffic actually uses *is* a bridge (`br0` on a vehicle PC, `br-lan` on a router-like unit), name it explicitly so changes on it trigger a recovery:

```Bash
PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES=br0,br-lan my_app
```

Comma-separated, whitespace around entries ignored. A named interface bypasses the name / kind / adapter-type exclusions but still has to be operationally up, non-loopback, and carrying a non-link-local address. On Windows, match either the adapter's friendly name or its GUID-style name.

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

**Cost during a reset:** typical end-to-end recovery time is a few seconds — about 3 s of event coalescing plus the time Fast-DDS needs for rediscovery and TypeLookup against the new participant. Each reset is logged (see [Logging](#logging) below).

For details see [include/provizio/dds/network_recovery.h](include/provizio/dds/network_recovery.h).

## Transport Selection

By default a participant uses the platform's standard transports: shared memory plus UDPv4 on Linux, and UDPv4 only on Windows/macOS (where shared memory is disabled to avoid a Boost.Interprocess cleanup bug). On every platform the UDP transport is tuned with enlarged (16 MiB) socket buffers so reliable delivery of large samples — camera frames, point clouds — works across hosts without extra configuration.

Pass `transport_mode::udp_only` (C++) / `TransportMode.UDP_ONLY` (Python) to `make_domain_participant` to disable shared memory on every platform. This helps when a participant bridges mismatched Fast-DDS major versions (e.g. a recorder relaying 2.x publishers), where cross-major shared-memory negotiation can degrade large-sample throughput.

```C++
#include "provizio/dds/domain_participant.h"

// Platform default — SHM + UDP on Linux, UDP-only on Windows/macOS:
auto participant_default = provizio::dds::make_domain_participant();

// UDP-only on every platform:
auto participant_udp_only = provizio::dds::make_domain_participant(
    0, provizio::dds::network_recovery_mode::env_var_controlled, {},
    provizio::dds::endpoint_kind::data_writer, provizio::dds::transport_mode::udp_only);
```

```Python
import provizio_dds

# Platform default:
participant_default = provizio_dds.make_domain_participant()

# UDP-only on every platform:
participant_udp_only = provizio_dds.make_domain_participant(transport=provizio_dds.TransportMode.UDP_ONLY)
```

The large-sample message types listed under [Publishing Data](#publishing-data) additionally default to asynchronous publishing with a small KEEP_LAST history, which complements the enlarged socket buffers for high-throughput data.

Three transport-level environment options are read once at participant creation. In C++ the first two are skipped when transports are configured through Fast-DDS's own `FASTDDS_BUILTIN_TRANSPORTS` variable or an [XML profile](#xml-profiles); in Python the socket-buffer request is applied *through* `FASTDDS_BUILTIN_TRANSPORTS` itself (an externally-set value always wins, and the first participant created in the process fixes it for the rest — the XML-profile skip does not apply). `PROVIZIO_DDS_MAX_MESSAGE_SIZE` caps the RTPS output path whichever transports carry it (XML-profile participants excepted):

| Environment variable | Default | Meaning |
|---|---|---|
| `PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE` | `16777216` | UDP send/receive socket buffer ceiling (bytes). The OS clamps it to `net.core.rmem_max` / `wmem_max` — raise those sysctls for the request to take full effect (C++ participants log a warning when they cap it). |
| `PROVIZIO_DDS_SHM_SEGMENT_SIZE` | derived by Fast-DDS | Shared-memory segment size (bytes) per participant, decoupled from the UDP socket buffer request Fast-DDS otherwise derives it from (16 MiB of socket buffer produces a ~33.5 MiB segment). Lower it on `/dev/shm`-constrained hosts. C++ participants only: the Python bindings configure transports through `FASTDDS_BUILTIN_TRANSPORTS`, which exposes no segment-size option. |
| `PROVIZIO_DDS_MAX_MESSAGE_SIZE` | `1400` | Send-side cap (bytes) for RTPS messages, mapping to Fast-DDS's `fastdds.max_message_size` property (output path only — reception from differently-configured peers is unaffected, so mixed fleets and default-configured ROS 2 peers stay compatible). The default keeps every UDP datagram within a single ~1500-byte-MTU link frame, so samples above it travel as individually-retransmittable RTPS fragments instead of one large UDP datagram that the IP layer splits into many link frames: with IP fragmentation, sustained frame loss makes large-sample delivery all-or-nothing and can exhaust the receiving kernel's reassembly cache (`net.ipv4.ipfrag_high_thresh` / `ipfrag_time`), blacking out every fragmented topic in ~30 s cycles while single-frame topics keep flowing — the dominant failure mode for radar point clouds on lossy links (Wi-Fi, embedded routers). Measured at 10% injected frame loss with 28 KB point clouds: ~36% delivery in blackout cycles at `65500` vs 99%+ at `1400`, p90 latency 160 ms. CycloneDDS keeps its fragments MTU-sized by default the same way (its `General/FragmentSize` default is 1344 bytes). Values below `576` are rejected (with a logged error): a complete discovery announcement must fit in one RTPS message, and Fast-DDS itself does not diagnose a property that breaks that. Values above Fast-DDS's `65500` maximum are clamped to it (with a logged warning). See the CPU trade-off note below. |

**Trade-off of the `1400` default — CPU when publishing very large samples.** The cap sets the size of every outgoing datagram, so per-datagram costs are paid per ~1.4 KB instead of per ~64 KB. Most messages published by Provizio components are far smaller than 64 KB, where the difference is negligible — radar point clouds and freespace polygons gain dramatically better deliverability on lossy networks at near-zero cost, which is why `1400` is the default. But bulk multi-MB streams pay for it: a 6 MB raw camera frame becomes ~4500 datagrams instead of ~100, costing roughly 10x the sender/receiver CPU. The cap is participant-wide, so same-host **shared-memory** traffic fragments at the same ~1.4 KB granularity even though no MTU is involved — an SHM-only multi-MB pipeline pays the same overhead and may equally prefer raising the cap. Hosts publishing such streams over clean, loss-free links (e.g. a wired lab bench) can set `PROVIZIO_DDS_MAX_MESSAGE_SIZE=65500` (Fast-DDS's maximum) to restore the single-datagram-per-64-KB behaviour and reclaim that CPU at the cost of loss resilience. Conversely, the cap can be lowered on paths with a smaller MTU — the cap is the UDP payload size, so it plus 28 bytes of UDP/IPv4 headers must fit the path MTU (e.g. `1350` keeps strict single-frame delivery through a WireGuard tunnel with its default 1420 MTU; exceeding a path's MTU is benign but splits each datagram into two IP fragments). One more sizing constraint: the participant's own discovery announcement grows with its addressed-interface count (~56 bytes per extra interface) and must fit the cap in one message, so the `1400` default accommodates roughly 15 addressed interfaces beyond a typical baseline — hosts with unusually many (dense container/VM networking) should raise the cap accordingly.

### Shared-Memory Cleanup

Fast-DDS never garbage-collects the shared-memory files of a participant that died without destroying itself. Every unclean process death — `SIGKILL`, a bare `exit()`, an uncaught exception — leaks that participant's data segment (~33.5 MiB at the default transport configuration), its lock file, and often its port files, **forever**. A service that exits and is restarted in a loop therefore fills `/dev/shm`, and once it is full every new participant *on the host* fails to register the shared-memory transport and silently falls back to UDP — a host-wide degradation with no symptom other than an obscure `Failed to create segment` line on the dying process's own stderr. (Measured on a deployed unit: 41,642 orphaned files, 3.87 GB, at ~850 files/hour.) eProsima's answer is to run `fastdds shm clean` by hand; provizio_dds runs the same algorithm automatically instead.

Every participant that may use shared memory sweeps the shared-memory directory **once per process, immediately before creating its first participant** — so a service restarted in a loop buries its own predecessor's corpse and the steady state is at most one dead generation, not unbounded growth — and again, rate-limited to once per 30 s, whenever it finds the filesystem nearly full, so a long-running process heals its host rather than only complaining about it. When something is reclaimed, one info line reports the counts and bytes freed; a healthy host stays silent.

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

provizio_dds emits diagnostic messages from background threads (network-recovery monitor, coalescer, participant reset) as well as from a few error paths in the request/response code. By default, info and warning messages go to `std::cout` and errors go to `std::cerr`, all prefixed with `[provizio_dds]`. To route the output into your application's logging system, install a callback:

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

## Request/Response

The library provides a lightweight request/response API built on top of DDS topics. A service subscribes to a request topic and publishes replies to a response topic with correlation tracking; clients send requests and await replies.

### Sending a Request

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <chrono>
#include <iostream>

int main()
{
    auto participant = provizio::dds::make_domain_participant();
    std_msgs::msg::String request;
    request.data("hello");
    auto future = provizio::dds::request<std_msgs::msg::StringPubSubType,
                                    std_msgs::msg::StringPubSubType>(
        participant, "echo_service", request);

    if (future.wait_for(std::chrono::seconds{2}) == std::future_status::ready) {
        const auto &response = future.get();
        // Print the received message
        std::cout << response.data() << std::endl;
    }

    return 0;
}
```

**Python Example:**

```Python
import asyncio
import provizio_dds

async def main():
    participant = provizio_dds.make_domain_participant()
    request = provizio_dds.String()
    request.data("hello")
    response = await provizio_dds.request(
        participant,
        provizio_dds.StringPubSubType,
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        request,
        service_name="echo_service",
    )
    print(response.data())

asyncio.run(main())
```

### Reusable Client (`service_client`)

`request(...)` is a one-shot helper: it creates a fresh client, waits for matching, sends a single request, and tears the client down. To issue **many requests** — especially concurrently — create a `service_client` once, up front. It matches the service(s) in the background (so it is ready by the time you need it) and supports many in-flight requests at once, each returning its own future. Optionally call `wait_for_service(...)` to block until the service(s) are ready before sending; like `request(...)`, that readiness wait includes a settling window, so all services sharing the topic (e.g. multiple sensors keyed by `frame_id`) are discovered first.

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <chrono>
#include <iostream>
#include <vector>

int main()
{
    auto participant = provizio::dds::make_domain_participant();
    auto client = provizio::dds::make_service_client<std_msgs::msg::StringPubSubType,
                                                      std_msgs::msg::StringPubSubType>(
        participant, "echo_service");

    // Optional: block until the service(s) are ready (includes the settling window).
    client->wait_for_service(std::chrono::seconds{5});

    // Many requests, each returning its own future_response; all may be in flight at once.
    std::vector<provizio::dds::future_response<std_msgs::msg::StringPubSubType,
                                               std_msgs::msg::StringPubSubType>> futures;
    for (int i = 0; i < 10; ++i) {
        std_msgs::msg::String request;
        request.data("hello " + std::to_string(i));
        futures.push_back(client->request(request));
    }
    for (auto &future : futures) {
        if (future.wait_for(std::chrono::seconds{2}) == std::future_status::ready) {
            std::cout << future.get().data() << std::endl;
        }
    }

    return 0;
}
```

**Python Example:**

```Python
import asyncio
import provizio_dds

async def main():
    participant = provizio_dds.make_domain_participant()
    client = provizio_dds.ServiceClient(
        participant,
        provizio_dds.StringPubSubType,  # request PubSub type
        provizio_dds.StringPubSubType,  # response PubSub type
        provizio_dds.String,            # response data type
        service_name="echo_service",
    )

    # Optional: block until the service(s) are ready (includes the settling window).
    await client.wait_for_service(timeout_sec=5.0)

    # Many requests in flight at once; each request() is awaitable.
    requests = []
    for i in range(10):
        request = provizio_dds.String()
        request.data(f"hello {i}")
        requests.append(client.request(request))
    for response in await asyncio.gather(*requests):
        print(response.data())

asyncio.run(main())
```

### Creating a Service

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <iostream>

int main() {
    auto participant = provizio::dds::make_domain_participant();

    // Create an "echo" service using a single service name for both topics
    auto service = provizio::dds::make_service<std_msgs::msg::StringPubSubType,
                                               std_msgs::msg::StringPubSubType>(
        participant,
        "echo_service",
        [](const std_msgs::msg::String &request) {
            std_msgs::msg::String response;
            response.data(request.data());
            return response; // synchronous handler, return std::future for async requests handling
        }
    );

    std::cin.get(); // Wait for any user input

    return 0;
}
```

**Python Example:**

```Python
import provizio_dds

participant = provizio_dds.make_domain_participant()

def handle_request(request: provizio_dds.String) -> provizio_dds.String:
    response = provizio_dds.String()
    response.data(request.data())
    return response

service = provizio_dds.Service(
    participant,
    provizio_dds.StringPubSubType,  # request PubSub type
    provizio_dds.String,            # request data type
    provizio_dds.StringPubSubType,  # response PubSub type
    handle_request,                 # can be sync or async
    service_name="echo_service",
)

input("Press any key to finish...")

# Ensure you call service.stop() on exit to stop internal threads, otherwise the application will keep running
service.stop()
```

**Notes:**

- Both synchronous and asynchronous service handlers are supported. Asynchronous handlers return `std::future` (C++) or are `async def` coroutines (Python).
- Reliability is set to RELIABLE for response readers/writers by default. The client’s request Publisher uses default (volatile) durability, while the service’s response Publisher uses TRANSIENT_LOCAL durability with a small history (depth 10) for robust delivery (compatible with ROS 2).
- Per-endpoint durability and history are configurable on both sides. `make_service(...)` / `provizio_dds.Service(...)` and `request(...)` accept an optional `durability_kind` and an `endpoint_history_depth` (the KEEP_LAST history depth of the underlying request/response DataWriters/DataReaders); leaving them at the defaults preserves the behaviour described above. Note that `max_history_depth` is a separate knob — it bounds the request **queue size**, not the endpoint history (`minimal_request_queue` / `MINIMAL_REQUEST_QUEUE`, i.e. `0`, selects a minimal bounded queue).
- Before publishing the first request, a short graph-based “readiness” wait is performed to ensure endpoints are matched and stable, avoiding races immediately after discovery.
- Optionally, the client request API lets you enforce a post-match settling window and (if desired) a finite matching timeout before the first request publish. See C++ `provizio::dds::request(..., std::chrono::milliseconds stable_matches_period = 1000ms, std::chrono::milliseconds service_match_timeout = 0ms)`, and Python `provizio_dds.request(..., stable_matches_period_sec=1.0, service_match_timeout_sec=0.0)` for the settling window control (use 0 to skip the extra wait).
- For issuing many requests (especially concurrently), reuse a single client created up front: `provizio::dds::make_service_client(...)` (C++) or `provizio_dds.ServiceClient(...)` (Python) — see [Reusable Client](#reusable-client-service_client) above. Its `request(...)` returns a `future_response` (C++) or an awaitable (Python), supports many requests in flight at once, and `wait_for_service(timeout[, stable_matches_period])` blocks until ready (settling window included). Its endpoints default to a bounded KEEP_LAST history (depth 10, matching the service's default response history), so a burst of up to that many concurrent requests/responses is retained and the reliable writer applies back-pressure beyond it; raise `endpoint_history_depth` for higher concurrency.
- To drop a request silently from a service handler, throw `provizio::dds::ignore_request` (C++) or raise `provizio_dds.Service.IgnoreRequest` (Python). Such requests are discarded without warnings.

### ROS 2 compatibility

- Publish/subscribe is compatible with ROS 2 starting from Humble.
- Request/response is compatible with ROS 2 starting from Jazzy.
- For both publish/subscribe and request/response to interoperate with ROS 2, either:
  - Keep the default `max_history_depth` on subscribers and services/requests, or
  - Configure ROS 2 QoS to use TRANSIENT_LOCAL durability on the ROS 2 publishers with appropriate history depth for better reliability.
- When running provizio_dds binaries in a shell with a **sourced ROS 2 environment** (`setup.bash`), note that ROS 2 distros bundle their own eProsima Fast-DDS. If its major.minor version matches the Fast-DDS bundled by provizio_dds (e.g. ROS 2 Lyrical ships Fast-DDS v3.6.1 while provizio_dds bundles v3.6.2 — both named `libfastdds.so.3.6`), the sourced `LD_LIBRARY_PATH` makes provizio_dds binaries load the ROS-bundled library. Fast-DDS doesn't guarantee ABI stability across patch releases, so this can break discovery or crash. To make an installed provizio_dds immune to it, build with `-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=ON` (Linux): installed binaries then link Fast-DDS by its fully-qualified name (e.g. `libfastdds.so.3.6.2.0`), which never collides with the ROS-bundled copy. Linking both provizio_dds and ROS 2 libraries into the *same process* is a different problem — see [provizio_radar_api_ros2](https://github.com/provizio/provizio_radar_api_ros2) for the namespace-isolation approach it requires.

## Reading and Creating PointCloud2 Messages

Utilities for reading and creating `PointCloud2` messages — both generically (any field layout) and in the standard Provizio radar format (see [TOPICS.md of provizio_dds_idls](https://github.com/provizio/provizio_dds_idls/blob/master/TOPICS.md)):

### Python Example

```Python
import provizio_dds

# Reading Provizio radar point clouds: a structured numpy array, one record per point with named fields
# x, y, z, radar_relative_radial_velocity, signal_to_noise_ratio, ground_relative_radial_velocity
# (access by name, e.g. points["x"]). Use read_points_numpy(...) for a plain (N, 6) float matrix.
points = provizio_dds.point_cloud2.read_points(point_cloud2_message)

# Or as named tuples (slower, but self-descriptive: point.x, point.signal_to_noise_ratio, ...):
points = provizio_dds.point_cloud2.read_points_list(point_cloud2_message)

# Reading ANY PointCloud2 generically, selecting fields by name:
xyz = provizio_dds.point_cloud2.read_points(point_cloud2_message, field_names=["x", "y", "z"])

# Creating radar point clouds:
message = provizio_dds.point_cloud2.make_radar_point_cloud(
    provizio_dds.point_cloud2.make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"),
    [[1.0, 2.0, 3.0, 0.5, 7.0, 0.0]],  # any iterable of per-point field values
)

# Creating and reading Provizio entity clouds (radar / camera / fused):
from types import SimpleNamespace
entity = SimpleNamespace(
    entity_id=42, entity_class=3, x=1.5, y=-2.5, z=0.5,
    radar_relative_radial_velocity=10.5, ground_relative_radial_velocity=9.5,
    orientation=(0.1, 0.2, 0.3, 0.4), size=(4.5, 1.8, 1.5),
    camera_bbox=(float("nan"),) * 4, entity_confidence=90, entity_class_confidence=80,
    camera_entity_id=provizio_dds.point_cloud2.NO_ENTITY_ID,
)
entities_message = provizio_dds.point_cloud2.make_entities_from(
    provizio_dds.point_cloud2.make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"),
    "radar",
    [entity],
)
entities = provizio_dds.point_cloud2.read_entities(entities_message)
entity_id = entities[0].entity_id
orientation = entities[0].orientation  # (x, y, z, w); fields absent from the cloud kind read as NaN
                                       # (floats) or NO_ENTITY_ID (ids)
entity_kind = entities[0].kind()  # per-entity: "radar"/"camera"/"fused" (see also entities_kind for the whole cloud)
kind = provizio_dds.point_cloud2.entities_kind(entities_message)  # "radar", "camera", "fused" or None
```

Arbitrary field layouts can be created with `provizio_dds.point_cloud2.create_cloud`; the flat-row makers
`make_radar_entities` / `make_camera_entities` / `make_fused_entities` also exist for pre-flattened data.

For more details see [python/point_cloud2.py](python/point_cloud2.py) and [test/python/accumulation_test.py](test/python/accumulation_test.py).

### C++ Example

```C++
#include <provizio/dds/point_cloud2.h>
namespace pc2 = provizio::dds::point_cloud2;

// Reading Provizio radar point clouds:
const std::vector<pc2::radar_point> radar_points = pc2::read_radar_points(point_cloud2_message);

// Reading ANY PointCloud2 generically, field by field:
const pc2::cloud_view view{point_cloud2_message};
const auto x_field = view.field("x");
for (const auto point : view)
{
    const auto x = x_field ? point.get<float>(*x_field) : std::numeric_limits<float>::quiet_NaN();
    // ...
}

// Creating radar point clouds (any iterable range of radar_point; contiguous ranges are bulk-copied):
const auto message = pc2::make_radar_point_cloud(
    pc2::make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"), radar_points);

// Creating arbitrary clouds (rows in field order; values are converted to each field's datatype):
const auto fields = pc2::make_fields({{"x", sensor_msgs::msg::PointField_Constants::FLOAT32},
                                      {"y", sensor_msgs::msg::PointField_Constants::FLOAT32},
                                      {"intensity", sensor_msgs::msg::PointField_Constants::UINT16}});
const auto custom_cloud = pc2::create_cloud(pc2::make_header(0, 0, "frame"), fields,
                                            std::vector<std::array<float, 3>>{{1.0F, 2.0F, 3.0F}});

// Creating and reading Provizio entity clouds (radar / camera / fused):
pc2::entity an_entity;
an_entity.entity_id = 42;
an_entity.entity_class = 3;
an_entity.x = 1.5F; an_entity.y = -2.5F; an_entity.z = 0.5F;
an_entity.orientation = {0.1F, 0.2F, 0.3F, 0.4F};
an_entity.size = {4.5F, 1.8F, 1.5F};
an_entity.radar_relative_radial_velocity = 10.5F;
an_entity.ground_relative_radial_velocity = 9.5F;
an_entity.entity_confidence = 90; an_entity.entity_class_confidence = 80;
const auto entities_message =
    pc2::make_entities(pc2::make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"),
                       pc2::entities_kind::radar, std::vector<pc2::entity>{an_entity});
const std::vector<pc2::entity> entities = pc2::read_entities(entities_message);
const auto entity_id = entities[0].entity_id;  // fields absent from the cloud kind read as NaN (floats) or
                                               // pc2::no_entity_id (ids)
const auto entity_kind = entities[0].kind();  // per-entity: radar/camera/fused (see also get_entities_kind for the whole cloud)
const auto kind = pc2::get_entities_kind(entities_message);  // entities_kind::radar/camera/fused or nullopt
```

The flat-row makers `make_radar_entities` / `make_camera_entities` / `make_fused_entities` also exist for pre-flattened data.

For more details see [include/provizio/dds/point_cloud2.h](include/provizio/dds/point_cloud2.h) and [test/point_cloud2/point_cloud2_test.cpp](test/point_cloud2/point_cloud2_test.cpp).

## Points Accumulation and Multi-Radar Fusion

Point clouds accumulation keeps some of the reflected points (normally ones from static objects) "visible" for a number of frames after they were originally received. It makes point clouds much denser and features of objects much clearer - similar to long exposure of dark scenes in photography. Usually, accumulation requires at least 2 types of inputs: point clouds and localization (both odometry and GNSS are supported). In case of static ego or single frame multi-radar fusion, accumulation can be used without odometry/localization inputs. When Provizio radar odometry is used it effectively turns into a SLAM (Simultaneous Localization and Mapping) solution, but external localization/odometry source is also supported.
Accumulation is also used for fusion of point clouds from multiple radars in a vehicle. In this case the sensors' extrinsics calibration data is required.

### Example of Point Clouds Accumulation

![Point Clouds Accumulation](media/point_clouds_accumulation.png)

### Python Example

```Python
import provizio_dds

max_accumulate_frames_per_radar = 30
# By default, relies on rt/provizio_radar_odometry as a localization data source and rt/provizio_extrinsics as the sensors' extrinsics data source
points_accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(max_accumulate_frames_per_radar)

# Then some time later, when data was received
print(f"Accumulated, relative to the local frame:")
for point in points_accumulator.get_points_local_frame_relative():
    print(f"x = {point.position[0]}, y = {point.position[1]}, z = {point.position[2]}, ground_relative_velocity = {point.ground_relative_velocity}, snr = {point.snr}")
print(f"Accumulated, relative to the current ego position:")
for point in points_accumulator.get_points_ego_relative():
    print(f"x = {point.position[0]}, y = {point.position[1]}, z = {point.position[2]}, ground_relative_velocity = {point.ground_relative_velocity}, snr = {point.snr}")
```

Instead of polling, an `on_point_cloud` callback can be invoked on receiving and accumulating every radar point cloud. It's fired outside the accumulator's internal lock, so `get_points_local_frame_relative` / `get_points_ego_relative` may be called right from the callback:

```Python
def on_point_cloud(accumulator):
    print(f"{len(accumulator.get_points_ego_relative())} points accumulated")

points_accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
    max_accumulate_frames_per_radar, on_point_cloud=on_point_cloud
)
```

For more details see [python/accumulation.py](python/accumulation.py) and [test/python/accumulation_test.py](test/python/accumulation_test.py).

### C++ Example

```C++
#include <provizio/dds/accumulation.h>

const std::size_t max_accumulate_frames_per_radar = 30;
// By default, relies on rt/provizio_radar_odometry as a localization data source and rt/provizio_extrinsics as the sensors' extrinsics data source
provizio::dds::accumulation::dds_point_clouds_accumulator points_accumulator{max_accumulate_frames_per_radar};

// Then some time later, when data was received
std::cout << "Accumulated, relative to the local frame:" << std::endl;
for (const auto &point : points_accumulator.get_points_local_frame_relative())
{
    std::cout << "x = " << point.position[0] << ", y = " << point.position[1] << ", z = " << point.position[2]
              << ", ground_relative_radial_velocity = " << point.ground_relative_radial_velocity
              << ", signal_to_noise_ratio = " << point.signal_to_noise_ratio << std::endl;
}
std::cout << "Accumulated, relative to the current ego position:" << std::endl;
for (const auto &point : points_accumulator.get_points_ego_relative())
{
    std::cout << "x = " << point.position[0] << ", y = " << point.position[1] << ", z = " << point.position[2]
              << ", ground_relative_radial_velocity = " << point.ground_relative_radial_velocity
              << ", signal_to_noise_ratio = " << point.signal_to_noise_ratio << std::endl;
}
```

Instead of polling, an `on_point_cloud` callback can be invoked on receiving and accumulating every radar point cloud. It's fired outside the accumulator's internal lock, so the `get_points_*` getters may be called right from the callback. It runs on a DDS thread though: never block in it, and never create or destroy DDS endpoints (publishers, subscribers, participants) from it:

```C++
provizio::dds::accumulation::dds_accumulation_options options;
options.on_point_cloud = [](provizio::dds::accumulation::dds_point_clouds_accumulator &accumulator) {
    std::cout << accumulator.get_points_ego_relative().size() << " points accumulated" << std::endl;
};
provizio::dds::accumulation::dds_point_clouds_accumulator points_accumulator{max_accumulate_frames_per_radar,
                                                                             options};
```

The C++ accumulation maths uses [Eigen](https://eigen.tuxfamily.org/) when YOUR compiler can see it (auto-detected via `__has_include(<Eigen/Dense>)` at the point your code compiles the accumulation headers — e.g. `find_package(Eigen3)` + linking `Eigen3::Eigen` in your CMake; `install_dependencies.sh` installs it). Without Eigen it falls back to less efficient plain-CPU processing (provizio_dds's CMake warns about it at configure time). Define `PROVIZIO_DDS_DISABLE_EIGEN` (or configure provizio_dds with `DISABLE_EIGEN=ON`) to force the fallback deliberately. Prebuilt provizio_dds binaries are identical either way — the maths-path choice is made when YOUR code compiles, never baked into the library.

#### Localization frame and extrinsics

The localization source has its own extrinsics (sensor → ego frame), read by default from the same `rt/provizio_extrinsics` topic as the radars. For **odometry** the localization frame defaults to `provizio_radar_front_center`: radar-based odometry is published in the front-center radar's frame, so it shares that radar's calibration — the same `rt/provizio_extrinsics` transform is applied to both the odometry and that radar's point clouds. The localization extrinsics is **assumed identity until that frame's transform is received**; when it arrives (or later changes) any already-accumulated frames are **retroactively re-placed**, so none are lost. For **GNSS** (`NavSatFix`) the localization frame is instead learned from the first fix and, having no radar extrinsics, stays ego-relative (identity). When multiple localization sources share the topic, only the one whose `child_frame_id` (odometry) / `header.frame_id` (GNSS) matches `localization_frame_id` is used — the rest are dropped. Set `localization_frame_id` / `localization_extrinsics_topic` to override these defaults.

For more details see [include/provizio/dds/accumulation.h](include/provizio/dds/accumulation.h) and [test/accumulation/accumulation_test.cpp](test/accumulation/accumulation_test.cpp).

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
