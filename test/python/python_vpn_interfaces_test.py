#!/usr/bin/env python3
# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Subcommand-driven tests for keeping VPN / tunnel interfaces out of the DDS
transports — Python mirror of the C++ test/vpn_interfaces/ suite.

They cover the interface classifier, the transport profile a blocked address
produces, that a participant configured that way still communicates, the
PROVIZIO_DDS_ALLOW_VPN_INTERFACES override, and the matching exclusion from
network-recovery change detection. Each subcommand is its own ctest entry so
per-case failure stays isolated — and because the override is parsed once per
process, so a case can set it before anything reads it."""

import os
import random
import sys
import threading
import traceback

import provizio_dds
from provizio_dds import network_recovery as _network_recovery

# Per process, not fixed: the pub/sub case below puts real traffic on the wire, and this
# suite is deliberately NOT loopback-confined -- main() pops FASTDDS_DEFAULT_PROFILES_FILE so
# that the transport path under test is the one that actually runs. Loopback confinement would
# not be enough anyway: it does not separate two ctest jobs on ONE host, and CI schedules
# several configurations of the same runner concurrently, each running all ~25 cases of this
# suite. A fixed domain has them discover each other by construction. Matches
# accumulation_test.py and python_discovery_tuning_test.py, which randomise for the same
# reason.
#
# Drawn ABOVE every domain the rest of the suite pins, rather than from the whole 1-200 range
# those two use. The suites with a fixed domain are always on it, so an overlap with one of
# them is not a coincidence between two random draws but a standing collision that some
# fraction of runs will hit -- and 1-200 covers all of them: 14; 42; 44 (the C++
# test/vpn_interfaces/ suite, the closest relative of this one and therefore the likeliest to
# be running beside it); 46; 71; 72; and the 100-127 band that test/transport_tuning and the
# reliability suites derive a domain from as 100 + (pid % 28) -- a band rather than a walk,
# and not always even that, since request_response_reliability_single_service passes a literal
# seed of 200 and so lands on 104 every run.
#
# 42 is the one that makes the argument best. It is CROSS_COMPAT_DOMAIN_ID, shared by the
# eight test/python/cross_compat_*.py scripts, and cross_version_compat_test.py states
# outright that those run on their own domain and their own topic and service names so that
# it can execute CONCURRENTLY with the same-version suite. In CI it always runs, because a
# missing legacy venv there is a hard failure rather than a skip. So the collision it would
# cause is scheduled by design rather than merely possible.
#
# Across this suite's ~25 cases a 1-in-200 per-case chance is roughly 1 in 8 per job, which is
# exactly the class of rare, unattributable failure the randomisation was added to remove.
_LOWEST_UNPINNED_DOMAIN = 130
_HIGHEST_SAFE_DOMAIN = 200
DOMAIN = random.randint(_LOWEST_UNPINNED_DOMAIN, _HIGHEST_SAFE_DOMAIN)
ALLOW_ENV_NAME = "PROVIZIO_DDS_ALLOW_VPN_INTERFACES"

# TEST-NET-3 (RFC 5737), documentation-only: no host carries it, so a case can use
# it as a stand-in "VPN address" and get a deterministic result on every runner.
SYNTHETIC_BLOCKED_ADDRESS = "203.0.113.7"

VPN_NAMES = (
    "tailscale0",
    "tun0",
    "tun9",
    "tap0",
    "utun3",
    "ipsec1",
    "wg0",
    "ztppmkbrz2",
    "Tailscale",
    "WireGuard Tunnel",
    "OpenVPN TAP-Windows6",
    "TAP-Windows Adapter V9",
    "Tunnel adapter Teredo",
)

# Physical NICs, container plumbing and loopback must never be classified as VPNs:
# blocking a Docker bridge or a vehicle PC's br0 would break real same-host /
# same-LAN DDS paths, which is not what this feature is for.
# The NETGEAR strings are the reason the short conventions (tun / tap / wg) are
# matched only as prefixes: "NETGEAR WG111v3" contains "wg", and misclassifying a
# host's only NIC would drop ALL of its DDS traffic — far worse than the duplicate
# traffic this feature removes.
ORDINARY_NAMES = (
    "NETGEAR WG111v3 54Mbps Wireless USB 2.0 Adapter",
    "NETGEAR WGA600N Wireless Gaming Adapter",
    "Intel(R) Ethernet Connection I219-LM",
    "Marvell AQtion 10Gbit Network Adapter",
    "eth0",
    "eno1",
    "enp8s0",
    "wlp9s0",
    "wlan0",
    "docker0",
    "br0",
    "br-lan",
    "veth1a2b",
    "lo",
    "l4tbr0",
    "ppp0",
    "usb0",
    "rndis0",
    "can0",
    "Ethernet",
    "Wi-Fi",
)


def _fail(message: str) -> bool:
    print(f"FAIL: {message}", file=sys.stderr)
    return False


def test_classifier() -> int:
    """The classifier recognises VPN / overlay tunnel endpoints and leaves
    everything a Provizio device actually carries DDS on alone."""
    passed = True
    for name in VPN_NAMES:
        if not _network_recovery._is_vpn_interface_name(name):
            passed = _fail(f"{name!r} not classified as a VPN interface")
    for name in ORDINARY_NAMES:
        if _network_recovery._is_vpn_interface_name(name):
            passed = _fail(f"{name!r} wrongly classified as a VPN interface")

    if sys.platform.startswith("linux"):
        # rtnetlink IFLA_INFO_KIND is what identifies a tunnel whose name follows no
        # convention (a renamed wg device, an xfrm interface); physical Ethernet and
        # Wi-Fi report no kind at all.
        for kind in ("tun", "wireguard", "ip6tnl", "gre", "vti", "xfrm"):
            if not _network_recovery._is_vpn_interface_kind(kind):
                passed = _fail(f"kind {kind!r} not classified as a VPN kind")
        for kind in ("", "bridge", "veth", "dummy", "vxlan", "macvlan", "ipvlan", "bond"):
            if _network_recovery._is_vpn_interface_kind(kind):
                passed = _fail(f"kind {kind!r} wrongly classified as a VPN kind")

    # A user-editable label (Windows' friendly name) is matched only against vendor
    # names: an Ethernet adapter renamed "WG-LAN" must not be mistaken for a tunnel,
    # because dropping it would take every bit of that host's DDS traffic with it.
    for name in ("Tailscale", "WireGuard Tunnel", "OpenVPN TAP-Windows6", "ZeroTier One"):
        if not _network_recovery._is_vpn_product_name(name):
            passed = _fail(f"{name!r} not recognised as a VPN product")
    for name in ("WG-LAN", "TUN-uplink", "tap-office", "Ethernet 2", "LAN", "Wi-Fi"):
        if _network_recovery._is_vpn_product_name(name):
            passed = _fail(f"{name!r} wrongly recognised as a VPN product")
    # The driver-supplied description keeps the full classifier, which is what still
    # catches OpenVPN's TAP adapter — an Ethernet-type device naming no vendor.
    if not _network_recovery._is_vpn_interface_name("TAP-Windows Adapter V9"):
        passed = _fail("TAP-Windows description no longer classified as a tunnel")

    print(f"classifier: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_transports_profile() -> int:
    """The generated transport profile blocks every address handed to it, and
    reproduces the transport stack the env-variable route would have built."""
    passed = True

    # The netmask filter below is decided by how many interfaces the exclusion leaves,
    # which is a property of the runner rather than an input of this case: on a
    # single-NIC host -- which is most CI runners -- the ON branch would be asserted
    # nowhere. Substituted, as the enumeration itself is elsewhere in this suite.
    original_interfaces = _network_recovery.allowed_interfaces
    # Loopback plus two real interfaces, then loopback plus one. Substituted because the
    # runner's own interfaces decide the per-entry filters, so on a single-NIC host one
    # branch of that decision would be asserted nowhere.
    _network_recovery.allowed_interfaces = lambda blocked: [
        ("127.0.0.1", True),
        ("198.51.100.10", False),
        ("203.0.113.10", False),
    ]
    try:
        shm_xml = provizio_dds._vpn_excluded_transports_xml(
            use_shared_memory=True,
            sockets_size=1048576,
            blocked_addresses=[SYNTHETIC_BLOCKED_ADDRESS],
        )
        _network_recovery.allowed_interfaces = lambda blocked: [
            ("127.0.0.1", True),
            ("198.51.100.10", False),
        ]
        single_interface_xml = provizio_dds._vpn_excluded_transports_xml(
            use_shared_memory=True,
            sockets_size=1048576,
            blocked_addresses=[SYNTHETIC_BLOCKED_ADDRESS],
        )
    finally:
        _network_recovery.allowed_interfaces = original_interfaces
    if f'<interface name="{SYNTHETIC_BLOCKED_ADDRESS}"/>' not in shm_xml:
        passed = _fail("blocked address missing from the generated profile")
    if "<blocklist>" not in shm_xml:
        passed = _fail("no blocklist element in the generated profile")
    # Without this the blocklist costs more traffic than it saves: any non-empty
    # interface list puts UDPv4 into whitelist mode, which replaces the single
    # any-address output socket with one per remaining interface, each of which sends
    # every unicast datagram. ON leaves each destination to the one socket whose
    # subnet contains it.
    # Two real interfaces: each would send its own copy of every unicast datagram, so
    # every entry is filtered, loopback included.
    if '<interface name="127.0.0.1" netmask_filter="ON"/>' not in shm_xml:
        passed = _fail("loopback not netmask-filtered in the generated allowlist")
    if '<interface name="198.51.100.10" netmask_filter="ON"/>' not in shm_xml:
        passed = _fail("real interface not filtered where two of them would duplicate")
    # The descriptor-level filter stays AUTO: it is the container each entry is validated
    # against, and ON there would override every entry's own answer.
    if "<netmask_filter>AUTO</netmask_filter>" not in shm_xml:
        passed = _fail("descriptor-level netmask filter is not AUTO")
    # One real interface -- the common vehicle host. Loopback is still filtered, because it
    # can reach no other host and every attempt costs a failed send plus a warning per
    # datagram; the real interface is left alone, so peers behind a gateway keep receiving.
    if '<interface name="127.0.0.1" netmask_filter="ON"/>' not in single_interface_xml:
        passed = _fail("loopback not filtered with a single real interface left")
    if '<interface name="198.51.100.10" netmask_filter="AUTO"/>' not in single_interface_xml:
        passed = _fail("lone real interface filtered, costing its routed peers")
    if f'<interface name="{SYNTHETIC_BLOCKED_ADDRESS}"/>' not in single_interface_xml:
        passed = _fail("blocked address missing from the generated profile")
    if "<type>SHM</type>" not in shm_xml or "<type>UDPv4</type>" not in shm_xml:
        passed = _fail("shared-memory profile must define both SHM and UDPv4 transports")
    if "<sendBufferSize>1048576</sendBufferSize>" not in shm_xml:
        passed = _fail("socket buffer size missing from the generated profile")
    # Fast-DDS derives the shared-memory segment from the socket buffer size
    # (setup_transports: max(send, listen) * 2), and the profile must reproduce that.
    if "<segment_size>2097152</segment_size>" not in shm_xml:
        passed = _fail("shared-memory segment size not derived from the socket buffer size")

    udp_only_xml = provizio_dds._vpn_excluded_transports_xml(
        use_shared_memory=False,
        sockets_size=1048576,
        blocked_addresses=[SYNTHETIC_BLOCKED_ADDRESS],
    )
    if "<type>SHM</type>" in udp_only_xml:
        passed = _fail("UDP-only profile must not define a shared-memory transport")

    print(f"transports_profile: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_profile_cache_follows_the_interfaces() -> int:
    """A NIC coming up must not be served the profile generated before it existed.

    The generated document names each allowed interface and the netmask filter that
    interface needs, so the cache has to key on that reading and not on the blocked set
    alone. Otherwise a process that starts with one LAN interface plus a tunnel, and later
    associates Wi-Fi, gets its first profile back on the rebuild: stale addresses in the
    allowlist and a filter decision taken when there was nothing to duplicate across --
    the duplication this feature removes, reinstated on the LAN."""
    passed = True
    original = _network_recovery.allowed_interfaces
    factory = provizio_dds.DomainParticipantFactory.get_instance()
    try:
        _network_recovery.allowed_interfaces = lambda blocked: [
            ("127.0.0.1", True),
            ("198.51.100.10", False),
        ]
        one_nic = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=False,
            sockets_size=1048576,
            blocked_addresses=[SYNTHETIC_BLOCKED_ADDRESS],
        )

        # The same tunnel blocked, but a second NIC has appeared since.
        _network_recovery.allowed_interfaces = lambda blocked: [
            ("127.0.0.1", True),
            ("198.51.100.10", False),
            ("203.0.113.10", False),
        ]
        two_nics = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=False,
            sockets_size=1048576,
            blocked_addresses=[SYNTHETIC_BLOCKED_ADDRESS],
        )

        if one_nic is None or two_nics is None:
            passed = _fail("a profile could not be generated at all")
        elif one_nic == two_nics:
            passed = _fail(
                f"the second interface set reused the first profile ({one_nic!r})"
            )

        # And the reverse, which is what stops the key from being a cache-buster: the same
        # reading twice must resolve to the same profile rather than registering a new
        # transport id per creation.
        again = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=False,
            sockets_size=1048576,
            blocked_addresses=[SYNTHETIC_BLOCKED_ADDRESS],
        )
        if again != two_nics:
            passed = _fail(f"an unchanged host generated a second profile ({again!r})")
    finally:
        _network_recovery.allowed_interfaces = original

    print(f"profile_cache_follows_the_interfaces: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_blocklist_read_failure_keeps_exclusion() -> int:
    """An unreadable interface list must not cost a participant the exclusion it already
    has.

    The failure and "this host has no tunnel" are the same empty set, so a rebuild that
    re-derived its transports from a failed read would drop the blocklist it was already
    running with -- unblocking a tunnel that is still up, on exactly the rebuild a network
    change triggered. The C++ side leaves its interface lists untouched for the same
    reason; here the last applied profile is reused."""
    passed = True
    original_entries = _network_recovery.vpn_interface_blocklist_entries
    original_failed = _network_recovery.blocklist_read_failed
    try:
        _network_recovery.vpn_interface_blocklist_entries = lambda: frozenset(
            {SYNTHETIC_BLOCKED_ADDRESS}
        )
        participant = provizio_dds.make_domain_participant(DOMAIN)
        applied = participant._last_vpn_profile_name
        if applied is None:
            return _fail("no transport profile was applied to begin with") or 1

        factory = provizio_dds.DomainParticipantFactory.get_instance()

        # The read fails: the same profile must come back, so the tunnel stays excluded.
        _network_recovery.vpn_interface_blocklist_entries = lambda: frozenset()
        _network_recovery.blocklist_read_failed = lambda: True
        after_failure = participant._resolve_transports(
            factory, provizio_dds.TransportMode.AUTOMATIC
        )
        if after_failure != applied:
            passed = _fail(
                f"a failed read dropped the exclusion ({after_failure!r} != {applied!r})"
            )

        # And the contrast that makes it mean something: a SUCCESSFUL read finding no
        # tunnel does drop it, because then there is genuinely nothing to exclude.
        _network_recovery.blocklist_read_failed = lambda: False
        after_empty = participant._resolve_transports(
            factory, provizio_dds.TransportMode.AUTOMATIC
        )
        if after_empty is not None:
            passed = _fail(
                f"a successful empty read kept the exclusion ({after_empty!r})"
            )
    finally:
        _network_recovery.vpn_interface_blocklist_entries = original_entries
        _network_recovery.blocklist_read_failed = original_failed

    print(f"blocklist_read_failure_keeps_exclusion: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_participant_communicates_with_blocked_interface() -> int:
    """A participant whose transports carry a blocklist still communicates over UDP.

    Forced with a synthetic blocked address so the path under test is exercised on
    every runner, including the ones with no VPN configured at all. Substituting the
    enumeration (rather than adding a production test hook) keeps the shipped code
    free of test-only branches."""
    _network_recovery.vpn_interface_blocklist_entries = lambda: frozenset(
        {SYNTHETIC_BLOCKED_ADDRESS}
    )

    received = threading.Event()
    participant = provizio_dds.make_domain_participant(
        DOMAIN, transport=provizio_dds.TransportMode.UDP_ONLY
    )
    subscriber = provizio_dds.Subscriber(
        participant,
        "vpn_interfaces_test_topic",
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        lambda message: received.set(),
    )
    publisher = provizio_dds.Publisher(
        participant, "vpn_interfaces_test_topic", provizio_dds.StringPubSubType
    )
    if publisher.get_num_matched_subscribers(30.0, 1.0) < 1:
        print("participant_communicates_with_blocked_interface: FAIL (never matched)")
        return 1

    message = provizio_dds.String()
    message.data("vpn_interfaces_test")
    for _ in range(20):
        publisher.publish(message)
        if received.wait(0.5):
            break

    passed = received.is_set()
    if not passed:
        _fail("no sample delivered while an interface blocklist was configured")
    # Keep the handles alive until here so neither is collected mid-test.
    del subscriber, publisher
    print(f"participant_communicates_with_blocked_interface: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_profile_loaded_once() -> int:
    """The generated profile is loaded once per distinct configuration.

    Fast-DDS keeps transport ids and profile names in a process-wide registry and
    logs "There is other transport with the same id" when one is re-registered, so
    re-loading the same document — which every participant creation and every
    network-recovery rebuild would otherwise do — is both noisy and pointless."""
    from provizio_dds import DomainParticipantFactory

    factory = DomainParticipantFactory.get_instance()
    loads = []
    real_load = factory.load_XML_profiles_string

    def counting_load(xml, length):
        loads.append(xml)
        return real_load(xml, length)

    factory.load_XML_profiles_string = counting_load
    try:
        first = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=True,
            sockets_size=1048576,
            blocked_addresses=frozenset({SYNTHETIC_BLOCKED_ADDRESS}),
        )
        second = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=True,
            sockets_size=1048576,
            blocked_addresses=frozenset({SYNTHETIC_BLOCKED_ADDRESS}),
        )
        # A different blocked set is a different configuration and needs its own
        # profile — with its own transport ids, or Fast-DDS would reject them.
        third = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=True,
            sockets_size=1048576,
            blocked_addresses=frozenset({"203.0.113.8"}),
        )
    finally:
        factory.load_XML_profiles_string = real_load

    passed = True
    if first is None or second is None or third is None:
        passed = _fail("profile could not be loaded")
    if first != second:
        passed = _fail(f"same configuration produced two profiles ({first}, {second})")
    if len(loads) != 2:
        passed = _fail(f"expected 2 profile loads (two distinct configurations), got {len(loads)}")
    if third == first:
        passed = _fail("a different blocked set reused the first profile's transport ids")

    print(f"profile_loaded_once: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


_CALLER_PROFILE_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>caller_configured_udpv4</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="caller_configured_default" is_default_profile="true">
        <rtps>
            <userTransports><transport_id>caller_configured_udpv4</transport_id></userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
            <builtin>
                <initialPeersList>
                    <locator><udpv4><address>192.0.2.10</address></udpv4></locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
"""


def test_leaves_caller_profile_alone() -> int:
    """A caller who configured the factory themselves keeps what they configured.

    ``load_XML_profiles_file`` / ``load_XML_profiles_string`` is invisible to every
    environment probe this library can make, yet ``load_profiles()`` folds such a profile
    into the factory's default QoS. Reading the whole QoS back out of our generated
    transport profile would then replace ``wire_protocol().builtin`` wholesale — dropping
    their initial peers, discovery server and lease configuration — and replace the
    transports they chose, on any host that happens to have a tunnel up and nowhere else.

    Substitutes the enumeration so the exclusion is live on any host, tunnel or not."""
    os.environ.pop(ALLOW_ENV_NAME, None)

    from provizio_dds import DomainParticipantFactory

    factory = DomainParticipantFactory.get_instance()
    loaded = factory.load_XML_profiles_string(
        _CALLER_PROFILE_XML, len(_CALLER_PROFILE_XML)
    )
    real_entries = _network_recovery.vpn_interface_blocklist_entries
    _network_recovery.vpn_interface_blocklist_entries = lambda: frozenset(
        {SYNTHETIC_BLOCKED_ADDRESS}
    )
    try:
        participant = provizio_dds.make_domain_participant(
            DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
        )
        qos = participant._participant.get_qos()
        transports = len(qos.transport().user_transports)
        peers = len(qos.wire_protocol().builtin.initialPeersList)
    finally:
        _network_recovery.vpn_interface_blocklist_entries = real_entries

    passed = True
    if loaded != provizio_dds.RETCODE_OK:
        passed = _fail("the caller profile failed to load")
    # Theirs, kept: the discovery configuration the profile asked for.
    if peers != 1:
        passed = _fail(f"expected the caller's 1 initial peer to survive, got {peers}")
    # Theirs, kept: the transport they configured, not a pair of ours grafted over it.
    if transports != 1:
        passed = _fail(
            f"expected the caller's single configured transport to survive, got {transports}"
        )

    print(
        f"leaves_caller_profile_alone: {'PASS' if passed else 'FAIL'} "
        f"({transports} transport(s), {peers} initial peer(s))"
    )
    return 0 if passed else 1


def test_profile_route_honours_process_transports() -> int:
    """The transport stack a participant ends up with must not depend on whether a tunnel
    happens to be up.

    FASTDDS_BUILTIN_TRANSPORTS is process-global: once an earlier participant has fixed
    it, a later one asking for something else keeps the existing stack and is warned. The
    VPN profile route has to reach the same decision and say the same thing — otherwise
    identical application code silently gets SHM+UDPv4 on one host and a UDP-only profile
    on another, with no warning on the second.

    Substitutes the enumeration so the profile route is taken on any host, tunnel or not."""
    os.environ.pop(ALLOW_ENV_NAME, None)
    os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)

    from provizio_dds import DomainParticipantFactory, TransportMode

    # Windows and macOS get UDPv4-only even for AUTOMATIC (Fast-DDS' bundled
    # Boost.Interprocess leaks shared-memory segments there), so what the process stack is
    # pinned to, how many transports the profile carries, and whether a later UDP_ONLY
    # request contradicts anything at all are all platform-dependent. Nothing here may
    # assume Linux's SHM+UDPv4.
    shared_memory_platform = sys.platform not in ("win32", "darwin")
    expected_kind = "DEFAULT" if shared_memory_platform else "UDPv4"
    expected_transports = 2 if shared_memory_platform else 1

    factory = DomainParticipantFactory.get_instance()
    blocked = frozenset({SYNTHETIC_BLOCKED_ADDRESS})
    real_entries = _network_recovery.vpn_interface_blocklist_entries
    _network_recovery.vpn_interface_blocklist_entries = lambda: blocked

    captured = []
    previous_callback = provizio_dds.set_log_callback(
        lambda level, message: captured.append((level, message))
    )
    try:
        # First participant, no preference: fixes the process-wide stack — shared memory
        # here, since this is where the library selects it — while taking the profile route.
        participant = provizio_dds.make_domain_participant(
            DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
        )
        pinned = os.environ.get("FASTDDS_BUILTIN_TRANSPORTS")
        transports = len(participant._participant.get_qos().transport().user_transports)

        # Second participant's decision, contradicting the process-wide stack. Only the
        # decision is exercised: creating it would prove nothing more and would need its
        # own discovery.
        warnings_before = len(captured)
        udp_only_profile = participant._resolve_transports(factory, TransportMode.UDP_ONLY)
        warnings = [
            message
            for level, message in captured[warnings_before:]
            if level == provizio_dds.LogLevel.WARNING
        ]

        sockets_size = provizio_dds._resolve_udp_socket_buffer_size()
        # The profile the process-wide stack implies, and the one it does not — the latter
        # only as a control that the two are distinguishable by name.
        process_stack_profile = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=shared_memory_platform,
            sockets_size=sockets_size,
            blocked_addresses=blocked,
        )
        other_stack_profile = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=not shared_memory_platform,
            sockets_size=sockets_size,
            blocked_addresses=blocked,
        )
    finally:
        provizio_dds.set_log_callback(previous_callback)
        _network_recovery.vpn_interface_blocklist_entries = real_entries

    passed = True
    if pinned is None:
        passed = _fail("the profile route left FASTDDS_BUILTIN_TRANSPORTS unpinned")
    elif not pinned.startswith(expected_kind):
        passed = _fail(f"expected the process stack pinned to {expected_kind}, got {pinned!r}")
    if transports != expected_transports:
        passed = _fail(
            f"expected {expected_transports} transport(s) from the profile route, got {transports}"
        )
    # Only where AUTOMATIC and UDP_ONLY differ can a later UDP_ONLY request contradict the
    # pinned stack — which is exactly where a warning is owed. Where they agree there is
    # nothing to warn about, and warning anyway would be the noise this check guards.
    if shared_memory_platform and not warnings:
        passed = _fail("a contradicted transport request produced no warning")
    if not shared_memory_platform and warnings:
        passed = _fail(f"UDP_ONLY matches this platform's stack, yet warned: {warnings}")
    # The profile is cached per configuration, so the same name means the same transports:
    # the UDP_ONLY request kept the process-wide stack it was told it would keep.
    if udp_only_profile != process_stack_profile:
        passed = _fail(
            f"UDP_ONLY got profile {udp_only_profile!r} instead of the process-wide "
            f"{process_stack_profile!r} it was warned it would keep"
        )
    # Control: the two configurations really are distinguishable, so the check above is
    # not comparing a name with itself by construction.
    if other_stack_profile == process_stack_profile:
        passed = _fail("two different transport configurations shared one profile")

    print(
        f"profile_route_honours_process_transports: {'PASS' if passed else 'FAIL'} "
        f"(pinned {pinned!r}, {transports} transport(s), {len(warnings)} warning(s))"
    )
    return 0 if passed else 1


def test_profile_limit() -> int:
    """Past the profile ceiling the participant falls back to the default transports
    rather than registering profiles forever.

    Fast-DDS cannot unload an XML profile or a transport descriptor, so a VPN that
    reconnects onto a new address every time must not be able to grow the registry for
    the life of the process. The fallback reinstates the duplicate-traffic behaviour,
    which is why it warns."""
    from provizio_dds import DomainParticipantFactory

    factory = DomainParticipantFactory.get_instance()
    limit = provizio_dds._VPN_TRANSPORTS_PROFILE_LIMIT

    accepted = 0
    for index in range(limit):
        name = provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=False,
            sockets_size=1048576,
            blocked_addresses=frozenset({f"203.0.113.{index}"}),
        )
        if name is not None:
            accepted += 1

    beyond = provizio_dds._vpn_excluded_transports_profile(
        factory,
        use_shared_memory=False,
        sockets_size=1048576,
        blocked_addresses=frozenset({"198.51.100.1"}),
    )
    # An already-registered configuration must still be served from the cache after the
    # ceiling is reached — the cap bounds registrations, not lookups.
    repeat = provizio_dds._vpn_excluded_transports_profile(
        factory,
        use_shared_memory=False,
        sockets_size=1048576,
        blocked_addresses=frozenset({"203.0.113.0"}),
    )

    # Refusing is not enough on its own: remembering each refused set would be an
    # unbounded dict keyed by whatever address the network hands us next, which is the
    # very shape the ceiling exists to prevent.
    for index in range(5):
        provizio_dds._vpn_excluded_transports_profile(
            factory,
            use_shared_memory=False,
            sockets_size=1048576,
            blocked_addresses=frozenset({f"198.51.100.{10 + index}"}),
        )
    cached = len(provizio_dds._vpn_transports_profiles)

    passed = True
    if accepted != limit:
        passed = _fail(f"expected {limit} profiles to be accepted, got {accepted}")
    if beyond is not None:
        passed = _fail(f"profile past the ceiling should have been refused, got {beyond!r}")
    if repeat is None:
        passed = _fail("a cached configuration stopped resolving once the ceiling was reached")
    if cached > limit:
        passed = _fail(f"the profile cache grew past the ceiling, to {cached} entries")

    print(f"profile_limit: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_override_named() -> int:
    """A named interface is re-admitted while every other tunnel stays excluded.

    Set before the first read, because the override is resolved once per process."""
    os.environ[ALLOW_ENV_NAME] = " tailscale0 ,definitely-not-an-interface"

    passed = True
    # The predicate takes the name directly, so this holds on every host rather than
    # only on one that happens to have the interface up.
    if _network_recovery._excluded_as_vpn_interface("tailscale0"):
        passed = _fail("a named interface was not re-admitted")
    if _network_recovery._excluded_as_vpn_interface(
        "definitely-not-an-interface", platform_says_vpn=True
    ):
        passed = _fail("whitespace around a list entry was not ignored")
    if not _network_recovery._excluded_as_vpn_interface("wg0"):
        passed = _fail("an unnamed tunnel was re-admitted")
    if not _network_recovery._excluded_as_vpn_interface("office0", platform_says_vpn=True):
        passed = _fail("an unnamed tunnel was re-admitted by its platform signal")
    # Whole-name, not prefix: a loose match would silently re-admit a different tunnel.
    if not _network_recovery._excluded_as_vpn_interface("tailscale1"):
        passed = _fail("the override matched a name it was not given")
    # Case, on the other hand, is ignored — on the same terms the classifier used. On
    # Windows the identity being classified is the adapter's friendly name, and
    # requiring its exact capitalisation would leave the escape hatch doing nothing.
    if _network_recovery._excluded_as_vpn_interface("TailScale0"):
        passed = _fail("the override did not match a name differing only in case")
    if "tailscale0" in _network_recovery.vpn_interface_blocklist_entries():
        passed = _fail("a re-admitted interface is still in the blocklist")

    print(f"override_named: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_override_unmatched_names_reported() -> int:
    """A name that re-admits nothing is reported, once, rather than leaving the escape
    hatch silently doing nothing.

    The gap this covers is a typo -- "tailscale" for a device called "tailscale0" -- or
    the wrong identity for the platform, both of which otherwise leave the tunnel
    excluded for the life of the process exactly as if the variable had never been set."""
    # One name that will match an interface offered to the classifier below and one that
    # will not, so the report has to distinguish them rather than echo what was set.
    os.environ[ALLOW_ENV_NAME] = "wg0,tailscale"

    passed = True
    # Nothing has been classified yet, so nothing has matched. The report is not asked for
    # here -- see take_unmatched_vpn_allow_override_names: before an enumeration every name
    # looks unmatched, which is why the caller asks only after one.
    if _network_recovery._excluded_as_vpn_interface("wg0"):
        passed = _fail("a named interface was not re-admitted")
    # "tailscale" does not name this one: whole-name matching, so the tunnel stays out.
    if not _network_recovery._excluded_as_vpn_interface("tailscale0"):
        passed = _fail("a name the override does not carry re-admitted an interface")

    unmatched = _network_recovery.take_unmatched_vpn_allow_override_names()
    if unmatched != ("tailscale",):
        passed = _fail(f"unexpected unmatched names: {unmatched}")
    # Once per process: a second participant creation on a host with a tunnel up must not
    # repeat it, and the caller does not latch for itself.
    if _network_recovery.take_unmatched_vpn_allow_override_names():
        passed = _fail("the unmatched-name report was made twice")

    print(f"override_unmatched_names_reported: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_override_all_matched_is_quiet() -> int:
    """Every name matched, so there is nothing to report.

    The other half of the case above, and what keeps the report from degenerating into
    "echo whatever was set": a working escape hatch must stay silent."""
    os.environ[ALLOW_ENV_NAME] = "wg0"

    passed = True
    if _network_recovery._excluded_as_vpn_interface("wg0"):
        passed = _fail("a named interface was not re-admitted")
    unmatched = _network_recovery.take_unmatched_vpn_allow_override_names()
    if unmatched:
        passed = _fail(f"a matched name was reported as unmatched: {unmatched}")

    print(f"override_all_matched_is_quiet: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_unmatched_name_survives_a_failed_profile_readback() -> int:
    """A failed transport-profile read-back must drop only the claim it invalidates.

    Both reports are queued on the same participant-creation pass. The read-back failure
    falsifies exactly one of them -- "these interfaces were excluded", which is untrue once
    the participant keeps the default transports -- but it used to clear the whole queue. The
    unmatched-name warning is the one that cannot survive that: the latch behind it hands the
    names out once per process, so discarding it means the operator never learns their
    PROVIZIO_DDS_ALLOW_VPN_INTERFACES name is a typo, not even on a later rebuild once the
    read-back succeeds again.

    Driven through the rule itself rather than by forcing a read-back failure: the factory
    method that fails there is also what verifies the generated profile a few lines earlier,
    so failing it stops the profile route being taken at all and the case would exercise
    nothing."""
    participant = provizio_dds.make_domain_participant(DOMAIN)

    exclusion_claim = (
        provizio_dds.LogLevel.INFO,
        "excluding VPN / tunnel interface(s) from the DDS transports on domain 0: 203.0.113.7",
        True,
    )
    unmatched_name_warning = (
        provizio_dds.LogLevel.WARNING,
        "PROVIZIO_DDS_ALLOW_VPN_INTERFACES names definitely-not-a-tunnel, which matched "
        "no VPN / tunnel interface on this host",
        False,
    )
    participant._pending_vpn_blocklist_logs = [exclusion_claim, unmatched_name_warning]
    participant._discard_reports_of_an_exclusion_that_did_not_apply()

    passed = True
    remaining = participant._pending_vpn_blocklist_logs
    if exclusion_claim in remaining:
        passed = _fail("an exclusion this participant did not apply was still queued")
    if unmatched_name_warning not in remaining:
        passed = _fail("the unmatched-name warning was discarded along with it")

    # And what survives is still emitted rather than merely retained.
    captured = []
    previous_callback = provizio_dds.set_log_callback(
        lambda level, message: captured.append(message)
    )
    try:
        participant._flush_pending_vpn_blocklist_log()
    finally:
        provizio_dds.set_log_callback(previous_callback)
    if not any("definitely-not-a-tunnel" in message for message in captured):
        passed = _fail("the surviving warning was never emitted")

    del participant
    print(f"unmatched_name_survives_a_failed_profile_readback: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_no_op_with_builtin_transports_env() -> int:
    """With FASTDDS_BUILTIN_TRANSPORTS set by the caller, Fast-DDS owns the transport
    stack and this layer must not replace it with a generated profile — the value could
    select LARGE_DATA, UDPv6 or a message-size limit that the profile would discard."""
    os.environ.pop(ALLOW_ENV_NAME, None)
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    _network_recovery.vpn_interface_blocklist_entries = lambda: frozenset(
        {SYNTHETIC_BLOCKED_ADDRESS}
    )

    participant = provizio_dds.make_domain_participant(DOMAIN)
    # The env-variable route leaves the transports to Fast-DDS, so the participant's QoS
    # carries no user transports of ours to have blocklisted.
    qos = participant._participant_qos
    passed = len(qos.transport().user_transports) == 0
    if not passed:
        _fail("a generated profile displaced the caller's FASTDDS_BUILTIN_TRANSPORTS")

    print(f"no_op_with_builtin_transports_env: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_caller_builtin_transports_matching_ours() -> int:
    """A caller who happens to set FASTDDS_BUILTIN_TRANSPORTS to the very value this layer
    would have pinned owns it exactly as much as one who sets anything else.

    The ownership flag exists so a LATER participant can recognise a value this library
    pinned earlier and still take the VPN-excluding profile route. Deciding that by
    comparing the value rather than by remembering that we set it makes a caller's own
    setting indistinguishable from ours whenever the strings agree -- and the profile route
    then runs over a deliberate, process-wide choice README.md promises to leave alone. The
    coincidence is what hides it: every other value behaves correctly, which is why
    test_no_op_with_builtin_transports_env (UDPv4) does not catch it.

    The coincidence is built rather than guessed at. One participant is created with nothing
    excluded, which pins the variable to whatever this layer would choose; forgetting that we
    set it then leaves exactly the state a caller who had typed that same string by hand
    would leave behind."""
    os.environ.pop(ALLOW_ENV_NAME, None)
    os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    _network_recovery.vpn_interface_blocklist_entries = lambda: frozenset()

    provizio_dds.make_domain_participant(DOMAIN)
    ours = os.environ.get("FASTDDS_BUILTIN_TRANSPORTS")
    passed = ours is not None
    if not passed:
        _fail("precondition: the first participant did not pin FASTDDS_BUILTIN_TRANSPORTS")
        print("caller_builtin_transports_matching_ours: FAIL")
        return 1

    # Whoever set it, it was not us as far as anything downstream can now tell.
    provizio_dds._builtin_transports_set_by_library = None

    _network_recovery.vpn_interface_blocklist_entries = lambda: frozenset(
        {SYNTHETIC_BLOCKED_ADDRESS}
    )
    participant = provizio_dds.make_domain_participant(DOMAIN)

    # Left to Fast-DDS through the env variable, so there are no user transports of ours to
    # carry a blocklist. A generated profile would have installed some.
    qos = participant._participant_qos
    if len(qos.transport().user_transports) != 0:
        passed = _fail(
            f"a generated profile displaced a caller-set FASTDDS_BUILTIN_TRANSPORTS "
            f"('{ours}') that happened to match the value this layer would have chosen"
        )
    if provizio_dds._builtin_transports_set_by_library is not None:
        passed = _fail("the caller's FASTDDS_BUILTIN_TRANSPORTS was recorded as library-set")

    # Where the damage actually surfaces. The ownership decision is taken before the flag is
    # written, so a participant that mis-records it still behaves correctly itself -- it is the
    # NEXT one that reads the corrupted flag, concludes the value is ours, and takes the
    # profile route over the caller's setting. Asserting only on the flag above would leave
    # this consequence untested.
    third = provizio_dds.make_domain_participant(DOMAIN)
    if len(third._participant_qos.transport().user_transports) != 0:
        passed = _fail(
            "a later participant took the profile route because the caller's "
            "FASTDDS_BUILTIN_TRANSPORTS had been recorded as library-set"
        )

    print(f"caller_builtin_transports_matching_ours: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_env_xml_profile_honoured_per_participant() -> int:
    """FASTDDS_DEFAULT_PROFILES_FILE must be honoured on every call, not just the first.

    Only the working-directory DEFAULT_FASTDDS_PROFILES.xml probe may be cached, because
    load_profiles() reads that directory once per process and every later participant lives
    with what it found. A profile NAMED by the environment is different: it is read per
    participant, and a participant created with the variable set defers to the caller's XML
    entirely. Caching the combined answer let the first participant's environment bind every
    later one, in both directions -- transports the caller owns reported as ours to rewrite,
    or the exclusion skipped for good.

    Mirrors transports_come_from_user_xml in src/domain_participant.cpp."""
    # The documented variable name, spelled out as the C++ test does: the class attribute
    # that holds it lives on a function-local class and is not reachable from here.
    env_name = "FASTDDS_DEFAULT_PROFILES_FILE"
    os.environ.pop(env_name, None)
    # Force the working-directory half to a known False so only the env half is under test.
    os.environ[provizio_dds._SKIP_DEFAULT_XML_FILE_ENV] = "1"
    provizio_dds._xml_profile_owns_transports_cache = None

    # First call: no profile anywhere, so nothing of the caller's owns the transports. This is
    # also the call that populates the cache.
    first = provizio_dds._xml_profile_owns_transports(env_name)

    # Now the caller names a profile. Any existing regular file will do -- the question asked
    # is "did this participant get its configuration from the caller's XML", which is decided
    # by the variable pointing at a real file, not by the file's contents.
    os.environ[env_name] = __file__
    second = provizio_dds._xml_profile_owns_transports(env_name)

    # And withdrawing it goes back to not-owned, rather than sticking at True.
    os.environ.pop(env_name, None)
    third = provizio_dds._xml_profile_owns_transports(env_name)

    passed = True
    if first:
        passed = _fail("no profile anywhere, yet the transports were reported as the caller's")
    if not second:
        passed = _fail(
            "FASTDDS_DEFAULT_PROFILES_FILE was set after the first call and ignored: the "
            "first participant's environment bound every later one"
        )
    if third:
        passed = _fail("FASTDDS_DEFAULT_PROFILES_FILE was withdrawn but still reported as owning")

    os.environ.pop(provizio_dds._SKIP_DEFAULT_XML_FILE_ENV, None)
    provizio_dds._xml_profile_owns_transports_cache = None

    print(f"env_xml_profile_honoured_per_participant: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_override_all() -> int:
    """PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1 restores the pre-2.0 behaviour of using
    every interface the OS offers — for the rare deployment that genuinely carries
    DDS over a tunnel (which needs unicast discovery configured too, since no VPN
    carries multicast)."""
    os.environ[ALLOW_ENV_NAME] = "1"

    addresses = _network_recovery.vpn_interface_blocklist_entries()
    passed = not addresses
    if not passed:
        _fail(f"override set but {addresses} still excluded")

    print(f"override_all: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def test_snapshot_excludes_vpn_ipv6() -> int:
    """A tunnel is kept out of the change-detection snapshot whatever address family it
    carries. The blocklist handed to the transports is IPv4-only on purpose (a UDPv4
    transport enumerates no IPv6 interface, and any non-empty blocklist costs netmask
    filtering), but that reasoning does not transfer here: a tunnel's IPv6 address is
    just as unable to change a locator, so letting it into the snapshot would rebuild
    every participant on a Tailscale re-auth that rotates the fd7a:: address — the exact
    disruption this exists to remove.

    Substitutes the enumeration so both outcomes are reachable on any host, including
    runners with no tunnel at all and runners whose tunnel has no IPv6 address."""
    os.environ.pop(ALLOW_ENV_NAME, None)

    kept_v4 = ("eth0", "", "192.168.1.5", 24)
    kept_v6 = ("eth0", "", "2001:db8::5", 64)
    tunnel_v4 = ("tailscale0", "tun", "100.72.1.5", 32)
    tunnel_v6 = ("tailscale0", "tun", "fd7a:115c:a1e0::1", 48)

    original = _network_recovery._iter_posix_interface_addresses
    _network_recovery._iter_posix_interface_addresses = lambda: iter(
        [kept_v4, kept_v6, tunnel_v4, tunnel_v6]
    )
    try:
        snapshot = _network_recovery._capture_snapshot_posix()
    finally:
        _network_recovery._iter_posix_interface_addresses = original

    addresses = {entry[1] for entry in snapshot}
    passed = True
    if tunnel_v6[2] in addresses:
        passed = _fail(f"tunnel IPv6 address {tunnel_v6[2]} present in the snapshot")
    if tunnel_v4[2] in addresses:
        passed = _fail(f"tunnel IPv4 address {tunnel_v4[2]} present in the snapshot")
    # Control: the same harness must let a real interface through on both families, or
    # the assertions above would pass on an empty snapshot.
    if kept_v4[2] not in addresses:
        passed = _fail(f"non-tunnel IPv4 address {kept_v4[2]} missing from the snapshot")
    if kept_v6[2] not in addresses:
        passed = _fail(f"non-tunnel IPv6 address {kept_v6[2]} missing from the snapshot")

    print(
        f"snapshot_excludes_vpn_ipv6: {'PASS' if passed else 'FAIL'} "
        f"({len(snapshot)} snapshot entry(ies): {sorted(addresses)})"
    )
    return 0 if passed else 1


def test_unreadable_interfaces_still_create_participant() -> int:
    """A failed interface read must not take participant creation down with it.

    The change detector needs an unreadable interface list distinguished from an empty one
    (they mean opposite things there), so the walk raises. This path asks a different
    question — which tunnels to keep out — where the safe answer to "cannot tell" is to
    exclude nothing for this participant and try again on the next creation or rebuild.
    Without that boundary a failed syscall would propagate out of make_domain_participant."""
    os.environ.pop(ALLOW_ENV_NAME, None)

    def raising_walk():
        raise OSError(12, "getifaddrs failed: Cannot allocate memory")
        yield  # pragma: no cover — makes this a generator, like the real walk

    captured = []
    previous_callback = provizio_dds.set_log_callback(
        lambda level, message: captured.append((level, message))
    )
    real_posix = _network_recovery._iter_posix_interface_addresses
    real_windows = getattr(_network_recovery, "_iter_windows_adapter_addresses", None)
    _network_recovery._iter_posix_interface_addresses = raising_walk
    if real_windows is not None:
        _network_recovery._iter_windows_adapter_addresses = raising_walk
    try:
        entries = _network_recovery.vpn_interface_blocklist_entries()
        participant = provizio_dds.make_domain_participant(
            DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
        )
    finally:
        _network_recovery._iter_posix_interface_addresses = real_posix
        if real_windows is not None:
            _network_recovery._iter_windows_adapter_addresses = real_windows
        provizio_dds.set_log_callback(previous_callback)

    passed = True
    if entries:
        passed = _fail(f"expected no exclusions from an unreadable interface list, got {sorted(entries)}")
    if participant is None:
        passed = _fail("participant creation failed on an unreadable interface list")
    warnings = [
        message for level, message in captured if level == provizio_dds.LogLevel.WARNING
    ]
    if not any("could not read this host's network interfaces" in message for message in warnings):
        passed = _fail(f"the degradation went unreported: {warnings}")

    print(
        f"unreadable_interfaces_still_create_participant: {'PASS' if passed else 'FAIL'} "
        f"({len(warnings)} warning(s))"
    )
    return 0 if passed else 1


def test_snapshot_excludes_vpn() -> int:
    """An interface the transports refuse to bind must not drive network-recovery
    either — its address churn (a Tailscale re-auth, a VPN reconnect) can no longer
    change any locator, so rebuilding every participant over it is pure disruption."""
    os.environ.pop(ALLOW_ENV_NAME, None)

    vpn_addresses = _network_recovery.vpn_interface_blocklist_entries()
    snapshot = _network_recovery._capture_address_snapshot()

    passed = True
    for entry in snapshot:
        if entry[1] in vpn_addresses:
            passed = _fail(f"VPN address {entry[1]} present in the change-detection snapshot")

    suffix = "" if vpn_addresses else "; host has no VPN interface up, empty case only"
    print(
        f"snapshot_excludes_vpn: {'PASS' if passed else 'FAIL'} "
        f"({len(snapshot)} snapshot entry(ies), {len(vpn_addresses)} VPN address(es){suffix})"
    )
    return 0 if passed else 1


_TESTS = {
    "classifier": test_classifier,
    "transports_profile": test_transports_profile,
    "profile_loaded_once": test_profile_loaded_once,
    "profile_limit": test_profile_limit,
    "profile_route_honours_process_transports": test_profile_route_honours_process_transports,
    "leaves_caller_profile_alone": test_leaves_caller_profile_alone,
    "participant_communicates": test_participant_communicates_with_blocked_interface,
    "blocklist_read_failure_keeps_exclusion": test_blocklist_read_failure_keeps_exclusion,
    "profile_cache_follows_the_interfaces": test_profile_cache_follows_the_interfaces,
    "override_all": test_override_all,
    "override_named": test_override_named,
    "override_unmatched_names_reported": test_override_unmatched_names_reported,
    "override_all_matched_is_quiet": test_override_all_matched_is_quiet,
    "unmatched_name_survives_a_failed_profile_readback": test_unmatched_name_survives_a_failed_profile_readback,
    "no_op_with_builtin_transports_env": test_no_op_with_builtin_transports_env,
    "caller_builtin_transports_matching_ours": test_caller_builtin_transports_matching_ours,
    "env_xml_profile_honoured_per_participant": test_env_xml_profile_honoured_per_participant,
    "snapshot_excludes_vpn": test_snapshot_excludes_vpn,
    "unreadable_interfaces_still_create_participant": test_unreadable_interfaces_still_create_participant,
    "snapshot_excludes_vpn_ipv6": test_snapshot_excludes_vpn_ipv6,
}


def main() -> int:
    if len(sys.argv) < 2:
        # No "run all in one process" fallback: the override is resolved once per
        # process (matching the C++ side), so an all-in-one run would lock the
        # decision for every case after the first one that triggered the cache.
        print(f"usage: {sys.argv[0]} <{'|'.join(_TESTS.keys())}>", file=sys.stderr)
        return 1
    name = sys.argv[1]
    if name not in _TESTS:
        print(f"Unknown subcommand: {name}", file=sys.stderr)
        return 1
    # Hermetic: an ambient FASTDDS_DEFAULT_PROFILES_FILE makes the library defer to
    # that profile, transports included, so the path under test would never run.
    os.environ.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
    # Same for a user-chosen transport set: with FASTDDS_BUILTIN_TRANSPORTS set,
    # Fast-DDS builds the transports and no profile of ours is involved.
    os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    try:
        return _TESTS[name]()
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
