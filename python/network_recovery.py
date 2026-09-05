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

"""Python implementation of provizio_dds network auto-recovery.

Mirrors the C++ implementation in include/provizio/dds/network_recovery.h
and src/network_recovery_coordinator.cpp, with three differences:

  1. Event detection is kernel-notification-based (netlink) on Linux, like the
     C++ side; macOS and Windows fall back to polling, and Linux falls back to
     polling too if AF_NETLINK is unavailable (e.g. a sandbox). Pure Python has
     no portable, dependency-free way to subscribe to macOS/Windows kernel
     address-change events; ctypes wrappers around PF_ROUTE /
     NotifyUnicastIpAddressChange would add a significant amount of
     platform-specific glue with limited benefit (recovery is intentionally
     debounced over a multi-second quiet period anyway). Polling there is
     simpler, dependency-free, and the latency is well within the design
     budget.

  2. The reset hook is supplied by the Python `_DomainParticipant`, not the
     coordinator. The Python participant knows how to tear down and rebuild
     its own Publisher / Subscriber / Service objects (which are pure-Python
     classes that wrap Fast-DDS-Python SWIG types).

  3. The cross-platform interface enumeration uses ctypes against libc's
     `getifaddrs` on POSIX and `GetAdaptersAddresses` on Windows, with the
     same filter list as the C++ side (loopback, link-local, docker /
     veth / etc.).

The detection interval — the polling cadence on the polling backends, or the
burst debounce (quiet period) on the netlink backend — is configurable via the
`PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC` env variable; default 3 s
(matches the C++ quiet_period).
"""

from __future__ import annotations

import contextlib
import ctypes
import errno
import math
import os
import queue
import re
import select
import socket
import struct
import sys
import threading
import time
import weakref
from enum import Enum
from typing import Any, Callable, Collection, Dict, FrozenSet, Iterator, List, Optional, Set, Tuple


# ---------------------------------------------------------------------------
# Public surface
# ---------------------------------------------------------------------------


class NetworkRecoveryMode(Enum):
    """How a participant participates in network auto-recovery.

    Mirrors the C++ ``provizio::dds::network_recovery_mode`` enum exactly so
    documentation and behaviour stay aligned between the two language bindings.
    """

    #: Honour the ``PROVIZIO_DDS_NETWORK_RECOVERY`` environment variable for
    #: the whole process. Recognised values (case-insensitive): ``on`` /
    #: ``1`` / ``true`` / ``yes`` to enable (default when the variable is
    #: unset); ``off`` / ``0`` / ``false`` / ``no`` to disable. Any other
    #: value is treated as enabled with a one-time warning.
    ENV_VAR_CONTROLLED = "env_var_controlled"

    #: Force auto-recovery on, regardless of env var.
    ON = "on"

    #: Force auto-recovery off, regardless of env var. The participant is
    #: excluded from the registry; the background monitor still runs if any
    #: other participant enables recovery.
    OFF = "off"


class LogLevel(Enum):
    """Severity level for provizio_dds log messages."""

    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


LogCallback = Callable[[LogLevel, str], None]


# Internal state for the env-var resolution and logging callback. These are
# module-globals because they are intentionally process-wide (the C++ side
# uses the same model).
_ENV_VAR_NAME = "PROVIZIO_DDS_NETWORK_RECOVERY"
_POLL_INTERVAL_ENV = "PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC"
_LOG_PREFIX = "[provizio_dds]"

# RLock so a user callback that itself calls set_log_callback() — e.g. to
# install a different sink from within a log emission — does not deadlock
# on the lock that _emit_log briefly holds to snapshot the active callback.
_log_callback_lock = threading.RLock()
_log_callback: Optional[LogCallback] = None

_env_resolution_lock = threading.Lock()
_env_resolution_cached: Optional[bool] = None


def set_log_callback(callback: Optional[LogCallback]) -> Optional[LogCallback]:
    """Install a custom log callback for all subsequent log emissions from
    provizio_dds. Pass ``None`` to restore the built-in stdout/stderr
    emitter.

    Returns the previously installed callback (``None`` if the default
    emitter was in use). Safe to call from any thread.

    The callback may be invoked from the network-recovery monitor thread or
    from a participant reset path. Implementations should be brief and
    reentrant; do any heavy work in their own background thread.

    A callback may use provizio_dds entities that already exist -- publishing
    the line onto a DDS topic is a supported use. It must NOT create or destroy
    a publisher, subscriber, service or client: some diagnostics are emitted
    while a participant's registration lock is held, and every endpoint
    constructor and destructor takes that same non-reentrant lock, so doing so
    from the callback deadlocks the calling thread. Creating a domain
    participant is fine. This is the same contract the C++ ``logging.h`` states.
    """

    global _log_callback
    with _log_callback_lock:
        previous = _log_callback
        _log_callback = callback
        return previous


def resolve_network_recovery_enabled(mode: NetworkRecoveryMode) -> bool:
    """Resolve a :class:`NetworkRecoveryMode` to a concrete on/off decision.

    For :attr:`NetworkRecoveryMode.ENV_VAR_CONTROLLED` the env variable is
    read exactly once per process; the result is cached for the lifetime of
    the process. Changing ``PROVIZIO_DDS_NETWORK_RECOVERY`` at runtime
    afterwards has no effect — to switch the default, relaunch the process.
    """

    if mode == NetworkRecoveryMode.ON:
        return True
    if mode == NetworkRecoveryMode.OFF:
        return False
    # ENV_VAR_CONTROLLED
    return _resolve_env_once()


# ---------------------------------------------------------------------------
# Internals: logging
# ---------------------------------------------------------------------------


def _emit_log(level: LogLevel, message: str) -> None:
    """Route a log message through the installed callback or the default
    stdout/stderr emitter. Exceptions raised by a custom callback are
    swallowed — a logger that throws cannot be allowed to crash the
    coordinator thread."""

    try:
        with _log_callback_lock:
            cb = _log_callback
        if cb is not None:
            try:
                cb(level, message)
            except Exception:
                # Same rationale as the C++ side: a throwing logger must
                # not propagate out of provizio internals.
                pass
            return
        stream = sys.stderr if level == LogLevel.ERROR else sys.stdout
        print(f"{_LOG_PREFIX} {message}", file=stream)
    except Exception:
        # Last-ditch swallow — log emission must never crash the caller.
        pass


# ---------------------------------------------------------------------------
# Internals: env-var resolution (one-shot cached)
# ---------------------------------------------------------------------------


def _resolve_env_once() -> bool:
    global _env_resolution_cached
    with _env_resolution_lock:
        if _env_resolution_cached is not None:
            return _env_resolution_cached

        raw = os.environ.get(_ENV_VAR_NAME)
        if raw is None or raw == "":
            _env_resolution_cached = True  # default-on
            return True

        value = raw.lower()
        if value in ("off", "0", "false", "no"):
            _env_resolution_cached = False
            return False
        if value in ("on", "1", "true", "yes"):
            _env_resolution_cached = True
            return True

        _emit_log(
            LogLevel.WARNING,
            f"{_ENV_VAR_NAME}={raw} is not recognised (use on/off); auto-recovery enabled",
        )
        _env_resolution_cached = True
        return True


# ---------------------------------------------------------------------------
# Fast-DDS interface-cache refresh — bridge to the C++ side via ctypes
# ---------------------------------------------------------------------------
#
# Fast-DDS keeps a process-wide static cache of network interfaces in its
# `eprosima::SystemInfo` class, populated exactly once at first participant
# creation and never automatically refreshed afterwards. When the Python
# `_DomainParticipant._reset_hook_locked` recreates a participant via
# `factory.create_participant(...)`, the new participant transparently picks
# up the stale cache and ends up bound to whatever interfaces were present
# at first-participant time. That is the auto-recovery bug fixed in
# APT-11792: the C++ provizio_dds library exposes an extern "C" shim
# (`provizio_dds_refresh_fastdds_interface_cache` in network_recovery.cpp)
# that calls `eprosima::SystemInfo::update_interfaces()`. Python loads
# libprovizio_dds via ctypes and calls that shim immediately before every
# `factory.create_participant` call.
#
# Lookup order for libprovizio_dds (kept narrow on purpose so a mismatched
# system-wide install can't silently shadow the package's own copy):
#   1. The shared library shipped alongside this module (Linux .so /
#      macOS .dylib / Windows .dll). This is the case for both the pip
#      wheel and the in-tree CTest layout.
#   2. The plain library name on the platform's default search path
#      (LD_LIBRARY_PATH / DYLD_LIBRARY_PATH / PATH), as a fallback for
#      developer setups that source the library via system paths.
#
# If neither resolves, `refresh_fastdds_interface_cache()` becomes a no-op
# that returns False. The error is logged once at WARNING so a missing
# library on a Python-only deployment surfaces visibly rather than silently
# disabling network recovery.


def _load_provizio_dds_lib() -> Optional[ctypes.CDLL]:
    here = os.path.dirname(os.path.abspath(__file__))

    if sys.platform == "win32":
        candidates = [os.path.join(here, "provizio_dds.dll")]
        fallback_name = "provizio_dds"
    elif sys.platform == "darwin":
        candidates = [os.path.join(here, "libprovizio_dds.dylib")]
        fallback_name = "libprovizio_dds.dylib"
    else:
        candidates = [os.path.join(here, "libprovizio_dds.so")]
        # Also try the versioned variants — some installs ship only the
        # SONAME-versioned file without the unversioned dev symlink.
        import glob

        candidates.extend(sorted(glob.glob(os.path.join(here, "libprovizio_dds.so.*"))))
        fallback_name = "libprovizio_dds.so"

    for path in candidates:
        if os.path.exists(path):
            try:
                return ctypes.CDLL(path)
            except OSError:
                continue

    try:
        return ctypes.CDLL(fallback_name)
    except OSError:
        return None


_provizio_dds_lib: Optional[ctypes.CDLL] = _load_provizio_dds_lib()
_refresh_fn = None
if _provizio_dds_lib is not None:
    try:
        _refresh_fn = _provizio_dds_lib.provizio_dds_refresh_fastdds_interface_cache
        _refresh_fn.argtypes = []
        _refresh_fn.restype = ctypes.c_bool
    except (AttributeError, OSError):
        _refresh_fn = None

if _refresh_fn is None:
    # Surface the degraded state once at import — network auto-recovery
    # will still run but won't refresh Fast-DDS's interface cache, so a
    # participant rebuilt after a network change may bind to stale
    # interfaces (the exact failure this module exists to prevent). Far
    # better to see this line than to silently lose recovery.
    _emit_log(
        LogLevel.WARNING,
        "network auto-recovery: could not bind "
        "provizio_dds_refresh_fastdds_interface_cache from libprovizio_dds; "
        "Fast-DDS interface cache will NOT be refreshed on reset "
        "(rebuilt participants may keep using stale interfaces)",
    )


def refresh_fastdds_interface_cache() -> bool:
    """Force Fast-DDS to re-enumerate the host's network interfaces.

    Counterpart of @c provizio::dds::refresh_fastdds_interface_cache on the C++
    side — see that function's docstring for the full rationale. Called by
    :class:`_DomainParticipant` before every ``factory.create_participant``
    invocation (both initial construction and auto-recovery reset) so the
    fresh participant binds to current host interfaces rather than the cached
    set from the first participant created in this process.

    Returns True on success, False if the underlying call failed OR if
    libprovizio_dds could not be loaded.
    """
    if _refresh_fn is None:
        return False
    try:
        return bool(_refresh_fn())
    except OSError:
        return False


# ---------------------------------------------------------------------------
# Address snapshot — cross-platform interface enumeration via ctypes
# ---------------------------------------------------------------------------


# Address-set type: (interface name, address text, CIDR prefix length). Two
# snapshots are equal if their contents match order-independently; frozenset gives
# us this for free.
#
# The prefix length is part of the identity because Fast-DDS derives its
# netmask-based locator filtering from it — re-subnetting an interface without
# changing its address (192.168.1.10/24 -> /16) changes which peers Fast-DDS treats
# as on-link, so it has to count as a network change. Mirrors the C++
# detail::interface_address.
AddressSnapshot = FrozenSet[Tuple[str, str, int]]


# Whether the current run of unreadable-interface failures has been reported, so the
# warning is emitted once per streak rather than once per attempt (a poller asks every few
# seconds). Cleared by the first successful read.
_enumeration_failure_reported = False


def _try_capture_address_snapshot() -> "Optional[AddressSnapshot]":
    """The current address snapshot, or ``None`` when the host's interfaces could not be
    read at all.

    The distinction matters because an empty snapshot is a legitimate reading — a container
    whose only device is a filtered-out veth reports exactly that — so a failed read that
    returned one would be taken for every address disappearing: every participant rebuilt
    for nothing, then rebuilt again once the next read succeeds, with any in-flight
    request/response lost to it. Mirrors
    ``network_recovery_coordinator::current_snapshot`` in
    src/network_recovery_coordinator.cpp."""
    global _enumeration_failure_reported

    try:
        snapshot = _capture_address_snapshot()
    except Exception as ex:
        if not _enumeration_failure_reported:
            _enumeration_failure_reported = True
            _emit_log(
                LogLevel.WARNING,
                f"could not read this host's network interfaces ({ex}); keeping the last "
                f"known address set and making no participant rebuild decision until it "
                f"can be read again (an unreadable interface list is not an interface "
                f"change)",
            )
        return None

    _enumeration_failure_reported = False
    return snapshot


def _capture_address_snapshot() -> AddressSnapshot:
    """Capture the host's current 'DDS-interesting' interface→address set.

    Excluded by platform:

    Common to all:
      - Loopback interfaces.
      - IPv6 link-local ``fe80::/10`` addresses.
      - Interfaces that are not *operationally* up — carrier-down as well as
        administratively down (POSIX ``IFF_RUNNING``, Windows ``OperStatus``).
        This mirrors Fast-DDS' own ``IPFinder::getIPs``, which also keys on
        ``IFF_RUNNING``: the snapshot exists to model the interface set Fast-DDS
        will bind to, so it must filter on the same flag. Keying on the weaker
        ``IFF_UP`` would make a switch power-cycle or cable replug invisible to
        the diff — the address stays in the snapshot across the outage while
        Fast-DDS silently stops binding a locator to it — and no participant
        would ever be rebuilt.

    Linux / macOS (via getifaddrs):
      - Names matching Docker / Kubernetes / LXC conventions
        (docker*, br-*, cni*, kube*, lxc*, flannel*, weave*, veth*).
      - Linux: container / virtual interface kinds (bridge, veth, dummy, vxlan,
        macvlan, ipvlan). Tunnel kinds are handled by the VPN filter instead
        (see :func:`_excluded_as_vpn_interface`), so one environment variable
        governs both change detection and the transports.
      - macOS: Apple-internal NICs (utun, awdl, llw, gif, stf, anpi, ap[0-9]).

    Windows (via GetAdaptersAddresses):
      - Interfaces with IfType not in {Ethernet, IEEE80211, PPP}.
      - Friendly-name / description match for known virtual adapters.

    Interfaces named in ``PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES`` bypass
    the name / kind / adapter-type heuristics above — see
    :func:`_force_included_interfaces`. They do NOT bypass the VPN filter;
    ``PROVIZIO_DDS_ALLOW_VPN_INTERFACES`` is the only knob for that.

    VPN / overlay-tunnel interfaces are excluded unless
    ``PROVIZIO_DDS_ALLOW_VPN_INTERFACES`` re-admits them: they are kept out of the
    DDS transports too, so their address churn can no longer change any locator
    and rebuilding participants over it would be pure disruption.

    Returns an empty snapshot if the OS call fails or the host has no
    operationally-up non-loopback interface.
    """

    if sys.platform == "win32":
        return _capture_snapshot_windows()
    return _capture_snapshot_posix()


# Common filtering helpers ---------------------------------------------------


def _is_link_local_ipv6(addr_text: str) -> bool:
    """fe80::/10 — see RFC 4291 §2.5.6.

    Matches the full /10 (high byte 0xfe with the top two bits of the
    next byte = 0b10, i.e. the second byte in 0x80–0xbf), matching the
    C++ ``IN6_IS_ADDR_LINKLOCAL`` macro. Although in practice the kernel
    only emits ``fe80::*`` for SLAAC link-local, doing the byte check
    keeps the Python and C++ filters bit-for-bit equivalent.
    """
    try:
        packed = socket.inet_pton(socket.AF_INET6, addr_text)
    except (OSError, ValueError):
        return False
    return len(packed) >= 2 and packed[0] == 0xFE and (packed[1] & 0xC0) == 0x80


_LINUX_NAME_PREFIXES = (
    "docker",
    "br-",
    "cni",
    "kube",
    "lxc",
    "flannel",
    "weave",
    "veth",
)


# Linux interface "kinds" (IFLA_INFO_KIND values) that identify container /
# virtual interfaces and are excluded from the snapshot, matching the C++ side in
# src/address_snapshot_linux.cpp. Physical Ethernet / Wi-Fi have no kind attribute
# and fall through.
#
# Tunnel kinds ("tun", "ip6tnl") are deliberately absent HERE because they are
# handled by the dedicated VPN filter below (_excluded_as_vpn_interface), which the
# PROVIZIO_DDS_ALLOW_VPN_INTERFACES override flows through.
_LINUX_EXCLUDED_KINDS = frozenset(
    {"bridge", "veth", "dummy", "vxlan", "macvlan", "ipvlan"}
)


# VPN / overlay-tunnel identification. Kept out of the DDS transports by default —
# see include/provizio/dds/detail/vpn_interfaces.h for the full rationale, in
# short: Fast-DDS announces an address on every interface it can bind, a writer
# sends every sample to ALL of a matched reader's announced locators, so two hosts
# that share a LAN and are both on a VPN exchange every sample twice — once over
# the LAN and once through the tunnel, which on a metered cellular uplink is
# charged for. A VPN can never be the path that *establishes* DDS communication
# (discovery is link-local multicast and no VPN carries multicast), so excluding
# one only ever removes a duplicate.
#
# Classification is by interface identity, never by address range: 100.64.0.0/10
# would catch Tailscale, but it is the RFC 6598 carrier-NAT range that a cellular
# uplink — and therefore a peer's real LAN interface behind such a router —
# legitimately holds. Mirrors src/vpn_interfaces.cpp.
#
# Distinctive vendor words, matched ANYWHERE in the string. They are long and
# specific enough that a physical NIC's name or description cannot contain one,
# which is what makes free substring matching safe here — and necessary, since
# Windows reports things like "OpenVPN TAP-Windows6" and "Tailscale Tunnel" where
# the identifying word is not at the start.
_VPN_VENDOR_NEEDLES = (
    "tailscale",  # Tailscale
    "wireguard",  # WireGuard (incl. the WireGuard NT Windows adapter)
    "openvpn",  # OpenVPN
    "zerotier",  # ZeroTier
)

# Generic tunnel conventions, matched only as a PREFIX. Anchoring matters: as free
# substrings these are short enough to appear inside an unrelated vendor string —
# "NETGEAR WG111v3 Wireless USB Adapter" contains "wg" — and a false positive is
# the worst outcome this feature can produce, since it would drop ALL DDS traffic
# on a real interface, where a false negative merely leaves the duplicate traffic
# this exists to remove. Prefix form still covers every convention in use: tun0,
# tap0, utun3, ipsec1, wg0, and Windows' own "TAP-Windows Adapter V9" /
# "Tunnel adapter ...".
_VPN_NAME_PREFIXES = (
    "utun",  # macOS user-space tunnel (VPN, IPsec)
    "ipsec",  # IPsec / strongSwan
    "tun",  # OpenVPN & friends, L3; also "Tunnel adapter ..." on Windows
    "tap",  # OpenVPN & friends, L2; also "TAP-Windows ..." on Windows
    "wg",  # WireGuard device convention
    # The two kernel tunnels macOS names itself. Linux's equivalents (ipip, sit, gre) are
    # caught by their rtnetlink kind instead; macOS reports no kind, so the name is the only
    # signal there is. They must be classified here and not only dropped from the address
    # snapshot: the snapshot drops exactly what the transports refuse to bind, and an
    # interface excluded from one but bound by the other is the disagreement the snapshot
    # contract says must never happen.
    "gif",  # generic IPv4/IPv6-in-IP tunnel (gif0)
    "stf",  # 6to4 tunnel (stf0)
)

# Tunnel forms as Windows' own strings spell them, matched as a PREFIX of a
# driver-supplied description. Whole words, unlike the device conventions above: that
# is what lets them be applied to a vendor string at all (see _is_vpn_description --
# a description beginning "WG111v3" is a NETGEAR radio, not a WireGuard device).
_VPN_DESCRIPTION_PREFIXES = (
    "tap-windows",  # OpenVPN's TAP driver: "TAP-Windows Adapter V9"
    "tunnel adapter",  # Windows' own naming: "Tunnel adapter Teredo"
)

# IFLA_INFO_KIND values that denote an overlay or tunnel device. The kind is the
# reliable half of the signal on Linux: it survives renaming (a wireguard device
# called office0) and covers tunnels that follow no naming convention (xfrm, vti).
_LINUX_VPN_KINDS = frozenset(
    {
        "tun",
        "wireguard",
        "xfrm",
        "vti",
        "vti6",
        "ipip",
        "ip6tnl",
        "gre",
        "gretap",
        "ip6gre",
        "sit",
    }
)

# "zt" is deliberately absent from the lists above: as a two-letter substring it
# would match far too much of a vendor string, so ZeroTier's device convention is
# matched only as a name prefix plus its eight-character node-derived suffix
# (ztppmkbrz2), which no physical NIC name collides with.
_ZEROTIER_NAME_PREFIX = "zt"
_ZEROTIER_NAME_SUFFIX_LENGTH = 8

_ALLOW_VPN_INTERFACES_ENV = "PROVIZIO_DDS_ALLOW_VPN_INTERFACES"
_ALLOW_VPN_KEYWORDS = ("1", "true", "yes", "on", "all")
# Values documented as leaving the default in place. Recognised explicitly rather than left
# to match nothing -- see _allow_vpn_override.
_ALLOW_VPN_INERT_VALUES = frozenset({"0", "false", "off", "no"})
_allow_vpn_cache: "Optional[Tuple[bool, FrozenSet[str]]]" = None
_allow_vpn_lock = threading.Lock()


def _lower_ascii(text: str) -> str:
    """Lower-case ASCII letters only, leaving every other code point alone.

    ``str.lower()`` is Unicode-aware and can even change a string's length, which would
    make the classifier and the allow list disagree with src/vpn_interfaces.cpp'
    to_lower_ascii on a non-ASCII Windows friendly name. These are contractual mirrors,
    so the folding has to match."""
    return "".join(
        chr(ord(char) + 32) if "A" <= char <= "Z" else char for char in text
    )


def _split_comma_separated(raw: str) -> List[str]:
    """Entries of a comma-separated environment value, ASCII whitespace trimmed off each
    and empty ones dropped, so that ``"br0, virbr2"`` behaves like ``"br0,virbr2"`` and a
    trailing comma contributes no unmatchable name.

    Shared by every PROVIZIO_DDS_* list this module reads, and trimming exactly the ASCII
    set that ``split_comma_separated`` in src/detail/env_utils.h trims -- ``str.strip()``
    would also remove Unicode whitespace, which is the kind of quiet difference that makes
    two contractual mirrors disagree about one host's configuration."""
    trimmed = (entry.strip(" \t\r\n") for entry in raw.split(","))
    return [entry for entry in trimmed if entry]


def _is_zerotier_device_name(lowered: str) -> bool:
    """Whether a lower-cased name is a ZeroTier device (zt + 8 alphanumerics)."""
    if len(lowered) != len(_ZEROTIER_NAME_PREFIX) + _ZEROTIER_NAME_SUFFIX_LENGTH:
        return False
    if not lowered.startswith(_ZEROTIER_NAME_PREFIX):
        return False
    # ASCII [0-9a-z] only, exactly as src/vpn_interfaces.cpp checks it: str.isalnum()
    # would additionally accept non-ASCII digits and letters, and the two
    # implementations are contractual mirrors.
    return all(
        "0" <= char <= "9" or "a" <= char <= "z"
        for char in lowered[len(_ZEROTIER_NAME_PREFIX):]
    )


def _is_vpn_interface_name(name: str) -> bool:
    """Whether an interface name looks like a VPN / overlay tunnel endpoint.

    Matched against the name the platform reports: the device name on POSIX
    (tailscale0, wg0, utun3), and the adapter's friendly name or description on
    Windows, where the device name is a GUID and carries no hint.

    Deliberately NOT matched: ``ppp`` (a PPP link is a real WAN uplink — cellular
    modems and DSL — not an overlay), bridges (br0 / br-lan can be the only
    interface a vehicle PC carries DDS on) and container plumbing (docker0,
    veth*), whose traffic is genuinely local and free."""
    lowered = _lower_ascii(name)
    if _is_zerotier_device_name(lowered):
        return True
    if any(needle in lowered for needle in _VPN_VENDOR_NEEDLES):
        return True
    return lowered.startswith(_VPN_NAME_PREFIXES)


def _is_vpn_product_name(name: str) -> bool:
    """Whether a string names a VPN *product* — the vendor half of
    :func:`_is_vpn_interface_name`, without the generic tun / tap / wg prefixes.

    For strings a user can rename, which on Windows means the adapter's friendly name.
    Renaming a real NIC to something starting with one of those prefixes ("WG-LAN")
    would otherwise drop it from the transports and take all of that host's DDS traffic
    with it. Mirrors is_vpn_product_name in src/vpn_interfaces.cpp."""
    lowered = _lower_ascii(name)
    return any(needle in lowered for needle in _VPN_VENDOR_NEEDLES)


def _is_vpn_description(description: str) -> bool:
    """Whether a driver-supplied adapter description names a tunnel.

    A third question, between the other two, and Windows' alone: a description is a
    vendor string rather than a device name, so the short conventions
    :func:`_is_vpn_interface_name` anchors -- tun, tap, wg -- match real hardware on it.
    NETGEAR's WG series is the example that matters: a description reading "WG111v3
    54Mbps Wireless USB 2.0 Adapter" begins with "wg", and taking it for a tunnel would
    blocklist that host's only NIC and every address on it.

    Matched instead: a VPN vendor's name anywhere in the string, plus the two forms
    Windows itself uses to describe a tunnel, which are whole words no model number
    begins with. Mirrors is_vpn_description in src/vpn_interfaces.cpp."""
    lowered = _lower_ascii(description)
    if any(needle in lowered for needle in _VPN_VENDOR_NEEDLES):
        return True
    return lowered.startswith(_VPN_DESCRIPTION_PREFIXES)


def _is_vpn_interface_kind(kind: str) -> bool:
    """Whether an rtnetlink IFLA_INFO_KIND identifies a VPN / tunnel device.
    Physical Ethernet and Wi-Fi report no kind, so an empty kind never matches."""
    return bool(kind) and kind in _LINUX_VPN_KINDS


def _allow_vpn_override() -> "Tuple[bool, FrozenSet[str]]":
    """``(allow_all, allowed_names)`` parsed from PROVIZIO_DDS_ALLOW_VPN_INTERFACES.

    Read exactly once per process, like every other PROVIZIO_DDS_* resolution, so
    the transports, the change-detection snapshot and every rebuilt participant
    agree for the process' lifetime. Names come back lower-cased, because that is
    how the classifier they have to match compared them."""
    global _allow_vpn_cache
    with _allow_vpn_lock:
        if _allow_vpn_cache is not None:
            return _allow_vpn_cache
        raw = os.environ.get(_ALLOW_VPN_INTERFACES_ENV, "")
        allow_all = False
        names = set()
        for entry in _split_comma_separated(raw):
            lowered_entry = _lower_ascii(entry)
            if lowered_entry in _ALLOW_VPN_KEYWORDS:
                allow_all = True
            elif lowered_entry in _ALLOW_VPN_INERT_VALUES:
                # Documented as leaving the default in place, so dropped rather than kept as
                # names that happen to match nothing: kept, they would have
                # take_unmatched_vpn_allow_override_names report a deliberately inert
                # setting as a typo on every host with a tunnel up.
                pass
            else:
                # Everything that is neither a blanket keyword nor one of the inert
                # values above is an interface name.
                #
                # Stored lower-cased, and compared against a lower-cased name below:
                # the classifier that decided this was a tunnel lower-cases too, and on
                # Windows the identity it classified is the adapter's friendly name or
                # description ("Tailscale", "WireGuard Tunnel"). Matching the raw
                # strings would make PROVIZIO_DDS_ALLOW_VPN_INTERFACES=tailscale
                # classify the adapter as a VPN and then fail to re-admit it — an escape
                # hatch that silently does nothing.
                names.add(lowered_entry)
        _allow_vpn_cache = (allow_all, frozenset(names))
        return _allow_vpn_cache


# Which PROVIZIO_DDS_ALLOW_VPN_INTERFACES names have actually re-admitted an interface,
# and whether the mismatch between those and the names given has been reported. Kept apart
# from _allow_vpn_cache, which is immutable once parsed: this is what the classifier learns
# as it runs, and it is the only evidence that a name given means anything on this host.
_matched_vpn_override_names: "Set[str]" = set()
_unmatched_vpn_override_reported = False
_matched_vpn_override_lock = threading.Lock()


def take_unmatched_vpn_allow_override_names() -> "Tuple[str, ...]":
    """The ``PROVIZIO_DDS_ALLOW_VPN_INTERFACES`` names that have re-admitted nothing,
    lower-cased and sorted; empty when every name given has matched an interface this
    process classified as a tunnel, when the variable names none, or when it was set to a
    blanket keyword.

    Exists because such a name is silent otherwise. ``PROVIZIO_DDS_ALLOW_VPN_INTERFACES=
    tailscale`` on a host whose device is ``tailscale0`` -- or a Windows adapter named
    under its device identity rather than the friendly name the classifier saw -- leaves
    the tunnel excluded for the life of the process, exactly as if the variable had never
    been set, and the deployment that needed DDS over the tunnel silently does not get it.

    "Matched" means the name re-admitted an interface that WAS classified as a tunnel,
    which is the only thing the variable can do; a name that matches a real but ordinary
    interface counts as unmatched, since naming it changes nothing either way.

    Takes: the names come back at most once per process, so a caller need not latch. Only a
    call that returns something spends that, so a host where every name matched leaves the
    report available for whichever participant later finds one that did not. Ask only where
    the answer is actionable, i.e. AFTER an enumeration that classified this host's
    interfaces and only where the exclusion actually excluded something; asking first would
    report every name as unmatched, because none of them has been offered an interface yet.
    Mirrors take_unmatched_vpn_allow_override_names in src/vpn_interfaces.cpp."""
    global _unmatched_vpn_override_reported
    allow_all, allowed_names = _allow_vpn_override()
    if allow_all or not allowed_names:
        return ()  # Nothing named, or everything allowed: no name can be wrong.
    with _matched_vpn_override_lock:
        unmatched = tuple(sorted(allowed_names - _matched_vpn_override_names))
        if not unmatched or _unmatched_vpn_override_reported:
            return ()
        _unmatched_vpn_override_reported = True
    return unmatched


def _excluded_as_vpn_interface(name: str, platform_says_vpn: bool = False) -> bool:
    """Whether an interface must be kept out of the DDS transports (and out of
    change detection) as a VPN endpoint.

    PROVIZIO_DDS_ALLOW_VPN_INTERFACES is the only knob that re-admits a tunnel:
    naming one in PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES does not, because
    that would put an interface back into change detection whose addresses the
    transports still refuse to bind. Names are matched case-insensitively, on the
    same terms the classifier used."""
    if not platform_says_vpn and not _is_vpn_interface_name(name):
        return False
    allow_all, allowed_names = _allow_vpn_override()
    if allow_all:
        return False
    lowered = _lower_ascii(name)
    if lowered not in allowed_names:
        return True
    # Recorded because the opposite -- a name that never matches anything -- is otherwise
    # indistinguishable from the variable not being set at all, and that is exactly what a
    # typo produces (see take_unmatched_vpn_allow_override_names). Recorded here rather
    # than at parse time because only the classifier knows the identities this host
    # actually has: the same name means different things on different machines, and on
    # Windows the identity compared is an adapter's friendly name or description rather
    # than a device name.
    with _matched_vpn_override_lock:
        _matched_vpn_override_names.add(lowered)
    return False


_EXTRA_INTERFACES_ENV = "PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES"
_extra_interfaces_cache: "Optional[FrozenSet[str]]" = None
_extra_interfaces_lock = threading.Lock()


def _force_included_interfaces() -> FrozenSet[str]:
    """Interface names force-included via
    ``PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES`` (comma-separated, e.g.
    ``"br0,virbr2"``). Such an interface bypasses the name / kind / adapter-type
    exclusions, but still has to be operationally up, carry a non-link-local
    address, and not be loopback — and it does NOT bypass the VPN filter, because
    re-admitting a tunnel there would put an interface back into change detection
    whose addresses the transports still refuse to bind
    (``PROVIZIO_DDS_ALLOW_VPN_INTERFACES`` is the knob for that).

    The escape hatch exists because the exclusions are heuristics: a host whose
    primary NIC *is* a bridge (``br0`` on a vehicle PC) would otherwise have its
    only DDS-relevant interface filtered out, and no change on it would ever
    trigger a recovery. Broadening the defaults instead is not an option —
    ``docker0`` / ``virbr0`` are bridges too, and their churn is exactly the noise
    the filters exist to drop.

    Read once per process; mirrors the C++ ``detail::force_included_interfaces``.

    Deliberately does NOT log: it is called from :func:`_capture_address_snapshot`,
    which ``register_participant`` calls while holding ``_registry_lock``. Emitting a
    log line there would run the user's callback under that lock, and a callback that
    constructs another participant — a documented, supported pattern — would deadlock
    re-entering ``register_participant``. The coordinator reports the force-included
    set from its own after-the-lock log instead.
    """

    global _extra_interfaces_cache
    with _extra_interfaces_lock:
        if _extra_interfaces_cache is None:
            raw = os.environ.get(_EXTRA_INTERFACES_ENV, "")
            _extra_interfaces_cache = frozenset(_split_comma_separated(raw))
        return _extra_interfaces_cache


# Netlink / rtnetlink constants we need. Re-declared here rather than reading
# from /usr/include/linux/... because Python's socket module exposes
# AF_NETLINK / NETLINK_ROUTE but not the rtnetlink ones, and on platforms
# where the kernel headers happen to differ this gives us a stable surface.
_NETLINK_ROUTE = 0
_RTM_GETLINK = 18
_RTM_NEWLINK = 16
_NLM_F_REQUEST = 0x01
_NLM_F_DUMP = 0x300  # NLM_F_ROOT | NLM_F_MATCH
_NLMSG_DONE = 3
_NLMSG_ERROR = 2
_IFLA_IFNAME = 3
_IFLA_LINKINFO = 18
_IFLA_INFO_KIND = 1

# Bound the netlink round-trip so it can never wedge the caller. The first
# recovery-enabled make_domain_participant captures the initial snapshot
# synchronously while the coordinator registry lock is held, so an untimed
# recv() on a delayed/lost kernel reply would stall every other participant.
# A dump from the local kernel completes in well under a millisecond; a few
# seconds is a generous ceiling whose only effect on timeout is to fall back
# to name-prefix filtering (an empty kind map), exactly as on any other error.
_NETLINK_RECV_TIMEOUT_SEC = 2.0

# Bound on datagrams discarded as not-from-the-kernel. The timeout above only fires
# while the socket is IDLE, so a CAP_NET_ADMIN process writing to this socket's netlink port
# keeps recvmsg() returning data and the discard would spin forever, holding up the
# registration that is waiting on the dump. A real dump is a handful of datagrams, so
# any host reaches NLMSG_DONE long before this; exceeding it means someone else is
# filling the socket, and bailing out degrades to name-prefix filtering exactly as
# every other failure here does. Mirrors src/address_snapshot_linux.cpp's
# max_foreign_datagrams.
_MAX_FOREIGN_NETLINK_DATAGRAMS = 64


def _align4(n: int) -> int:
    return (n + 3) & ~3


def _no_link_kinds() -> Dict[int, str]:
    """What every exit that could not produce an authoritative kind map returns, so the
    report cannot be missed by a branch added later -- and so the fallback to name-prefix
    matching is never silent (see :func:`report_interface_kind_lookup_failed`).

    Empty rather than partial for the reason the recv loop gives: a half-parsed dump
    misclassifies virtual interfaces as "no kind" instead of leaving them to the names.
    Mirrors ``no_link_kinds`` in src/address_snapshot_linux.cpp."""
    report_interface_kind_lookup_failed()
    return {}


def _fetch_link_kinds_linux() -> Dict[int, str]:
    """Issue an RTM_GETLINK netlink dump and return a {ifindex: kind} map.

    Mirrors the C++ ``fetch_link_kinds`` in src/address_snapshot_linux.cpp
    one-for-one. Uses Python's stdlib socket.AF_NETLINK (no ctypes) plus
    the ``struct`` module to lay out the request and parse replies. Returns
    an empty dict on any failure — callers should treat a missing entry as
    "no kind" (i.e. assume physical Ethernet / Wi-Fi and include).

    Why this exists in Python at all (the C++ side does the same thing):
    without an IFLA_INFO_KIND filter, the Python snapshot would include
    every bridge / vxlan / macvlan / tun device on the host, so an
    interface that the C++ side ignores would trigger Python-side resets
    on every churn event — Docker / Kubernetes / WireGuard hosts would
    see the asymmetry immediately.
    """
    if sys.platform != "linux":
        return {}

    try:
        sock = socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, _NETLINK_ROUTE)
    except (OSError, AttributeError):
        return _no_link_kinds()

    sock.settimeout(_NETLINK_RECV_TIMEOUT_SEC)

    # Bound explicitly so this socket's port id is known (getsockname below), which is
    # what lets every reply be checked against THIS request from THIS socket. Note what
    # the bind does not buy: port id 0 asks for the same autobind the implicit bind
    # performs, so the port id is no less guessable than before — and rtnetlink is
    # registered without NL_CFG_F_NONROOT_SEND, so an unprivileged process cannot unicast
    # to another socket's port id at all (sendto() to a non-zero port id returns EPERM).
    # The checks below are therefore about correctness rather than defence: a reply that
    # answers someone else's dump, or that did not come from the kernel, must not be
    # parsed as if it described this host's links — and a CAP_NET_ADMIN process that could
    # forge one could simply create a real wireguard device instead. Mirrors
    # src/address_snapshot_linux.cpp.
    try:
        sock.bind((0, 0))
        local_port_id = sock.getsockname()[0]
    except OSError:
        sock.close()
        return _no_link_kinds()

    request_sequence = 1
    kinds: Dict[int, str] = {}
    # Only a dump terminated by a clean NLMSG_DONE is authoritative. A dump cut
    # short by NLMSG_ERROR, a truncated datagram, or a recv timeout would leave
    # a partial map that silently misclassifies virtual interfaces as "no kind"
    # and admits them into the snapshot — the exact asymmetry this function
    # exists to prevent. On any such failure we return {} so the caller falls
    # back to name-prefix filtering, identical to the C++ side.
    dump_complete = False
    try:
        # nlmsghdr (16 bytes) + ifinfomsg (16 bytes) = 32-byte request.
        # nlmsghdr: u32 len, u16 type, u16 flags, u32 seq, u32 pid
        # ifinfomsg: u8 family, u8 pad, u16 type, s32 index, u32 flags, u32 change
        request = struct.pack(
            "=IHHII" "BBHiII",
            32,
            _RTM_GETLINK,
            _NLM_F_REQUEST | _NLM_F_DUMP,
            request_sequence,
            0,
            socket.AF_UNSPEC,
            0,
            0,
            0,
            0,
            0,
        )
        sock.send(request)

        # Multi-datagram dump — loop until NLMSG_DONE. recvmsg (not recv) so we
        # can see the MSG_TRUNC flag the kernel sets when a datagram is larger
        # than the buffer; a silently truncated datagram would drop links on
        # hosts with very many interfaces.
        stop = False
        foreign_datagrams = 0
        while not stop:
            buf, _ancdata, msg_flags, addr = sock.recvmsg(16 * 1024)
            # Checked before the emptiness test, so a zero-length datagram cannot reach
            # the break without having passed it, and failing CLOSED on an address the
            # kernel did not give us — matching the C++ side, which requires the source to
            # be exactly a sockaddr_nl. Only the kernel (port id 0) may answer; anything
            # else is discarded rather than parsed — see the bind() above.
            if not addr or addr[0] != 0:
                foreign_datagrams += 1
                if foreign_datagrams > _MAX_FOREIGN_NETLINK_DATAGRAMS:
                    # Silent: falls back to name-prefix filtering, which is correct.
                    break
                continue
            if not buf:
                break
            if msg_flags & socket.MSG_TRUNC:
                # Silent: falls back to name-prefix filtering, which is correct.
                break
            offset = 0
            while offset + 16 <= len(buf):
                nlmsg_len, nlmsg_type, _flags, seq, port_id = struct.unpack_from(
                    "=IHHII", buf, offset
                )
                if nlmsg_len < 16 or offset + nlmsg_len > len(buf):
                    break
                # Step to the next message before parsing this one, and read the body off
                # message_start: that way no branch below can strand the cursor by
                # skipping the step, which is what hung the Windows unicast walk. The
                # C++ side gets the same guarantee from NLMSG_NEXT in a for-increment.
                message_start = offset
                message_end = offset + nlmsg_len
                offset += _align4(nlmsg_len)
                # Belongs to THIS request from THIS socket. With the source-port check
                # above, this is what keeps a reply that answers someone else's dump — or
                # that is addressed to another socket's port id — from being parsed as if
                # it described this host's links.
                if seq != request_sequence or port_id != local_port_id:
                    continue
                if nlmsg_type == _NLMSG_DONE:
                    dump_complete = True
                    stop = True
                    break
                if nlmsg_type == _NLMSG_ERROR:
                    # Silent: falls back to name-prefix filtering, which is correct.
                    stop = True
                    break
                if nlmsg_type == _RTM_NEWLINK:
                    # ifinfomsg starts immediately after nlmsghdr (16 B).
                    ifinfo_offset = message_start + 16
                    if ifinfo_offset + 16 <= message_end:
                        _family, _pad, _itype, ifindex, _iflags, _change = struct.unpack_from(
                            "=BBHiII", buf, ifinfo_offset
                        )
                        # Walk top-level rtattrs after the ifinfomsg.
                        rta_offset = ifinfo_offset + 16
                        rta_end = message_end
                        kind = ""
                        while rta_offset + 4 <= rta_end:
                            rta_len, rta_type = struct.unpack_from("=HH", buf, rta_offset)
                            if rta_len < 4 or rta_offset + rta_len > rta_end:
                                break
                            if rta_type == _IFLA_LINKINFO:
                                # Walk nested attributes for IFLA_INFO_KIND.
                                nested_offset = rta_offset + 4
                                nested_end = rta_offset + rta_len
                                while nested_offset + 4 <= nested_end:
                                    n_rta_len, n_rta_type = struct.unpack_from(
                                        "=HH", buf, nested_offset
                                    )
                                    if n_rta_len < 4 or nested_offset + n_rta_len > nested_end:
                                        break
                                    if n_rta_type == _IFLA_INFO_KIND:
                                        raw = bytes(buf[nested_offset + 4 : nested_offset + n_rta_len])
                                        kind = raw.rstrip(b"\x00").decode("ascii", errors="replace")
                                        break
                                    nested_offset += _align4(n_rta_len)
                                break
                            rta_offset += _align4(rta_len)
                        kinds[ifindex] = kind
    except OSError:
        # Includes socket.timeout: a stalled kernel reply must not be treated
        # as a complete dump. dump_complete stays False → empty map below.
        pass
    finally:
        sock.close()

    return kinds if dump_complete else _no_link_kinds()


def _linux_kind_excluded(kind: str) -> bool:
    """Mirror src/address_snapshot_linux.cpp::kind_excluded — physical
    Ethernet / Wi-Fi report no kind so the empty string is included."""
    return bool(kind) and kind in _LINUX_EXCLUDED_KINDS


_MACOS_NAME_PREFIXES = (
    "awdl",
    "llw",
    "utun",
    "bridge",
    "gif",
    "stf",
    "anpi",
)


def _macos_name_excluded(name: str) -> bool:
    """Apple-internal NICs and Wi-Fi AP-mode interfaces (ap0, ap1, …) — but
    NOT user names that merely start with 'ap' (require a digit after)."""
    for prefix in _MACOS_NAME_PREFIXES:
        if name.startswith(prefix):
            return True
    # ap\d+ specifically — ap alone or "appliance0" does not match.
    if name.startswith("ap") and len(name) > 2 and name[2].isdigit():
        return True
    return False


def _linux_name_excluded(name: str) -> bool:
    for prefix in _LINUX_NAME_PREFIXES:
        if name.startswith(prefix):
            return True
    return False


# POSIX (Linux + macOS) — getifaddrs via libc -------------------------------


# BSD vs Linux sockaddr layout: BSD systems (macOS / FreeBSD / OpenBSD) put
# a one-byte `sa_len` at offset 0, then a one-byte `sa_family` at offset 1.
# Linux / glibc has no `sa_len` — `sa_family` is the leading 16-bit field.
# Reading two bytes as a `c_ushort` against the BSD layout yields
# `(sa_family << 8) | sa_len` (e.g. 0x1002 for an AF_INET with sa_len=16),
# silently failing every family check. So we branch on platform — the whole BSD
# family, matching the C++ counterpart's __APPLE__ || __FreeBSD__ || __OpenBSD__
# || __NetBSD__ guard (src/detail/netmask_prefix.h), so the bounded sockaddr read
# holds on any BSD this ever runs on, not just macOS.
_IS_BSD_SOCKADDR = sys.platform.startswith(("darwin", "freebsd", "openbsd", "netbsd"))


class _In6Addr(ctypes.Structure):
    _fields_ = [("s6_addr", ctypes.c_ubyte * 16)]


if _IS_BSD_SOCKADDR:

    class _Sockaddr(ctypes.Structure):
        _fields_ = [
            ("sa_len", ctypes.c_ubyte),
            ("sa_family", ctypes.c_ubyte),
            ("sa_data", ctypes.c_byte * 14),
        ]

    class _SockaddrIn(ctypes.Structure):
        # sin_addr laid out as a 4-byte array — see Linux branch for the
        # rationale.
        _fields_ = [
            ("sin_len", ctypes.c_ubyte),
            ("sin_family", ctypes.c_ubyte),
            ("sin_port", ctypes.c_ushort),
            ("sin_addr", ctypes.c_ubyte * 4),
            ("sin_zero", ctypes.c_byte * 8),
        ]

    class _SockaddrIn6(ctypes.Structure):
        _fields_ = [
            ("sin6_len", ctypes.c_ubyte),
            ("sin6_family", ctypes.c_ubyte),
            ("sin6_port", ctypes.c_ushort),
            ("sin6_flowinfo", ctypes.c_uint32),
            ("sin6_addr", _In6Addr),
            ("sin6_scope_id", ctypes.c_uint32),
        ]

else:

    class _Sockaddr(ctypes.Structure):
        _fields_ = [
            ("sa_family", ctypes.c_ushort),
            ("sa_data", ctypes.c_byte * 14),
        ]

    class _SockaddrIn(ctypes.Structure):
        # sin_addr is laid out as a 4-byte array in network byte order. We
        # declare it as `c_ubyte * 4` so `bytes(sin.sin_addr)` yields the
        # network-order representation that `socket.inet_ntop(AF_INET, ...)`
        # expects directly. Declaring it as `c_uint32` would force a manual
        # byteswap on little-endian hosts.
        _fields_ = [
            ("sin_family", ctypes.c_ushort),
            ("sin_port", ctypes.c_ushort),
            ("sin_addr", ctypes.c_ubyte * 4),
            ("sin_zero", ctypes.c_byte * 8),
        ]

    class _SockaddrIn6(ctypes.Structure):
        _fields_ = [
            ("sin6_family", ctypes.c_ushort),
            ("sin6_port", ctypes.c_ushort),
            ("sin6_flowinfo", ctypes.c_uint32),
            ("sin6_addr", _In6Addr),
            ("sin6_scope_id", ctypes.c_uint32),
        ]


class _Ifaddrs(ctypes.Structure):
    pass


_Ifaddrs._fields_ = [
    ("ifa_next", ctypes.POINTER(_Ifaddrs)),
    ("ifa_name", ctypes.c_char_p),
    ("ifa_flags", ctypes.c_uint),
    ("ifa_addr", ctypes.POINTER(_Sockaddr)),
    ("ifa_netmask", ctypes.POINTER(_Sockaddr)),
    ("ifa_broadaddr_or_dstaddr", ctypes.POINTER(_Sockaddr)),
    ("ifa_data", ctypes.c_void_p),
]


# IFF_LOOPBACK / IFF_RUNNING — defined in net/if.h, and identical on Linux and
# macOS. Hard-coded to avoid an additional ctypes lookup against libc constants that
# differ by platform. IFF_RUNNING (operationally up: administratively up AND carrier
# present) is the flag Fast-DDS' IPFinder filters on, so it is the one that governs
# snapshot membership — see _capture_address_snapshot.
_IFF_RUNNING_VALUE = 0x40
_IFF_LOOPBACK_VALUE = 0x8


def _iter_linked_nodes(head: Any, next_field: str) -> "Iterator[Any]":
    """Yield every node of a singly linked list the OS handed us, starting at ``head``
    and stopping at the first null pointer. A null ``head`` yields nothing.

    Stepping to the next node belongs here rather than in the caller's loop body:
    a body that skips a node with ``continue`` would otherwise have to remember to
    step first, and one that forgot stranded the cursor and spun on the same node
    forever. ``next_field`` names the node's pointer-to-next member, which differs
    between the lists this walks (``ifa_next`` from getifaddrs, ``Next`` from
    GetAdaptersAddresses)."""
    cursor = head
    while cursor:
        node = cursor.contents
        yield node
        cursor = getattr(node, next_field)


def _prefix_length_from_netmask(netmask_ptr) -> int:
    """CIDR prefix length of a ``getifaddrs`` netmask, e.g. 24 for 255.255.255.0.

    Returns 0 for a null mask (some point-to-point / tunnel devices report none) or
    a family other than AF_INET / AF_INET6. Mirrors the C++
    ``detail::prefix_length_from_netmask``.
    """

    if not netmask_ptr:
        return 0
    family = netmask_ptr.contents.sa_family
    if family == _AF_INET:
        address_offset = _SockaddrIn.sin_addr.offset
        width = 4
    elif family == _AF_INET6:
        address_offset = _SockaddrIn6.sin6_addr.offset
        width = 16
    else:
        return 0

    available = width
    if _IS_BSD_SOCKADDR:
        # BSD-family getifaddrs copies each sockaddr using only its self-reported
        # sa_len, and a netmask is the canonical short one: trailing all-zero bytes are
        # omitted, so a /8 mask can be as little as sin_len = offsetof(sin_addr) + 1.
        # Omitted bytes are zero by definition and prefix counting stops at the first
        # zero bit, so clamping loses nothing. Linux has no sa_len and glibc always
        # materialises a full-size mask, so the clamp is a no-op there.
        available = min(width, max(0, netmask_ptr.contents.sa_len - address_offset))
    if available <= 0:
        return 0

    # Read ONLY the bytes that are actually present. Casting to _SockaddrIn /
    # _SockaddrIn6 and taking bytes(...sin_addr) would materialise the full fixed-size
    # field FIRST and clamp afterwards — i.e. the out-of-bounds read would already have
    # happened, reaching into the neighbouring sockaddr of the same buffer or past the
    # allocation entirely for the last entry.
    mask = ctypes.string_at(ctypes.addressof(netmask_ptr.contents) + address_offset, available)

    bits = 0
    for byte in mask:
        # The kernel only ever produces contiguous masks, so counting whole 0xFF
        # bytes and then the leading bits of the first partial byte is exact.
        if byte == 0xFF:
            bits += 8
            continue
        for shift in range(7, -1, -1):
            if not byte & (1 << shift):
                break
            bits += 1
        break
    return bits

_AF_INET = socket.AF_INET
_AF_INET6 = socket.AF_INET6


def _load_libc():
    if sys.platform == "darwin":
        return ctypes.CDLL("libc.dylib", use_errno=True)
    return ctypes.CDLL("libc.so.6", use_errno=True)


def _iter_posix_interface_addresses(
    include_loopback: bool = False,
) -> "Iterator[Tuple[str, str, str, int]]":
    """Yield ``(name, kind, addr_text, prefix_length)`` for every interface address
    the kernel reports that Fast-DDS could bind a locator to.

    Applies only the *operational* filters — non-loopback (unless ``include_loopback``,
    which :func:`allowed_interfaces` needs because Fast-DDS gives loopback a sender socket
    of its own), IFF_RUNNING, IPv4/IPv6, no IPv6 link-local — and leaves every policy
    filter to the callers, which want
    different ones: the snapshot drops container plumbing and tunnels, while
    :func:`vpn_interface_blocklist_entries` wants exactly the tunnels. Sharing the walk is
    what keeps the two from drifting apart on which addresses exist in the first
    place. ``kind`` is the rtnetlink IFLA_INFO_KIND on Linux and always empty on
    macOS. Mirrors ``enumerate_interface_addresses`` in
    src/address_snapshot_linux.cpp.

    Raises ``OSError`` when the interfaces cannot be read at all, rather than yielding
    nothing: an empty result is a legitimate reading, so a failure that looked like one
    would be taken for every address disappearing (see
    :func:`_try_capture_address_snapshot`)."""
    libc = _load_libc()

    getifaddrs = libc.getifaddrs
    getifaddrs.argtypes = [ctypes.POINTER(ctypes.POINTER(_Ifaddrs))]
    getifaddrs.restype = ctypes.c_int

    freeifaddrs = libc.freeifaddrs
    freeifaddrs.argtypes = [ctypes.POINTER(_Ifaddrs)]
    freeifaddrs.restype = None

    head = ctypes.POINTER(_Ifaddrs)()
    if getifaddrs(ctypes.byref(head)) != 0:
        # On macOS this is a sysctl(NET_RT_IFLIST) size-then-fetch pair that can lose a
        # race with a routing-table change, so the failure is a live possibility rather
        # than a formality.
        errno_value = ctypes.get_errno()
        raise OSError(errno_value, f"getifaddrs failed: {os.strerror(errno_value)}")

    is_macos = sys.platform == "darwin"

    # On Linux the interface's IFLA_INFO_KIND attribute is what identifies virtual /
    # container / tunnel devices — mirrors src/address_snapshot_linux.cpp.
    # if_nametoindex maps each ifa entry's name to the ifindex key in the kind map.
    kinds_by_index: Dict[int, str] = _fetch_link_kinds_linux() if not is_macos else {}
    if_nametoindex = None
    if kinds_by_index:
        try:
            if_nametoindex = libc.if_nametoindex
            if_nametoindex.argtypes = [ctypes.c_char_p]
            if_nametoindex.restype = ctypes.c_uint
        except (AttributeError, OSError):
            if_nametoindex = None

    try:
        for entry in _iter_linked_nodes(head, "ifa_next"):
            # surrogateescape, not "replace": a non-ASCII device name must round-trip to
            # the bytes Fast-DDS compares, and U+FFFD placeholders could never match them.
            name = (
                entry.ifa_name.decode("ascii", errors="surrogateescape")
                if entry.ifa_name
                else ""
            )
            flags = entry.ifa_flags
            if (flags & _IFF_LOOPBACK_VALUE) != 0 and not include_loopback:
                continue
            if (flags & _IFF_RUNNING_VALUE) == 0:
                continue
            if not name:
                continue
            kind = ""
            if if_nametoindex is not None:
                kind = kinds_by_index.get(int(if_nametoindex(entry.ifa_name)), "")
            if entry.ifa_addr:
                family = entry.ifa_addr.contents.sa_family
                addr_text: Optional[str] = None
                if family == _AF_INET:
                    sin = ctypes.cast(entry.ifa_addr, ctypes.POINTER(_SockaddrIn)).contents
                    addr_text = socket.inet_ntop(_AF_INET, bytes(sin.sin_addr))
                elif family == _AF_INET6:
                    sin6 = ctypes.cast(entry.ifa_addr, ctypes.POINTER(_SockaddrIn6)).contents
                    addr_text = socket.inet_ntop(_AF_INET6, bytes(sin6.sin6_addr.s6_addr))
                    if addr_text and _is_link_local_ipv6(addr_text):
                        continue
                if addr_text:
                    yield (
                        name,
                        kind,
                        addr_text,
                        _prefix_length_from_netmask(entry.ifa_netmask),
                    )
    finally:
        freeifaddrs(head)


# Whether every participant in this process managed to apply the VPN exclusion to its
# transports. Set once and never cleared, and read by the snapshot policy below.
#
# A tunnel this library kept the transports off cannot move any locator, so its churn must
# not rebuild anything -- that is the whole point of dropping it from the snapshot. But
# where the exclusion could NOT be applied (the caller owns the transports through their
# own XML, through FASTDDS_BUILTIN_TRANSPORTS or through descriptors they configured, the
# generated-profile ceiling refused one, or this host's interfaces could not be read when
# the participant was configured) DDS binds and announces the tunnel after all,
# and dropping it here would leave a re-auth or a reconnect with a dead locator that no
# rebuild replaces. The two filters disagreeing about one interface is the one outcome
# neither may produce. Mirrors report_vpn_exclusion_not_applied /
# vpn_exclusion_applies_to_transports in src/vpn_interfaces.cpp.
#
# A plain module-level bool rather than a lock: it is written once, with the same value,
# and a read that races a write can only see the stale True, whose cost is one unnecessary
# rebuild -- the safe direction, and the same one-way conservatism the C++ latch has.
_vpn_exclusion_not_applied = False


def report_vpn_exclusion_not_applied() -> None:
    """Record that a participant could NOT apply the VPN exclusion to its transports, so
    nothing in this process may assume a tunnel is unbound."""
    global _vpn_exclusion_not_applied
    _vpn_exclusion_not_applied = True


def vpn_exclusion_applies_to_transports() -> bool:
    """Whether the VPN exclusion is believed to reach the transports of every participant
    in this process -- i.e. :func:`report_vpn_exclusion_not_applied` was never called."""
    return not _vpn_exclusion_not_applied


# Whether an interface classification has run without the platform's own interface-kind
# information since the last time anyone asked. On Linux that information is the rtnetlink
# IFLA_INFO_KIND, which is what keeps the classification working for a tunnel renamed away
# from the conventional prefixes (a WireGuard device called office0); when the dump cannot
# be issued or does not complete, the classifier falls back to names and such a device is
# taken for ordinary hardware, so DDS binds and announces it.
#
# Recorded rather than logged where it happens, and taken by whoever reports it, for the
# same reason the unmatched override names are: the enumeration runs deep inside a
# participant creation, and the report belongs with the other transport-configuration
# lines. Re-armable, unlike the one-way latch above: this describes one enumeration rather
# than a property of the process. Mirrors report_interface_kind_lookup_failed /
# take_interface_kind_lookup_failure_report in src/vpn_interfaces.cpp.
_interface_kind_lookup_failed = False


def report_interface_kind_lookup_failed() -> None:
    """Record that an interface classification ran without the platform's own
    interface-kind information, so every verdict it produced came from names alone."""
    global _interface_kind_lookup_failed
    _interface_kind_lookup_failed = True


def take_interface_kind_lookup_failure_report() -> bool:
    """Whether an interface classification has run on names alone since this was last
    asked. Answers ``True`` at most once per occurrence, so the caller need not latch."""
    global _interface_kind_lookup_failed
    failed = _interface_kind_lookup_failed
    _interface_kind_lookup_failed = False
    return failed


def _snapshot_policy_excludes_posix(name: str, kind: str) -> bool:
    """Whether the snapshot drops a POSIX interface on *policy* grounds: the VPN
    filter, the name / kind heuristics, and the force-include escape hatch that
    bypasses them. The operational filters (loopback, carrier, address family,
    link-local) belong to the walk, which applies them before asking this.

    Its own function for the same two reasons the C++
    ``snapshot_policy_excludes_interface`` is: the policy is then stated exactly once,
    so this filter and ``vpn_interface_blocklist_entries`` cannot come to disagree
    about one interface, and it can be tested for a tunnel on a host that has none up.
    """
    platform_says_vpn = _is_vpn_interface_kind(kind)

    # Only where the exclusion actually reached the transports -- see
    # _vpn_exclusion_not_applied for why a tunnel DDS ended up binding must stay watched.
    may_drop_tunnels = vpn_exclusion_applies_to_transports()

    # A VPN interface is excluded before anything else and regardless of
    # force-inclusion: the transports refuse to bind it (see
    # vpn_interface_blocklist_entries), so its address churn — a Tailscale re-auth, a
    # tunnel reconnect — can no longer change any locator, and rebuilding every
    # participant over it would be pure disruption.
    # Every address family, unlike the IPv4-only blocklist in
    # _vpn_blocklist_entries_posix: that restriction is about what a UDPv4 transport
    # can be told to block, which has no bearing on what is worth rebuilding for. A
    # tunnel's IPv6 address is exactly as unable to change a locator as its IPv4 one,
    # so a rotated fd7a:: address must not trigger a rebuild either.
    if may_drop_tunnels and _excluded_as_vpn_interface(name, platform_says_vpn):
        return True

    # Reached for a tunnel only when PROVIZIO_DDS_ALLOW_VPN_INTERFACES re-admitted it,
    # and the heuristics below must not then drop it again: "utun" is in the macOS
    # prefix list, so consulting it for an allowed utunN would leave change detection
    # ignoring an interface the transports do bind — the one disagreement between the
    # two filters that must never happen, and one no runner without a live tunnel
    # would notice.
    if platform_says_vpn or _is_vpn_interface_name(name):
        return False

    # A force-included interface skips the name / kind heuristics but not the
    # loopback, carrier and link-local checks applied by the walk.
    if name in _force_included_interfaces():
        return False

    name_excluded = (
        _macos_name_excluded if sys.platform == "darwin" else _linux_name_excluded
    )
    return name_excluded(name) or _linux_kind_excluded(kind)


def _capture_snapshot_posix() -> AddressSnapshot:
    result: set = set()
    for name, kind, addr_text, prefix_length in _iter_posix_interface_addresses():
        if _snapshot_policy_excludes_posix(name, kind):
            continue
        result.add((name, addr_text, prefix_length))

    return frozenset(result)


def _vpn_blocklist_entries_posix() -> "FrozenSet[str]":
    """POSIX backend of :func:`vpn_interface_blocklist_entries`."""
    entries = set()
    for name, kind, addr_text, _ in _iter_posix_interface_addresses():
        # IPv4 only, deliberately: this library configures SHM + UDPv4, and a UDPv4
        # transport enumerates no IPv6 interface — so an IPv6 entry, or the name of a
        # tunnel with no IPv4 address, could never match anything it binds, while ANY
        # non-empty blocklist still forces whitelist mode and netmask filtering on (and
        # with it the loss of peers reachable only through a gateway). Mirrors
        # src/address_snapshot_linux.cpp.
        if ":" not in addr_text and _excluded_as_vpn_interface(
            name, _is_vpn_interface_kind(kind)
        ):
            # Both forms — see vpn_interface_blocklist_entries for why the device name
            # matters alongside the address. A non-ASCII name is carried by its addresses
            # alone: it would have to go through the generated XML profile, where a
            # surrogate byte is not representable, so the whole document would be rejected
            # and nothing at all would be excluded.
            if name.isascii():
                entries.add(name)
            entries.add(addr_text)
    return frozenset(entries)


# Windows — GetAdaptersAddresses via iphlpapi --------------------------------


_WIN_EXCLUDED_DESCRIPTIONS = (
    "Hyper-V Virtual",
    "WSL",
    "VirtualBox Host-Only",
    "VMware Virtual",
    "TAP-Windows",
    "Loopback Pseudo-Interface",
    "Tunnel adapter",
)

# IfOperStatusUp and the kept IfTypes — values from iptypes.h.
_IF_OPER_STATUS_UP = 1
_IF_TYPE_ETHERNET_CSMACD = 6
_IF_TYPE_SOFTWARE_LOOPBACK = 24
_IF_TYPE_PPP = 23
_IF_TYPE_IEEE80211 = 71
# IF_TYPE_TUNNEL — the adapter type Windows reports for encapsulation interfaces
# (Tailscale, WireGuard NT, Teredo / ISATAP), i.e. the platform's own VPN signal.
_IF_TYPE_TUNNEL = 131
# IpDadStatePreferred — the only DAD state whose address is usable, per iptypes.h.
_IP_DAD_STATE_PREFERRED = 4
_KEPT_IF_TYPES = (_IF_TYPE_ETHERNET_CSMACD, _IF_TYPE_PPP, _IF_TYPE_IEEE80211)


def _description_excluded_win(description: str) -> bool:
    for needle in _WIN_EXCLUDED_DESCRIPTIONS:
        if needle in description:
            return True
    return False


# Windows adapter-enumeration ctypes layouts. Defined ONCE at module scope, not inside
# the walk that uses them: a ctypes.Structure subclass is a heavyweight object and
# ctypes.POINTER() memoises a pointer type per class forever, so re-declaring them on
# every call would grow the interpreter's object count for the life of the process —
# visible as a leak once the walk runs per participant creation, which it now does.
# Only the fields we read are declared; skipped regions are opaque byte arrays. The
# sockaddr layouts shared with the POSIX path are fine here too (same binary layout on
# Windows); IP_ADAPTER_ADDRESSES is the large one, laid out only as far as the fields
# this module reads.


class _WinSocketAddress(ctypes.Structure):
    _fields_ = [
        ("lpSockaddr", ctypes.POINTER(_Sockaddr)),
        ("iSockaddrLength", ctypes.c_int),
    ]


class _WinIpAdapterUnicastAddress(ctypes.Structure):
    pass

# The whole struct is declared through OnLinkPrefixLength, which the snapshot
# needs for the prefix length. The enum-typed fields (PrefixOrigin, SuffixOrigin,
# DadState) are c_int, the lifetimes are c_ulong, and OnLinkPrefixLength is the
# trailing UINT8 — matching iptypes.h. DadState IS filtered on this path (see the walk
# below), matching src/address_snapshot_windows.cpp; the POSIX paths cannot, because
# getifaddrs reports no DAD state at all.
_WinIpAdapterUnicastAddress._fields_ = [
    ("Length", ctypes.c_ulong),
    ("Flags", ctypes.c_ulong),
    ("Next", ctypes.POINTER(_WinIpAdapterUnicastAddress)),
    ("Address", _WinSocketAddress),
    ("PrefixOrigin", ctypes.c_int),
    ("SuffixOrigin", ctypes.c_int),
    ("DadState", ctypes.c_int),
    ("ValidLifetime", ctypes.c_ulong),
    ("PreferredLifetime", ctypes.c_ulong),
    ("LeaseLifetime", ctypes.c_ulong),
    ("OnLinkPrefixLength", ctypes.c_ubyte),
]


class _WinIpAdapterAddresses(ctypes.Structure):
    pass

_WinIpAdapterAddresses._fields_ = [
    ("Length", ctypes.c_ulong),
    ("IfIndex", ctypes.c_ulong),
    ("Next", ctypes.POINTER(_WinIpAdapterAddresses)),
    ("AdapterName", ctypes.c_char_p),
    ("FirstUnicastAddress", ctypes.POINTER(_WinIpAdapterUnicastAddress)),
    ("FirstAnycastAddress", ctypes.c_void_p),
    ("FirstMulticastAddress", ctypes.c_void_p),
    ("FirstDnsServerAddress", ctypes.c_void_p),
    ("DnsSuffix", ctypes.c_wchar_p),
    ("Description", ctypes.c_wchar_p),
    ("FriendlyName", ctypes.c_wchar_p),
    ("PhysicalAddress", ctypes.c_ubyte * 8),
    ("PhysicalAddressLength", ctypes.c_ulong),
    ("Flags", ctypes.c_ulong),
    ("Mtu", ctypes.c_ulong),
    ("IfType", ctypes.c_ulong),
    ("OperStatus", ctypes.c_int),
    # Remaining fields elided — Python only reads what's above.
]

_GAA_FLAG_SKIP_ANYCAST = 0x2
_GAA_FLAG_SKIP_MULTICAST = 0x4
_GAA_FLAG_SKIP_DNS_SERVER = 0x8
_WIN_AF_UNSPEC = 0
_WIN_ERROR_BUFFER_OVERFLOW = 111
_WIN_NO_ERROR = 0


def _iter_windows_adapter_addresses(
    include_loopback: bool = False,
) -> "Iterator[Tuple[str, str, str, int, str, int]]":
    """Yield ``(name, friendly, description, if_type, addr_text, prefix_length)`` for
    every adapter address Fast-DDS could bind a locator to.

    Applies only the *operational* filters (adapter up, not loopback, IPv4/IPv6, no
    IPv6 link-local) and leaves policy to the callers — the Windows counterpart of
    :func:`_iter_posix_interface_addresses`, and for the same reason.

    The Windows path uses ctypes against iphlpapi.GetAdaptersAddresses. We
    deliberately keep the struct definitions minimal — only the fields we read are
    declared. Padding to match the actual layout is achieved by declaring opaque
    byte arrays for the skipped regions.

    ``include_loopback`` admits the software loopback adapter, which the snapshot and the
    blocklist never want (a force-included interface still cannot be loopback) and
    :func:`allowed_interfaces` always does, exactly as the POSIX walk's flag of the same
    name works.
    """
    # Raised, not swallowed into an empty walk — see _try_capture_address_snapshot.
    iphlpapi = ctypes.windll.iphlpapi  # type: ignore[attr-defined]

    iphlpapi.GetAdaptersAddresses.argtypes = [
        ctypes.c_ulong,  # Family
        ctypes.c_ulong,  # Flags
        ctypes.c_void_p,  # Reserved
        ctypes.c_void_p,  # AdapterAddresses
        ctypes.POINTER(ctypes.c_ulong),  # SizePointer
    ]
    iphlpapi.GetAdaptersAddresses.restype = ctypes.c_ulong

    buf_size = ctypes.c_ulong(16 * 1024)
    buffer = ctypes.create_string_buffer(buf_size.value)
    for _ in range(3):
        ret = iphlpapi.GetAdaptersAddresses(
            _WIN_AF_UNSPEC,
            _GAA_FLAG_SKIP_ANYCAST | _GAA_FLAG_SKIP_MULTICAST | _GAA_FLAG_SKIP_DNS_SERVER,
            None,
            ctypes.cast(buffer, ctypes.c_void_p),
            ctypes.byref(buf_size),
        )
        if ret == _WIN_ERROR_BUFFER_OVERFLOW:
            buffer = ctypes.create_string_buffer(buf_size.value)
            continue
        break
    if ret != _WIN_NO_ERROR:
        raise OSError(f"GetAdaptersAddresses failed with error {ret}")

    for adapter_data in _iter_linked_nodes(
        ctypes.cast(buffer, ctypes.POINTER(_WinIpAdapterAddresses)), "Next"
    ):
        # OperStatus is Windows' operational (carrier-aware) state, the counterpart of
        # POSIX IFF_RUNNING: a disconnected adapter reports down and is dropped here.
        # Loopback is excluded unless asked for, matching the POSIX walk's IFF_LOOPBACK
        # check: the snapshot and the blocklist never want it (a force-included interface
        # still cannot be loopback), the allowlist always does -- Windows is the one
        # platform where this library never selects shared memory, so the loopback sender
        # socket IS the same-host path there.
        if adapter_data.OperStatus == _IF_OPER_STATUS_UP and (
            include_loopback or adapter_data.IfType != _IF_TYPE_SOFTWARE_LOOPBACK
        ):
            friendly = adapter_data.FriendlyName or ""
            description = adapter_data.Description or ""
            name = (
                adapter_data.AdapterName.decode("ascii", errors="replace")
                if adapter_data.AdapterName
                else friendly
            )
            for unicast in _iter_linked_nodes(adapter_data.FirstUnicastAddress, "Next"):
                if unicast.DadState != _IP_DAD_STATE_PREFERRED:
                    continue  # tentative / deprecated / duplicate
                sa = unicast.Address.lpSockaddr
                if sa:
                    family = sa.contents.sa_family
                    addr_text: Optional[str] = None
                    if family == _AF_INET:
                        sin = ctypes.cast(sa, ctypes.POINTER(_SockaddrIn)).contents
                        addr_text = socket.inet_ntop(_AF_INET, bytes(sin.sin_addr))
                    elif family == _AF_INET6:
                        sin6 = ctypes.cast(sa, ctypes.POINTER(_SockaddrIn6)).contents
                        addr_text = socket.inet_ntop(_AF_INET6, bytes(sin6.sin6_addr.s6_addr))
                        if addr_text and _is_link_local_ipv6(addr_text):
                            addr_text = None
                    if addr_text:
                        # OnLinkPrefixLength is already a CIDR prefix length, so
                        # unlike the POSIX path there is no netmask to convert.
                        yield (
                            name,
                            friendly,
                            description,
                            int(adapter_data.IfType),
                            addr_text,
                            int(unicast.OnLinkPrefixLength),
                        )


def _adapter_is_vpn_win(friendly: str, description: str, if_type: int) -> bool:
    """Whether an adapter is a VPN / tunnel endpoint: an IF_TYPE_TUNNEL adapter, one
    whose driver-supplied description names a tunnel, or one whose friendly name names a
    VPN product. The GUID-ish AdapterName carries no hint on Windows, which is why the
    human-readable fields are what get classified."""
    # Neither string gets the device-name classifier, and for the same reason: both are
    # prose. A user can rename the friendly name ("WG-LAN" on an Ethernet adapter), and a
    # description is the driver's own vendor string, where a model number can begin with
    # the very letters a device convention anchors on -- a NETGEAR WG-series radio
    # describes itself "WG111v3 54Mbps Wireless USB 2.0 Adapter". _is_vpn_description
    # still catches what the description exists to catch: OpenVPN's "TAP-Windows Adapter
    # V9". See src/address_snapshot_windows.cpp.
    return (
        if_type == _IF_TYPE_TUNNEL
        or _is_vpn_description(description)
        or _is_vpn_product_name(friendly)
    )


def _excluded_as_vpn_adapter_win(
    name: str, friendly: str, description: str, if_type: int
) -> bool:
    """VPN exclusion for a Windows adapter, honouring the override against any of the
    three identities Windows reports for it: the GUID-ish adapter name, the friendly
    name, and the driver-supplied description.

    All three, because any of them can be both what classified the adapter and what a
    user would name it by -- an adapter matched on its description alone
    ("TAP-Windows Adapter V9" on a NIC called "Ethernet 3") has no other identity the
    variable could name. Mirrors ``excluded_as_vpn_adapter`` in
    src/address_snapshot_windows.cpp."""
    if not _adapter_is_vpn_win(friendly, description, if_type):
        return False
    return (
        _excluded_as_vpn_interface(friendly, platform_says_vpn=True)
        and _excluded_as_vpn_interface(name, platform_says_vpn=True)
        and _excluded_as_vpn_interface(description, platform_says_vpn=True)
    )


def _snapshot_policy_excludes_windows(
    name: str, friendly: str, description: str, if_type: int
) -> bool:
    """Windows counterpart of :func:`_snapshot_policy_excludes_posix`."""
    # A VPN adapter is excluded before anything else and regardless of
    # force-inclusion — see the matching comment in _snapshot_policy_excludes_posix, and
    # _vpn_exclusion_not_applied for why this only holds where the exclusion reached the
    # transports at all.
    if vpn_exclusion_applies_to_transports() and _excluded_as_vpn_adapter_win(
        name, friendly, description, if_type
    ):
        return True

    # Reached for a tunnel only when the override re-admitted it, and the adapter-type
    # gate below must not then drop it again: IF_TYPE_TUNNEL is none of Ethernet /
    # Wi-Fi / PPP, so consulting that gate for an allowed tunnel would leave change
    # detection ignoring an interface the transports do bind. The description gate is
    # skipped for the same reason -- "Tunnel adapter ..." is exactly what such an
    # adapter is called.
    if _adapter_is_vpn_win(friendly, description, if_type):
        return False

    # Match a force-include on either the GUID-ish AdapterName or the friendly
    # name — a user cannot reasonably be expected to know the former.
    force_included = _force_included_interfaces()
    if name in force_included or friendly in force_included:
        return False

    if if_type not in _KEPT_IF_TYPES:
        return True

    return _description_excluded_win(friendly) or _description_excluded_win(description)


def _capture_snapshot_windows() -> AddressSnapshot:
    result: set = set()
    for (
        name,
        friendly,
        description,
        if_type,
        addr_text,
        prefix_length,
    ) in _iter_windows_adapter_addresses():
        if _snapshot_policy_excludes_windows(name, friendly, description, if_type):
            continue
        result.add((name, addr_text, prefix_length))
    return frozenset(result)


def _vpn_blocklist_entries_windows() -> "FrozenSet[str]":
    """Windows backend of :func:`vpn_interface_blocklist_entries`."""
    entries = set()
    for (
        name,
        friendly,
        description,
        if_type,
        addr_text,
        _,
    ) in _iter_windows_adapter_addresses():
        # IPv4 only, and the adapter's name only alongside an IPv4 address of its own —
        # see the rationale in _vpn_blocklist_entries_posix and
        # src/address_snapshot_windows.cpp.
        if ":" in addr_text:
            continue
        if _excluded_as_vpn_adapter_win(name, friendly, description, if_type):
            # The GUID-ish AdapterName is what Fast-DDS compares a blocklist entry to as
            # IPFinder::info_IP::dev, and unlike an address it cannot later belong to a
            # real interface. Falls back to the friendly name when Windows reported no
            # AdapterName, matching the C++ side.
            device_name = name or friendly
            if device_name:
                entries.add(device_name)
            entries.add(addr_text)
    return frozenset(entries)


# Whether the last real read of this host's interfaces failed, rather than finding no
# tunnel. The two are the same empty set, and a caller about to REPLACE entries it applied
# earlier has to tell them apart: re-deriving from a failed read would unblock a tunnel
# that is still up, on exactly the rebuild a network change triggered.
#
# Module-level state rather than a second return value because tests substitute
# :func:`vpn_interface_blocklist_entries` and :func:`allowed_interfaces` wholesale; a
# substitution never touches this, so it stays False and a substituted set reads as a
# successful one, which is what those tests mean. Mirrors the enumeration_failed
# out-parameter on the C++ side -- and, like an out-parameter, it is PER THREAD: two
# participants being created concurrently each read the host for themselves, and a
# process-wide flag would let one thread's successful read clear another thread's failure
# between that thread's read and its check, making it conclude the host simply has no
# tunnel and skip telling change detection that its transports may bind one.
_read_failures = threading.local()


# Per-thread cache behind :func:`vpn_interface_blocklist_entries`, active only inside a
# :func:`scoped_vpn_blocklist_cache` block. One network event rebuilds every
# recovery-enabled participant in the process, and each rebuild otherwise re-runs a full
# getifaddrs walk plus a full netlink RTM_GETLINK dump to answer the same question about
# the same host at the same instant; this collapses that to one reading, which also means
# every participant rebuilt for one event is configured from the same view of the host.
#
# Thread-local on purpose, exactly as the C++ scoped_vpn_blocklist_cache is: a participant
# being constructed on another thread is not part of this event and must read the host for
# itself. Nesting is counted so an inner scope does not release an outer one's cache.
_blocklist_cache = threading.local()


@contextlib.contextmanager
def scoped_vpn_blocklist_cache() -> "Iterator[None]":
    """Collapse the host enumeration behind :func:`vpn_interface_blocklist_entries` to at
    most one for the duration of the block, on this thread only.

    Mirrors ``provizio::dds::detail::scoped_vpn_blocklist_cache``."""
    depth = getattr(_blocklist_cache, "depth", 0)
    _blocklist_cache.depth = depth + 1
    try:
        yield
    finally:
        _blocklist_cache.depth -= 1
        if _blocklist_cache.depth == 0:
            _blocklist_cache.entries = None


def blocklist_read_failed() -> bool:
    """Whether the last :func:`vpn_interface_blocklist_entries` call on this thread failed
    to read the host, as opposed to finding no VPN interface."""
    return bool(getattr(_read_failures, "blocklist", False))


def allowed_interfaces_read_failed() -> bool:
    """Whether the last :func:`allowed_interfaces` call on this thread failed to read the
    host, as opposed to finding nothing left once the tunnels are excluded. Per thread,
    like :func:`blocklist_read_failed` and for the same reason."""
    return bool(getattr(_read_failures, "allowed", False))


def vpn_interface_blocklist_entries() -> "FrozenSet[str]":
    """Everything the DDS transports should be handed as their interface blocklist
    for the host's excluded VPN interfaces: each one's device name AND each of its
    addresses.

    Both forms, because Fast-DDS matches a blocklist entry against either the device
    name (``IPFinder::info_IP::dev`` — ``ifa_name`` on POSIX, ``AdapterName`` on
    Windows) or the address. The name is the stable identity; the address form alone
    would carry a hazard the name does not, since an address a tunnel has released
    can later be handed to a real interface. Empty when the host has no VPN interface
    up, or when PROVIZIO_DDS_ALLOW_VPN_INTERFACES allows them all. Mirrors
    ``detail::vpn_interface_blocklist_entries`` in the C++ library.

    Unreadable interfaces yield no entries rather than an error: unlike the change
    detection — where the walk raises and :func:`_try_capture_address_snapshot` translates
    it — here the answer is only ever "which tunnels to keep out", and raising would take a
    failed syscall all the way out of make_domain_participant.

    Yielding nothing is not the same as deciding to exclude nothing, though, and an empty
    reading means opposite things here too: :func:`blocklist_read_failed` is what tells the
    two apart, and the participant reads it to keep whatever exclusion it already had and
    to tell change detection that its transports may be binding a tunnel after all. The C++
    side draws the same line, reporting the failure through the ``enumeration_failed``
    out-parameter of ``detail::vpn_interface_blocklist_entries`` rather than by throwing."""
    if getattr(_blocklist_cache, "depth", 0) != 0:
        cached = getattr(_blocklist_cache, "entries", None)
        if cached is not None:
            return cached
    try:
        entries = (
            _vpn_blocklist_entries_windows()
            if sys.platform == "win32"
            else _vpn_blocklist_entries_posix()
        )
        _read_failures.blocklist = False
        if getattr(_blocklist_cache, "depth", 0) != 0:
            # Successful readings only. Caching a failure would hand every participant
            # rebuilt for this event the same wrong answer -- "this host has no tunnels" --
            # where reading again might well succeed.
            _blocklist_cache.entries = entries
        return entries
    except Exception:
        # Not logged here, for three reasons the C++ enumeration shares (it never logs): the
        # participant already reports a failed read, once per participant rather than on
        # every rebuild; a second line would duplicate it; and this runs under both lifecycle
        # locks, where the caller's log callback may not be invoked (see set_log_callback).
        _read_failures.blocklist = True
        return frozenset()


def _is_loopback_address(address: str) -> bool:
    """Whether an address text names the loopback interface: 127.0.0.0/8 or ``::1``."""
    return address.startswith("127.") or address == "::1"


def allowed_interfaces(blocked: "Collection[str]") -> "List[Tuple[str, bool]]":
    """The interfaces Fast-DDS will let the transports use once ``blocked`` is excluded,
    as ``(address, is_loopback)`` pairs.

    Loopback is INCLUDED, and that is the whole point: any non-empty interface list puts
    UDPv4 into whitelist mode, where every interface that is not blocked gets an output
    socket of its own -- loopback among them, since nothing blocks it. That socket can
    reach no other host, so unless it is netmask-filtered every unicast datagram costs it
    a failed sendto and a Fast-DDS warning. IPv4 only: a UDPv4 transport can neither bind
    an IPv6-only interface nor own a sender socket for it.

    Unreadable interfaces answer an empty list AND set :func:`allowed_interfaces_read_failed`,
    which the caller must check before writing anything: a blocklist without an allowlist
    puts UDPv4 into whitelist mode with none of the per-interface netmask filters the
    allowlist carries, so every remaining interface sends its own copy of every unicast
    datagram -- the duplication this feature exists to remove, moved from the tunnel onto
    the LAN -- and an allowlist matching nothing is no better, since Fast-DDS fills an empty
    whitelist with a sentinel that allows nothing at all. The caller therefore leaves the
    transports alone, as the C++ ``refresh_vpn_interface_blocklist`` does when
    ``vpn_allowed_interfaces`` reports failure. Mirrors ``vpn_allowed_interfaces`` in
    src/vpn_interfaces.cpp."""
    blocked_set = frozenset(blocked)
    found: "List[Tuple[str, bool]]" = []
    seen: set = set()
    try:
        if sys.platform == "win32":
            walk = (
                (name, addr)
                for name, _friendly, _description, _if_type, addr, _prefix in (
                    _iter_windows_adapter_addresses(include_loopback=True)
                )
            )
        else:
            walk = (
                (name, addr)
                for name, _kind, addr, _prefix in _iter_posix_interface_addresses(
                    include_loopback=True
                )
            )
        for name, address in walk:
            if ":" in address:
                continue  # IPv6: no UDPv4 sender socket can exist for it.
            if name in blocked_set or address in blocked_set:
                continue
            if address in seen:
                continue
            seen.add(address)
            found.append((address, _is_loopback_address(address)))
    except Exception:
        _read_failures.allowed = True
        return []
    _read_failures.allowed = False
    return found


# ---------------------------------------------------------------------------
# Listener drain — detach + wait for in-flight callbacks
# ---------------------------------------------------------------------------


# How long a drain waits before reporting that it is still waiting. Purely
# diagnostic: it does not bound the total wait (see ListenerDrain.detach_and_drain).
_DRAIN_STALL_WARNING_SEC = 5.0


class ListenerDrain:
    """Detach + drain primitive for Fast-DDS listener callbacks, mirroring
    the C++ ``provizio::dds::detail::listener_drain``.

    A subscriber / publisher listener wraps every callback body in a
    ``with drain.scope() as ok:`` block. The reset path calls
    :meth:`detach_and_drain` BEFORE acquiring the participant's lifecycle
    lock; this lets any in-flight callback that re-enters provizio APIs
    complete (those re-entries acquire the lifecycle lock shared, which is
    free at this point), and then the drain waits for the in-flight count
    to fall to zero.
    """

    def __init__(self, stall_warning_period: float = _DRAIN_STALL_WARNING_SEC) -> None:
        """``stall_warning_period`` is how long the drain waits before reporting that it
        is still waiting, and how long between repeats of that report. Purely diagnostic:
        it never bounds the total wait (see :meth:`detach_and_drain`). Every shipped
        endpoint takes the default; the argument exists so a test can observe the
        reporting without spending the default period per line, exactly as the C++
        listener_drain constructor's does."""
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._in_flight = 0
        self._detached = False
        self._stall_warning_period = stall_warning_period

    def enter(self) -> bool:
        """Record entry to a callback. Returns ``True`` if the callback body
        should proceed, ``False`` if a reset is in progress. Either way the
        counter is incremented; every :meth:`enter` MUST be paired with
        :meth:`leave`."""
        with self._lock:
            self._in_flight += 1
            return not self._detached

    def leave(self) -> None:
        with self._lock:
            self._in_flight -= 1
            if self._in_flight == 0:
                self._cv.notify_all()

    def scope(self):
        """Context-manager form of enter/leave. Yields a single boolean —
        whether the callback body should run."""
        drain = self

        class _Scope:
            def __enter__(self_inner):
                self_inner._run = drain.enter()
                return self_inner._run

            def __exit__(self_inner, exc_type, exc, tb):
                drain.leave()
                return False

        return _Scope()

    def detach_and_drain(self) -> None:
        """Block until every in-flight callback has returned. MUST NOT be
        called while holding the participant's lifecycle lock — see the
        module docstring for the rationale.

        Waits in bounded slices with an unbounded total: the contract is still
        "until every in-flight callback has returned", but a callback that never
        returns (one blocked on a lock the caller holds, or one creating an
        endpoint during a reset — both forbidden) would otherwise hang here
        forever with nothing in the log to say why. The slice only decides how
        soon that is reported; it never ends the wait early, because proceeding
        while a callback is still running would tear the Fast-DDS entity down
        underneath it. Mirrors detach_and_drain in
        include/provizio/dds/detail/listener_drain.h."""
        with self._cv:
            # Stored once, before the wait, exactly as the C++ side does: repeating it in
            # the loop would silently re-detach if reattach() ever raced this drain.
            self._detached = True

        # Reported once when the stall starts and again on every slice after it, because one
        # line cannot say whether the stall is over: the wait is unbounded and the caller
        # holds the registration lock throughout, so an operator who sees a single warning
        # cannot tell a stall that cleared from one still going hours later. The slice is
        # long enough that a wedged process produces a line a few times a minute.
        stalled_slices = 0
        # The real elapsed time, because the completion line below must report what the reset
        # actually cost. Deriving it from the slice count would undercount by however far
        # into the final slice the drain finished -- a stall cleared 2.5 s into its second
        # slice would be reported as 5 s rather than 7.5 s.
        stall_started = time.monotonic()
        # An absolute deadline, not a fresh slice per wakeup, so the first report lands one
        # period after the drain began however many times the condition is notified —
        # matching wait_for(lock, period, pred) on the C++ side. Re-armed after each report,
        # which is what turns the one line into a heartbeat.
        warn_at = time.monotonic() + self._stall_warning_period
        while True:
            drained = False
            in_flight = 0
            with self._cv:
                if self._in_flight == 0:
                    drained = True
                else:
                    remaining = warn_at - time.monotonic()
                    self._cv.wait(
                        timeout=remaining if remaining > 0 else self._stall_warning_period
                    )
                    in_flight = self._in_flight
                    drained = in_flight == 0

            # Every _emit_log below runs with the condition released: it calls the user's
            # log callback, which is documented as free to re-enter provizio APIs, and
            # holding this primitive's condition across that would silently add it to that
            # contract.
            if drained:
                if stalled_slices:
                    # Only where a stall was reported: the log needs an end as well as a
                    # beginning, or the last warning stands as the final word on a reset
                    # that in fact completed.
                    _emit_log(
                        LogLevel.WARNING,
                        f"listener drain completed after "
                        f"{time.monotonic() - stall_started:.1f} s; the reset is no "
                        f"longer blocked.",
                    )
                return

            if time.monotonic() < warn_at:
                continue

            stalled_slices += 1
            warn_at = time.monotonic() + self._stall_warning_period
            _emit_log(
                LogLevel.WARNING,
                f"listener drain has been waiting for {in_flight} in-flight "
                f"callback(s) for over {self._stall_warning_period * stalled_slices:g} s. A "
                f"callback that never returns blocks the network-recovery reset (and "
                f"endpoint teardown) indefinitely: callbacks must not block and must not "
                f"create publishers / subscribers / services.",
            )

    def reattach(self) -> None:
        with self._lock:
            self._detached = False

    @property
    def is_detached(self) -> bool:
        with self._lock:
            return self._detached


class DeferredReaper:
    """Runs endpoint-teardown callables on a dedicated thread.

    Fast-DDS tears an endpoint down by joining its listener / event thread, so
    ``delete_data{writer,reader}`` must never run on a Fast-DDS callback
    thread — it would self-join and deadlock. When an endpoint's ``__del__``
    fires on such a thread, it submits its teardown here instead, to run on
    this dedicated thread. The C++ side does not need this: its listeners hold
    a non-owning raw reference to the handle, so a C++ callback thread can
    never be the last owner — something Python cannot express for a value
    handed to a user callback.
    """

    def __init__(self) -> None:
        self._queue: "queue.SimpleQueue[Callable[[], None]]" = queue.SimpleQueue()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def _ensure_thread(self) -> None:
        with self._lock:
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._run, name="provizio_dds_reaper", daemon=True
                )
                self._thread.start()

    def _run(self) -> None:
        while True:
            task = self._queue.get()
            try:
                task()
            except Exception:
                # A teardown task must never crash the reaper thread; the next
                # submitted teardown still needs to run.
                pass
            del task

    def submit(self, task: "Callable[[], None]") -> None:
        """Queue ``task`` to run on the reaper thread (started lazily)."""
        self._ensure_thread()
        self._queue.put(task)


# Process-wide reaper. Its worker thread starts on the first submit().
_reaper = DeferredReaper()


def submit_off_thread(task: "Callable[[], None]") -> None:
    """Run ``task`` on the dedicated reaper thread — see :class:`DeferredReaper`."""

    _reaper.submit(task)


# ---------------------------------------------------------------------------
# Network monitor — event-driven netlink on Linux, polling elsewhere/fallback
# ---------------------------------------------------------------------------


# The monitor reports each settled burst as (old_snapshot, new_snapshot,
# burst_start_snapshot). burst_start is the snapshot captured at the FIRST event
# of the burst (event-driven backends only); the coordinator uses it to detect a
# transient flap whose end-state equals old. Polling backends pass burst_start as
# None (they cannot observe a sub-interval transient).
OnNetworkEvent = Callable[[AddressSnapshot, AddressSnapshot, Optional[AddressSnapshot]], None]

# Invoked by the monitor on every safety-net tick, before the snapshot re-check, so
# the coordinator can retry participants a previous reset left torn down.
OnSafetyNetTick = Callable[[], None]

# Coalescing parameters, mirroring the C++ network_recovery_coordinator
# quiet_period / max_debounce. The quiet period is the env-configurable
# detection interval (also the polling cadence on non-event-driven backends).
_MAX_DEBOUNCE_SEC = 60.0

# How often an event-driven monitor re-verifies the snapshot directly when no burst
# is pending, mirroring the C++ default_safety_net_period. This is the backstop for
# everything the event channel cannot report: a dropped netlink datagram (ENOBUFS),
# a socket that died on an unrecoverable error (the tick reopens it), a change on a
# channel we do not subscribe to, and a participant left unrecovered by a rebuild
# that failed. Without it, any one of those leaves the process unable to
# communicate for the rest of its life.
#
# Set PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC to 0 to disable the periodic
# check — which also disables the socket-revival path that rides on it.
_SAFETY_NET_ENV = "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC"
_DEFAULT_SAFETY_NET_SEC = 30.0


# Upper bound on both the safety-net period and the poll interval. `select.select`
# raises OverflowError for a timeout that does not fit the platform's time type, and
# `inf` / `nan` slip through a naive `value < 0` check — `nan` compares False against
# everything, so it would silently disable the safety net, and `inf` would kill the
# monitor thread outright. A day is far past any sane cadence.
_MAX_INTERVAL_SEC = 24 * 60 * 60.0

# Lower bound on the poll interval: each poll is a full getifaddrs walk (plus a retry
# pass), so a denormal like 1e-9 — strictly positive, hence past the zero check — would
# busy-spin the monitor thread. 100 ms is far below any useful cadence already.
_MIN_INTERVAL_SEC = 0.1


def _sanitise_env_value_for_log(raw: str) -> str:
    """Cap and de-control-character an env value before quoting it in a warning, so a
    pathological value cannot flood the log or forge log lines downstream."""
    capped = raw[:32]
    cleaned = "".join("?" if ord(c) < 0x20 or ord(c) == 0x7F else c for c in capped)
    return cleaned + "..." if len(raw) > 32 else cleaned


def _resolve_positive_interval(env_name: str, default: float) -> float:
    """Parser for the poll-interval cadence env variable. Rejects non-numbers,
    non-positives, ``nan`` and ``inf``, clamps to [:data:`_MIN_INTERVAL_SEC`,
    :data:`_MAX_INTERVAL_SEC`]; every rejection falls back to @p default with a single
    warning. (The safety-net period is NOT parsed here — it allows 0 and uses the
    stricter cross-language integer grammar, see :func:`_resolve_safety_net_period`.)"""

    raw = os.environ.get(env_name)
    if not raw:
        return default

    quoted = _sanitise_env_value_for_log(raw)

    def reject(reason: str) -> float:
        _emit_log(LogLevel.WARNING, f"{env_name}={quoted} {reason}; using the default {default}s")
        return default

    try:
        value = float(raw)
    except (ValueError, TypeError):
        return reject("is not a number of seconds")
    # isfinite rejects both nan and inf. Checked before the comparisons below, because
    # every comparison against nan is False and would fall through as "valid".
    if not math.isfinite(value):
        return reject("is not a finite number of seconds")
    if value <= 0:
        return reject("is not a positive number of seconds")
    if value < _MIN_INTERVAL_SEC:
        _emit_log(
            LogLevel.WARNING,
            f"{env_name}={quoted} is below the minimum of {_MIN_INTERVAL_SEC}s; clamping to it",
        )
        return _MIN_INTERVAL_SEC
    if value > _MAX_INTERVAL_SEC:
        _emit_log(
            LogLevel.WARNING,
            f"{env_name}={quoted} exceeds the maximum of {_MAX_INTERVAL_SEC}s; clamping to it",
        )
        return _MAX_INTERVAL_SEC
    return value


def _resolve_safety_net_period() -> float:
    """Read the safety-net cadence from
    ``PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC``; 0 disables it.

    Parsed as a whole number of seconds with the same grammar as the C++ side
    (``resolve_safety_net_period`` in src/network_recovery_coordinator.cpp), so one
    deployment value cannot behave differently per language: fractional, unparseable,
    trailing-character or negative values are logged once and ignored, oversized
    values are clamped to a day."""

    raw = os.environ.get(_SAFETY_NET_ENV)
    if not raw:
        return _DEFAULT_SAFETY_NET_SEC

    quoted = _sanitise_env_value_for_log(raw)

    def reject(reason: str) -> float:
        _emit_log(
            LogLevel.WARNING,
            f"{_SAFETY_NET_ENV}={quoted} {reason}; using the default of {_DEFAULT_SAFETY_NET_SEC}s",
        )
        return _DEFAULT_SAFETY_NET_SEC

    # Integer full-consume, mirroring the C++ stoll + trailing-character check: "30s",
    # "0x10" and "0.5" are all rejected rather than silently misread. re.ASCII keeps
    # \s to ASCII whitespace — C++ isspace on raw bytes rejects e.g. a leading NBSP,
    # and Python's int() would otherwise accept it.
    if not re.fullmatch(r"\s*[+-]?[0-9]+", raw, re.ASCII):
        return reject("is not an integer number of seconds")
    try:
        value = int(raw)
    except ValueError:
        # int() refuses digit strings beyond its conversion-length limit (4300 digits by
        # default on Python 3.11+); such input is invalid here anyway, never an error.
        return reject("is not an integer number of seconds")
    if not -(2**63) <= value <= 2**63 - 1:
        # C++ stoll throws out_of_range past int64 and the value is rejected; reject
        # here too rather than clamping, so both languages fall back identically.
        return reject("is not an integer number of seconds")
    if value < 0:
        return reject("is negative")
    if value > _MAX_INTERVAL_SEC:
        _emit_log(
            LogLevel.WARNING,
            f"{_SAFETY_NET_ENV}={quoted} exceeds the maximum of {_MAX_INTERVAL_SEC}s; "
            f"clamping to it (use 0 to disable the periodic check entirely)",
        )
        return _MAX_INTERVAL_SEC
    return float(value)


def _log_missing_baseline(snapshot: AddressSnapshot) -> None:
    """Report that the first readable interface list is being treated as all-new.

    Reached only when the read at construction failed, so there is no baseline -- and, with
    it, no knowing what the participants actually bound to. An interface may have come up
    while the list was unreadable, so every address now visible counts as new: that rebuilds
    once, where quietly adopting them would lose the rebuild permanently (an adopted address
    is no longer a gain against any later snapshot). Mirrors
    network_recovery_coordinator::adopt_first_readable_snapshot.

    One deliberate difference from that mirror, and the only one: the C++ side counts the
    empty-first-readable-list case as a skipped reset, because it reaches its decision
    function to do so. Here the caller's own "changed?" comparison absorbs the case before
    any handler runs, so ``skipped_reset_count`` does not move. Nothing but a test can
    observe it -- no rebuild happens on either side -- which is why the counters are left
    as they are rather than routing an empty set through the handler just to increment one.
    """
    if not snapshot:
        # Nothing to bind, so nothing to rebuild for; the comparison in the caller reduces
        # to "unchanged" on its own.
        return
    _emit_log(
        LogLevel.INFO,
        "network monitor: no interface baseline to compare against (the list could not be "
        f"read at startup), so all {len(snapshot)} interface address(es) now visible are "
        "treated as new",
    )


class _PollingNetworkMonitor:
    """Fallback monitor: polls :func:`_capture_address_snapshot` on an interval
    and reports a burst whenever the result changes from the last observed value.

    Being a poller, it is its own safety net — every interval is a direct re-check —
    so it needs no separate periodic tick beyond calling ``on_safety_net_tick`` for
    the coordinator's failed-rebuild retries.

    LIMITATION: a sub-interval transient (an address removed and re-added between
    two polls) is invisible — both polls observe the same set, so no burst is
    reported. The Linux backend (:class:`_NetlinkNetworkMonitor`) is event-driven
    and does not have this blind spot. Used on macOS / Windows, or on Linux only
    if the netlink socket cannot be opened.
    """

    def __init__(
        self,
        on_event: OnNetworkEvent,
        poll_interval_sec: float,
        on_safety_net_tick: Optional[OnSafetyNetTick] = None,
    ):
        self._on_event = on_event
        self._on_safety_net_tick = on_safety_net_tick
        self._poll_interval_sec = poll_interval_sec
        self._stop_event = threading.Event()
        # Initial snapshot is captured BEFORE the worker thread starts so
        # the first observation has a baseline to diff against. An unreadable interface list
        # leaves NO baseline (None) rather than failing construction — and, importantly, rather
        # than an empty one: the empty set is a real state some hosts genuinely have (a
        # container whose only device is a filtered-out veth), so seeding it would make the
        # first readable snapshot look like every address arriving at once and rebuild every
        # participant for nothing. Not free either, despite happening early: the rebuild would
        # land on the next successful read, seconds later, by which time endpoints are carrying
        # traffic. The first readable snapshot is adopted as the baseline instead.
        self._last_snapshot: Optional[AddressSnapshot] = _try_capture_address_snapshot()
        self._thread = threading.Thread(target=self._run, name="provizio_dds.network_monitor", daemon=True)
        self._thread.start()

    def initial_snapshot(self) -> Optional[AddressSnapshot]:
        """The baseline, or ``None`` while the interface list has never been readable."""
        return self._last_snapshot

    def is_alive(self) -> bool:
        """A poller has no kernel channel that can die, so it is alive while its
        thread runs. Mirrors :meth:`_NetlinkNetworkMonitor.is_alive`."""
        return self._thread.is_alive()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join()

    def _run(self) -> None:
        while not self._stop_event.wait(self._poll_interval_sec):
            self._safety_net_check()

    def _safety_net_check(self) -> None:
        """One poll: let the coordinator retry failed rebuilds, then re-verify the
        snapshot. For a poller this *is* the safety net — there is no separate event
        channel — so it is also what :meth:`run_safety_net_tick_for_test` runs."""

        if self._on_safety_net_tick is not None:
            try:
                self._on_safety_net_tick()
            except Exception as ex:
                _emit_log(LogLevel.ERROR, f"network monitor: safety-net tick raised ({ex})")
        new_snapshot = _try_capture_address_snapshot()
        if new_snapshot is None:
            return  # Unreadable interfaces are not a change — see the helper.
        if self._last_snapshot is None:
            # Stand in the empty set for THIS comparison, so everything visible reads as new
            # and rebuilds. Deliberately not what __init__ stores -- see there -- because the
            # difference between "unknown" and "no addresses" is what makes this rebuild, while
            # a genuinely empty list below still reduces to "unchanged" and rebuilds nothing.
            _log_missing_baseline(new_snapshot)
            self._last_snapshot = frozenset()
        if new_snapshot != self._last_snapshot:
            old = self._last_snapshot
            self._last_snapshot = new_snapshot
            try:
                self._on_event(old, new_snapshot, None)  # no burst-start: poll can't see transients
            except Exception as ex:
                _emit_log(LogLevel.ERROR, f"network monitor: on_event handler raised ({ex})")

    def run_safety_net_tick_for_test(self) -> None:
        """Test-only: run one poll synchronously on the calling thread, so the same
        tests cover this backend as cover the event-driven one. Present here as well as
        on :class:`_NetlinkNetworkMonitor` so the coordinator's hook behaves identically
        on macOS / Windows, where polling is the only backend."""
        self._safety_net_check()

    def kill_for_test(self) -> bool:
        """No kernel channel to kill — a poller re-reads the interface list every
        interval and so has nothing that can die short of its thread. Reported as False
        so the revival test skips rather than asserting a property this backend cannot
        have."""
        return False


class _NetlinkNetworkMonitor:
    """Linux event-driven monitor. Subscribes an ``AF_NETLINK`` / ``NETLINK_ROUTE``
    socket to ``RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR | RTMGRP_LINK`` and coalesces
    the resulting bursts, mirroring the C++ network_monitor + coordinator coalescer.

    The link group matters as much as the address groups: the snapshot only admits
    operationally-up interfaces (``IFF_RUNNING``, as Fast-DDS' ``IPFinder`` does), so
    a switch being powered on or a cable replugged changes the snapshot while
    emitting only ``RTM_NEWLINK``. Subscribing to addresses alone would leave this
    loop asleep through exactly the transition that needs a rebuild. The extra
    wake-ups a flapping link causes are harmless — the coordinator's snapshot diff
    drops every burst that nets out to no change.

    Crucially it captures a snapshot at the FIRST event of each burst — before a
    quick flap can revert — and reports it as ``burst_start``. That is what lets
    the coordinator catch a transient (an address removed and re-added within the
    quiet period): the end-state matches the last known set, but the Fast-DDS
    sockets bound to that address were torn down while it was gone and must be
    rebuilt. A pure end-state diff (what polling and the old code did) skips it.

    Like the C++ side, the netlink payload is NOT parsed — any event is just a
    trigger; the actual decision is made from the captured snapshots, so the
    existing filters (loopback / link-local / container kinds) apply for free and
    container/veth churn cannot cause spurious resets.

    When no burst is pending, the loop also wakes every ``safety_net_sec`` to
    re-verify the snapshot directly and to reopen a socket that died — see
    ``_SAFETY_NET_ENV``.
    """

    _RTMGRP_LINK = 0x1
    _RTMGRP_IPV4_IFADDR = 0x10
    _RTMGRP_IPV6_IFADDR = 0x100
    _GROUPS = _RTMGRP_LINK | _RTMGRP_IPV4_IFADDR | _RTMGRP_IPV6_IFADDR

    def __init__(
        self,
        on_event: OnNetworkEvent,
        quiet_period_sec: float,
        on_safety_net_tick: Optional[OnSafetyNetTick] = None,
        safety_net_sec: Optional[float] = None,
    ):
        self._on_event = on_event
        self._on_safety_net_tick = on_safety_net_tick
        self._quiet = quiet_period_sec
        self._safety_net = _resolve_safety_net_period() if safety_net_sec is None else safety_net_sec
        # Open and bind the socket BEFORE capturing the baseline snapshot. Any event
        # that fires from here on is queued on the (bound) socket and delivered to the
        # loop, which then captures a fresh snapshot and diffs it against the baseline.
        # In the other order, an event landing between the capture and the bind would
        # be lost AND already reflected in the baseline — so the change would never be
        # noticed, which is precisely the boot-time race this monitor exists for.
        # Mirrors the ordering the C++ coordinator's register_participant documents.
        #
        # Subscribing to the multicast groups can raise OSError; the caller
        # (_make_network_monitor) falls back to polling on failure.
        self._sock: Optional[socket.socket] = socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, _NETLINK_ROUTE)
        self._stop_r = self._stop_w = -1
        try:
            self._sock.bind((0, self._GROUPS))
            # An unreadable interface list leaves NO baseline rather than failing construction,
            # and specifically not an empty one — see the matching comment in the polling monitor.
            self._last_known: Optional[AddressSnapshot] = _try_capture_address_snapshot()
            # Self-pipe to wake the select() loop for a clean shutdown (mirrors the
            # C++ eventfd). Closing the socket from another thread is not a reliable
            # way to unblock a blocked recv/select on every kernel.
            self._stop_r, self._stop_w = os.pipe()
            self._thread = threading.Thread(target=self._run, name="provizio_dds.network_monitor", daemon=True)
            self._thread.start()
        except BaseException:
            # Construction failed partway: close whatever was opened so a half-built monitor doesn't leak the
            # socket / pipe fds. The object never finishes constructing, so stop() is never reachable to do it.
            self._sock.close()
            for fd in (self._stop_r, self._stop_w):
                if fd >= 0:
                    os.close(fd)
            raise

    def initial_snapshot(self) -> Optional[AddressSnapshot]:
        """The baseline, or ``None`` while the interface list has never been readable."""
        return self._last_known

    def is_alive(self) -> bool:
        """Whether the kernel channel is currently being watched. False between a
        socket dying on an unrecoverable error and the next safety-net tick
        reopening it (and permanently, if reopening keeps failing) — mirrors the C++
        ``network_monitor::is_alive``."""
        return self._sock is not None and self._thread.is_alive()

    def _reopen_socket(self) -> None:
        """Re-establish the netlink channel after an unrecoverable error. Leaves
        ``_sock`` as None if it fails, so the next tick tries again."""
        try:
            sock = socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, _NETLINK_ROUTE)
        except OSError as ex:
            _emit_log(LogLevel.ERROR, f"network monitor: could not recreate the netlink socket ({ex})")
            return
        try:
            sock.bind((0, self._GROUPS))
        except OSError as ex:
            sock.close()
            _emit_log(LogLevel.ERROR, f"network monitor: could not rebind the netlink socket ({ex})")
            return
        self._sock = sock
        # Silent: died and was repaired here, with the missed events covered by this same check.

    def _safety_net_check(self) -> None:
        """One periodic tick: revive the socket, let the coordinator retry failed
        rebuilds, then re-verify the snapshot directly to catch anything no event
        reported."""

        if self._sock is None:
            self._reopen_socket()

        if self._on_safety_net_tick is not None:
            try:
                self._on_safety_net_tick()
            except Exception as ex:
                _emit_log(LogLevel.ERROR, f"network monitor: safety-net tick raised ({ex})")

        new_snapshot = _try_capture_address_snapshot()
        if new_snapshot is None:
            return  # Unreadable interfaces are not a change — see the helper.
        if self._last_known is None:
            # See the polling monitor: the empty set stands in for "unknown" here so the first
            # readable list rebuilds instead of being silently absorbed.
            _log_missing_baseline(new_snapshot)
            self._last_known = frozenset()
        if new_snapshot == self._last_known:
            # Deliberately silent: this runs on a timer for the life of the process.
            return

        _emit_log(
            LogLevel.INFO,
            "network monitor: change found by the periodic safety-net check -- no kernel event "
            f"reported it ({len(self._last_known)} -> {len(new_snapshot)} interface address(es))",
        )
        try:
            self._on_event(self._last_known, new_snapshot, None)
        except Exception as ex:
            _emit_log(LogLevel.ERROR, f"network monitor: on_event handler raised ({ex})")
        self._last_known = new_snapshot

    def _run(self) -> None:
        pending = False
        # None means "no burst-start snapshot", which is exactly what a missing baseline
        # amounts to as well, so the two collapse into one value.
        burst_start: Optional[AddressSnapshot] = None
        first_event_mono = 0.0
        last_event_mono = 0.0

        while True:
            timeout: Optional[float] = None
            if pending:
                now = time.monotonic()
                wake_at = min(last_event_mono + self._quiet, first_event_mono + _MAX_DEBOUNCE_SEC)
                timeout = max(0.0, wake_at - now)
            elif self._safety_net > 0:
                timeout = self._safety_net

            # Read the socket ONCE per iteration and use that value throughout: it can be
            # replaced under us (kill_for_test from another thread, or a revival from a
            # tick), and the error paths below must only ever discard the socket they
            # actually failed on — never a freshly reopened one.
            sock = self._sock
            watched = [self._stop_r] if sock is None else [sock, self._stop_r]
            try:
                readable, _, _ = select.select(watched, [], [], timeout)
            except OverflowError:
                # A timeout the platform's time type cannot represent. The resolvers
                # clamp their inputs, so this is belt-and-braces — but it must not kill
                # the thread, which is what an uncaught exception here used to do.
                # Silent: internal clamp, no effect the caller can observe.
                self._safety_net = min(self._safety_net, _DEFAULT_SAFETY_NET_SEC)
                continue
            except ValueError:
                # select() raises ValueError (not OSError) for a socket whose fd is
                # already -1 — reachable when another thread closes it between the read
                # of self._sock above and this call. Not terminal: drop that socket and
                # let the periodic check reopen it. An uncaught ValueError here used to
                # kill the worker thread outright, which is exactly the permanent-outage
                # failure mode this feature exists to prevent.
                # Silent: reopened below; self-healed.
                self._close_socket(sock)
                continue
            except OSError as ex:
                # EINTR: select was interrupted by a signal — retry rather than tear
                # down the monitor.
                if ex.errno == errno.EINTR:
                    continue
                # EBADF means one of the fds we are watching is gone. If it is the stop
                # pipe (stop() closes it after a join that can time out while a slow
                # snapshot or reset hook is still running) there is nothing left to watch
                # and retrying would spin, one ERROR log per iteration. Terminal.
                if ex.errno == errno.EBADF:
                    _emit_log(LogLevel.ERROR, f"network monitor: select() on a closed descriptor ({ex}); stopping")
                    self._close_socket()
                    break
                # Anything else is terminal for this socket, but not for recovery:
                # drop the socket and let the periodic check keep working (and try to
                # reopen it). Killing the monitor here is what used to leave a process
                # without auto-recovery for the rest of its life.
                _emit_log(
                    LogLevel.ERROR,
                    f"network monitor: select() failed ({ex}); falling back to the periodic "
                    f"safety-net check and retrying the netlink channel",
                )
                self._close_socket(sock)
                continue

            if self._stop_r in readable:
                break

            if sock is not None and sock in readable:
                try:
                    sock.recv(65536)  # content ignored — any address event is a trigger
                except OSError as ex:
                    # Mirror the C++ Linux monitor's recv classification. EINTR/EAGAIN
                    # are spurious — retry. ENOBUFS means the kernel's netlink multicast
                    # buffer overflowed because address events arrived faster than we
                    # drained them — precisely a rapid flap, the scenario auto-recovery
                    # exists to handle. The kernel drops the unread events and subsequent
                    # recvs succeed, so log and keep monitoring (the next snapshot capture
                    # reflects whatever state the kernel settles into); breaking here would
                    # silently kill the monitor under the very load it must survive. Only
                    # genuinely terminal errors stop it.
                    if ex.errno in (errno.EINTR, errno.EAGAIN, errno.EWOULDBLOCK):
                        continue
                    if ex.errno == errno.ENOBUFS:
                        # Silent: the periodic capture picks up whatever state the kernel ends up in.
                        continue
                    # Terminal for this socket only — see the select() branch above.
                    _emit_log(
                        LogLevel.ERROR,
                        f"network monitor: recv() failed ({ex}); falling back to the periodic "
                        f"safety-net check and retrying the netlink channel",
                    )
                    self._close_socket(sock)
                    continue
                now = time.monotonic()
                if not pending:
                    pending = True
                    first_event_mono = now
                    # Burst START snapshot — capture immediately, before a quick
                    # flap can revert. capture_address_snapshot() is slow; while it
                    # runs the kernel buffers further events for the next iteration.
                    # An unreadable list leaves the burst with the last known set as its
                    # start, which makes the transient test below a no-op rather than
                    # reporting every address as having returned during the burst. With no
                    # baseline yet that yields None, which the coordinator already reads as
                    # "no burst-start snapshot" — the same no-op by a different route.
                    captured_start = _try_capture_address_snapshot()
                    burst_start = captured_start if captured_start is not None else self._last_known
                last_event_mono = now
                continue

            # select() timed out. Either the burst has gone quiet (or hit
            # max_debounce), or there was no burst at all and it is time for a
            # safety-net tick.
            if not pending:
                self._safety_net_check()
                continue

            if (
                last_event_mono + self._quiet <= time.monotonic()
                or first_event_mono + _MAX_DEBOUNCE_SEC <= time.monotonic()
            ):
                end_snapshot = _try_capture_address_snapshot()
                if end_snapshot is None:
                    # Unreadable interfaces are not a change — see the helper. The burst is
                    # dropped; the safety-net tick re-checks once a read succeeds.
                    pending = False
                    burst_start = None
                    continue
                if self._last_known is None:
                    # See the polling monitor: with no baseline the empty set stands in, so this
                    # burst reads as every visible address being new and rebuilds.
                    _log_missing_baseline(end_snapshot)
                    self._last_known = frozenset()
                try:
                    self._on_event(self._last_known, end_snapshot, burst_start)
                except Exception as ex:
                    _emit_log(LogLevel.ERROR, f"network monitor: on_event handler raised ({ex})")
                self._last_known = end_snapshot
                pending = False
                burst_start = None

    def _close_socket(self, expected: "Optional[socket.socket]" = None) -> None:
        """Drop the netlink socket, leaving the loop running on its periodic check.
        ``_safety_net_check`` reopens it on the next tick.

        @param expected When given, do nothing unless it is still the current socket.
        The worker's error paths pass the socket they actually failed on, so a socket
        that has meanwhile been reopened (by a tick, or after ``kill_for_test``) is
        never closed by a stale failure.
        """
        if expected is not None and self._sock is not expected:
            return
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def run_safety_net_tick_for_test(self) -> None:
        """Test-only: run one safety-net tick synchronously on the calling thread.
        Only safe while the monitor is idle — see the C++
        ``run_safety_net_tick_for_test``."""
        self._safety_net_check()

    def kill_for_test(self) -> bool:
        """Test-only: drop the netlink socket as if it had failed unrecoverably, so
        the revival path can be exercised without a real kernel fault. The loop keeps
        running; the next tick reopens the socket."""
        if self._sock is None:
            return False
        self._close_socket()
        return True

    def stop(self) -> None:
        try:
            os.write(self._stop_w, b"x")
        except OSError:
            pass
        self._thread.join(timeout=2.0)
        for closer in (self._close_socket, lambda: os.close(self._stop_r), lambda: os.close(self._stop_w)):
            try:
                closer()
            except OSError:
                pass


def _make_network_monitor(
    on_event: OnNetworkEvent,
    interval_sec: float,
    on_safety_net_tick: Optional[OnSafetyNetTick] = None,
):
    """Construct the platform-appropriate monitor: event-driven netlink on Linux,
    polling elsewhere. Falls back to polling on Linux if the netlink socket cannot
    be opened (e.g. a sandbox without ``AF_NETLINK``)."""
    if sys.platform.startswith("linux"):
        try:
            return _NetlinkNetworkMonitor(on_event, interval_sec, on_safety_net_tick)
        except OSError as ex:
            _emit_log(
                LogLevel.WARNING,
                f"network monitor: netlink unavailable ({ex}); falling back to polling "
                f"(sub-interval transient flaps may be missed)",
            )
    return _PollingNetworkMonitor(on_event, interval_sec, on_safety_net_tick)


# ---------------------------------------------------------------------------
# Coordinator — process-wide registry of recovery-enabled participants
# ---------------------------------------------------------------------------


# Default polling interval, in seconds. Mirrors the C++ quiet_period so the
# end-to-end recovery cadence is comparable.
_DEFAULT_POLL_INTERVAL_SEC = 3.0


def _resolve_poll_interval() -> float:
    """Read the per-process polling cadence from
    ``PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC``; fall back to the
    default. Unparseable, non-positive, non-finite or oversized values are logged once
    and ignored — ``nan`` in particular used to slip through the ``v <= 0`` check and
    make the polling backend spin on a zero-length wait."""

    return _resolve_positive_interval(_POLL_INTERVAL_ENV, _DEFAULT_POLL_INTERVAL_SEC)


class _NetworkRecoveryCoordinator:
    """Process-wide singleton, constructed lazily on the first
    ``register_participant`` call from a recovery-enabled participant.

    Failed-rebuild retries are bounded by ``_MAX_CONSECUTIVE_RETRY_PASSES``
    (mirroring the C++ ``max_consecutive_retry_passes``): after that many
    consecutive fruitless passes the coordinator gives up — with one error log —
    until the next real network change re-arms it.

    Each registered participant supplies a ``reset_hook`` bound method;
    on a confirmed network change the coordinator walks the live
    registry and invokes every hook IN-LINE on the polling monitor's
    worker thread (not on a per-hook dedicated thread — the hook is
    expected to be brief; any heavy lifting should be off-loaded by the
    hook itself). The hook is responsible for tearing down its Fast-DDS
    objects and rebuilding them — the coordinator does not see into the
    Python participant's internals.

    The hook is stored as a ``weakref.WeakMethod`` so the registry
    doesn't keep the participant (the method's ``__self__``) alive past
    its natural lifetime. When the participant is GC'd, the weak method
    resolves to ``None`` and the entry is dropped on the next registry
    walk.
    """

    # Mirrors the C++ max_consecutive_retry_passes (network_recovery_coordinator.h).
    _MAX_CONSECUTIVE_RETRY_PASSES = 5

    _instance_lock = threading.Lock()
    _instance: "Optional[_NetworkRecoveryCoordinator]" = None

    @classmethod
    def instance(cls) -> "_NetworkRecoveryCoordinator":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        self._registry_lock = threading.Lock()
        # WeakMethod entries — a weak reference to a BOUND METHOD that
        # resolves to None when the participant (the method's __self__)
        # is GC'd. Storing a strong reference to the bound method would
        # pin the participant via __self__ and defeat the weak-registry
        # design.
        self._registered: list[weakref.WeakMethod] = []
        # _PollingNetworkMonitor or _NetlinkNetworkMonitor (see _make_network_monitor).
        self._monitor: Optional[object] = None
        # Test observability counters — always present so behaviour stays
        # comparable to the C++ side. See network_recovery_coordinator.h.
        self.reset_count = 0
        self.skipped_reset_count = 0
        self._idle_lock = threading.Lock()
        self._idle_cv = threading.Condition(self._idle_lock)
        self._reset_in_progress = False
        # Failed-rebuild retry bound — mirrors the C++ coordinator's
        # consecutive_retry_passes / retry_exhaustion_reported (see
        # network_recovery_coordinator.h): a participant that can never be rebuilt
        # must not churn its healthy siblings once per safety-net tick forever.
        # Touched only on the monitor's worker thread (ticks and network events are
        # both delivered there), so no extra lock is needed.
        self._consecutive_retry_passes = 0
        self._retry_exhaustion_reported = False

    def register_participant(self, participant: object,
                              reset_hook: Callable[[AddressSnapshot, AddressSnapshot], None]) -> None:
        """Add a participant to the recovery registry.

        ``reset_hook`` must be a BOUND METHOD on the participant (e.g.
        ``participant._reset_hook``). It is invoked on every confirmed
        network change as ``reset_hook(old_snapshot, new_snapshot)`` and
        must tear down the participant's Fast-DDS state and rebuild it.
        The entry is stored as a ``weakref.WeakMethod`` so the registry
        doesn't keep the participant alive; when the caller drops their
        last strong reference, the entry is garbage-collected on the
        next registry walk. Re-registering after GC is idempotent and
        safe.
        """

        try:
            hook_ref = weakref.WeakMethod(reset_hook)
        except TypeError as ex:
            # weakref.WeakMethod requires a bound method. A non-method
            # callable would silently keep the participant alive — make
            # the API contract loud rather than papering over with a
            # strong reference.
            raise TypeError(
                f"register_participant requires reset_hook to be a bound method "
                f"on the participant, not {type(reset_hook).__name__}"
            ) from ex

        # Resolve the env-driven settings and construct the monitor OUTSIDE
        # _registry_lock: resolution and construction emit warnings (invalid env
        # values, the netlink→polling fallback — the latter fires on EVERY first
        # registration in a sandbox without AF_NETLINK), _emit_log runs the user log
        # callback, and a callback that constructs another participant re-enters
        # register_participant on the same thread — a deadlock on the non-reentrant
        # lock if it were held. Mirrors the C++ pattern of routing every log out of
        # the locked region. The unsynchronised `self._monitor is None` peek is a
        # benign race: two racing first registrations both build a monitor, and the
        # loser's is stopped below without ever becoming THE monitor.
        prebuilt_monitor = None
        init_error: Optional[str] = None
        init_poll_interval = 0.0
        if self._monitor is None:
            try:
                init_poll_interval = _resolve_poll_interval()
                prebuilt_monitor = _make_network_monitor(
                    self._on_network_event, init_poll_interval, self._on_safety_net_tick
                )
            except Exception as ex:
                init_error = str(ex)

        with self._registry_lock:
            # GC expired entries inline.
            self._registered = [m for m in self._registered if m() is not None]
            # Idempotent: already registered?
            for m in self._registered:
                bound = m()
                if bound is not None and bound.__self__ is participant:
                    break
            else:
                self._registered.append(hook_ref)

            if self._monitor is None and prebuilt_monitor is not None:
                self._monitor = prebuilt_monitor
                prebuilt_monitor = None

        if prebuilt_monitor is not None:
            # Lost the construction race: another registration installed its monitor
            # first. Ours never became visible to anything — stop its worker thread.
            prebuilt_monitor.stop()
            init_error = None

        # Logged with no provizio_dds lock held — a user callback that re-enters via
        # make_domain_participant can run freely.
        #
        # Nothing is logged when the monitor starts successfully: that auto-recovery is
        # enabled, and what it saw, is this library's internal state. Only its FAILURE to
        # start is reported, because that leaves auto-recovery unavailable.
        if init_error is not None:
            _emit_log(
                LogLevel.ERROR,
                f"network auto-recovery: monitor failed to start ({init_error}); "
                f"auto-recovery unavailable until the next recovery-enabled "
                f"participant creation retries the initialization",
            )

    def _on_safety_net_tick(self) -> None:
        """Called by the monitor on each periodic tick, before its own snapshot
        re-check. Retries participants a previous reset left torn down: their
        Fast-DDS state is gone right now, independently of whether the network has
        moved since, and no further event is guaranteed to arrive. Bounded by
        ``_MAX_CONSECUTIVE_RETRY_PASSES`` so an endpoint that can never come back
        does not churn its healthy siblings once per tick forever; a real network
        change re-arms the retrying (see :meth:`_on_network_event`). Mirrors the
        retry branch of the C++ ``safety_net_tick``."""

        hooks = self._live_hooks()
        retry = [hook for hook in hooks if getattr(hook.__self__, "_recovery_retry_needed", False)]
        if not retry:
            self._consecutive_retry_passes = 0
            self._retry_exhaustion_reported = False
            return

        if self._consecutive_retry_passes >= self._MAX_CONSECUTIVE_RETRY_PASSES:
            if not self._retry_exhaustion_reported:
                self._retry_exhaustion_reported = True
                _emit_log(
                    LogLevel.ERROR,
                    f"network auto-recovery: gave up rebuilding participant(s) after "
                    f"{self._MAX_CONSECUTIVE_RETRY_PASSES} consecutive attempts; they stay "
                    f"inactive (publish/take report failure) until the next network change. "
                    f"Retrying further would keep tearing down their healthy siblings.",
                )
            return

        # NO snapshot read gates this pass, deliberately, and that is the whole point of
        # the retry: it re-attempts a rebuild that already failed, against whatever the host
        # looks like now, and needs to know nothing about what changed. Gating it on a read
        # would let an unreadable interface list silently cancel it -- the counter would
        # never move, so the bound would never be reached, so the exhaustion warning would
        # never fire, and a participant left torn down by a failed rebuild could stay dead
        # for as long as the reads kept failing, with nothing logged. That failure is not
        # hypothetical on macOS, where getifaddrs is a sysctl(NET_RT_IFLIST) size-then-fetch
        # pair that can lose a race with a routing-table change.
        #
        # Matching the C++ counterpart: safety_net_tick runs apply_reset(retry_only) before
        # it reads a snapshot at all, and hands participants no snapshot when it does
        # (trigger_network_recovery_reset takes none). The hooks here ignore both arguments
        # too -- they are passed only because this side's signature carries them -- so an
        # empty set stands in for them rather than a read that could fail.
        snapshot: AddressSnapshot = frozenset()

        self._consecutive_retry_passes += 1
        # Silent: retrying is internal; only exhausting every attempt is reported (below).
        with self._idle_lock:
            self._reset_in_progress = True
        try:
            for hook in retry:
                try:
                    hook(snapshot, snapshot)
                except Exception as ex:
                    _emit_log(LogLevel.ERROR, f"participant reset retry failed: {ex}")
        finally:
            with self._idle_lock:
                self.reset_count += 1
                self._reset_in_progress = False
                self._idle_cv.notify_all()

    def _live_hooks(self) -> "list":
        """Resolve the weak registry to the bound reset hooks still alive, dropping
        entries whose participant has been garbage-collected."""
        with self._registry_lock:
            resolved = [m() for m in self._registered]
        return [hook for hook in resolved if hook is not None]

    def _on_network_event(self, old: AddressSnapshot, new: AddressSnapshot,
                          burst_start: Optional[AddressSnapshot] = None) -> None:
        """Decide whether a settled burst warrants a participant rebuild.

        Rebuilds when the host has GAINED an address to bind: one that is not in
        ``old``, or — for event-driven backends that supply ``burst_start`` — one that
        is here now but was missing when the burst opened. The second case is a
        transient flap (a DDS-relevant address left and returned within the quiet
        period): the end-state looks unchanged but the Fast-DDS sockets bound to that
        address were torn down while it was gone and must be rebuilt.

        Addresses merely going away do NOT rebuild: nothing can be bound to what is
        gone, and tearing down endpoints that still work over the remaining interfaces
        buys nothing. The smaller set is adopted instead, so a later return reads as a
        gain and rebuilds then — which is the moment a rebuild can achieve something.

        Mirrors the C++ ``network_recovery_coordinator::run_reset``.
        """
        added = len(new - old)
        removed = len(old - new)
        returned = 0 if burst_start is None else len(new - burst_start)

        if added == 0 and returned == 0:
            if removed == 0:
                # Genuine no-op (or container/veth/link-local churn, which is filtered
                # out of both snapshots). Bump under _idle_lock to match reset_count so
                # a test sampling the counter has a happens-before edge to this write.
                # Silent: a burst that changed nothing is a non-event.
                with self._idle_lock:
                    self.skipped_reset_count += 1
                return

            # Loss only — see the docstring. The cost of waiting is that this participant
            # keeps announcing a locator that no longer answers until the next real
            # change, which is cheaper than dropping every in-flight sample now.
            _emit_log(
                LogLevel.INFO,
                f"network change detected: +0 / -{removed} interface address(es) "
                f"({len(old)} -> {len(new)}); not rebuilding for a loss alone -- will "
                f"rebuild if address(es) return",
            )
            with self._idle_lock:
                self.skipped_reset_count += 1
            return

        if added != 0 or removed != 0:
            _emit_log(
                LogLevel.INFO,
                f"network change detected: +{added} / -{removed} interface address(es) "
                f"({len(old)} -> {len(new)}); resetting recovery-enabled participants",
            )
        else:
            _emit_log(
                LogLevel.INFO,
                "network change detected: transient interface change within the debounce window "
                "(a DDS-relevant address left and returned, end-state unchanged); "
                "resetting recovery-enabled participants",
            )

        # A real change re-arms the failed-rebuild retrying: whatever made a rebuild
        # fail before may well be gone now, so an earlier give-up must not be
        # permanent. Mirrors the C++ safety_net_tick snapshot-change branch.
        self._consecutive_retry_passes = 0
        self._retry_exhaustion_reported = False

        with self._idle_lock:
            self._reset_in_progress = True

        # Snapshot the registry under the lock; operate without it so a
        # participant being destroyed concurrently doesn't deadlock on us.
        # Resolving the WeakMethod returns None if the participant has
        # been GC'd since registration — those entries are dropped here.
        live = self._live_hooks()

        # One reading of the host for the whole pass: each hook rebuilds a participant,
        # and each rebuild re-resolves the VPN exclusion. Without this every one of them
        # pays a fresh getifaddrs walk and netlink dump to answer the same question about
        # the same instant -- and they could answer it differently, which would leave one
        # event's participants configured from two views of the host.
        with scoped_vpn_blocklist_cache():
            for hook in live:
                try:
                    hook(old, new)
                except Exception as ex:
                    _emit_log(LogLevel.ERROR, f"participant reset failed: {ex}")

        # No completion line: the change was already announced, and a failure logs its own error.

        # Bump the observability counter UNDER the idle lock so a thread
        # that wakes from wait_for_idle and then samples reset_count cannot
        # see the old value: clearing _reset_in_progress and incrementing
        # reset_count must happen atomically with respect to the CV-wait
        # predicate.
        with self._idle_lock:
            self.reset_count += 1
            self._reset_in_progress = False
            self._idle_cv.notify_all()

    def wait_for_idle(self, timeout_sec: Optional[float] = None) -> bool:
        """Block until no reset is currently running. Test-only — production
        code interacts with the coordinator via the participant lifecycle.
        Returns ``True`` if idle was observed, ``False`` if the timeout
        elapsed."""

        with self._idle_cv:
            if not self._reset_in_progress:
                return True
            return self._idle_cv.wait_for(lambda: not self._reset_in_progress, timeout=timeout_sec)

    def inject_change_for_test(self, old: AddressSnapshot, new: AddressSnapshot) -> None:
        """Test-only: synthesize a network change event without waiting for
        the monitor. Drives the same decision path as a real change."""

        self._on_network_event(old, new)

    def inject_transient_for_test(self) -> None:
        """Test-only: drive a transient-flap reset — an address that left and
        returned within the quiet period, so the end-state equals the last known
        set but a rebuild is still required. Mirrors the C++
        ``network_recovery_coordinator::inject_transient_for_test``. No host
        interface manipulation needed: the end-state is the current real snapshot
        and the (synthetic) burst-start differs from it."""

        # The synthetic address is present in the end-state (and therefore in `old`, so
        # the end-state is unchanged) and ABSENT from the burst-start: that is the shape
        # that needs a rebuild — it went away and came back, so whatever was bound to it
        # died in between. The reverse shape (an extra address at burst start, gone by the
        # end) is an address that appeared and vanished inside the window; nothing was ever
        # bound to it, so it needs no rebuild. Synthetic rather than real so the case works
        # on a host with no addresses at all, e.g. a CI container.
        synthetic = ("provizio_test_transient_if", "203.0.113.7", 24)  # TEST-NET-3
        end_state = frozenset(_capture_address_snapshot()) | {synthetic}
        burst_start = end_state - {synthetic}
        self._on_network_event(end_state, end_state, burst_start)

    def run_safety_net_tick_for_test(self) -> bool:
        """Test-only: run one safety-net tick synchronously — reopen a dead monitor
        channel, retry participants left unrecovered by a failed rebuild, and reset
        everything if the live snapshot no longer matches the last known one.

        Only safe while the coordinator is idle; call :meth:`wait_for_idle` first.

        Returns False if no monitor has been started yet.
        """

        monitor = self._monitor
        if monitor is None:
            return False
        # Both backends implement this — the netlink one as its periodic re-verify, the
        # polling one as a single poll — so the same tests cover every platform.
        monitor.run_safety_net_tick_for_test()
        return True

    def kill_monitor_for_test(self) -> bool:
        """Test-only: drop the monitor's kernel channel as if it had failed
        unrecoverably, so the revival path can be exercised. Returns False where
        there is nothing to kill."""

        monitor = self._monitor
        kill = getattr(monitor, "kill_for_test", None) if monitor is not None else None
        return bool(kill()) if kill is not None else False

    def monitor_alive_for_test(self) -> bool:
        """Test-only: whether the monitor's kernel channel is currently watched."""
        monitor = self._monitor
        return bool(monitor is not None and monitor.is_alive())
