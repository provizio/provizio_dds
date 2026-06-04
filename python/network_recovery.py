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

  1. Event detection is polling-based rather than kernel-notification-based.
     Pure Python has no portable, dependency-free way to subscribe to kernel
     address-change events; ctypes wrappers around netlink / PF_ROUTE /
     NotifyUnicastIpAddressChange would add a significant amount of
     platform-specific glue with limited benefit (recovery is intentionally
     debounced over a multi-second quiet period anyway). Polling is simpler,
     dependency-free, and the latency is well within the design budget.

  2. The reset hook is supplied by the Python `_DomainParticipant`, not the
     coordinator. The Python participant knows how to tear down and rebuild
     its own Publisher / Subscriber / Service objects (which are pure-Python
     classes that wrap Fast-DDS-Python SWIG types).

  3. The cross-platform interface enumeration uses ctypes against libc's
     `getifaddrs` on POSIX and `GetAdaptersAddresses` on Windows, with the
     same filter list as the C++ side (loopback, link-local, docker /
     veth / etc.).

The polling cadence is configurable via the
`PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC` env variable; default 3 s
(matches the C++ quiet_period).
"""

from __future__ import annotations

import ctypes
import os
import queue
import socket
import struct
import sys
import threading
import weakref
from enum import Enum
from typing import Any, Callable, Dict, FrozenSet, Optional, Tuple


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


# Address-set type. Two snapshots are equal if their contents match
# order-independently; frozenset gives us this for free.
AddressSnapshot = FrozenSet[Tuple[str, str]]


def _capture_address_snapshot() -> AddressSnapshot:
    """Capture the host's current 'DDS-interesting' interface→address set.

    Excluded by platform:

    Common to all:
      - Loopback interfaces.
      - IPv6 link-local ``fe80::/10`` addresses.

    Linux / macOS (via getifaddrs):
      - Names matching Docker / Kubernetes / LXC conventions
        (docker*, br-*, cni*, kube*, lxc*, flannel*, weave*, veth*).
      - macOS: Apple-internal NICs (utun, awdl, llw, gif, stf, anpi, ap[0-9]).

    Windows (via GetAdaptersAddresses):
      - Interfaces with IfType not in {Ethernet, IEEE80211, PPP}.
      - Friendly-name / description match for known virtual adapters.

    Returns an empty snapshot if the OS call fails or the host has only
    loopback up.
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
# virtual / tunnel interfaces and are excluded from the snapshot, matching the
# C++ side in src/address_snapshot_linux.cpp. Physical Ethernet / Wi-Fi have
# no kind attribute and fall through.
_LINUX_EXCLUDED_KINDS = frozenset(
    {"bridge", "veth", "dummy", "vxlan", "macvlan", "ipvlan", "ip6tnl", "tun"}
)


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


def _align4(n: int) -> int:
    return (n + 3) & ~3


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
        return {}

    sock.settimeout(_NETLINK_RECV_TIMEOUT_SEC)

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
            1,
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
        while not stop:
            buf, _ancdata, msg_flags, _addr = sock.recvmsg(16 * 1024)
            if not buf:
                break
            if msg_flags & socket.MSG_TRUNC:
                _emit_log(
                    LogLevel.WARNING,
                    "network monitor: netlink RTM_GETLINK reply truncated (buffer too small); "
                    "falling back to name-prefix interface filtering",
                )
                break
            offset = 0
            while offset + 16 <= len(buf):
                nlmsg_len, nlmsg_type, _flags, _seq, _pid = struct.unpack_from("=IHHII", buf, offset)
                if nlmsg_len < 16 or offset + nlmsg_len > len(buf):
                    break
                if nlmsg_type == _NLMSG_DONE:
                    dump_complete = True
                    stop = True
                    break
                if nlmsg_type == _NLMSG_ERROR:
                    _emit_log(
                        LogLevel.WARNING,
                        "network monitor: netlink RTM_GETLINK dump returned an error; "
                        "falling back to name-prefix interface filtering",
                    )
                    stop = True
                    break
                if nlmsg_type == _RTM_NEWLINK:
                    # ifinfomsg starts immediately after nlmsghdr (16 B).
                    ifinfo_offset = offset + 16
                    if ifinfo_offset + 16 <= offset + nlmsg_len:
                        _family, _pad, _itype, ifindex, _iflags, _change = struct.unpack_from(
                            "=BBHiII", buf, ifinfo_offset
                        )
                        # Walk top-level rtattrs after the ifinfomsg.
                        rta_offset = ifinfo_offset + 16
                        rta_end = offset + nlmsg_len
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
                offset += _align4(nlmsg_len)
    except OSError:
        # Includes socket.timeout: a stalled kernel reply must not be treated
        # as a complete dump. dump_complete stays False → empty map below.
        pass
    finally:
        sock.close()

    return kinds if dump_complete else {}


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
# silently failing every family check. So we branch on platform.
_IS_BSD_SOCKADDR = sys.platform == "darwin"


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


# IFF_LOOPBACK / IFF_UP — defined in net/if.h. Hard-coded to avoid an
# additional ctypes lookup against libc constants that differ by platform.
_IFF_UP_VALUE = 0x1
_IFF_LOOPBACK_VALUE = 0x8

_AF_INET = socket.AF_INET
_AF_INET6 = socket.AF_INET6


def _load_libc():
    if sys.platform == "darwin":
        return ctypes.CDLL("libc.dylib", use_errno=True)
    return ctypes.CDLL("libc.so.6", use_errno=True)


def _capture_snapshot_posix() -> AddressSnapshot:
    try:
        libc = _load_libc()
    except OSError as ex:
        _emit_log(LogLevel.WARNING, f"network monitor: libc dlopen failed ({ex}); empty snapshot")
        return frozenset()

    getifaddrs = libc.getifaddrs
    getifaddrs.argtypes = [ctypes.POINTER(ctypes.POINTER(_Ifaddrs))]
    getifaddrs.restype = ctypes.c_int

    freeifaddrs = libc.freeifaddrs
    freeifaddrs.argtypes = [ctypes.POINTER(_Ifaddrs)]
    freeifaddrs.restype = None

    head = ctypes.POINTER(_Ifaddrs)()
    if getifaddrs(ctypes.byref(head)) != 0:
        return frozenset()

    is_macos = sys.platform == "darwin"
    name_excluded = _macos_name_excluded if is_macos else _linux_name_excluded

    # On Linux, also exclude virtual / container / tunnel kinds by their
    # IFLA_INFO_KIND attribute — mirrors src/address_snapshot_linux.cpp.
    # if_nametoindex maps each ifa entry's name to the ifindex key in the
    # kind map.
    kinds_by_index: Dict[int, str] = _fetch_link_kinds_linux() if not is_macos else {}
    if_nametoindex = None
    if kinds_by_index:
        try:
            if_nametoindex = libc.if_nametoindex
            if_nametoindex.argtypes = [ctypes.c_char_p]
            if_nametoindex.restype = ctypes.c_uint
        except (AttributeError, OSError):
            if_nametoindex = None

    result: set = set()
    try:
        cur = head
        while cur:
            entry = cur.contents
            name = entry.ifa_name.decode("ascii", errors="replace") if entry.ifa_name else ""
            flags = entry.ifa_flags
            if (flags & _IFF_LOOPBACK_VALUE) != 0 or (flags & _IFF_UP_VALUE) == 0:
                cur = entry.ifa_next
                continue
            if not name or name_excluded(name):
                cur = entry.ifa_next
                continue
            if if_nametoindex is not None:
                idx = if_nametoindex(entry.ifa_name)
                kind = kinds_by_index.get(int(idx), "")
                if _linux_kind_excluded(kind):
                    cur = entry.ifa_next
                    continue
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
                        cur = entry.ifa_next
                        continue
                if addr_text:
                    result.add((name, addr_text))
            cur = entry.ifa_next
    finally:
        freeifaddrs(head)

    return frozenset(result)


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
_IF_TYPE_PPP = 23
_IF_TYPE_IEEE80211 = 71
_KEPT_IF_TYPES = (_IF_TYPE_ETHERNET_CSMACD, _IF_TYPE_PPP, _IF_TYPE_IEEE80211)


def _description_excluded_win(description: str) -> bool:
    for needle in _WIN_EXCLUDED_DESCRIPTIONS:
        if needle in description:
            return True
    return False


def _capture_snapshot_windows() -> AddressSnapshot:
    # The Windows path uses ctypes against iphlpapi.GetAdaptersAddresses.
    # We deliberately keep the struct definitions minimal — only the fields
    # we read are declared. Padding to match the actual layout is achieved
    # by declaring opaque byte arrays for the skipped regions.
    try:
        iphlpapi = ctypes.windll.iphlpapi  # type: ignore[attr-defined]
    except (AttributeError, OSError):
        return frozenset()

    # Sockaddr layouts shared with the POSIX path are fine here too (same
    # binary layout on Windows). What's different is IP_ADAPTER_ADDRESSES,
    # which is large; we lay out only the fields we need and skip the rest
    # via byte arrays computed by offset.

    SOCKET_ADDRESS_size = ctypes.sizeof(ctypes.c_void_p) + ctypes.sizeof(ctypes.c_int)

    class SOCKET_ADDRESS(ctypes.Structure):
        _fields_ = [
            ("lpSockaddr", ctypes.POINTER(_Sockaddr)),
            ("iSockaddrLength", ctypes.c_int),
        ]

    class IP_ADAPTER_UNICAST_ADDRESS(ctypes.Structure):
        pass

    # Fields up to and including Address — anything we don't need beyond
    # Address is collapsed into a single tail array. We also include
    # DadState (offset varies). To stay portable across SDK versions, we
    # treat unknown fields as a void block and only read by struct layout.
    IP_ADAPTER_UNICAST_ADDRESS._fields_ = [
        ("Length", ctypes.c_ulong),
        ("Flags", ctypes.c_ulong),
        ("Next", ctypes.POINTER(IP_ADAPTER_UNICAST_ADDRESS)),
        ("Address", SOCKET_ADDRESS),
        # We only need the head of the struct; remaining fields (PrefixOrigin,
        # SuffixOrigin, DadState, ValidLifetime, PreferredLifetime, LeaseLifetime,
        # OnLinkPrefixLength) sum to 7 * 4 bytes on x64 (some are ulong, one is
        # uchar). We don't filter on DadState in Python — same simplification
        # the original Linux/macOS paths take.
    ]

    class IP_ADAPTER_ADDRESSES(ctypes.Structure):
        pass

    IP_ADAPTER_ADDRESSES._fields_ = [
        ("Length", ctypes.c_ulong),
        ("IfIndex", ctypes.c_ulong),
        ("Next", ctypes.POINTER(IP_ADAPTER_ADDRESSES)),
        ("AdapterName", ctypes.c_char_p),
        ("FirstUnicastAddress", ctypes.POINTER(IP_ADAPTER_UNICAST_ADDRESS)),
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

    GAA_FLAG_SKIP_ANYCAST = 0x2
    GAA_FLAG_SKIP_MULTICAST = 0x4
    GAA_FLAG_SKIP_DNS_SERVER = 0x8
    AF_UNSPEC = 0
    ERROR_BUFFER_OVERFLOW = 111
    NO_ERROR = 0

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
            AF_UNSPEC,
            GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER,
            None,
            ctypes.cast(buffer, ctypes.c_void_p),
            ctypes.byref(buf_size),
        )
        if ret == ERROR_BUFFER_OVERFLOW:
            buffer = ctypes.create_string_buffer(buf_size.value)
            continue
        break
    if ret != NO_ERROR:
        return frozenset()

    result: set = set()
    adapter = ctypes.cast(buffer, ctypes.POINTER(IP_ADAPTER_ADDRESSES))
    while adapter:
        a = adapter.contents
        if a.OperStatus == _IF_OPER_STATUS_UP and a.IfType in _KEPT_IF_TYPES:
            friendly = a.FriendlyName or ""
            description = a.Description or ""
            if not _description_excluded_win(friendly) and not _description_excluded_win(description):
                name = a.AdapterName.decode("ascii", errors="replace") if a.AdapterName else friendly
                uni = a.FirstUnicastAddress
                while uni:
                    u = uni.contents
                    sa = u.Address.lpSockaddr
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
                            result.add((name, addr_text))
                    uni = u.Next
        adapter = a.Next
    return frozenset(result)


# ---------------------------------------------------------------------------
# Listener drain — detach + wait for in-flight callbacks
# ---------------------------------------------------------------------------


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

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._in_flight = 0
        self._detached = False

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
        module docstring for the rationale."""
        with self._cv:
            self._detached = True
            while self._in_flight > 0:
                self._cv.wait()

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
# Network monitor — polling-based
# ---------------------------------------------------------------------------


class _NetworkMonitor:
    """Polls :func:`_capture_address_snapshot` on an interval and fires a
    callback when the result changes from the last observed value.

    The polling interval doubles as the coalescing quiet period: a burst of
    OS-level changes that all settle within one interval is observed as a
    single delta.
    """

    def __init__(self, on_change: Callable[[AddressSnapshot, AddressSnapshot], None],
                 poll_interval_sec: float):
        self._on_change = on_change
        self._poll_interval_sec = poll_interval_sec
        self._stop_event = threading.Event()
        # Initial snapshot is captured BEFORE the worker thread starts so
        # the first observation has a baseline to diff against.
        self._last_snapshot: AddressSnapshot = _capture_address_snapshot()
        self._thread = threading.Thread(target=self._run, name="provizio_dds.network_monitor", daemon=True)
        self._thread.start()

    def initial_snapshot(self) -> AddressSnapshot:
        return self._last_snapshot

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join()

    def _run(self) -> None:
        while not self._stop_event.wait(self._poll_interval_sec):
            try:
                new_snapshot = _capture_address_snapshot()
            except Exception as ex:
                _emit_log(LogLevel.WARNING, f"network monitor: snapshot capture failed ({ex})")
                continue
            if new_snapshot != self._last_snapshot:
                old = self._last_snapshot
                self._last_snapshot = new_snapshot
                try:
                    self._on_change(old, new_snapshot)
                except Exception as ex:
                    _emit_log(LogLevel.ERROR, f"network monitor: on_change handler raised ({ex})")


# ---------------------------------------------------------------------------
# Coordinator — process-wide registry of recovery-enabled participants
# ---------------------------------------------------------------------------


# Default polling interval, in seconds. Mirrors the C++ quiet_period so the
# end-to-end recovery cadence is comparable.
_DEFAULT_POLL_INTERVAL_SEC = 3.0


def _resolve_poll_interval() -> float:
    """Read the per-process polling cadence from
    ``PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC``; fall back to the
    default. Unparseable values are logged once and ignored."""
    raw = os.environ.get(_POLL_INTERVAL_ENV)
    if not raw:
        return _DEFAULT_POLL_INTERVAL_SEC
    try:
        v = float(raw)
        if v <= 0:
            raise ValueError("non-positive")
        return v
    except (ValueError, TypeError):
        _emit_log(
            LogLevel.WARNING,
            f"{_POLL_INTERVAL_ENV}={raw} is not a positive number; "
            f"using default {_DEFAULT_POLL_INTERVAL_SEC}s",
        )
        return _DEFAULT_POLL_INTERVAL_SEC


class _NetworkRecoveryCoordinator:
    """Process-wide singleton, constructed lazily on the first
    ``register_participant`` call from a recovery-enabled participant.

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
        self._monitor: Optional[_NetworkMonitor] = None
        # Test observability counters — always present so behaviour stays
        # comparable to the C++ side. See network_recovery_coordinator.h.
        self.reset_count = 0
        self.skipped_reset_count = 0
        self._idle_lock = threading.Lock()
        self._idle_cv = threading.Condition(self._idle_lock)
        self._reset_in_progress = False

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

        # Capture the outcome inside the locked section, emit the log
        # AFTER releasing. A user-installed log callback could otherwise
        # re-enter make_domain_participant → register_participant and
        # recursively acquire _registry_lock — deadlock. Mirrors the C++
        # network_recovery_coordinator::register_participant pattern.
        initialised_monitor_now = False
        initial_snapshot_size = 0
        init_error: Optional[str] = None
        init_poll_interval = 0.0

        with self._registry_lock:
            # GC expired entries inline.
            self._registered = [m for m in self._registered if m() is not None]
            # Idempotent: already registered?
            for m in self._registered:
                bound = m()
                if bound is not None and bound.__self__ is participant:
                    return
            self._registered.append(hook_ref)

            if self._monitor is None:
                try:
                    init_poll_interval = _resolve_poll_interval()
                    self._monitor = _NetworkMonitor(self._on_network_change, init_poll_interval)
                    initialised_monitor_now = True
                    initial_snapshot_size = len(self._monitor.initial_snapshot())
                except Exception as ex:
                    init_error = str(ex)
                    self._monitor = None

        # Logs emitted with no provizio_dds lock held — a user callback
        # that re-enters via make_domain_participant can run freely.
        if initialised_monitor_now:
            _emit_log(
                LogLevel.INFO,
                f"network auto-recovery: enabled "
                f"(initial snapshot: {initial_snapshot_size} interface address(es), "
                f"poll interval {init_poll_interval}s)",
            )
        elif init_error is not None:
            _emit_log(
                LogLevel.ERROR,
                f"network auto-recovery: monitor failed to start ({init_error}); "
                f"auto-recovery unavailable until the next recovery-enabled "
                f"participant creation retries the initialization",
            )

    def _on_network_change(self, old: AddressSnapshot, new: AddressSnapshot) -> None:
        added = len(new - old)
        removed = len(old - new)
        if not added and not removed:
            # The monitor's polling loop only invokes us when new != old,
            # so this should not happen in practice. Guard anyway. Bump under
            # _idle_lock to match reset_count, so a test sampling the counter
            # has a happens-before edge to this write.
            with self._idle_lock:
                self.skipped_reset_count += 1
            return

        _emit_log(
            LogLevel.INFO,
            f"network change detected: +{added} / -{removed} interface address(es) "
            f"({len(old)} → {len(new)}); resetting recovery-enabled participants",
        )

        with self._idle_lock:
            self._reset_in_progress = True

        # Snapshot the registry under the lock; operate without it so a
        # participant being destroyed concurrently doesn't deadlock on us.
        # Resolving the WeakMethod returns None if the participant has
        # been GC'd since registration — those entries are dropped here.
        with self._registry_lock:
            live = [m() for m in self._registered]
        live = [hook for hook in live if hook is not None]

        for hook in live:
            try:
                hook(old, new)
            except Exception as ex:
                _emit_log(LogLevel.ERROR, f"participant reset failed: {ex}")

        _emit_log(LogLevel.INFO, f"reset complete ({len(live)} participant(s))")

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
        the polling loop. Drives the same code path as a real change."""

        self._on_network_change(old, new)
