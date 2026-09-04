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

"""High-resolution recorder of what the host's interfaces are doing during a test run.

A CI job that reports "network change detected: +0 / -1 interface address(es) (1 -> 0)"
mid-test says the library saw an address vanish, but not whether the OS really lost it,
which interface it was, or how long it was gone. This records all three, independently of
the library:

  * every 100 ms, the COMPLETE ``getifaddrs`` reading -- every interface, its flags
    (IFF_UP / IFF_RUNNING in particular) and every address, with NO filtering -- and a line
    is emitted only when that reading changes;
  * alongside it, the library's own policy-filtered snapshot (imported from
    python/network_recovery.py), so a change the library reacted to can be told from one it
    filtered out;
  * the kernel's routing-socket event stream (``route -n monitor`` on macOS, ``ip monitor``
    on Linux), timestamped, which is the same feed the library's network monitor watches.

Usage:
    ci_net_watch.py [--out DIR] [--poll SEC]
"""

import argparse
import ctypes
import os
import socket
import subprocess
import sys
import threading
import time

_AF_INET = socket.AF_INET
_AF_INET6 = socket.AF_INET6
_IS_MACOS = sys.platform == "darwin"

# net/if.h. Same values on Darwin and Linux.
_IFF_UP = 0x1
_IFF_BROADCAST = 0x2
_IFF_LOOPBACK = 0x8
_IFF_POINTOPOINT = 0x10
_IFF_RUNNING = 0x40
_IFF_MULTICAST = 0x8000 if _IS_MACOS else 0x1000

_FLAG_NAMES = (
    (_IFF_UP, "UP"),
    (_IFF_BROADCAST, "BROADCAST"),
    (_IFF_LOOPBACK, "LOOPBACK"),
    (_IFF_POINTOPOINT, "P2P"),
    (_IFF_RUNNING, "RUNNING"),
    (_IFF_MULTICAST, "MULTICAST"),
)


class _SockaddrMac(ctypes.Structure):
    _fields_ = [("sa_len", ctypes.c_uint8), ("sa_family", ctypes.c_uint8), ("sa_data", ctypes.c_uint8 * 14)]


class _SockaddrLinux(ctypes.Structure):
    _fields_ = [("sa_family", ctypes.c_uint16), ("sa_data", ctypes.c_uint8 * 14)]


_Sockaddr = _SockaddrMac if _IS_MACOS else _SockaddrLinux


class _SockaddrIn(ctypes.Structure):
    if _IS_MACOS:
        _fields_ = [
            ("sin_len", ctypes.c_uint8),
            ("sin_family", ctypes.c_uint8),
            ("sin_port", ctypes.c_uint16),
            ("sin_addr", ctypes.c_uint8 * 4),
        ]
    else:
        _fields_ = [
            ("sin_family", ctypes.c_uint16),
            ("sin_port", ctypes.c_uint16),
            ("sin_addr", ctypes.c_uint8 * 4),
        ]


class _SockaddrIn6(ctypes.Structure):
    if _IS_MACOS:
        _fields_ = [
            ("sin6_len", ctypes.c_uint8),
            ("sin6_family", ctypes.c_uint8),
            ("sin6_port", ctypes.c_uint16),
            ("sin6_flowinfo", ctypes.c_uint32),
            ("sin6_addr", ctypes.c_uint8 * 16),
            ("sin6_scope_id", ctypes.c_uint32),
        ]
    else:
        _fields_ = [
            ("sin6_family", ctypes.c_uint16),
            ("sin6_port", ctypes.c_uint16),
            ("sin6_flowinfo", ctypes.c_uint32),
            ("sin6_addr", ctypes.c_uint8 * 16),
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
    ("ifa_dstaddr", ctypes.POINTER(_Sockaddr)),
    ("ifa_data", ctypes.c_void_p),
]


def _libc():
    return ctypes.CDLL("libc.dylib" if _IS_MACOS else "libc.so.6", use_errno=True)


def _flags_text(flags):
    names = [name for bit, name in _FLAG_NAMES if flags & bit]
    return f"0x{flags:x}<{','.join(names)}>"


def raw_reading():
    """Every (interface, flags, address) the kernel reports, unfiltered.

    Returns a sorted tuple of strings, or ``("<getifaddrs failed: ...>",)``: an
    enumeration failure has to be visible as itself, since reporting it as an empty
    reading is exactly the confusion this recorder exists to resolve.
    """
    libc = _libc()
    getifaddrs = libc.getifaddrs
    getifaddrs.argtypes = [ctypes.POINTER(ctypes.POINTER(_Ifaddrs))]
    getifaddrs.restype = ctypes.c_int
    freeifaddrs = libc.freeifaddrs
    freeifaddrs.argtypes = [ctypes.POINTER(_Ifaddrs)]
    freeifaddrs.restype = None

    head = ctypes.POINTER(_Ifaddrs)()
    if getifaddrs(ctypes.byref(head)) != 0:
        return (f"<getifaddrs failed: {os.strerror(ctypes.get_errno())}>",)

    entries = []
    try:
        node = head
        while node:
            entry = node.contents
            name = entry.ifa_name.decode("ascii", "surrogateescape") if entry.ifa_name else "?"
            addr_text = "-"
            if entry.ifa_addr:
                family = entry.ifa_addr.contents.sa_family
                if family == _AF_INET:
                    sin = ctypes.cast(entry.ifa_addr, ctypes.POINTER(_SockaddrIn)).contents
                    addr_text = socket.inet_ntop(_AF_INET, bytes(sin.sin_addr))
                elif family == _AF_INET6:
                    sin6 = ctypes.cast(entry.ifa_addr, ctypes.POINTER(_SockaddrIn6)).contents
                    addr_text = socket.inet_ntop(_AF_INET6, bytes(sin6.sin6_addr))
                else:
                    addr_text = f"<family {family}>"
            entries.append(f"{name} {_flags_text(entry.ifa_flags)} {addr_text}")
            node = entry.ifa_next
    finally:
        freeifaddrs(head)
    return tuple(sorted(entries))


def library_snapshot():
    """The library's own policy-filtered snapshot, or a one-line reason it is unavailable."""
    try:
        import network_recovery  # noqa: PLC0415 - optional, resolved at first use
    except ImportError as ex:
        return (f"<network_recovery unavailable: {ex}>",)
    try:
        snapshot = network_recovery._capture_address_snapshot()  # noqa: SLF001 - diagnostic
    except Exception as ex:  # noqa: BLE001 - a diagnostic must report, not raise
        return (f"<snapshot failed: {ex!r}>",)
    return tuple(sorted(str(item) for item in snapshot))


def _stamp():
    return time.strftime("%H:%M:%S", time.localtime()) + f".{int((time.time() % 1) * 1000):03d}"


def _emit(handle, text):
    line = f"[net-watch {_stamp()}] {text}"
    handle.write(line + "\n")
    handle.flush()


# Routing-message types, macOS/BSD (net/route.h). Only the ones an interface change can
# produce are named; anything else is reported by number.
_RTM_NAMES = {
    1: "RTM_ADD", 2: "RTM_DELETE", 3: "RTM_CHANGE", 4: "RTM_GET", 5: "RTM_LOSING",
    6: "RTM_REDIRECT", 7: "RTM_MISS", 8: "RTM_LOCK", 11: "RTM_RESOLVE",
    12: "RTM_NEWADDR", 13: "RTM_DELADDR", 14: "RTM_IFINFO", 15: "RTM_NEWMADDR",
    16: "RTM_DELMADDR", 17: "RTM_IFINFO2", 18: "RTM_NEWMADDR2", 19: "RTM_GET2",
}

# What src/network_monitor_macos.cpp compares every routing message against before it will
# look at the message at all: it requires more than sizeof(struct rt_msghdr) bytes. That
# struct is the ROUTE-entry header (92 bytes on macOS: 36 bytes of header plus a 56-byte
# rt_metrics); an address message uses struct ifa_msghdr, which is 20 bytes plus its
# sockaddrs and is routinely shorter than 92 in total. If that is right, every RTM_NEWADDR
# and RTM_DELADDR is silently discarded there -- so this records the size of each message
# as the kernel actually delivers it, which settles it on the real runner.
_CPP_MONITOR_MIN_ACCEPTED_BYTES = 92


def _raw_route_socket_monitor(handle, stop):
    """Log every routing message's type and LENGTH, straight off a PF_ROUTE socket."""
    if not _IS_MACOS:
        return
    try:
        route_socket = socket.socket(socket.AF_ROUTE, socket.SOCK_RAW, 0)
    except (AttributeError, OSError) as ex:
        _emit(handle, f"raw PF_ROUTE socket unavailable: {ex!r}")
        return
    _emit(handle, "raw PF_ROUTE socket open; logging every message's type and length")
    dropped_by_cpp_rule = 0
    try:
        route_socket.settimeout(1.0)
        while not stop.is_set():
            try:
                message = route_socket.recv(4096)
            except socket.timeout:
                continue
            except OSError as ex:
                _emit(handle, f"raw PF_ROUTE recv failed: {ex!r}")
                return
            if len(message) < 4:
                continue
            msglen = int.from_bytes(message[0:2], "little")
            version = message[2]
            rtm_type = message[3]
            name = _RTM_NAMES.get(rtm_type, f"type {rtm_type}")
            would_drop = len(message) <= _CPP_MONITOR_MIN_ACCEPTED_BYTES
            if would_drop:
                dropped_by_cpp_rule += 1
            _emit(
                handle,
                f"RTM {name} recv={len(message)}B rtm_msglen={msglen} version={version}"
                + (
                    f" -- SHORTER THAN sizeof(rt_msghdr)={_CPP_MONITOR_MIN_ACCEPTED_BYTES}, so"
                    f" network_monitor_macos.cpp would DISCARD it (#{dropped_by_cpp_rule})"
                    if would_drop
                    else ""
                ),
            )
    finally:
        route_socket.close()


def _route_monitor(handle, stop):
    """Timestamp and record the kernel's own interface / route event stream."""
    argv = ["route", "-n", "monitor"] if _IS_MACOS else ["ip", "monitor", "address", "link", "route"]
    try:
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except OSError as ex:
        _emit(handle, f"route monitor unavailable: {ex!r}")
        return
    try:
        for raw in proc.stdout:
            if stop.is_set():
                break
            text = raw.decode("utf-8", "replace").rstrip("\n")
            if text.strip():
                _emit(handle, f"ROUTE-EVENT {text}")
    except Exception as ex:  # noqa: BLE001 - a diagnostic must report, not raise
        _emit(handle, f"route monitor stopped: {ex!r}")
    finally:
        try:
            proc.terminate()
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None)
    parser.add_argument("--poll", type=float, default=0.1)
    parser.add_argument("--raw-poll", type=float, default=0.02,
                        help="interval for the unfiltered getifaddrs reading")
    parser.add_argument("--tag", default="", help="suffix for the output file, so consecutive runs do not overwrite")
    args = parser.parse_args()

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        suffix = f".{args.tag}" if args.tag else ""
        handle = open(os.path.join(args.out, f"net_watch{suffix}.txt"), "w", encoding="utf-8")
    else:
        handle = sys.stderr

    raw_poll = args.raw_poll
    stop = threading.Event()
    threading.Thread(target=_route_monitor, args=(handle, stop), daemon=True).start()
    threading.Thread(target=_raw_route_socket_monitor, args=(handle, stop), daemon=True).start()
    _emit(handle, f"started (snapshot poll {args.poll:g}s, raw poll {raw_poll:g}s, platform {sys.platform})")

    previous_raw = None
    previous_snapshot = None
    changes = [0]
    zero_address_readings = [0]

    def watch_raw():
        """The kernel's own reading, as fast as it can be taken.

        Faster than the library's own poller on purpose: the flap this is looking for is
        reported by a 3-second poller as a state change, which means the true event may be
        far shorter than the interval that noticed it. A reading is timed, so a getifaddrs
        call that itself takes an unusual time is visible too.
        """
        nonlocal previous_raw
        while not stop.is_set():
            started = time.monotonic()
            current = raw_reading()
            took_ms = (time.monotonic() - started) * 1000.0
            if took_ms > 50.0:
                _emit(handle, f"SLOW getifaddrs: {took_ms:.0f} ms")
            if current != previous_raw:
                changes[0] += 1
                _emit(handle, f"RAW CHANGE #{changes[0]} ({len(current)} entries, read in {took_ms:.1f} ms)")
                if previous_raw is not None:
                    for gone in sorted(set(previous_raw) - set(current)):
                        _emit(handle, f"  - {gone}")
                    for added in sorted(set(current) - set(previous_raw)):
                        _emit(handle, f"  + {added}")
                else:
                    for item in current:
                        _emit(handle, f"  = {item}")
                previous_raw = current
            # The condition the library turns into a participant rebuild: no non-loopback
            # interface with a usable address. Counted separately because it is the whole
            # question, and it can be brief enough that a change line alone understates it.
            usable = [e for e in current if "LOOPBACK" not in e and e.rsplit(" ", 1)[-1] not in ("-",)
                      and not e.rsplit(" ", 1)[-1].startswith("<")]
            if not usable:
                zero_address_readings[0] += 1
                if zero_address_readings[0] <= 200 or zero_address_readings[0] % 100 == 0:
                    _emit(handle, f"ZERO-USABLE-ADDRESS READING #{zero_address_readings[0]}: {current}")
            time.sleep(raw_poll)

    threading.Thread(target=watch_raw, daemon=True).start()

    try:
        while True:
            current_snapshot = library_snapshot()
            if current_snapshot != previous_snapshot:
                _emit(handle, f"LIBRARY SNAPSHOT CHANGE ({len(current_snapshot)} entries)")
                if previous_snapshot is not None:
                    for gone in sorted(set(previous_snapshot) - set(current_snapshot)):
                        _emit(handle, f"  - {gone}")
                    for added in sorted(set(current_snapshot) - set(previous_snapshot)):
                        _emit(handle, f"  + {added}")
                else:
                    for item in current_snapshot:
                        _emit(handle, f"  = {item}")
                previous_snapshot = current_snapshot
            time.sleep(args.poll)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        _emit(handle, f"stopped after {changes[0]} raw change(s), "
                      f"{zero_address_readings[0]} reading(s) with no usable address")
        if handle is not sys.stderr:
            handle.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
