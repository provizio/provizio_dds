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
"""Cross-language parity of the VPN / tunnel interface classifier.

The rule lives twice: in src/vpn_interfaces.cpp and, hand-mirrored, in
python/network_recovery.py -- vendor needles, the anchored generic prefixes, the
ZeroTier suffix rule and (on Linux) the tunnel kind set. Nothing but this test ties
the two together, so a needle added on one side only would let a C++ process and a
Python process on the SAME host classify the same physical interface differently: one
excludes the tunnel from its transports and its change detection, the other binds and
announces on it. Both suites' own cases would stay green, because each asserts against
its own language's list.

The table of names lives here and is passed to both sides, so neither can be right
about a name the other was never asked about. The C++ verdicts come from the
vpn_interfaces_test executable's ``classifier_table`` subcommand (path given as this
test's argument by the CMake registration); the Python ones come straight from the
module. Any disagreement is a failure, in either direction.

Import-only on the Python side and one short subprocess on the C++ side -- no DDS
participants, so it returns in milliseconds."""

import os
import subprocess
import sys

import network_recovery

# Names chosen to cover every branch of the classifier, in both directions:
#   - each vendor needle, and one embedded in a longer Windows-style string;
#   - each anchored generic prefix, plus a name that CONTAINS one without starting
#     with it ("NETGEAR WG111v3"), which is the whole reason they are anchored;
#   - the ZeroTier device convention and near-misses on its length and alphabet;
#   - Linux tunnel kinds and non-tunnel kinds (the kind column is 0 off Linux on both
#     sides, so the same table is usable everywhere);
#   - ordinary NICs, container plumbing, and a renamed NIC that must NOT be taken for
#     a product name.
NAMES = [
    # Tunnels by vendor.
    "tailscale0",
    "Tailscale",
    "tailscale-office",
    "wireguard",
    "WireGuard Tunnel",
    "openvpn",
    "OpenVPN TAP-Windows6",
    "zerotier",
    "ZeroTier One",
    # Tunnels by generic prefix.
    "tun0",
    "tun9",
    "tap0",
    "utun3",
    "ipsec1",
    "wg0",
    # The kernel tunnels macOS names itself, which the address snapshot has always dropped
    # and the transports must therefore refuse to bind as well.
    "gif0",
    "stf0",
    "TAP-Windows Adapter V9",
    "Tunnel adapter Teredo",
    # ZeroTier device names: the convention, and near-misses on it.
    "ztppmkbrz2",
    "ztabcdefgh",
    "zt",
    "ztshort",
    "ztppmkbrz2x",
    "zt-ppmkbrz",
    # Linux interface kinds.
    "vti",
    "vti6",
    "xfrm",
    "ipip",
    "ip6tnl",
    "gre",
    "gretap",
    "ip6gre",
    "sit",
    "bridge",
    "veth",
    "dummy",
    "vxlan",
    "macvlan",
    "ipvlan",
    "bond",
    "",
    # Ordinary interfaces and container plumbing, which must never classify as tunnels.
    "eth0",
    "eno1",
    "enp8s0",
    "wlp9s0",
    "wlan0",
    "en0",
    "docker0",
    "br0",
    "br-lan",
    "veth1a2b",
    "lo",
    "l4tbr0",
    "ppp0",
    "usb0",
    "can0",
    "Ethernet",
    "Wi-Fi",
    "Ethernet 2",
    # Renamed NICs: a user-editable label must not be matched against generic prefixes.
    "WG-LAN",
    "TUN-uplink",
    "tap-office",
    "NETGEAR WG111v3 54Mbps Wireless USB 2.0 Adapter",
    # A driver description with the model number FIRST, which is what makes the
    # description a different question from a device name: it begins with "wg" and is a
    # real radio. is_vpn_description must refuse it while still accepting the two
    # Windows tunnel forms above.
    "WG111v3 54Mbps Wireless USB 2.0 Adapter",
    "TUN-2000 Gigabit Adapter",
    "Intel(R) Ethernet Connection I219-LM",
    "Marvell AQtion 10Gbit Network Adapter",
]

_failures = []


def expect(condition, description):
    if not condition:
        print(f"FAIL: {description}")
        _failures.append(description)
    return condition


# Budget for one run of the C++ classifier. Inside the ctest TIMEOUT of this test (60 s,
# which provizio_dds_finalize_tests multiplies by the same factor for sanitizer builds), so
# that a hung binary is reported by this test's own message rather than by ctest's kill;
# the classifier itself answers in well under a second, instrumented or not.
_TIMEOUT_SCALE = float(os.environ.get("PROVIZIO_DDS_TEST_TIMEOUT_SCALE", "1") or "1")
SUBPROCESS_TIMEOUT_SEC = 30.0 * _TIMEOUT_SCALE


def cpp_verdicts(executable):
    """``{name: (vpn_name, vpn_product, vpn_description, vpn_kind)}`` as C++ sees it."""
    finished = subprocess.run(
        [executable, "classifier_table", *NAMES],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=SUBPROCESS_TIMEOUT_SEC,
        check=False,
    )
    text = finished.stdout.decode("utf-8", "replace")
    if finished.returncode != 0:
        print(text)
        raise RuntimeError(
            f"{executable} classifier_table exited with {finished.returncode}"
        )

    verdicts = {}
    for line in text.splitlines():
        # An empty name is one of the inputs, so a line can legitimately start with the
        # separator; anything without the three separators is not one of our lines (a
        # Fast-DDS log line, say) and is ignored.
        parts = line.split("|")
        if len(parts) != 5 or any(part not in ("0", "1") for part in parts[1:]):
            continue
        verdicts[parts[0]] = tuple(part == "1" for part in parts[1:])
    return verdicts


# Addresses put to both languages' loopback predicate, which decides the allowlist's
# netmask filter: the whole 127.0.0.0/8 and ::1 are loopback, nothing else is. Fast-DDS' own
# IP4_LOCAL is an exact 127.0.0.1 compare, which is the disagreement this table exists to
# catch.
ADDRESSES = [
    "127.0.0.1",
    "127.0.0.2",
    "127.0.1.1",
    "127.255.255.254",
    "::1",
    "192.0.2.1",
    "10.0.0.1",
    "12.7.0.1",
    "1270.0.0.1",
    "::1:1",
    "fe80::1",
]


def cpp_loopback_verdicts(executable):
    """``{address: is_loopback}`` as C++ sees it (the ``loopback_table`` subcommand)."""
    finished = subprocess.run(
        [executable, "loopback_table", *ADDRESSES],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=SUBPROCESS_TIMEOUT_SEC,
        check=False,
    )
    text = finished.stdout.decode("utf-8", "replace")
    if finished.returncode != 0:
        print(text)
        raise RuntimeError(f"{executable} loopback_table exited with {finished.returncode}")
    verdicts = {}
    for line in text.splitlines():
        parts = line.split("|")
        if len(parts) != 2 or parts[1] not in ("0", "1"):
            continue
        verdicts[parts[0]] = parts[1] == "1"
    return verdicts


def python_verdicts():
    """The same, as python/network_recovery.py sees it."""
    is_kind = (
        network_recovery._is_vpn_interface_kind
        if sys.platform.startswith("linux")
        # Only Linux has an interface kind; the C++ side reports 0 off Linux for the
        # same reason, so the table stays comparable on every platform.
        else (lambda _name: False)
    )
    return {
        name: (
            network_recovery._is_vpn_interface_name(name),
            network_recovery._is_vpn_product_name(name),
            network_recovery._is_vpn_description(name),
            is_kind(name),
        )
        for name in NAMES
    }


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <path/to/vpn_interfaces_test>", file=sys.stderr)
        return 1

    from_cpp = cpp_verdicts(sys.argv[1])
    from_python = python_verdicts()

    loopback_cpp = cpp_loopback_verdicts(sys.argv[1])
    expect(
        len(loopback_cpp) == len(ADDRESSES),
        f"C++ answered for {len(loopback_cpp)} of {len(ADDRESSES)} addresses -- the "
        f"loopback_table subcommand drifted",
    )
    for address in ADDRESSES:
        expect(
            loopback_cpp.get(address) == network_recovery._is_loopback_address(address),
            f"is_loopback_address({address!r}): C++ {loopback_cpp.get(address)!r} vs "
            f"Python {network_recovery._is_loopback_address(address)!r}",
        )

    expect(
        len(from_cpp) == len(NAMES),
        f"C++ answered for {len(from_cpp)} of {len(NAMES)} names -- the table format or "
        f"the classifier_table subcommand drifted",
    )

    columns = (
        "is_vpn_interface_name",
        "is_vpn_product_name",
        "is_vpn_description",
        "is_vpn_interface_kind",
    )
    for name in NAMES:
        if name not in from_cpp:
            expect(False, f"C++ reported no verdict for {name!r}")
            continue
        for column, cpp_value, python_value in zip(
            columns, from_cpp[name], from_python[name]
        ):
            expect(
                cpp_value == python_value,
                f"{column}({name!r}): C++ says {cpp_value}, Python says {python_value}",
            )

    # Guards against the whole comparison passing because both sides answered "no" to
    # everything -- a classifier that classified nothing would otherwise be in perfect
    # parity with one that did.
    expect(
        sum(any(verdict) for verdict in from_python.values()) >= 20,
        "fewer than 20 names classified as VPN-ish at all -- the table no longer "
        "exercises the classifier",
    )

    if _failures:
        print(
            f"vpn_classifier_parity_test: FAILED ({len(_failures)} disagreement(s) over "
            f"{len(NAMES)} name(s))"
        )
        return 1
    print(
        f"vpn_classifier_parity_test: C++ and Python agree on all {len(NAMES)} name(s)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
