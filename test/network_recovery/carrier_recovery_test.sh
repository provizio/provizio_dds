#!/usr/bin/env bash
# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#
# End-to-end regression test for network auto-recovery against a REAL kernel
# interface transition, rather than an injected event. Everything happens inside
# a throw-away network namespace, so the host's networking is untouched.
#
# Two things are proven, each of which used to leave a participant permanently
# unable to communicate:
#
#   Phase 1 (link-state trigger). An interface that already holds its address is
#     brought up. With IPv6 disabled on it, the kernel emits ONLY RTM_NEWLINK —
#     no address event whatsoever. A monitor subscribed to the address groups
#     alone (which is what provizio_dds used to do) never even wakes up.
#
#   Phase 2 (carrier vs. administrative state). A bond with no members is
#     administratively up and holds an address, but has no carrier. Fast-DDS'
#     IPFinder::getIPs skips it (it filters on IFF_RUNNING), so a participant
#     created now binds no locator to it. Enslaving a live member gives it
#     carrier WITHOUT touching the address or the administrative flag: a snapshot
#     keyed on IFF_UP (which is what provizio_dds used to do) is byte-identical
#     across the transition and no rebuild is triggered — the exact shape of the
#     "no DDS messages after the network switch was powered on late" report.
#     Skipped when the bonding driver is unavailable.
#
# Both phases run with PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0, so only the
# kernel event path can satisfy them. Otherwise the periodic safety-net check would
# eventually notice either transition on its own and mask a regression in the event
# subscription. (The safety net has its own dedicated tests.)
#
# Exits 77 (ctest SKIP_RETURN_CODE) when it cannot set up a namespace.
#
# SECURITY: this script executes its first argument under sudo. That is safe only
# because the caller must already hold root or a passwordless-sudo grant. Do NOT add
# this script to a sudoers NOPASSWD rule — doing so would turn it into a one-argument
# root shell for anyone permitted to run it.

set -u -o pipefail

TEST_BINARY="${1:?usage: carrier_recovery_test.sh <path to network_recovery_test>}"
NS="provizio_dds_carrier_test_$$"
IFACE_LINK=provizio0
IFACE_BOND=bond0
BOND_MEMBER=provizio1
ADDR_LINK=203.0.113.5     # TEST-NET-3, RFC 5737 — never routable
ADDR_BOND=203.0.113.6
AWAIT_TIMEOUT=40

# Run privileged helpers directly when root, else via non-interactive sudo.
if [ "$(id -u)" = "0" ]; then
    SUDO=""
elif command -v sudo > /dev/null 2>&1 && sudo -n true > /dev/null 2>&1; then
    SUDO="sudo -n"
else
    echo "SKIP: needs root or passwordless sudo to create a network namespace"
    exit 77
fi

if ! command -v ip > /dev/null 2>&1; then
    echo "SKIP: iproute2 (ip) not available"
    exit 77
fi

created_ns=0
await_pid=""
out_file=""

cleanup() {
    # Kill the in-namespace child first: it runs as root inside the namespace for up to
    # AWAIT_TIMEOUT seconds, and deleting the namespace from under it would leave it
    # running in an anonymous one until it exits.
    if [ -n "$await_pid" ]; then
        kill "$await_pid" 2> /dev/null || true
        wait "$await_pid" 2> /dev/null || true
    fi
    [ -n "$out_file" ] && rm -f "$out_file"
    # Only ever delete a namespace this run created. $$ is reused across runs, so an
    # unconditional delete here could destroy a same-named namespace left behind by an
    # earlier aborted run — including on the SKIP path, where we created nothing.
    [ "$created_ns" = "1" ] && $SUDO ip netns del "$NS" > /dev/null 2>&1
    return 0
}
trap cleanup EXIT INT TERM

if ! $SUDO ip netns add "$NS" > /dev/null 2>&1; then
    echo "SKIP: cannot create a network namespace (unprivileged container?)"
    exit 77
fi
created_ns=1

# Everything below runs inside the namespace. The test binary too — so the
# interfaces it enumerates are only the ones this script creates.
nsx() { $SUDO ip netns exec "$NS" "$@"; }

# Disabling IPv6 on a test interface is LOAD-BEARING, not cosmetic. With IPv6 live, the very
# transition under test (a link coming up in phase 1, carrier arriving in phase 2) also makes the
# kernel add an fe80:: address — and that is an ADDRESS event, which would wake the monitor even if
# the link-state subscription were broken. The test would then pass against the pre-fix code and
# prove nothing. So verify the knob actually took effect, and skip rather than pretend.
#
# Returns 0 when no fe80:: can appear (IPv6 disabled, or absent from this kernel entirely),
# 1 when it could and the caller must skip.
require_no_ipv6() {
    _rni_iface="$1"
    _rni_knob="/proc/sys/net/ipv6/conf/${_rni_iface}/disable_ipv6"
    # Read via cat rather than `test -e`: it needs no external test(1) and tells us in one step
    # both whether the knob exists and what it says.
    if ! nsx cat "$_rni_knob" > /dev/null 2>&1; then
        # No IPv6 support in this kernel at all, so nothing can add an fe80:: address.
        return 0
    fi
    nsx sysctl -q -w "net.ipv6.conf.${_rni_iface}.disable_ipv6=1" > /dev/null 2>&1 || true
    _rni_value="$(nsx cat "$_rni_knob" 2>/dev/null || echo "")"
    if [ "$_rni_value" = "1" ]; then
        return 0
    fi
    echo "SKIP: cannot disable IPv6 on ${_rni_iface} (disable_ipv6='${_rni_value}'). With IPv6 live,"
    echo "      the transition under test also adds an fe80:: address, whose ADDRESS event would wake"
    echo "      the monitor even with the link-state subscription broken — this phase would pass"
    echo "      regardless of the fix and prove nothing."
    return 1
}

# Asserts, after the fact, that the interface really never gained an IPv6 address, so the reset the
# phase observed cannot have been triggered by an fe80:: address event.
assert_no_ipv6_address() {
    if nsx ip -6 addr show dev "$1" 2>/dev/null | grep -q "inet6"; then
        echo "FAIL: $1 gained an IPv6 address, so this phase cannot distinguish a link-state event"
        echo "      from an address event — treating it as a failure rather than a false pass"
        return 1
    fi
    return 0
}

failures=0

# --------------------------------------------------------------------------
# Phase 1: link-state-only transition (no address event at all)
# --------------------------------------------------------------------------
echo "=== Phase 1: interface brought up with its address already assigned ==="

# A dummy device is universally available and, unlike a veth, needs no peer. Its
# kind is on the snapshot's exclusion list, so force-include it by name — which
# also exercises PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES end to end.
if ! nsx ip link add "$IFACE_LINK" type dummy > /dev/null 2>&1; then
    echo "SKIP: cannot create a dummy interface (dummy module unavailable)"
    exit 77
fi

# See require_no_ipv6: without this the phase cannot tell a link event from an address event.
require_no_ipv6 "$IFACE_LINK" || exit 77
nsx ip addr add "${ADDR_LINK}/24" dev "$IFACE_LINK"

# Address assigned while the link is down: not operationally up, so not in the
# snapshot and not bound by Fast-DDS.
before="$(nsx env PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES="$IFACE_LINK" \
    "$TEST_BINARY" print_snapshot)"
echo "snapshot while down: [${before//$'\n'/, }]"
if echo "$before" | grep -q "$IFACE_LINK"; then
    echo "FAIL: $IFACE_LINK is in the snapshot while it is not operationally up"
    failures=$((failures + 1))
fi

# Arm the participant, then bring the link up underneath it.
out_file="$(mktemp)"
nsx env PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES="$IFACE_LINK" \
    PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0 \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    "$TEST_BINARY" await_reset "$AWAIT_TIMEOUT" > "$out_file" 2>&1 &
await_pid=$!

# Wait for "armed" so the transition cannot land before the monitor is watching.
for _ in $(seq 1 100); do
    grep -q "await_reset: armed" "$out_file" 2>/dev/null && break
    sleep 0.1
done

nsx ip link set "$IFACE_LINK" up
wait "$await_pid"
await_status=$?
await_pid=""
sed 's/^/  /' "$out_file"
rm -f "$out_file"
out_file=""

if [ "$await_status" -ne 0 ]; then
    echo "FAIL: bringing $IFACE_LINK up triggered no participant reset"
    failures=$((failures + 1))
fi

after="$(nsx env PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES="$IFACE_LINK" \
    "$TEST_BINARY" print_snapshot)"
echo "snapshot while up: [${after//$'\n'/, }]"
if ! echo "$after" | grep -q "${IFACE_LINK} ${ADDR_LINK}/24"; then
    echo "FAIL: $IFACE_LINK ${ADDR_LINK}/24 missing from the snapshot once operationally up"
    failures=$((failures + 1))
fi
assert_no_ipv6_address "$IFACE_LINK" || failures=$((failures + 1))

# --------------------------------------------------------------------------
# Phase 2: carrier-only transition (administrative state and address unchanged)
# --------------------------------------------------------------------------
echo "=== Phase 2: carrier appearing on an already-up, already-addressed interface ==="

if ! nsx ip link add "$IFACE_BOND" type bond > /dev/null 2>&1; then
    echo "SKIP(phase 2): bonding driver unavailable; carrier-only case not covered here"
    if [ "$failures" -eq 0 ]; then
        echo "carrier_recovery: PASS (phase 1 only)"
        exit 0
    fi
    echo "carrier_recovery: FAIL ($failures failure(s))"
    exit 1
fi

if ! require_no_ipv6 "$IFACE_BOND"; then
    echo "SKIP(phase 2): carrier-only case not covered here (see above)"
    if [ "$failures" -eq 0 ]; then
        echo "carrier_recovery: PASS (phase 1 only)"
        exit 0
    fi
    echo "carrier_recovery: FAIL ($failures failure(s))"
    exit 1
fi
nsx ip addr add "${ADDR_BOND}/24" dev "$IFACE_BOND"
nsx ip link set "$IFACE_BOND" up
nsx ip link add "$BOND_MEMBER" type dummy

# A memberless bond is IFF_UP but has no carrier. Confirm that from the kernel's
# own flags, so a future change to how the snapshot is built cannot make this
# phase silently vacuous.
flags="$(nsx cat "/sys/class/net/${IFACE_BOND}/flags")"
if (((flags & 0x1) == 0)) || (((flags & 0x40) != 0)); then
    echo "SKIP(phase 2): $IFACE_BOND is not in the expected up-without-carrier state (flags=$flags)"
else
    before_bond="$(nsx "$TEST_BINARY" print_snapshot)"
    echo "snapshot without carrier: [${before_bond//$'\n'/, }]"
    if echo "$before_bond" | grep -q "$IFACE_BOND"; then
        echo "FAIL: $IFACE_BOND is in the snapshot while it has no carrier"
        failures=$((failures + 1))
    fi

    out_file="$(mktemp)"
    nsx env PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
        "$TEST_BINARY" await_reset "$AWAIT_TIMEOUT" > "$out_file" 2>&1 &
    await_pid=$!
    for _ in $(seq 1 100); do
        grep -q "await_reset: armed" "$out_file" 2>/dev/null && break
        sleep 0.1
    done

    # Carrier appears. The address and IFF_UP are untouched.
    nsx ip link set "$BOND_MEMBER" master "$IFACE_BOND"
    nsx ip link set "$BOND_MEMBER" up

    wait "$await_pid"
    await_status=$?
    await_pid=""
    sed 's/^/  /' "$out_file"
    rm -f "$out_file"
    out_file=""

    if [ "$await_status" -ne 0 ]; then
        echo "FAIL: carrier appearing on $IFACE_BOND triggered no participant reset"
        failures=$((failures + 1))
    fi

    after_bond="$(nsx "$TEST_BINARY" print_snapshot)"
    echo "snapshot with carrier: [${after_bond//$'\n'/, }]"
    if ! echo "$after_bond" | grep -q "${IFACE_BOND} ${ADDR_BOND}/24"; then
        echo "FAIL: $IFACE_BOND ${ADDR_BOND}/24 missing from the snapshot once it has carrier"
        failures=$((failures + 1))
    fi
    assert_no_ipv6_address "$IFACE_BOND" || failures=$((failures + 1))
fi

if [ "$failures" -eq 0 ]; then
    echo "carrier_recovery: PASS"
    exit 0
fi
echo "carrier_recovery: FAIL ($failures failure(s))"
exit 1
