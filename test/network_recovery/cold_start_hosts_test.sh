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
# End-to-end regression test for two hosts that both start DDS before their network
# has carrier -- the shape of a fleet whose services come up at boot, seconds before
# the network switch does -- and are then rebuilt by network auto-recovery once it
# arrives. The two rebuilt participants must be able to talk to each other.
#
# They could not. Fast-DDS derives a process-wide "host id" once, at first use, from the
# IPv4 addresses of the interfaces that have carrier at that moment, and falls back to
# one fixed value when there are none. Every process started before the link came up, on
# every host, therefore carried the same id -- and shared-memory locators embed it, so
# each such host took the other's shared-memory locators for its own, sent the other's
# traffic into its own segment, and never heard from it. A network-recovery rebuild
# creates a new participant but cannot change that id, so the pair stayed deaf to each
# other for the life of both processes, while a freshly started process (whose id was
# right) talked to either of them at once. The fix derives the id from the machine id
# where no interface has carrier (see cmake/fast_dds/host_id_without_interfaces.cmake).
#
# Two hosts are emulated on one machine: a network namespace each, joined by a veth pair,
# and for each peer process a private mount namespace in which /dev/shm is its own tmpfs
# (its own shared-memory "host") and /etc/machine-id is its own identity. The veth kind is
# on the snapshot's exclusion list, so PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES
# names the interfaces explicitly, exactly as carrier_recovery_test.sh does for its dummy.
#
#   Control run: both interfaces up before either peer starts. Proves the two namespaces
#     can exchange samples at all, so a failure of the cold run cannot be the harness.
#   Cold run: both interfaces down while the peers start, then brought up together. Each
#     peer prints the host id its participant was given, and the subscriber reports
#     whether a sample from the other host arrived after the rebuild.
#
# Exits 77 (ctest SKIP_RETURN_CODE) when it cannot set up the namespaces.
#
# SECURITY: this script executes its first argument under sudo. That is safe only
# because the caller must already hold root or a passwordless-sudo grant. Do NOT add
# this script to a sudoers NOPASSWD rule -- doing so would turn it into a one-argument
# root shell for anyone permitted to run it.

set -u -o pipefail

TEST_BINARY="${1:?usage: cold_start_hosts_test.sh <path to network_recovery_test>}"
TEST_BINARY="$(readlink -f "$TEST_BINARY")"
TAG="$$"
NS_A="provizio_dds_cold_a_${TAG}"
NS_B="provizio_dds_cold_b_${TAG}"
# Neither name is on the snapshot's name block-list; the kind (veth) is what excludes them,
# and PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES re-admits them by name. Suffixed with
# the pid because the pair is created in the HOST namespace before its ends are moved, so
# two concurrent runs must not both ask for the same names (IFNAMSIZ leaves room for a
# seven-digit pid).
IF_A="pvzc0${TAG}"
IF_B="pvzc1${TAG}"
ADDR_A=203.0.113.21     # TEST-NET-3, RFC 5737 -- never routable
ADDR_B=203.0.113.22
# What Fast-DDS reads from /etc/machine-id: exactly 32 characters. Two distinct
# identities, one per emulated host.
MACHINE_ID_A=0123456789abcdef0123456789abcdef
MACHINE_ID_B=fedcba9876543210fedcba9876543210
# Fast-DDS' no-interface fallback host id, 0x017F, as the participant GUID prefix shows it
# (little-endian, bytes 2 and 3).
FALLBACK_HOST_ID=7f01
# Scaled like the ctest TIMEOUT around this script (provizio_dds_finalize_tests exports the
# factor): under a sanitizer everything is several times slower, and a budget that stays put
# would report a slow runner as a host-id regression.
SCALE="${PROVIZIO_DDS_TEST_TIMEOUT_SCALE:-1}"
WARM_TIMEOUT=$((30 * SCALE))
COLD_TIMEOUT=$((60 * SCALE))
ARM_TIMEOUT=$((30 * SCALE))

# Run privileged helpers directly when root, else via non-interactive sudo.
if [ "$(id -u)" = "0" ]; then
    SUDO=""
elif command -v sudo > /dev/null 2>&1 && sudo -n true > /dev/null 2>&1; then
    SUDO="sudo -n"
else
    echo "SKIP: needs root or passwordless sudo to create network namespaces"
    exit 77
fi

for tool in ip unshare mount; do
    if ! command -v "$tool" > /dev/null 2>&1; then
        echo "SKIP: $tool not available"
        exit 77
    fi
done

# Each peer gets its identity by bind-mounting a file over /etc/machine-id, which needs a
# mount point to exist. A minimal container may have none; that is an environment the test
# cannot run in, not a failure of the code under test.
if [ ! -f /etc/machine-id ]; then
    echo "SKIP: /etc/machine-id does not exist on this host, so a per-host identity cannot be bind-mounted"
    exit 77
fi

created_a=0
created_b=0
# Set while the veth pair still sits in the host namespace (created, not yet moved): only
# then does cleanup have to delete it, since afterwards it goes with the namespaces.
veth_in_host=0
peer_pids=""
tmpdir="$(mktemp -d)"

cleanup() {
    # Kill the in-namespace children first: they run as root inside the namespaces for up to
    # COLD_TIMEOUT seconds, and deleting a namespace from under one would leave it running in
    # an anonymous namespace until it exits.
    for pid in $peer_pids; do
        # $pid is this shell's own background subshell (start_peer backgrounds the nsa/nsb
        # FUNCTION, so the job runs as this user); the peer beneath it is root's, launched
        # through sudo. A plain kill would end the subshell only and orphan the peer until
        # its own timeout ran out, so reach the peer first, through sudo, as the subshell's
        # child: sudo relays the signal and exits once the peer has.
        $SUDO pkill -TERM -P "$pid" 2> /dev/null || true
        kill "$pid" 2> /dev/null || true
        wait "$pid" 2> /dev/null || true
    done
    [ "$veth_in_host" = "1" ] && $SUDO ip link del "$IF_A" > /dev/null 2>&1
    # Only ever delete namespaces this run created (see carrier_recovery_test.sh for why).
    [ "$created_a" = "1" ] && $SUDO ip netns del "$NS_A" > /dev/null 2>&1
    [ "$created_b" = "1" ] && $SUDO ip netns del "$NS_B" > /dev/null 2>&1
    rm -rf "$tmpdir"
    return 0
}
trap cleanup EXIT INT TERM

if ! $SUDO ip netns add "$NS_A" > /dev/null 2>&1; then
    echo "SKIP: cannot create a network namespace (unprivileged container?)"
    exit 77
fi
created_a=1
if ! $SUDO ip netns add "$NS_B" > /dev/null 2>&1; then
    echo "SKIP: cannot create a second network namespace"
    exit 77
fi
created_b=1

nsa() { $SUDO ip netns exec "$NS_A" "$@"; }
nsb() { $SUDO ip netns exec "$NS_B" "$@"; }

# The veth pair is created in the host namespace and each end moved into its namespace;
# it stays administratively down until a phase brings it up.
if ! $SUDO ip link add "$IF_A" type veth peer name "$IF_B" > /dev/null 2>&1; then
    echo "SKIP: cannot create a veth pair"
    exit 77
fi
veth_in_host=1
if ! $SUDO ip link set "$IF_A" netns "$NS_A" || ! $SUDO ip link set "$IF_B" netns "$NS_B"; then
    # cleanup removes the pair from the host namespace: not moved, it would not go with
    # the namespaces.
    echo "SKIP: cannot move the veth ends into the namespaces"
    exit 77
fi
veth_in_host=0
nsa ip link set lo up
nsb ip link set lo up
nsa ip addr add "${ADDR_A}/24" dev "$IF_A"
nsb ip addr add "${ADDR_B}/24" dev "$IF_B"

mid_a="$tmpdir/machine-id-a"
mid_b="$tmpdir/machine-id-b"
printf '%s\n' "$MACHINE_ID_A" > "$mid_a"
printf '%s\n' "$MACHINE_ID_B" > "$mid_b"
chmod 644 "$mid_a" "$mid_b"
# The peers' working directory: empty, so no DEFAULT_FASTDDS_PROFILES.xml can be picked up.
workdir="$tmpdir/work"
mkdir -p "$workdir"

# start_peer <nsx-function> <iface> <machine-id-file> <pub|sub> <topic> <timeout> <cold|warm> <out-file>
# Starts one peer in the background inside its namespace; its pid is left in
# last_peer_pid (not echoed: a $(...) capture would make it a child of the subshell,
# and this shell could not wait for it).
last_peer_pid=""
start_peer() {
    local nsx="$1" iface="$2" mid="$3" role="$4" topic="$5" timeout="$6" mode="$7" out="$8"
    # unshare -m: a mount namespace of this peer's own, in which /dev/shm is a fresh tmpfs
    # (the emulated host's own shared memory) and /etc/machine-id its own identity.
    # FASTDDS_DEFAULT_PROFILES_FILE is dropped: CI points it at the loopback-only profile,
    # which would confine the participant to 127.0.0.1 and make the veth unreachable.
    "$nsx" unshare -m --propagation private bash -c '
        set -e
        mount -t tmpfs -o size=64m,mode=1777 tmpfs /dev/shm
        mount --bind "$1" /etc/machine-id
        cd "$2"
        exec env -u FASTDDS_DEFAULT_PROFILES_FILE \
            PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES="$3" \
            "$4" cold_start_peer "$5" "$6" "$7" "$8"' \
        _ "$mid" "$workdir" "$iface" "$TEST_BINARY" "$role" "$topic" "$timeout" "$mode" > "$out" 2>&1 &
    last_peer_pid=$!
}

# wait_for_line <file> <pattern> <timeout-seconds> <pid>: polls for a line matching
# <pattern>, giving up early once the process writing the file has exited.
wait_for_line() {
    local file="$1" pattern="$2" timeout="$3" pid="$4"
    local tries=$((timeout * 10))
    for _ in $(seq 1 "$tries"); do
        grep -q -- "$pattern" "$file" 2> /dev/null && return 0
        # $pid is this shell's own subshell around the (root-owned) peer, so this needs no
        # sudo and answers for the peer: the subshell lives exactly as long as sudo does,
        # and sudo as long as the peer.
        kill -0 "$pid" 2> /dev/null || return 1
        sleep 0.1
    done
    return 1
}

# wait_for_carrier <nsx-function> <iface> <0|1>: polls the kernel's operational state.
wait_for_carrier() {
    local nsx="$1" iface="$2" want="$3"
    for _ in $(seq 1 100); do
        local state
        state="$("$nsx" cat "/sys/class/net/${iface}/carrier" 2> /dev/null || echo 0)"
        [ "$state" = "$want" ] && return 0
        sleep 0.1
    done
    return 1
}

host_id_of() {
    sed -n 's/^cold_start_peer: host id \([0-9a-f]*\).*/\1/p' "$1" | head -1
}

show() {
    echo "  --- $1 ---"
    sed 's/^/  /' "$2"
}

failures=0

# --------------------------------------------------------------------------
# Control run: interfaces up first, then the peers -- can the namespaces talk at all?
# --------------------------------------------------------------------------
echo "=== Control run: both interfaces up before the peers start ==="
nsa ip link set "$IF_A" up
nsb ip link set "$IF_B" up
if ! wait_for_carrier nsa "$IF_A" 1 || ! wait_for_carrier nsb "$IF_B" 1; then
    echo "SKIP: the veth pair never gained carrier"
    exit 77
fi

out_pub="$tmpdir/warm_pub.log"
out_sub="$tmpdir/warm_sub.log"
start_peer nsa "$IF_A" "$mid_a" pub "cold_start_warm_${TAG}" "$WARM_TIMEOUT" warm "$out_pub"
pub_pid=$last_peer_pid
start_peer nsb "$IF_B" "$mid_b" sub "cold_start_warm_${TAG}" "$WARM_TIMEOUT" warm "$out_sub"
sub_pid=$last_peer_pid
peer_pids="$pub_pid $sub_pid"
wait "$sub_pid"
sub_status=$?
wait "$pub_pid" 2> /dev/null
peer_pids=""
show "control publisher" "$out_pub"
show "control subscriber" "$out_sub"
if [ "$sub_status" -ne 0 ]; then
    echo "FAIL: the control run exchanged no sample, so the two namespaces cannot talk;"
    echo "      the cold run below cannot be judged"
    echo "cold_start_hosts: FAIL (control run)"
    exit 1
fi
warm_id_a="$(host_id_of "$out_pub")"
warm_id_b="$(host_id_of "$out_sub")"
echo "control host ids: A=${warm_id_a} B=${warm_id_b}"

# --------------------------------------------------------------------------
# Cold run: both peers start with their interface down, then both come up together.
# --------------------------------------------------------------------------
echo "=== Cold run: peers start without carrier, interfaces come up, participants rebuild ==="
nsa ip link set "$IF_A" down
nsb ip link set "$IF_B" down
if ! wait_for_carrier nsa "$IF_A" 0 || ! wait_for_carrier nsb "$IF_B" 0; then
    echo "SKIP: cannot take the veth pair down"
    exit 77
fi

out_pub="$tmpdir/cold_pub.log"
out_sub="$tmpdir/cold_sub.log"
start_peer nsa "$IF_A" "$mid_a" pub "cold_start_cold_${TAG}" "$COLD_TIMEOUT" cold "$out_pub"
pub_pid=$last_peer_pid
start_peer nsb "$IF_B" "$mid_b" sub "cold_start_cold_${TAG}" "$COLD_TIMEOUT" cold "$out_sub"
sub_pid=$last_peer_pid
peer_pids="$pub_pid $sub_pid"

# Both peers must have created their participants -- and seen no interface -- before the
# links come up; otherwise this would be a warm start.
if ! wait_for_line "$out_pub" "cold_start_peer: armed" "$ARM_TIMEOUT" "$pub_pid" ||
    ! wait_for_line "$out_sub" "cold_start_peer: armed" "$ARM_TIMEOUT" "$sub_pid"; then
    show "cold publisher" "$out_pub"
    show "cold subscriber" "$out_sub"
    echo "cold_start_hosts: FAIL (a peer did not arm within ${ARM_TIMEOUT}s)"
    exit 1
fi
for f in "$out_pub" "$out_sub"; do
    if ! grep -q "cold_start_peer: armed (.*, 0 interface address(es) visible)" "$f"; then
        echo "FAIL: a peer saw an interface address while its link was down -- not a cold start"
        failures=$((failures + 1))
    fi
done

nsa ip link set "$IF_A" up
nsb ip link set "$IF_B" up

wait "$sub_pid"
sub_status=$?
wait "$pub_pid" 2> /dev/null
peer_pids=""
show "cold publisher" "$out_pub"
show "cold subscriber" "$out_sub"

cold_id_a="$(host_id_of "$out_pub")"
cold_id_b="$(host_id_of "$out_sub")"
echo "cold host ids: A=${cold_id_a} B=${cold_id_b} (Fast-DDS' no-interface fallback is ${FALLBACK_HOST_ID})"
for id in "$cold_id_a" "$cold_id_b"; do
    if [ -z "$id" ]; then
        echo "FAIL: a peer did not report its host id"
        failures=$((failures + 1))
    elif [ "$id" = "$FALLBACK_HOST_ID" ]; then
        echo "FAIL: a participant created without any interface with carrier was given Fast-DDS'"
        echo "      fixed fallback host id ${FALLBACK_HOST_ID}, which every such process on every host shares"
        failures=$((failures + 1))
    fi
done
if [ -n "$cold_id_a" ] && [ "$cold_id_a" = "$cold_id_b" ]; then
    echo "FAIL: the two hosts were given the same host id (${cold_id_a}); each will take the other's"
    echo "      shared-memory locators for its own and never hear from it"
    failures=$((failures + 1))
fi
if [ "$sub_status" -ne 0 ]; then
    echo "FAIL: after both participants were rebuilt, no sample crossed from one host to the other"
    failures=$((failures + 1))
fi

if [ "$failures" -eq 0 ]; then
    echo "cold_start_hosts: PASS"
    exit 0
fi
echo "cold_start_hosts: FAIL ($failures failure(s))"
exit 1
