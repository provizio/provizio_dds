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
# End-to-end regression test for two processes of ONE host, started before and after
# the host's network came up, the first in transport_mode::localhost_only.
#
# Fast-DDS derives a process-wide "host id" once, at first use, from the IPv4 addresses of the
# interfaces that have carrier at that moment, and puts it in every GUID prefix. The
# discovery filter localhost_only turns on (FILTER_DIFFERENT_HOST) compared those GUID bytes
# to decide whether a remote participant is on this host, so a process started before the
# network came up and one started after it -- the same machine, different ids -- took each
# other for participants of another host and never matched. The fix judges the host by the
# machine id each participant announces instead (see
# cmake/fast_dds/same_host_by_machine_id.cmake).
#
# That is not all a localhost_only participant started before the network came up needs. A peer
# started after it in an ordinary transport (a recorder, say) announces the addresses of the
# interfaces that had come up by then and no loopback one, and the confined participant can reach it
# only over loopback -- which it does by recognising those addresses as this host's own. Fast-DDS
# answered that from the interfaces it saw when the process started, which had none of them, so the
# peer's addresses were taken for another host's and dropped: the two never exchanged a sample
# (cmake/fast_dds/local_address_by_current_interfaces.cmake). Both directions are run.
#
# One network namespace is the host: a dummy interface is down while the first peer of a case
# starts and up, with an address, when the second does. Both see the same /etc/machine-id, as two
# processes of a machine do. The two host ids are printed and asserted to differ -- without that,
# a pass would prove nothing.
#
# Exits 77 (ctest SKIP_RETURN_CODE) when it cannot set up the namespace.
#
# SECURITY: this script executes its first argument under sudo. That is safe only
# because the caller must already hold root or a passwordless-sudo grant. Do NOT add
# this script to a sudoers NOPASSWD rule -- doing so would turn it into a one-argument
# root shell for anyone permitted to run it.

set -u -o pipefail

TEST_BINARY="${1:?usage: same_host_filter_test.sh <path to network_recovery_test>}"
TEST_BINARY="$(readlink -f "$TEST_BINARY")"
TAG="$$"
NS="provizio_dds_samehost_${TAG}"
IFACE="pvzs${TAG}"
ADDR=203.0.113.31 # TEST-NET-3, RFC 5737 -- never routable
# Scaled like the ctest TIMEOUT around this script (provizio_dds_finalize_tests exports the
# factor): under a sanitizer everything is several times slower.
SCALE="${PROVIZIO_DDS_TEST_TIMEOUT_SCALE:-1}"
if ! [[ "$SCALE" =~ ^[1-9][0-9]*$ ]]; then
    echo "PROVIZIO_DDS_TEST_TIMEOUT_SCALE must be a positive integer, got '$SCALE'" >&2
    exit 1
fi
PEER_TIMEOUT=$((60 * SCALE))
ARM_TIMEOUT=$((30 * SCALE))
# How long a peer is given to end once it should have. A publisher ends when it sees its subscriber
# go, which without the subscriber's goodbye takes the 30 s participant lease to expire.
EXIT_GRACE=$((45 * SCALE))

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

# Fast-DDS reads this host's machine id from /etc/machine-id alone. Without one (a minimal container)
# both peers announce none and fall back to the very GUID comparison under test, so the case could only
# fail for a reason that is not the code's.
if [ ! -s /etc/machine-id ]; then
    echo "SKIP: /etc/machine-id is missing or empty, so there is no machine id for the peers to share"
    exit 77
fi

# shellcheck source=test/network_recovery/netns_reaper.sh
. "$(dirname "$0")/netns_reaper.sh"
provizio_reap_stale_test_netns

created=0
peer_pids=""
tmpdir="$(mktemp -d)"

stop_peers() {
    for pid in $peer_pids; do
        # See cold_start_hosts_test.sh: reach the root-owned peer beneath the subshell first.
        $SUDO pkill -TERM -P "$pid" 2> /dev/null || true
        kill "$pid" 2> /dev/null || true
        wait "$pid" 2> /dev/null || true
    done
    peer_pids=""
}

cleanup() {
    stop_peers
    [ "$created" = "1" ] && $SUDO ip netns del "$NS" > /dev/null 2>&1
    rm -rf "$tmpdir"
    return 0
}
# A signal ends the script (and so runs the EXIT trap once) instead of cleaning up and carrying on.
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if ! $SUDO ip netns add "$NS" > /dev/null 2>&1; then
    echo "SKIP: cannot create a network namespace (unprivileged container?)"
    exit 77
fi
created=1
ns() { $SUDO ip netns exec "$NS" "$@"; }

if ! ns ip link set lo up || ! $SUDO ip netns exec "$NS" ip link add "$IFACE" type dummy > /dev/null 2>&1; then
    echo "SKIP: cannot create a dummy interface (kernel module missing?)"
    exit 77
fi
ns ip addr add "${ADDR}/24" dev "$IFACE"

# The peers' working directory: empty, so no DEFAULT_FASTDDS_PROFILES.xml can be picked up.
workdir="$tmpdir/work"
mkdir -p "$workdir"

# start_peer <pub|sub> <localhost_only|automatic> <topic> <out-file>: one peer in the background in
# the namespace; its pid is left in last_peer_pid. FASTDDS_DEFAULT_PROFILES_FILE is dropped: CI points
# it at a profile that would make the library skip the transport configuration under test, as
# FASTDDS_BUILTIN_TRANSPORTS would; both are dropped. PROVIZIO_DDS_LOCALHOST_ONLY is dropped too: it
# would turn the automatic peer into a confined one. The environment is set with unset/export rather
# than env(1), which would take a binary path holding a "=" for an assignment and run the next
# argument instead.
# The dummy kind is on the address snapshot's exclusion list, so PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES
# names the interface, as carrier_recovery_test.sh does: the count of addresses each peer reports seeing
# is what shows the first started cold and the second warm.
# Each peer gets a private mount namespace with its own tmpfs /dev/shm: it runs as root (entering the
# network namespace needs it) with shared memory on, and on the host's own /dev/shm it would leave
# root-owned segments, named by port alone, that later non-root DDS processes of the machine could not
# reopen -- and that nothing sweeps. The peers differ in host id, so they never share shared memory anyway.
last_peer_pid=""
start_peer() {
    local role="$1" transport="$2" topic="$3" out="$4"
    ns unshare -m --propagation private bash -c '
        set -e
        mount -t tmpfs -o size=64m,mode=1777 tmpfs /dev/shm
        cd "$1"
        unset FASTDDS_DEFAULT_PROFILES_FILE FASTDDS_BUILTIN_TRANSPORTS PROVIZIO_DDS_LOCALHOST_ONLY
        export PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES="$2"
        exec "${@:3}"' \
        _ "$workdir" "$IFACE" "$TEST_BINARY" same_host_peer "$role" "$topic" "$PEER_TIMEOUT" "$transport" > "$out" 2>&1 &
    last_peer_pid=$!
}

wait_for_line() {
    local file="$1" pattern="$2" timeout="$3" pid="$4"
    local tries=$((timeout * 10))
    for _ in $(seq 1 "$tries"); do
        grep -q -- "$pattern" "$file" 2> /dev/null && return 0
        kill -0 "$pid" 2> /dev/null || return 1
        sleep 0.1
    done
    return 1
}

# descendants <pid>: every process below <pid>, deepest last
descendants() {
    local child
    for child in $(pgrep -P "$1"); do
        echo "$child"
        descendants "$child"
    done
}

# wait_for_exit <pid> <seconds>: waits for the peer (the subshell running it) to end, and returns its
# status; after <seconds>, stops it and returns 124 instead -- so that a peer hung past its own
# deadline cannot run the script into ctest's TIMEOUT, which would discard every log shown after it.
# TERM goes through the sudo between the subshell and the peer, which relays it, as stop_peers does;
# what is still there after a grace is KILLed by pid, peer included, as sudo cannot relay a KILL.
wait_for_exit() {
    local pid="$1" seconds="$2" stragglers
    local tries=$((seconds * 10))
    for _ in $(seq 1 "$tries"); do
        if ! kill -0 "$pid" 2> /dev/null; then
            wait "$pid"
            return $?
        fi
        sleep 0.1
    done
    $SUDO pkill -TERM -P "$pid" 2> /dev/null || true
    for _ in $(seq 1 50); do
        kill -0 "$pid" 2> /dev/null || break
        sleep 0.1
    done
    stragglers="$(descendants "$pid")"
    # shellcheck disable=SC2086 # one pid per word
    [ -n "$stragglers" ] && $SUDO kill -KILL $stragglers 2> /dev/null
    kill -KILL "$pid" 2> /dev/null || true
    wait "$pid" 2> /dev/null || true
    return 124
}

# check_peer_transport <name> <role> <transport> <log>: what the peer reports it was built with -- the
# mode its request resolved to and the interfaces its socket transports are confined to -- must be
# what the case asked for, and the library must not have reported it unconfined: a localhost_only peer
# left unconfined, or an automatic one confined, would let a case pass without exercising the patches.
check_peer_transport() {
    local name="$1" role="$2" transport="$3" log="$4" expected_allowlist="none"
    [ "$transport" = "localhost_only" ] && expected_allowlist="127.0.0.1"
    if ! grep -q "^same_host_peer: socket allowlist ${expected_allowlist}\$" "$log"; then
        echo "FAIL (${name}): the ${role} (${transport}) does not report socket allowlist ${expected_allowlist}:" \
            "$(grep "socket allowlist" "$log" | head -1)"
        failures=$((failures + 1))
    fi
    if grep -q "is NOT confined to this host" "$log"; then
        echo "FAIL (${name}): the library reported the ${role} (${transport}) as not confined to this host"
        failures=$((failures + 1))
    fi
}

host_id_of() {
    sed -n 's/^same_host_peer: host id \([0-9a-f]*\).*/\1/p' "$1" | head -1
}

show() {
    echo "  --- $1 ---"
    sed 's/^/  /' "$2"
}

failures=0

# run_case <name> <first role> <first transport> <second role> <second transport>: the first peer
# starts while the dummy interface is down, the second once it is up with its address.
run_case() {
    local name="$1" first_role="$2" first_transport="$3" second_role="$4" second_transport="$5"
    local topic="same_host_filter_${TAG}_${name}"
    local out_first="$tmpdir/${name}_first.log" out_second="$tmpdir/${name}_second.log"
    local first_pid second_pid first_status second_status id_first id_second

    echo "=== ${name}: the ${first_role} (${first_transport}) starts with no interface carrying an address ==="
    ns ip link set "$IFACE" down
    start_peer "$first_role" "$first_transport" "$topic" "$out_first"
    first_pid=$last_peer_pid
    peer_pids="$first_pid"
    if ! wait_for_line "$out_first" "same_host_peer: armed" "$ARM_TIMEOUT" "$first_pid"; then
        stop_peers
        show "${name}: ${first_role} (${first_transport})" "$out_first"
        echo "FAIL (${name}): the ${first_role} did not arm within ${ARM_TIMEOUT}s"
        failures=$((failures + 1))
        return
    fi
    if ! grep -q "armed (${first_role}, ${first_transport}, 0 interface address(es) visible)" "$out_first"; then
        echo "FAIL (${name}): the ${first_role} saw an interface address before the link came up -- not a cold start"
        failures=$((failures + 1))
    fi

    echo "=== ${name}: the link comes up, then the ${second_role} (${second_transport}) starts ==="
    ns ip link set "$IFACE" up
    for _ in $(seq 1 100); do
        [ "$(ns cat "/sys/class/net/${IFACE}/carrier" 2> /dev/null || echo 0)" = "1" ] && break
        sleep 0.1
    done
    if [ "$(ns cat "/sys/class/net/${IFACE}/carrier" 2> /dev/null || echo 0)" != "1" ]; then
        # A case that ran before this one and failed must not be reported as a skip
        if [ "$failures" -gt 0 ]; then
            echo "FAIL (${name}): the dummy interface never gained carrier, after ${failures} failure(s) above"
            exit 1
        fi
        echo "SKIP: the dummy interface never gained carrier"
        exit 77
    fi
    start_peer "$second_role" "$second_transport" "$topic" "$out_second"
    second_pid=$last_peer_pid
    peer_pids="$first_pid $second_pid"
    # Each peer ends by its own deadline (PEER_TIMEOUT) at the latest, and the first, once the second has
    # gone, within the lease
    wait_for_exit "$second_pid" $((PEER_TIMEOUT + EXIT_GRACE))
    second_status=$?
    wait_for_exit "$first_pid" "$EXIT_GRACE"
    first_status=$?
    peer_pids=""
    show "${name}: ${first_role} (${first_transport}, started first)" "$out_first"
    show "${name}: ${second_role} (${second_transport}, started second)" "$out_second"

    check_peer_transport "$name" "$first_role" "$first_transport" "$out_first"
    check_peer_transport "$name" "$second_role" "$second_transport" "$out_second"
    # The peer started second must have been warm too, or the case is not the one under test
    if ! grep -q "armed (${second_role}, ${second_transport}, [1-9][0-9]* interface address(es) visible)" "$out_second"; then
        echo "FAIL (${name}): the ${second_role} saw no interface address although the link was up"
        failures=$((failures + 1))
    fi
    id_first="$(host_id_of "$out_first")"
    id_second="$(host_id_of "$out_second")"
    echo "host ids (${name}): first=${id_first} second=${id_second}"
    if [ -z "$id_first" ] || [ -z "$id_second" ]; then
        echo "FAIL (${name}): a peer did not report its host id"
        failures=$((failures + 1))
    elif [ "$id_first" = "$id_second" ]; then
        echo "FAIL (${name}): both peers got the same host id, so the case under test (a process started"
        echo "      before the network came up and one after it) was not produced"
        failures=$((failures + 1))
    fi
    # The subscriber exits 0 on its first sample. The publisher exits 0 once it has seen the subscriber
    # match and then go: a crash, or a run that never matched, must not hide behind the subscriber's verdict.
    if [ "$first_status" -ne 0 ]; then
        echo "FAIL (${name}): the ${first_role} (${first_transport}, started first) exited with status ${first_status}"
        failures=$((failures + 1))
    fi
    if [ "$second_status" -ne 0 ]; then
        echo "FAIL (${name}): the ${second_role} (${second_transport}, started second) exited with status ${second_status}"
        failures=$((failures + 1))
    fi
}

# Both confined: the discovery filter, which took the two for different hosts
run_case localhost_only_both pub localhost_only sub localhost_only
# The confined peer started first meets an ordinary one, which announces no loopback address: its
# samples must reach that peer, and that peer's must reach it
run_case automatic_subscriber pub localhost_only sub automatic
run_case automatic_publisher sub localhost_only pub automatic

if [ "$failures" -eq 0 ]; then
    echo "same_host_filter: PASS"
    exit 0
fi
echo "same_host_filter: FAIL ($failures failure(s))"
exit 1
