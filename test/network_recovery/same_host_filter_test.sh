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
# the host's network came up, in transport_mode::localhost_only.
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
# One network namespace is the host: a dummy interface is down while the first peer (the
# publisher) starts and up, with an address, when the second (the subscriber) does. Both see the
# same /etc/machine-id, as two processes of a machine do. The two host ids are printed and
# asserted to differ -- without that, a pass would prove nothing.
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

cleanup() {
    for pid in $peer_pids; do
        # See cold_start_hosts_test.sh: reach the root-owned peer beneath the subshell first.
        $SUDO pkill -TERM -P "$pid" 2> /dev/null || true
        kill "$pid" 2> /dev/null || true
        wait "$pid" 2> /dev/null || true
    done
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

# start_peer <pub|sub> <topic> <out-file>: one peer in the background in the namespace; its
# pid is left in last_peer_pid. FASTDDS_DEFAULT_PROFILES_FILE is dropped: CI points it at a
# profile that would make the library skip the localhost_only configuration under test.
# Each peer gets a private mount namespace with its own tmpfs /dev/shm: it runs as root (entering the
# network namespace needs it) with shared memory on, and on the host's own /dev/shm it would leave
# root-owned segments, named by port alone, that later non-root DDS processes of the machine could not
# reopen -- and that nothing sweeps. The peers differ in host id, so they never share shared memory anyway.
last_peer_pid=""
start_peer() {
    local role="$1" topic="$2" out="$3"
    ns unshare -m --propagation private bash -c '
        set -e
        mount -t tmpfs -o size=64m,mode=1777 tmpfs /dev/shm
        cd "$1" && shift
        exec env -u FASTDDS_DEFAULT_PROFILES_FILE "$@"' \
        _ "$workdir" "$TEST_BINARY" same_host_peer "$role" "$topic" "$PEER_TIMEOUT" > "$out" 2>&1 &
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

host_id_of() {
    sed -n 's/^same_host_peer: host id \([0-9a-f]*\).*/\1/p' "$1" | head -1
}

show() {
    echo "  --- $1 ---"
    sed 's/^/  /' "$2"
}

failures=0
topic="same_host_filter_${TAG}"
out_pub="$tmpdir/pub.log"
out_sub="$tmpdir/sub.log"

echo "=== Publisher starts with no interface carrying an address ==="
start_peer pub "$topic" "$out_pub"
pub_pid=$last_peer_pid
peer_pids="$pub_pid"
if ! wait_for_line "$out_pub" "same_host_peer: armed" "$ARM_TIMEOUT" "$pub_pid"; then
    show "publisher" "$out_pub"
    echo "same_host_filter: FAIL (the publisher did not arm within ${ARM_TIMEOUT}s)"
    exit 1
fi
if ! grep -q "armed (pub, 0 interface address(es) visible)" "$out_pub"; then
    echo "FAIL: the publisher saw an interface address before the link came up -- not a cold start"
    failures=$((failures + 1))
fi

echo "=== The link comes up, then the subscriber starts ==="
ns ip link set "$IFACE" up
for _ in $(seq 1 100); do
    [ "$(ns cat "/sys/class/net/${IFACE}/carrier" 2> /dev/null || echo 0)" = "1" ] && break
    sleep 0.1
done
if [ "$(ns cat "/sys/class/net/${IFACE}/carrier" 2> /dev/null || echo 0)" != "1" ]; then
    echo "SKIP: the dummy interface never gained carrier"
    exit 77
fi
start_peer sub "$topic" "$out_sub"
sub_pid=$last_peer_pid
peer_pids="$pub_pid $sub_pid"
wait "$sub_pid"
sub_status=$?
wait "$pub_pid"
pub_status=$?
peer_pids=""
show "publisher" "$out_pub"
show "subscriber" "$out_sub"

id_pub="$(host_id_of "$out_pub")"
id_sub="$(host_id_of "$out_sub")"
echo "host ids: publisher=${id_pub} subscriber=${id_sub}"
if [ -z "$id_pub" ] || [ -z "$id_sub" ]; then
    echo "FAIL: a peer did not report its host id"
    failures=$((failures + 1))
elif [ "$id_pub" = "$id_sub" ]; then
    echo "FAIL: both peers got the same host id, so the case under test (a process started before"
    echo "      the network came up and one after it) was not produced"
    failures=$((failures + 1))
fi
if [ "$sub_status" -ne 0 ]; then
    echo "FAIL: two localhost_only processes of one host with different host ids exchanged no sample"
    failures=$((failures + 1))
fi
# The publisher exits 0 once it has seen the subscriber match and then go: a crash, or a run that never
# matched, must not hide behind the subscriber's verdict.
if [ "$pub_status" -ne 0 ]; then
    echo "FAIL: the publisher exited with status ${pub_status}"
    failures=$((failures + 1))
fi

if [ "$failures" -eq 0 ]; then
    echo "same_host_filter: PASS"
    exit 0
fi
echo "same_host_filter: FAIL ($failures failure(s))"
exit 1
