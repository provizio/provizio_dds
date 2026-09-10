# shellcheck shell=bash
# (A `shell` directive rather than a shebang: this file is sourced, never executed.)
#
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

# Reclaims what a KILLED run of the privileged network-recovery tests left behind.
# Sourced by carrier_recovery_test.sh and cold_start_hosts_test.sh; not executable on its
# own. Expects $SUDO to be set by the caller (empty when already root).
#
# Why this exists: both scripts clean up through `trap ... EXIT INT TERM`, and CTest kills a
# timed-out test with SIGKILL, which no handler can catch (test/CMakeLists.txt says so, and
# it was verified there). A cancelled CI run can escalate the same way. What survives such a
# kill is a named network namespace -- a root-owned bind mount under /run/netns -- plus, in
# one narrow window, a veth pair still in the host namespace. On a throw-away GitHub runner
# that is irrelevant; on the SELF-HOSTED jetson-20.04 runner in the same test matrix it
# accumulates, one set per killed run, until someone reboots the box.
#
# Reaping is by name prefix AND by liveness of the pid the name carries (see
# provizio_pid_is_alive for why that takes two checks, not one). Deleting every
# matching namespace would be wrong: test/run_parallel.py runs cases concurrently and several
# CI jobs share the self-hosted runner, so a sibling's namespace can legitimately exist right
# now. A namespace whose pid is gone belonged to a run that is over. Pid reuse can only make
# this MISS a stale namespace (some unrelated live process now owns that number), never make
# it delete a live one -- the safe direction.

# Whether a pid is still running. Deliberately answers "alive" unless BOTH checks say
# otherwise, because the cost of the two mistakes is not symmetric: believing a dead run
# alive leaks one namespace until the next run, while believing a LIVE run dead deletes a
# namespace out from under it.
#
# Neither check alone is enough. `kill -0` runs as the INVOKING user even when the deletes go
# through `sudo -n`, and it cannot distinguish "no such process" from "alive but owned by
# someone else" (EPERM) -- so on its own it reports a foreign-owned live process as dead.
# /proc/<pid> answers regardless of owner, but a /proc mounted with hidepid hides other
# users' entries. Requiring both to say "gone" makes a wrong deletion need both to be wrong.
provizio_pid_is_alive() {
    [ -d "/proc/$1" ] && return 0
    kill -0 "$1" 2> /dev/null && return 0
    return 1
}

# Deletes stale namespaces and host-namespace veths belonging to dead runs of this suite.
# Never touches anything named for a live pid, and never anything outside this suite's own
# prefixes. Silent: a failure here is not a test failure.
provizio_reap_stale_test_netns() {
    provizio_reap_stale_netns_by_prefix "provizio_dds_carrier_test_"
    provizio_reap_stale_netns_by_prefix "provizio_dds_cold_a_"
    provizio_reap_stale_netns_by_prefix "provizio_dds_cold_b_"
    provizio_reap_stale_host_veths
}

provizio_reap_stale_netns_by_prefix() {
    _prefix="$1"
    # `ip netns list` prints "name (id: N)" or just "name"; take the first field.
    $SUDO ip netns list 2> /dev/null | awk '{print $1}' | while IFS= read -r _ns; do
        case "$_ns" in
            "$_prefix"*) ;;
            *) continue ;;
        esac
        _pid="${_ns#"$_prefix"}"
        # Only a pure decimal suffix is ours; anything else is a name we did not make.
        # The digits are enumerated rather than written as the range [!0-9], because a range
        # is collation-dependent: under en_US.UTF-8 it also admits non-ASCII digit-likes
        # (U+0664, U+FF15, U+2464), which would then reach `ip netns del` as a name this
        # suite never creates.
        case "$_pid" in
            '' | *[!0123456789]*) continue ;;
        esac
        if provizio_pid_is_alive "$_pid"; then
            continue # A run with that pid is still alive -- possibly a sibling job's.
        fi
        $SUDO ip netns del "$_ns" > /dev/null 2>&1 || true
    done
}

provizio_reap_stale_host_veths() {
    # Only the cold-start case leaves these, and only if killed between creating the pair and
    # moving its ends into the namespaces. Same pid-liveness rule.
    $SUDO ip -o link show 2> /dev/null | awk -F': ' '{print $2}' | cut -d'@' -f1 |
        while IFS= read -r _link; do
            case "$_link" in
                pvzc0* | pvzc1*) ;;
                *) continue ;;
            esac
            _pid="${_link#pvzc?}"
            # Enumerated digits, not a range -- see the namespace loop above.
            case "$_pid" in
                '' | *[!0123456789]*) continue ;;
            esac
            if provizio_pid_is_alive "$_pid"; then
                continue
            fi
            $SUDO ip link del "$_link" > /dev/null 2>&1 || true
        done
}
