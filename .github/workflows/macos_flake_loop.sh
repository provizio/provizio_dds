#!/bin/bash

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
#
# TEMPORARY investigation driver for .github/workflows/macos-flake-hunt.yml. Runs the test
# suite in a loop until HUNT_LOOP_SECONDS have passed, keeping every iteration's output and
# running two recorders alongside it (see the workflow's header comment). It deliberately
# does NOT stop at the first failure: the point is a failure RATE and as many stall captures
# as the time budget allows.
#
# Environment:
# Argument 1 is the chunk number, so consecutive chunks of one job do not overwrite each
# other's iteration logs. The job runs several chunks with an artifact upload between them:
# a macos-15-intel runner that dies mid-job uploads nothing and its log cannot be fetched
# afterwards either, so everything since the last upload is simply lost.
#
# Environment:
#   HUNT_ARTIFACTS              directory for all output (required)
#   HUNT_LOOP_SECONDS           how long to keep looping, per chunk (default 4200)
#   PROVIZIO_DDS_CTEST_EXCLUDE  ctest -E regex (optional)
#   HUNT_FLAP_UP_SECONDS        when set, inject an interface flap on that cycle (optional)
#   HUNT_FLAP_DOWN_SECONDS      how long the injected address stays away (default 2)
#   HUNT_GHOST_PERIOD_MS        when set, cycle SIGKILLed participants on domain 0 (optional)
#   HUNT_GHOST_PROCESSES        how many at a time (default 2)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd -P)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
BUILD_DIR="${REPO_DIR}/build"
ARTIFACTS="${HUNT_ARTIFACTS:?HUNT_ARTIFACTS must be set}"
LOOP_SECONDS="${HUNT_LOOP_SECONDS:-4200}"
CHUNK="${1:-1}"

mkdir -p "${ARTIFACTS}"

# Same environment the real test.sh builds, minus the legacy-1.10.1 venv: the
# cross-version test is excluded from every hunt shard, and setting that venv up costs
# ~13 minutes of a budget that is better spent on iterations.
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/python_venv.sh"

export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${REPO_DIR}/test/fast_dds_localhost_profile.xml}"
_SAN_DIR="${REPO_DIR}/test/sanitizers"
export ASAN_OPTIONS="halt_on_error=1:detect_leaks=0:detect_odr_violation=0:detect_stack_use_after_return=1:${ASAN_OPTIONS:-}"
export LSAN_OPTIONS="suppressions=${_SAN_DIR}/lsan.supp:${LSAN_OPTIONS:-}"
export UBSAN_OPTIONS="halt_on_error=0:print_stacktrace=1:suppressions=${_SAN_DIR}/ubsan.supp:${UBSAN_OPTIONS:-}"

CTEST_EXTRA_ARGS=()
if [[ -n "${PROVIZIO_DDS_CTEST_EXCLUDE:-}" ]]; then
    CTEST_EXTRA_ARGS+=(-E "${PROVIZIO_DDS_CTEST_EXCLUDE}")
fi

# The two recorders. Both are read-only observers; neither can change a verdict.
# PYTHONPATH is set for the net watcher alone -- it wants the working tree's
# network_recovery module (the snapshot policy under test), while the test suite must keep
# resolving whatever the venv installed.
PYTHONPATH="${REPO_DIR}/python${PYTHONPATH:+:${PYTHONPATH}}" \
python3 -u "${REPO_DIR}/test/ci_net_watch.py" --out "${ARTIFACTS}" --poll 0.1 --tag "c${CHUNK}" \
    > "${ARTIFACTS}/net_watch.c${CHUNK}.stdout.txt" 2>&1 &
NET_WATCH_PID=$!

# macOS's own account of why an interface changed. configd runs DHCP and applies every
# interface reconfiguration, and mDNSResponder reacts to them, so this is what turns "the
# address vanished" into "the DHCP lease was renewed at 01:23:45".
CONFIGD_LOG_PID=""
if command -v log > /dev/null 2>&1; then
    log stream --style compact \
        --predicate 'process == "configd" OR process == "mDNSResponder" OR process == "networkd"' \
        > "${ARTIFACTS}/system_network_log.c${CHUNK}.txt" 2>&1 &
    CONFIGD_LOG_PID=$!
fi
# Optional injected interface flap (see the shard matrix). Nothing else in this script
# depends on it: without it the shard simply waits for the host's own churn.
FLAP_PID=""
if [[ -n "${HUNT_FLAP_UP_SECONDS:-}" ]]; then
    "${REPO_DIR}/test/ci_interface_flap.sh" "${HUNT_FLAP_UP_SECONDS}" "${HUNT_FLAP_DOWN_SECONDS:-2}" \
        "${ARTIFACTS}/interface_flap.c${CHUNK}.txt" &
    FLAP_PID=$!
fi

# Optional ghost participants: the accelerator for the Fast-DDS lease-reaping deadlock.
GHOST_PID=""
if [[ -n "${HUNT_GHOST_PERIOD_MS:-}" ]]; then
    "${REPO_DIR}/test/ci_ghost_participants.sh" "${HUNT_GHOST_PERIOD_MS}" "${HUNT_GHOST_PROCESSES:-2}" \
        "${BUILD_DIR}/test/simplest_pub_sub/simplest_subscriber/simplest_subscriber" \
        "${ARTIFACTS}/ghost_participants.c${CHUNK}.txt" &
    GHOST_PID=$!
fi

python3 -u "${REPO_DIR}/test/ci_stall_watchdog.py" --build-dir "${BUILD_DIR}" --out "${ARTIFACTS}" \
    --margin 8 --poll 1 > "${ARTIFACTS}/stall_watchdog.c${CHUNK}.stdout.txt" 2>&1 &
WATCHDOG_PID=$!

cleanup() {
    kill "${NET_WATCH_PID}" "${WATCHDOG_PID}" ${CONFIGD_LOG_PID} ${FLAP_PID} ${GHOST_PID} 2>/dev/null
    wait "${NET_WATCH_PID}" "${WATCHDOG_PID}" ${CONFIGD_LOG_PID} ${FLAP_PID} ${GHOST_PID} 2>/dev/null
    pkill -9 -f "simplest_subscriber" 2>/dev/null
}
trap cleanup EXIT

# ctest's own output carries no clock, so a stall capture and an interface event cannot be
# lined up with the test that was running. Prefix every line with one.
stamp() {
    python3 -u -c 'import sys, time
for line in sys.stdin:
    sys.stdout.write(time.strftime("%H:%M:%S ") + line)'
}

cd "${BUILD_DIR}" || exit 1

DEADLINE=$(( $(date +%s) + LOOP_SECONDS ))
ITERATION=0
FAILURES=0
echo "hunt: chunk ${CHUNK}, looping for ${LOOP_SECONDS}s, excluding '${PROVIZIO_DDS_CTEST_EXCLUDE:-<nothing>}'"

while [ "$(date +%s)" -lt "${DEADLINE}" ]; do
    ITERATION=$(( ITERATION + 1 ))
    LOG="${ARTIFACTS}/ctest.c${CHUNK}.$(printf '%03d' "${ITERATION}").log"
    echo "chunk ${CHUNK} iteration ${ITERATION} started $(date -u +%H:%M:%S)Z"
    # -V, not --output-on-failure: the library logs a network change (and therefore a
    # participant rebuild) into the output of whatever test was running, and
    # --output-on-failure throws that away for every test that passed -- which is most of
    # them, and exactly the ones needed to tell how often a rebuild happens at all. The
    # full output goes to the artifact; only the summary lines reach the job log, which
    # could not hold the rest.
    ctest -V "${CTEST_EXTRA_ARGS[@]}" 2>&1 | stamp > "${LOG}"
    STATUS=${PIPESTATUS[0]}
    grep -E "Test +#[0-9]+:|tests passed|The following tests FAILED|\*\*\*|network change detected|network snapshot diff" "${LOG}" \
        | grep -vE "Test +#[0-9]+: .* Passed" | head -60
    if [ "${STATUS}" -eq 0 ]; then
        echo "hunt: iteration ${ITERATION} PASS"
    else
        FAILURES=$(( FAILURES + 1 ))
        echo "::error::hunt: iteration ${ITERATION} FAILED (ctest exit ${STATUS})"
        grep -E '\*\*\*|The following tests FAILED|tests passed' "${LOG}" | tail -20
        echo "--- output of the failing test(s) ---"
        grep -E "^[0-9:]+ [0-9]+: " "${LOG}" | tail -120
        {
            echo "=== after failing iteration ${ITERATION} at $(date -u +%H:%M:%SZ) ==="
            ifconfig -a
            echo "--- netstat -an (udp) ---"
            netstat -an -p udp 2>/dev/null | head -60
            echo "--- surviving processes ---"
            ps -axww -o pid=,ppid=,etime=,state=,command= | grep -E "provizio|/build/test/" | grep -v grep
        } >> "${ARTIFACTS}/after_failure.txt" 2>&1
        # macOS writes a full backtrace for a crashed process here. The stall watchdog cannot
        # help with a crash -- there is nothing left to sample -- so this is the only way a
        # SegFault like the one seen in network_recovery_concurrent_make_publisher_during_reset
        # gets explained.
        mkdir -p "${ARTIFACTS}/crash_reports"
        find "${HOME}/Library/Logs/DiagnosticReports" -name "*.ips" -newermt "-20 minutes" \
            -exec cp {} "${ARTIFACTS}/crash_reports/" \; 2>/dev/null
        echo "--- crash reports collected so far: $(ls -1 "${ARTIFACTS}/crash_reports" 2>/dev/null | wc -l | tr -d ' ') ---"
        echo "--- last 40 net-watch lines ---"
        tail -40 "${ARTIFACTS}/net_watch.c${CHUNK}.txt" 2>/dev/null
    fi
    # -V output is large; every iteration is kept, but compressed.
    gzip -f "${LOG}"
    # Anything the suite left behind changes what the next iteration sees, so it is
    # recorded (never killed: a leak is itself a finding, and killing it would hide it).
    LEFTOVER=$(ps -axww -o pid=,etime=,command= | grep -E "/build/test/" | grep -v grep | wc -l | tr -d ' ')
    if [ "${LEFTOVER}" != "0" ]; then
        {
            echo "=== ${LEFTOVER} leftover test process(es) after iteration ${ITERATION} ==="
            ps -axww -o pid=,ppid=,etime=,state=,command= | grep -E "/build/test/" | grep -v grep
        } >> "${ARTIFACTS}/leftover_processes.txt"
    fi
done

echo "hunt: chunk ${CHUNK}: ${ITERATION} iteration(s), ${FAILURES} failing"
echo "chunk ${CHUNK}: ${ITERATION} iterations, ${FAILURES} failures" >> "${ARTIFACTS}/summary.txt"
# The loop's own exit status is deliberately 0: a failing iteration is the RESULT of this
# job, recorded in the artifacts and in the summary step, not a reason to abandon the run
# and lose the iterations that would have followed.
exit 0
