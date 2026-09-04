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
#   HUNT_ARTIFACTS              directory for all output (required)
#   HUNT_LOOP_SECONDS           how long to keep looping (default 14400)
#   PROVIZIO_DDS_CTEST_EXCLUDE  ctest -E regex (optional)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd -P)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
BUILD_DIR="${REPO_DIR}/build"
ARTIFACTS="${HUNT_ARTIFACTS:?HUNT_ARTIFACTS must be set}"
LOOP_SECONDS="${HUNT_LOOP_SECONDS:-14400}"

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
python3 -u "${REPO_DIR}/test/ci_net_watch.py" --out "${ARTIFACTS}" --poll 0.1 \
    > "${ARTIFACTS}/net_watch.stdout.txt" 2>&1 &
NET_WATCH_PID=$!
python3 -u "${REPO_DIR}/test/ci_stall_watchdog.py" --build-dir "${BUILD_DIR}" --out "${ARTIFACTS}" \
    --margin 8 --poll 1 > "${ARTIFACTS}/stall_watchdog.stdout.txt" 2>&1 &
WATCHDOG_PID=$!

cleanup() {
    kill "${NET_WATCH_PID}" "${WATCHDOG_PID}" 2>/dev/null
    wait "${NET_WATCH_PID}" "${WATCHDOG_PID}" 2>/dev/null
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
echo "hunt: looping for ${LOOP_SECONDS}s, excluding '${PROVIZIO_DDS_CTEST_EXCLUDE:-<nothing>}'"

while [ "$(date +%s)" -lt "${DEADLINE}" ]; do
    ITERATION=$(( ITERATION + 1 ))
    LOG="${ARTIFACTS}/ctest.$(printf '%03d' "${ITERATION}").log"
    echo "::group::iteration ${ITERATION} (started $(date -u +%H:%M:%S)Z)"
    ctest --output-on-failure "${CTEST_EXTRA_ARGS[@]}" 2>&1 | stamp | tee "${LOG}"
    STATUS=${PIPESTATUS[0]}
    echo "::endgroup::"
    if [ "${STATUS}" -eq 0 ]; then
        echo "hunt: iteration ${ITERATION} PASS"
    else
        FAILURES=$(( FAILURES + 1 ))
        echo "::error::hunt: iteration ${ITERATION} FAILED (ctest exit ${STATUS})"
        grep -E '\*\*\*|The following tests FAILED|tests passed' "${LOG}" | tail -20
        {
            echo "=== after failing iteration ${ITERATION} at $(date -u +%H:%M:%SZ) ==="
            ifconfig -a
            echo "--- netstat -an (udp) ---"
            netstat -an -p udp 2>/dev/null | head -60
            echo "--- surviving processes ---"
            ps -axww -o pid=,ppid=,etime=,state=,command= | grep -E "provizio|/build/test/" | grep -v grep
        } >> "${ARTIFACTS}/after_failure.txt" 2>&1
        echo "--- last 40 net-watch lines ---"
        tail -40 "${ARTIFACTS}/net_watch.txt" 2>/dev/null
    fi
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

echo "hunt: ${ITERATION} iteration(s), ${FAILURES} failing"
echo "${ITERATION} iterations, ${FAILURES} failures" > "${ARTIFACTS}/summary.txt"
# The loop's own exit status is deliberately 0: a failing iteration is the RESULT of this
# job, recorded in the artifacts and in the summary step, not a reason to abandon the run
# and lose the iterations that would have followed.
exit 0
