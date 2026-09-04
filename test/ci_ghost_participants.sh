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
# TEMPORARY investigation tool. Makes the trigger for the Fast-DDS deadlock this
# investigation caught happen often enough to measure.
#
# The deadlock (eProsima/Fast-DDS#6502, open) needs a remote participant to be reaped by
# LEASE EXPIRY on the ResourceEvent thread at the moment another thread deletes an endpoint:
# the reaping callback wants a writer mutex the deleting thread holds, and the deleting
# thread is inside ResourceEvent::unregister_timer waiting for the very callback that is
# blocked. In CI that lines up whenever a test process is SIGKILLed or crashes, because every
# test in this suite runs on domain 0 and therefore sees every other test process on the host.
#
# This produces the same condition on purpose: a participant is created on domain 0 and then
# SIGKILLed (never disposed) on a cycle, so every running test process has a lease to reap
# every few seconds. Reproduced locally with an artificially widened window: with peers to
# reap a teardown deadlocked after 53 cycles, with none 438 cycles ran with a 454 ms worst
# case.
#
# Usage: ci_ghost_participants.sh <period-ms> <processes> <ghost-binary> [log-file]

set -uo pipefail

PERIOD_MS="${1:-3000}"
PROCESSES="${2:-2}"
GHOST_BINARY="${3:?a binary that creates a participant and waits is required}"
LOG_FILE="${4:-/dev/stderr}"

say() {
    echo "[ghost-participants $(date -u +%H:%M:%S)Z] $*" >> "${LOG_FILE}"
}

if [[ ! -x "${GHOST_BINARY}" ]]; then
    say "no ghost binary at ${GHOST_BINARY}; not accelerating"
    exit 0
fi

# Deliberately NOT inherited: a ghost must sit on the domain the tests use when they are not
# isolated (0), which is the whole point of the comparison.
unset PROVIZIO_DDS_FORCE_DOMAIN_ID

# The ghost announces a three-second lease, so a test process reaps it three seconds after
# the SIGKILL rather than the twenty Fast-DDS defaults to -- by which time almost every test
# in this suite has already finished. A lease duration is a property of the announcing
# participant, so this changes only how long a peer waits, never the tests' own timing.
GHOST_PROFILE="$(cd "$(dirname "$0")" && pwd -P)/fast_dds_ghost_profile.xml"
if [[ -f "${GHOST_PROFILE}" ]]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="${GHOST_PROFILE}"
    say "ghosts announce the 3 s lease from ${GHOST_PROFILE}"
else
    say "WARNING: ${GHOST_PROFILE} missing; ghosts keep the default 20 s lease and will rarely be reaped inside a test"
fi

# Killing a background job makes the shell announce it ("Killed"), once per ghost per cycle;
# that is thousands of lines of nothing in the job log, so this script's stderr goes to its
# own file from here on.
if [[ "${LOG_FILE}" != "/dev/stderr" ]]; then
    exec 2>> "${LOG_FILE}"
fi

say "cycling ${PROCESSES} ghost participant(s) every ${PERIOD_MS} ms using ${GHOST_BINARY}"

CYCLE=0
PIDS=()

cleanup() {
    for pid in "${PIDS[@]:-}"; do
        [[ -n "${pid}" ]] && kill -9 "${pid}" 2>/dev/null
    done
    say "stopped after ${CYCLE} cycle(s)"
}
trap cleanup EXIT

while true; do
    CYCLE=$(( CYCLE + 1 ))
    PIDS=()
    for _ in $(seq 1 "${PROCESSES}"); do
        # A ghost's own output is noise; only its existence and its death matter.
        "${GHOST_BINARY}" > /dev/null 2>&1 &
        PIDS+=($!)
    done
    # Long enough for the tests' participants to discover the ghosts before they vanish --
    # a ghost nobody discovered leaves no lease to expire.
    perl -e "select(undef, undef, undef, ${PERIOD_MS} / 1000.0)" 2>/dev/null || sleep 3
    for pid in "${PIDS[@]}"; do
        kill -9 "${pid}" 2>/dev/null
    done
    wait 2>/dev/null
    if [ $(( CYCLE % 20 )) -eq 0 ]; then
        say "cycle ${CYCLE}"
    fi
done
