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
# TEMPORARY investigation tool (macOS). Makes the rare host event that appears to cause the
# macos-15-intel test flakes happen on demand.
#
# Three separate macOS CI failures each show the library reporting an interface address
# disappearing and returning, and the test that was running at that moment either timing out
# or losing a response. The event itself is rare, so waiting for it costs hours. This
# creates a fake-ethernet pair the library's snapshot accepts (feth is matched by none of the
# macOS exclusion prefixes: lo, awdl, llw, utun, bridge, gif, stf, anpi, apN), then adds and
# removes its address on a cycle -- which is exactly the "+1 / -0" then "+0 / -1" the failing
# jobs logged, at a rate that fits inside one test-suite pass.
#
# Usage: ci_interface_flap.sh <up-seconds> <down-seconds> [log-file]
#
# Exits 0 without flapping when the platform or privileges do not allow it: the shard then
# simply behaves like an un-accelerated one, which is worth more than a failed job.

set -uo pipefail

UP_SECONDS="${1:-45}"
DOWN_SECONDS="${2:-2}"
LOG_FILE="${3:-/dev/stderr}"

DEVICE_A="feth9"
DEVICE_B="feth10"
FLAP_ADDRESS="10.99.99.1"
FLAP_NETMASK="255.255.255.0"

say() {
    echo "[interface-flap $(date -u +%H:%M:%S)Z] $*" >> "${LOG_FILE}"
}

if [[ "$(uname -s)" != "Darwin" ]]; then
    say "not macOS; nothing to flap"
    exit 0
fi
if ! sudo -n true 2>/dev/null; then
    say "no passwordless sudo; cannot create an interface. Shard runs un-accelerated."
    exit 0
fi

cleanup() {
    sudo -n ifconfig "${DEVICE_A}" destroy 2>/dev/null
    sudo -n ifconfig "${DEVICE_B}" destroy 2>/dev/null
    say "destroyed ${DEVICE_A} / ${DEVICE_B}"
}
trap cleanup EXIT

# A peered pair, because a lone feth never reaches IFF_RUNNING -- and IFF_RUNNING is exactly
# what the library's snapshot filters on (matching Fast-DDS' IPFinder), so an interface
# without it would change nothing and the whole shard would be a no-op that looked like a
# negative result.
if ! sudo -n ifconfig "${DEVICE_A}" create 2>>"${LOG_FILE}" ||
   ! sudo -n ifconfig "${DEVICE_B}" create 2>>"${LOG_FILE}" ||
   ! sudo -n ifconfig "${DEVICE_A}" peer "${DEVICE_B}" 2>>"${LOG_FILE}"; then
    say "could not create a feth pair. Shard runs un-accelerated."
    exit 0
fi
sudo -n ifconfig "${DEVICE_A}" up
sudo -n ifconfig "${DEVICE_B}" up
say "created ${DEVICE_A} peered with ${DEVICE_B}:"
ifconfig "${DEVICE_A}" >> "${LOG_FILE}" 2>&1

if ! ifconfig "${DEVICE_A}" | grep -q "RUNNING"; then
    say "WARNING: ${DEVICE_A} is not RUNNING, so the snapshot will ignore it and no flap will register"
fi

say "flapping ${FLAP_ADDRESS} on ${DEVICE_A}: ${UP_SECONDS}s present, ${DOWN_SECONDS}s absent"
CYCLE=0
while true; do
    CYCLE=$(( CYCLE + 1 ))
    sudo -n ifconfig "${DEVICE_A}" inet "${FLAP_ADDRESS}" netmask "${FLAP_NETMASK}" alias 2>>"${LOG_FILE}"
    say "cycle ${CYCLE}: address ADDED"
    sleep "${UP_SECONDS}"
    sudo -n ifconfig "${DEVICE_A}" inet "${FLAP_ADDRESS}" -alias 2>>"${LOG_FILE}"
    say "cycle ${CYCLE}: address REMOVED"
    sleep "${DOWN_SECONDS}"
done
