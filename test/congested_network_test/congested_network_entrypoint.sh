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

# Container entry point for the congested-network test: applies this container's share of
# the simulated congestion, starts the counter reporter, and runs the publisher or the
# subscriber.
#
# The shaping is deliberately applied to IP traffic ONLY, through a prio band, rather than
# to everything the interface sends. netem on the root qdisc shapes every egress frame,
# and that includes ARP -- which is not the traffic under test and which the kernel needs
# in order to send any of it. At this test's harshest profile (75% loss, 500 ms delay) an
# ARP resolution round of three solicitations fails about 42% of the time, and a neighbour
# entry that reaches NUD_FAILED stops being usable: from then on the kernel discards every
# datagram addressed to that peer locally, below the qdisc, until the entry resolves
# again. That is a total blackout rather than a lossy link, and it was measured here --
# a run where the subscriber's entry for the publisher went INCOMPLETE at 39 s and never
# recovered, so the publisher stopped hearing from it, dropped its participant on the
# 20-second lease and exited, while the subscriber sat waiting for a DONE that could no
# longer come. Nothing provizio_dds does or could do affects that outcome, and the failure
# it produces looks exactly like a delivery bug in the log.
#
# So: IPv4 goes through the netem band and gets the configured loss, delay and rate, byte
# for byte as before. ARP takes the unshaped band, which is what lets the shaped traffic
# be delivered at all.
#
# Environment (all set by the image, see congested_network_test.dockerfile):
#   SERVICE        publisher | subscriber
#   NETWORK_DELAY  netem delay, e.g. 500ms
#   PACKETS_LOSS   netem loss, e.g. 75%
#   NETWORK_RATE   netem rate, e.g. 128kbit
#   XML_PROFILE    Fast-DDS profile to load, or a path that does not exist for none

set -eu

INTERFACE="${INTERFACE:-eth0}"
HERE="$(cd "$(dirname "$0")" && pwd -P)"

# Optional by design: the simple-discovery profiles of this test pass a path that does
# not exist, which means "no XML profile".
if [ -f "${XML_PROFILE:-none}" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="${XML_PROFILE}"
fi

# Band 1:3 carries the shaped traffic; bands 1:1 and 1:2 stay unshaped for everything the
# filter below does not claim.
tc qdisc add dev "${INTERFACE}" root handle 1: prio bands 3
tc qdisc add dev "${INTERFACE}" parent 1:3 handle 30: netem \
    delay "${NETWORK_DELAY}" loss "${PACKETS_LOSS}" rate "${NETWORK_RATE}"
# "match u32 0 0" is match-anything, so this claims all IPv4 regardless of its TOS bits,
# which the prio priomap would otherwise spread across the bands.
tc filter add dev "${INTERFACE}" parent 1: protocol ip prio 1 u32 match u32 0 0 flowid 1:3
tc qdisc show dev "${INTERFACE}"

# Runs for the container's lifetime. Its counters are what tell a lost run apart from a
# lossy one: see the header of congested_network_netstats.sh.
"${HERE}/congested_network_netstats.sh" "${INTERFACE}" 5 &

# -u because compose stops the containers with SIGKILL once one of them exits, and Python
# block-buffers stdout into a pipe: without it a killed container loses everything it had
# printed, which is what left the first failure of this test to be reconstructed from
# timestamps alone.
exec python3 -u "${HERE}/congested_network_${SERVICE}.py"
