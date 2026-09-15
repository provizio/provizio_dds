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

# Prints one line of network counters every few seconds, from inside a congested-network
# test container, so a failed run says WHERE the traffic stopped instead of only that it
# did. Every way this test can lose packets looks the same in its own log -- the
# subscriber simply does not get a sample -- and they are told apart only here:
#
#   sent / dropped / backlog
#                The netem band's counters. "dropped" growing in step with the offered
#                load is the configured loss doing its job. A backlog near the qdisc limit
#                (1000 packets) instead means the shaper itself is the bottleneck and is
#                tail-dropping, which is not what this test means to simulate.
#   udp_out      /proc/net/snmp Udp OutDatagrams. Flat while the publisher keeps reporting
#                samples published means the process never handed them to the kernel: a
#                DDS-side problem, such as a matched reader with no usable locator.
#   udp_in       Udp InDatagrams. On the subscriber it separates "the samples never
#                arrived" from "they arrived and the test did not report them" -- a
#                subscriber killed before it flushed its stdout prints nothing either way.
#   udp_snderr   Udp SndbufErrors, which counts sendto() failing. Fast-DDS logs a failed
#                send at WARNING, and its default verbosity (Error) does not print that,
#                so this counter is the only trace such a failure leaves.
#   neigh        The peer's ARP entry state. An entry in FAILED or INCOMPLETE blackholes
#                that destination inside the kernel, below the qdisc, so nothing reaches
#                the wire however healthy the shaping looks. The entry point deliberately
#                leaves ARP out of the shaped band to keep that from happening (its header
#                explains why); this field is here to show if it happens anyway.
#
# Usage: congested_network_netstats.sh [<interface>] [<interval_sec>] [<peer_ip>]

set -u

INTERFACE="${1:-eth0}"
INTERVAL="${2:-5}"
PEER="${3:-}"

# The value of one column of the /proc/net/snmp IPv4 UDP block, by name, so a kernel that
# adds or reorders columns cannot silently shift the reading.
udp_counter() {
    awk -v want="$1" '
        /^Udp:/ {
            if (header == "") { for (i = 2; i <= NF; i++) { column[$i] = i }; header = "yes"; next }
            if (want in column) { print $column[want] }
        }' /proc/net/snmp 2>/dev/null || echo "?"
}

# The shaped band's own counters. The test's qdisc is a prio whose third band holds the
# netem, so the root's totals would mix in the unshaped traffic; this reads the netem
# section, and falls back to the first section for a plain netem root.
qdisc_field() {
    tc -s qdisc show dev "${INTERFACE}" 2>/dev/null | awk -v want="$1" '
        /^qdisc / { in_netem = ($2 == "netem"); if (in_netem) { seen_netem = 1 } }
        {
            if (want == "sent" && $1 == "Sent") { value[in_netem] = $4 }
            if (want == "dropped" && $1 == "Sent") {
                # Found by name rather than by position: the counters after it differ
                # between qdisc kinds and iproute2 versions.
                for (i = 1; i < NF; i++) {
                    if ($i == "(dropped") { dropped = $(i + 1); sub(",", "", dropped); value[in_netem] = dropped }
                }
            }
            if (want == "backlog" && $1 == "backlog") { sub("p$", "", $3); value[in_netem] = $3 }
        }
        END { print (seen_netem && (1 in value)) ? value[1] : ((0 in value) ? value[0] : "?") }'
}

while true; do
    neigh="$(ip neigh show ${PEER:+to "${PEER}"} dev "${INTERFACE}" 2>/dev/null |
        awk '{ printf "%s=%s ", $1, $NF }')"
    echo "[netstats] $(date -u +%H:%M:%S)" \
        "sent=$(qdisc_field sent) dropped=$(qdisc_field dropped) backlog=$(qdisc_field backlog)" \
        "udp_in=$(udp_counter InDatagrams) udp_out=$(udp_counter OutDatagrams)" \
        "udp_snderr=$(udp_counter SndbufErrors)" \
        "neigh: ${neigh:-none}"
    sleep "${INTERVAL}"
done
