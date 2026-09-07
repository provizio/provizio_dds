# congested_network_test

This test validates reliable provizio_dds functioning in congested networks, simulating packets loss and delay between 2 Docker containers setup via Docker Compose. It's not part of CMake tests, but is instead invoked by `.github/workflows/test_congested_network.sh`.

## What the simulated congestion covers

`congested_network_entrypoint.sh` applies the loss, delay and rate to **IPv4 traffic only**, through a `prio` band, rather than to everything the interface sends. ARP takes the unshaped band deliberately: it is not the traffic under test, and shaping it breaks the test rather than hardening it. At the harshest profile (75% loss, 500 ms delay) an ARP resolution round fails about 42% of the time, and a neighbour entry that reaches `NUD_FAILED` makes the kernel discard every datagram to that peer locally, below the qdisc. The result is a total blackout that no DDS implementation can do anything about, and that reads in the log exactly like a delivery bug: the subscriber matches, the publisher reports every sample published, and nothing arrives.

`congested_network_netstats.sh` prints the counters that tell those cases apart (netem sent/dropped/backlog, UDP in/out/send-errors, and the peer's neighbour state) every 5 seconds from inside both containers, so a failing run in CI carries its own diagnosis.
