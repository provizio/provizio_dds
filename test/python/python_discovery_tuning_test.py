#!/usr/bin/env python3
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
"""Subcommand-driven tests for participant auto-discovery tuning — Python mirror
of the C++ test/discovery_tuning/ suite.

They cover the de-escalated discovery defaults (a modest initial-announcement
burst and a relaxed periodic re-announcement that keep the library from being a
primary source of UDP congestion when many participants run), the
PROVIZIO_DDS_DISCOVERY_* environment overrides (and their fallback to the
defaults on malformed input), and many-participant discovery convergence under
the gentler defaults. Each subcommand is its own ctest entry so per-case failure
stays isolated, mirroring python_match_publisher_default_test.py."""

import os
import random
import sys
import time
import traceback

import provizio_dds

DOMAIN = 0
# The mesh case runs on a per-process domain, picked once at random within the
# DDS-safe range and away from 0. These tests must unset FASTDDS_DEFAULT_PROFILES_FILE
# to exercise the code-driven discovery tuning, so unlike the rest of the suite they
# cannot be confined to loopback; with a fixed domain, concurrent ctest runs on other
# self-hosted CI hosts sharing the LAN could cross-match into the mesh and break the
# exact "matched == K" assertion. Randomising per process makes such a collision
# improbable.
MESH_DOMAIN = random.randint(1, 200)  # DDS-safe range, excluding domain 0

# The expected de-escalated defaults (kept in sync with the C++
# test/discovery_tuning/ suite, src/domain_participant.cpp and the
# QosDefaults / _resolve_discovery_* helpers in provizio_dds.py).
DEFAULT_INITIAL_COUNT = 15
DEFAULT_INITIAL_PERIOD_NS = 100 * 1000 * 1000  # 100 ms
DEFAULT_ANNOUNCEMENT_PERIOD_NS = 3 * 1000 * 1000 * 1000  # 3 s
DEFAULT_LEASE_DURATION_NS = 30 * 1000 * 1000 * 1000  # 30 s

_failures = []


def expect(condition, description):
    if not condition:
        print(f"FAIL: {description}")
        _failures.append(description)
    return condition


def _make_participant(domain=DOMAIN):
    # network_recovery OFF keeps each case self-contained / fast. Mirrors the C++
    # make_domain_participant(..., network_recovery_mode::off).
    return provizio_dds.make_domain_participant(
        domain, provizio_dds.NetworkRecoveryMode.OFF
    )


def _read_cfg(participant):
    """(initial_count, initial_period_ns, announcement_period_ns, lease_duration_ns) off the
    participant's effective QoS."""
    discovery = participant._participant_qos.wire_protocol().builtin.discovery_config
    return (
        discovery.initial_announcements.count,
        discovery.initial_announcements.period.to_ns(),
        discovery.leaseDuration_announcementperiod.to_ns(),
        discovery.leaseDuration.to_ns(),
    )


_DISCOVERY_ENV_VARS = (
    "PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT",
    "PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS",
    "PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS",
    "PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS",
)


def _clear_discovery_env():
    """Remove any ambient PROVIZIO_DDS_DISCOVERY_* overrides so the test exercises the library defaults."""
    for var in _DISCOVERY_ENV_VARS:
        os.environ.pop(var, None)


def test_defaults():
    """With no env overrides a participant is configured with the de-escalated
    defaults — NOT the historical 200-shot / 1 s flood."""
    _clear_discovery_env()
    participant = _make_participant()
    count, initial_ns, announcement_ns, lease_ns = _read_cfg(participant)

    expect(count == DEFAULT_INITIAL_COUNT, f"initial count {count} == {DEFAULT_INITIAL_COUNT}")
    expect(initial_ns == DEFAULT_INITIAL_PERIOD_NS, f"initial period {initial_ns} ns == {DEFAULT_INITIAL_PERIOD_NS}")
    expect(
        announcement_ns == DEFAULT_ANNOUNCEMENT_PERIOD_NS,
        f"announcement period {announcement_ns} ns == {DEFAULT_ANNOUNCEMENT_PERIOD_NS}",
    )
    expect(lease_ns == DEFAULT_LEASE_DURATION_NS, f"lease duration {lease_ns} ns == {DEFAULT_LEASE_DURATION_NS}")
    print(f"defaults: {'PASS' if not _failures else 'FAIL'} (count={count}, "
          f"initial_period_ns={initial_ns}, announcement_period_ns={announcement_ns}, lease_ns={lease_ns})")


def test_env_overrides():
    """Valid PROVIZIO_DDS_DISCOVERY_* env values are honoured verbatim."""
    os.environ["PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT"] = "42"
    os.environ["PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS"] = "250"
    os.environ["PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS"] = "5000"
    os.environ["PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS"] = "25000"

    participant = _make_participant()
    count, initial_ns, announcement_ns, lease_ns = _read_cfg(participant)

    expect(count == 42, f"initial count {count} == 42")
    expect(initial_ns == 250 * 1000 * 1000, f"initial period {initial_ns} ns == 250 ms")
    expect(announcement_ns == 5 * 1000 * 1000 * 1000, f"announcement period {announcement_ns} ns == 5 s")
    expect(lease_ns == 25 * 1000 * 1000 * 1000, f"lease duration {lease_ns} ns == 25 s")
    print(f"env_overrides: {'PASS' if not _failures else 'FAIL'} (count={count})")


def test_env_invalid():
    """Malformed / out-of-range env values are ignored and the defaults win, so a
    typo can never silently disable or wildly misconfigure discovery."""
    os.environ["PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT"] = "not-a-number"
    os.environ["PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS"] = "0"
    os.environ["PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS"] = "-5"
    os.environ["PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS"] = "abc"

    participant = _make_participant()
    count, initial_ns, announcement_ns, lease_ns = _read_cfg(participant)

    expect(count == DEFAULT_INITIAL_COUNT, f"initial count {count} == {DEFAULT_INITIAL_COUNT}")
    expect(initial_ns == DEFAULT_INITIAL_PERIOD_NS, f"initial period {initial_ns} ns == {DEFAULT_INITIAL_PERIOD_NS}")
    expect(
        announcement_ns == DEFAULT_ANNOUNCEMENT_PERIOD_NS,
        f"announcement period {announcement_ns} ns == {DEFAULT_ANNOUNCEMENT_PERIOD_NS}",
    )
    expect(lease_ns == DEFAULT_LEASE_DURATION_NS, f"lease duration {lease_ns} ns == {DEFAULT_LEASE_DURATION_NS}")
    print(f"env_invalid: {'PASS' if not _failures else 'FAIL'} (count={count})")


def test_multi_participant():
    """Many participants must still all discover and match each other under the
    de-escalated defaults. K publisher participants and K subscriber participants
    share one topic; every subscriber must match all K publishers — a
    2K-participant discovery mesh, the congested-deployment shape the
    de-escalation must not break."""
    _clear_discovery_env()  # exercise the de-escalated defaults, not whatever the runner exports
    k = 8
    # Generous: 2K participants doing SPDP/SEDP on a loaded runner, with the
    # gentler announcement cadence, may take a while to fully converge.
    convergence_timeout_sec = 60.0
    # Suffixed with an independent per-process random draw (a separate random.randint call, NOT derived from
    # MESH_DOMAIN) so that even the rare case of two concurrent runs picking the same MESH_DOMAIN still does not
    # cross-match this mesh's topic — the two draws must vary independently of each other. A host-local PID
    # would not do: it is not unique ACROSS hosts, which is exactly the cross-host collision this guards.
    topic_name = f"rt/provizio_dds_discovery_tuning_mesh_topic_{random.randint(0, 2**32 - 1)}"

    pub_participants = []
    publishers = []
    sub_participants = []
    subscribers = []
    for _ in range(k):
        pub_participant = _make_participant(MESH_DOMAIN)
        publishers.append(
            provizio_dds.Publisher(
                pub_participant, topic_name, provizio_dds.StringPubSubType,
                reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
            )
        )
        pub_participants.append(pub_participant)

        sub_participant = _make_participant(MESH_DOMAIN)
        # Explicit reliability (eager reader) so this measures pure discovery
        # convergence rather than the match-publisher deferral path.
        subscribers.append(
            provizio_dds.Subscriber(
                sub_participant, topic_name, provizio_dds.StringPubSubType, provizio_dds.String,
                lambda _message: None, reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
            )
        )
        sub_participants.append(sub_participant)

    deadline = time.monotonic() + convergence_timeout_sec
    for i, subscriber in enumerate(subscribers):
        matched = 0
        while time.monotonic() < deadline:
            # settle_time 0: return the current matched count as soon as it is
            # positive, so we can poll it up toward the full mesh size.
            matched = subscriber.get_num_matched_publishers(0.5, 0.0)
            if matched >= k:
                break
            time.sleep(0.2)
        expect(matched == k, f"subscriber {i} matched {matched} of {k} publishers")

    print(f"multi_participant: {'PASS' if not _failures else 'FAIL'} (k={k})")


_SUBCOMMANDS = {
    "defaults": test_defaults,
    "env_overrides": test_env_overrides,
    "env_invalid": test_env_invalid,
    "multi_participant": test_multi_participant,
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in _SUBCOMMANDS:
        print(f"usage: {sys.argv[0]} <{'|'.join(_SUBCOMMANDS)}>", file=sys.stderr)
        return 1
    # Hermetic across the whole suite: an ambient FASTDDS_DEFAULT_PROFILES_FILE pointing at a real XML
    # profile makes the library skip its code-driven discovery tuning entirely, which would break every
    # subcommand here (defaults and env overrides alike). Clear it so the tests exercise the code path.
    os.environ.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
    try:
        _SUBCOMMANDS[sys.argv[1]]()
    except Exception:  # noqa: BLE001 — surface any unexpected error as a failure
        traceback.print_exc()
        return 1
    return 0 if not _failures else 1


if __name__ == "__main__":
    sys.exit(main())
