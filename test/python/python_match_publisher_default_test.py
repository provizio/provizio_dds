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
"""Subcommand-driven tests for the "match publisher" reader-reliability default.
Python mirror of the C++ test/match_publisher_default/ suite.

A Subscriber left at the default reliability (provizio_dds.MATCH_PUBLISHER_RELIABILITY_QOS)
DEFERS creating its DataReader until the first matching DataWriter is discovered on
its topic, then builds the reader with that writer's offered reliability. An explicit
BEST_EFFORT / RELIABLE reliability still builds eagerly. Each ctest entry runs the same
script with a different subcommand so per-case failure is isolated, mirroring
python_discovered_endpoints_test.py."""

import sys
import threading
import time
import traceback

import provizio_dds

# Discovery / matching is asynchronous; generous upper bound for a
# cross-participant match to settle on a busy runner. Mirrors the C++
# k_match_timeout / the discovered-endpoints DISCOVERY_TIMEOUT_SEC.
MATCH_TIMEOUT_SEC = 15.0

# Timeout for the "no writer ever appears" deferral assertion. get_num_matched_publishers
# must HONOUR this timeout on a deferred reader (block ~this long, then return 0) rather
# than short-circuiting to 0 at t=0 — measured below. Well under the per-test ctest timeout.
DEFERRED_OBSERVE_WINDOW_SEC = 3.0

# Two participants per cross-participant case (one for the publisher, one for the
# subscriber) so the match is a real remote SEDP match, matching how the library
# is used. Same domain.
DOMAIN = 0

REPUBLISH_INTERVAL_SEC = 0.2


def _make_participant():
    # network_recovery OFF: these tests don't exercise resets, and OFF keeps the
    # case self-contained / fast. Mirrors the C++ make_domain_participant(..., off).
    return provizio_dds.make_domain_participant(
        DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
    )


class _Receiver:
    """Records received payloads under a lock + condition, like the C++ receiver."""

    def __init__(self):
        self._cv = threading.Condition()
        self.received_total = 0
        self.last_message = None

    def on_data(self, message):
        with self._cv:
            self.last_message = message.data()
            self.received_total += 1
            self._cv.notify_all()

    def wait_for_payload(self, payload, baseline):
        with self._cv:
            return self._cv.wait_for(
                lambda: self.received_total > baseline and self.last_message == payload,
                timeout=REPUBLISH_INTERVAL_SEC,
            )


def _publish_and_wait_for(publisher, sink, payload, timeout_sec):
    """Republish ``payload`` periodically until ``sink`` receives that exact
    payload at least once beyond the baseline, or the timeout elapses. The
    republish loop (rather than a single send + sleep) keeps the test
    deterministic across discovery jitter. Mirrors the C++ publish_and_wait_for."""
    baseline = sink.received_total
    message = provizio_dds.String()
    message.data(payload)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        publisher.publish(message)
        if sink.wait_for_payload(payload, baseline):
            return True
    return False


def _make_matching_subscriber(participant, topic_name, sink):
    # Left at the DEFAULT reliability_kind (omitted) on purpose — that is the case
    # under test (deferred DataReader creation, adopts the publisher's reliability).
    return provizio_dds.Subscriber(
        participant,
        topic_name,
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        sink.on_data,
    )


def _make_explicit_subscriber(participant, topic_name, sink, reliability):
    # Explicit reliability override -> eager DataReader (no defer).
    return provizio_dds.Subscriber(
        participant,
        topic_name,
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        sink.on_data,
        reliability_kind=reliability,
    )


def test_deferral():
    """Case 1 — Deferral: a default-reliability subscriber on a topic with NO
    publisher must not create its DataReader, yet get_num_matched_publishers must
    still HONOUR its timeout (rather than short-circuiting to 0 at t=0 while
    reliability discovery is in progress). Asserted through the public API:
      * with no writer, get_num_matched_publishers blocks ~the full timeout and
        returns 0 (it did NOT short-circuit), and get_guid() is the unknown GUID
        (a built reader would report a real one);
      * a writer that appears DURING the wait is observed within the timeout — the
        deferred reader is built and matched, and the call returns a positive count
        without running out the clock (the case the early return used to break).
    Mirrors the C++ test_deferral."""
    participant = _make_participant()
    sink = _Receiver()
    topic_name = "provizio_dds_py_match_publisher_deferral_topic"
    subscriber = _make_matching_subscriber(participant, topic_name, sink)

    # No writer will ever appear: the call must wait out its timeout and report 0,
    # proving it honours the timeout instead of early-returning on the null reader.
    start = time.monotonic()
    matched = subscriber.get_num_matched_publishers(DEFERRED_OBSERVE_WINDOW_SEC, 0.0)
    elapsed = time.monotonic() - start

    if matched != 0:
        print(f"deferred subscriber reported {matched} matched publishers, expected 0", file=sys.stderr)
        return 1
    # It blocked for ~the timeout. Lower bound comfortably under the full window
    # (wide margin for a fast scheduler) proves it did NOT short-circuit at t=0.
    if elapsed < DEFERRED_OBSERVE_WINDOW_SEC - 1.0:
        print(f"get_num_matched_publishers returned after only {elapsed:.3f}s — it short-circuited instead of honouring the timeout", file=sys.stderr)
        return 1
    # The reader was never built, so there is no DataReader GUID to report.
    if subscriber.get_guid() != provizio_dds.GUID_t.unknown():
        print("deferred subscriber reports a concrete GUID — DataReader was created despite no publisher", file=sys.stderr)
        return 1

    # A writer that appears mid-wait is observed within the timeout. Create the
    # publisher from a separate thread a short moment after the (blocking) wait
    # begins, so the wait must span discovery + the deferred build and return a
    # positive count — not 0, and not by timing out.
    appears_sink = _Receiver()
    appears_topic = "provizio_dds_py_match_publisher_deferral_appears_topic"
    appears_subscriber = _make_matching_subscriber(participant, appears_topic, appears_sink)
    pub_participant = _make_participant()
    holder = {}

    def _create_writer():
        time.sleep(0.3)
        holder["pub"] = provizio_dds.Publisher(
            pub_participant,
            appears_topic,
            provizio_dds.StringPubSubType,
            reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
        )

    writer_thread = threading.Thread(target=_create_writer)
    writer_thread.start()
    appears_start = time.monotonic()
    appears_matched = appears_subscriber.get_num_matched_publishers(MATCH_TIMEOUT_SEC, 0.0)
    appears_elapsed = time.monotonic() - appears_start
    writer_thread.join()

    if appears_matched <= 0:
        print(f"writer appeared mid-wait but get_num_matched_publishers returned {appears_matched}", file=sys.stderr)
        return 1
    if appears_elapsed >= MATCH_TIMEOUT_SEC:
        print(f"get_num_matched_publishers ran out the {MATCH_TIMEOUT_SEC}s timeout instead of returning on match", file=sys.stderr)
        return 1

    print(f"deferral: PASS (matched={matched}, waited {elapsed * 1000:.0f} ms; "
          f"appears_matched={appears_matched}, waited {appears_elapsed * 1000:.0f} ms)")
    return 0


def _run_adopts(writer_reliability, topic_name, label):
    """Shared body for the two "default subscriber adopts the publisher's
    reliability" cases. The publisher is created FIRST (so the subscriber's
    deferred build resolves against an already-discoverable writer, also
    exercising the writer-before-subscriber cache path), then the default
    subscriber. Success = the subscriber matches AND receives a published sample.

    For writer_reliability == BEST_EFFORT this is the discriminating case: a
    blanket-RELIABLE reader is RxO-INCOMPATIBLE with a BEST_EFFORT writer and would
    never match, so reception here proves the subscriber adopted BEST_EFFORT rather
    than forcing RELIABLE. Mirrors the C++ run_adopts."""
    pub_participant = _make_participant()
    sub_participant = _make_participant()

    # Publisher first, with the reliability under test.
    publisher = provizio_dds.Publisher(
        pub_participant,
        topic_name,
        provizio_dds.StringPubSubType,
        reliability_kind=writer_reliability,
    )

    sink = _Receiver()
    subscriber = _make_matching_subscriber(sub_participant, topic_name, sink)

    # The default subscriber must match and RECEIVE. _publish_and_wait_for
    # republishes until a sample lands, so this also implicitly proves the
    # deferred reader got built (no reader -> no reception).
    if not _publish_and_wait_for(publisher, sink, "match-" + label, MATCH_TIMEOUT_SEC):
        print(f"{label}: default subscriber never received from the {label} publisher within timeout", file=sys.stderr)
        return 1

    # Once it has received, it must report a matched publisher — i.e. the deferred
    # reader exists now and the listener saw the writer.
    matched = subscriber.get_num_matched_publishers(MATCH_TIMEOUT_SEC, 0.0)
    if matched <= 0:
        print(f"{label}: subscriber received but reports {matched} matched publishers", file=sys.stderr)
        return 1

    # A real DataReader was built, so a concrete GUID is now reported (it was the
    # unknown GUID while deferred — see test_deferral).
    if subscriber.get_guid() == provizio_dds.GUID_t.unknown():
        print(f"{label}: subscriber received but still reports the unknown GUID (reader not built?)", file=sys.stderr)
        return 1

    print(f"{label}: PASS (received={sink.received_total}, matched={matched})")
    return 0


def test_adopts_reliable():
    """Case 2 — default subscriber adopts a RELIABLE publisher."""
    return _run_adopts(
        provizio_dds.RELIABLE_RELIABILITY_QOS,
        "provizio_dds_py_match_publisher_adopts_reliable_topic",
        "adopts_reliable",
    )


def test_adopts_best_effort():
    """Case 3 — default subscriber adopts a BEST_EFFORT publisher (discriminating
    case: proves the reader did NOT silently force RELIABLE)."""
    return _run_adopts(
        provizio_dds.BEST_EFFORT_RELIABILITY_QOS,
        "provizio_dds_py_match_publisher_adopts_best_effort_topic",
        "adopts_best_effort",
    )


def _run_explicit(reliability, topic_name, label):
    """Shared body for the two "explicit reliability override builds eagerly"
    cases. With an explicit reliability the subscriber must NOT defer: its
    DataReader is created up-front. We prove eager creation by asserting get_guid()
    is a real GUID immediately after construction, BEFORE any publisher exists — a
    deferred reader would still report the unknown GUID here. Then a matching
    publisher is created and reception is confirmed, i.e. the historical behaviour
    is unchanged. Mirrors the C++ run_explicit."""
    sub_participant = _make_participant()

    # Subscriber first, with an explicit reliability and NO publisher yet.
    sink = _Receiver()
    subscriber = _make_explicit_subscriber(sub_participant, topic_name, sink, reliability)

    # Eager build: a concrete DataReader GUID is available straight away, with no
    # publisher in sight. (The match-mode default would report the unknown GUID
    # until a writer is discovered — see test_deferral.)
    if subscriber.get_guid() == provizio_dds.GUID_t.unknown():
        print(f"{label}: explicit-reliability subscriber reports the unknown GUID — reader was deferred, not built eagerly", file=sys.stderr)
        return 1

    # Now bring up a matching publisher and confirm reception is unchanged.
    pub_participant = _make_participant()
    publisher = provizio_dds.Publisher(
        pub_participant,
        topic_name,
        provizio_dds.StringPubSubType,
        reliability_kind=reliability,
    )

    if not _publish_and_wait_for(publisher, sink, "explicit-" + label, MATCH_TIMEOUT_SEC):
        print(f"{label}: explicit-reliability subscriber never received within timeout", file=sys.stderr)
        return 1

    matched = subscriber.get_num_matched_publishers(MATCH_TIMEOUT_SEC, 0.0)
    if matched <= 0:
        print(f"{label}: subscriber received but reports {matched} matched publishers", file=sys.stderr)
        return 1

    print(f"{label}: PASS (received={sink.received_total}, matched={matched})")
    return 0


def test_explicit_best_effort():
    """Case 4a — explicit BEST_EFFORT subscriber builds eagerly and receives."""
    return _run_explicit(
        provizio_dds.BEST_EFFORT_RELIABILITY_QOS,
        "provizio_dds_py_match_publisher_explicit_best_effort_topic",
        "explicit_best_effort",
    )


def test_explicit_reliable():
    """Case 4b — explicit RELIABLE subscriber builds eagerly and receives."""
    return _run_explicit(
        provizio_dds.RELIABLE_RELIABILITY_QOS,
        "provizio_dds_py_match_publisher_explicit_reliable_topic",
        "explicit_reliable",
    )


def test_participant_dropped():
    """Case 5 — Lifetime: a match-publisher (default) subscriber must keep working
    after the caller releases its OWN reference to the participant. The Subscriber holds
    a strong reference to the participant, so the participant — and the discovery
    listener that drives the deferred reader build — stays alive and resolves the
    deferred reader once a writer appears, even though the caller no longer holds the
    participant. The publisher is brought up AFTER the drop (subscriber-first), so the
    subscriber is parked waiting for writer discovery — exactly the path that must
    survive the caller dropping its participant handle. Mirrors the C++
    test_participant_dropped."""
    sub_participant = _make_participant()
    sink = _Receiver()
    topic_name = "provizio_dds_py_match_publisher_participant_dropped_topic"
    subscriber = _make_matching_subscriber(sub_participant, topic_name, sink)

    # Release the caller's participant reference: only the subscriber keeps it alive now.
    sub_participant = None

    pub_participant = _make_participant()
    publisher = provizio_dds.Publisher(
        pub_participant,
        topic_name,
        provizio_dds.StringPubSubType,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    if not _publish_and_wait_for(publisher, sink, "participant-dropped", MATCH_TIMEOUT_SEC):
        print("participant_dropped: subscriber never received after its participant reference was dropped", file=sys.stderr)
        return 1

    matched = subscriber.get_num_matched_publishers(MATCH_TIMEOUT_SEC, 0.0)
    if matched <= 0:
        print(f"participant_dropped: subscriber received but reports {matched} matched publishers", file=sys.stderr)
        return 1

    if subscriber.get_guid() == provizio_dds.GUID_t.unknown():
        print("participant_dropped: subscriber received but still reports the unknown GUID (reader not built?)", file=sys.stderr)
        return 1

    print(f"participant_dropped: PASS (received={sink.received_total}, matched={matched})")
    return 0


def test_heterogeneous_rederive():
    """Case 6 — Heterogeneous-writer re-derive: when the writer whose reliability the topic
    adopted (match-first) leaves while a differently-configured writer remains, the adopted
    reliability must be re-derived from a still-live writer — otherwise a subscriber created
    afterwards adopts a stale reliability matching none of the remaining writers (the
    RELIABLE-cached / only-BEST_EFFORT-left data-loss path).

    Sequence (synchronised via the discovery callback, which fires AFTER the internal
    resolve/remove on the same discovery thread, so observing an event means the cache is
    already updated):
      1. RELIABLE writer discovered first   -> topic adopts RELIABLE.
      2. BEST_EFFORT writer discovered next  -> match-first keeps RELIABLE; BEST_EFFORT counted.
      3. RELIABLE writer removed             -> re-derive: only BEST_EFFORT remains -> adopted BEST_EFFORT.
      4. A NEW default subscriber must adopt BEST_EFFORT and receive from the surviving writer.
         If adopted stayed RELIABLE, a RELIABLE reader would not match a BEST_EFFORT writer and
         would receive nothing — so reception is the discriminating assertion.
    Mirrors the C++ test_heterogeneous_rederive."""
    cond = threading.Condition()
    writer_events = []  # list of (reliability, discovered)
    topic_name = "provizio_dds_py_match_publisher_heterogeneous_rederive_topic"

    sub_participant = _make_participant()

    def _on_discovered(_participant, topic, _type, kind, discovered, reliability, _durability):
        if kind != provizio_dds.EndpointKind.DATA_WRITER or topic != topic_name:
            return
        with cond:
            writer_events.append((reliability, discovered))
            cond.notify_all()

    sub_participant.on_discovered_endpoint(
        _on_discovered, kinds=provizio_dds.EndpointKind.DATA_WRITER
    )

    def _wait_for_writer_event(reliability, discovered):
        with cond:
            return cond.wait_for(
                lambda: any(r == reliability and d == discovered for (r, d) in writer_events),
                timeout=MATCH_TIMEOUT_SEC,
            )

    # Separate participants so each writer can be discovered / removed independently.
    pub_reliable_participant = _make_participant()
    pub_best_effort_participant = _make_participant()

    # 1. RELIABLE writer discovered FIRST -> topic adopts RELIABLE.
    reliable_writer = provizio_dds.Publisher(
        pub_reliable_participant,
        topic_name,
        provizio_dds.StringPubSubType,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    if not _wait_for_writer_event(provizio_dds.RELIABLE_RELIABILITY_QOS, True):
        print("heterogeneous_rederive: RELIABLE writer discovery not observed", file=sys.stderr)
        return 1

    # 2. BEST_EFFORT writer discovered next -> match-first keeps RELIABLE adopted.
    best_effort_writer = provizio_dds.Publisher(
        pub_best_effort_participant,
        topic_name,
        provizio_dds.StringPubSubType,
        reliability_kind=provizio_dds.BEST_EFFORT_RELIABILITY_QOS,
    )
    if not _wait_for_writer_event(provizio_dds.BEST_EFFORT_RELIABILITY_QOS, True):
        print("heterogeneous_rederive: BEST_EFFORT writer discovery not observed", file=sys.stderr)
        return 1

    # 3. Remove the RELIABLE writer -> re-derive adopted to the surviving BEST_EFFORT.
    reliable_writer = None  # noqa: F841 — drop the last reference to remove the writer
    if not _wait_for_writer_event(provizio_dds.RELIABLE_RELIABILITY_QOS, False):
        print("heterogeneous_rederive: RELIABLE writer removal not observed", file=sys.stderr)
        return 1

    # 4. A new default subscriber must adopt BEST_EFFORT and receive from the surviving writer.
    sink = _Receiver()
    subscriber = _make_matching_subscriber(sub_participant, topic_name, sink)
    if not _publish_and_wait_for(best_effort_writer, sink, "rederive", MATCH_TIMEOUT_SEC):
        print("heterogeneous_rederive: new default subscriber did not receive from the surviving BEST_EFFORT writer (stale RELIABLE adopted?)", file=sys.stderr)
        return 1

    matched = subscriber.get_num_matched_publishers(MATCH_TIMEOUT_SEC, 0.0)
    if matched <= 0:
        print(f"heterogeneous_rederive: subscriber received but reports {matched} matched publishers", file=sys.stderr)
        return 1

    print(f"heterogeneous_rederive: PASS (received={sink.received_total}, matched={matched})")
    return 0


_TESTS = {
    "deferral": test_deferral,
    "adopts_reliable": test_adopts_reliable,
    "adopts_best_effort": test_adopts_best_effort,
    "explicit_best_effort": test_explicit_best_effort,
    "explicit_reliable": test_explicit_reliable,
    "participant_dropped": test_participant_dropped,
    "heterogeneous_rederive": test_heterogeneous_rederive,
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in _TESTS:
        print(
            f"usage: {sys.argv[0]} <{'|'.join(_TESTS.keys())}>",
            file=sys.stderr,
        )
        return 1
    try:
        return _TESTS[sys.argv[1]]()
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
