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
"""Subcommand-driven tests for the discovery-callback / type-registry
additions to provizio_dds._DomainParticipant. Mirrors the C++
discovered_endpoints_test suite."""

import sys
import threading
import time
import traceback

import provizio_dds

# Discovery is asynchronous — Fast-DDS's default initial announcement period
# is sub-second but anything cross-participant can drift on a busy runner.
DISCOVERY_TIMEOUT_SEC = 15.0


def _wait_for(predicate, timeout_sec):
    """Spin-wait (50 ms granularity) until predicate() returns True or the
    timeout elapses. Returns the predicate's last value."""
    deadline = time.monotonic() + timeout_sec
    while True:
        result = predicate()
        if result:
            return result
        if time.monotonic() >= deadline:
            return result
        time.sleep(0.05)


def test_type_registry():
    participant = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )

    assert not participant.is_known_type(
        "std_msgs::msg::dds_::String_"
    ), "is_known_type returned True before any register_type call"
    assert (
        participant.known_types() == []
    ), "known_types() not empty before any register_type call"

    participant.register_type(provizio_dds.StringPubSubType())
    assert participant.is_known_type("std_msgs::msg::dds_::String_"), (
        "is_known_type returned False after register_type"
    )
    names = participant.known_types()
    assert "std_msgs::msg::dds_::String_" in names, (
        "known_types() missing String_ after registration"
    )

    # Idempotent.
    participant.register_type(provizio_dds.StringPubSubType())
    assert len(participant.known_types()) == len(names), (
        "duplicate register_type changed known_types() size"
    )
    return 0


def _record_events(lock, events):
    """Build a callback that records every discovery event under ``lock``.

    Records the observed (participant, topic_name, type_name, kind, discovered)
    tuple. Tests inspect ``events`` by index — the test predicates below assume
    the same positional layout. The discovery callback also receives the
    discovered endpoint's reliability and durability QoS kinds as two trailing
    arguments; these tests don't assert on them, so they are accepted and
    ignored here."""
    def cb(participant, topic_name, type_name, kind, discovered, reliability, durability):
        with lock:
            events.append((participant, topic_name, type_name, kind, discovered))
    return cb


def test_discovers_writer():
    pa = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )
    pb = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )

    lock = threading.Lock()
    events = []
    pa.on_discovered_endpoint(_record_events(lock, events))

    topic_name = "provizio_dds_py_discovery_test_topic"
    pub = provizio_dds.Publisher(pb, topic_name, provizio_dds.StringPubSubType)

    def predicate():
        with lock:
            return any(
                d
                and k == provizio_dds.EndpointKind.DATA_WRITER
                and t == topic_name
                and ty == "std_msgs::msg::dds_::String_"
                for (_p, t, ty, k, d) in events
            )

    if not _wait_for(predicate, DISCOVERY_TIMEOUT_SEC):
        print(
            f"no writer discovery event observed within {DISCOVERY_TIMEOUT_SEC}s",
            file=sys.stderr,
        )
        return 1

    # Sanity-check the participant arg the callback received: every event must
    # carry the same participant wrapper the test called on_discovered_endpoint
    # on. Catches a regression where set_owner doesn't run before the listener
    # starts firing.
    with lock:
        for (p, _t, _ty, _k, _d) in events:
            if p is not pa:
                print(
                    "callback received a different participant wrapper than the one it was registered on",
                    file=sys.stderr,
                )
                return 1
    return 0


def test_kind_filter():
    pa = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )
    pb = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )

    lock = threading.Lock()
    events = []
    pa.on_discovered_endpoint(
        _record_events(lock, events),
        kinds=provizio_dds.EndpointKind.DATA_READER,
    )

    topic_name = "provizio_dds_py_discovery_test_topic_filter"
    # Publisher first — should NOT fire the DATA_READER-only callback.
    pub = provizio_dds.Publisher(pb, topic_name, provizio_dds.StringPubSubType)
    # Subscriber — SHOULD fire.
    sub = provizio_dds.Subscriber(
        pb,
        topic_name,
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        lambda _msg: None,
    )

    def predicate_reader():
        with lock:
            return any(
                d and k == provizio_dds.EndpointKind.DATA_READER and t == topic_name
                for (_p, t, ty, k, d) in events
            )

    if not _wait_for(predicate_reader, DISCOVERY_TIMEOUT_SEC):
        print("no reader discovery event observed within timeout", file=sys.stderr)
        return 1

    with lock:
        writer_fired = any(
            k == provizio_dds.EndpointKind.DATA_WRITER for (_p, _t, _ty, k, _d) in events
        )
    assert not writer_fired, (
        "data_writer-kind event delivered despite kinds=DATA_READER"
    )
    return 0


def test_constructor_callback():
    # Callback passed at make_domain_participant time must be attached
    # BEFORE Fast-DDS starts discovery. Bring B up first with a writer, give
    # it time to steadily announce, then create A with the callback wired
    # via the constructor — A must observe B's writer.
    pb = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )
    topic_name = "provizio_dds_py_discovery_test_topic_ctor"
    pub = provizio_dds.Publisher(pb, topic_name, provizio_dds.StringPubSubType)

    # Let B steady-state announce before A joins.
    time.sleep(0.5)

    lock = threading.Lock()
    events = []
    pa = provizio_dds.make_domain_participant(
        0,
        provizio_dds.NetworkRecoveryMode.OFF,
        initial_discovery_callback=_record_events(lock, events),
    )

    def predicate():
        with lock:
            return any(
                d
                and k == provizio_dds.EndpointKind.DATA_WRITER
                and t == topic_name
                and ty == "std_msgs::msg::dds_::String_"
                for (_p, t, ty, k, d) in events
            )

    if not _wait_for(predicate, DISCOVERY_TIMEOUT_SEC):
        print(
            f"no writer discovery event via constructor-time callback within {DISCOVERY_TIMEOUT_SEC}s",
            file=sys.stderr,
        )
        return 1

    # Same constraint as test_discovers_writer: the constructor-time callback
    # must receive the wrapper we get back from make_domain_participant.
    with lock:
        for (p, _t, _ty, _k, _d) in events:
            if p is not pa:
                print(
                    "constructor-time callback received a different participant wrapper",
                    file=sys.stderr,
                )
                return 1
    return 0


def test_callback_receives_participant():
    # Mirrors the C++ test of the same name: verifies the participant arg of
    # the discovery callback is the same wrapper make_domain_participant
    # returns even when the very first event fires during the constructor-time
    # SEDP exchange. Also exercises the documented recorder pattern of a
    # constructor-time callback querying the type registry through that
    # participant — the path that AttributeError'd when the registries were
    # initialised after create_participant (the listener swallows callback
    # exceptions, so cb captures any error to surface it as a test failure).
    pb = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )
    topic_name = "provizio_dds_py_discovery_test_topic_callback_ref"
    pub = provizio_dds.Publisher(pb, topic_name, provizio_dds.StringPubSubType)

    # Let B steady-state-announce before A appears so A's first discovery
    # fires close to participant creation time.
    time.sleep(0.5)

    state = {
        "lock": threading.Lock(),
        "seen_participant": None,
        "registry_error": None,
    }

    def cb(participant, _topic, _type_name, _kind, discovered, _reliability, _durability):
        if not discovered:
            return
        try:
            participant.is_known_type("std_msgs::msg::dds_::String_")
            participant.known_types()
        except Exception as ex:  # noqa: BLE001
            with state["lock"]:
                state["registry_error"] = repr(ex)
        with state["lock"]:
            state["seen_participant"] = participant

    pa = provizio_dds.make_domain_participant(
        0,
        provizio_dds.NetworkRecoveryMode.OFF,
        initial_discovery_callback=cb,
    )

    def seen_predicate():
        with state["lock"]:
            return state["seen_participant"] is not None

    if not _wait_for(seen_predicate, DISCOVERY_TIMEOUT_SEC):
        print("no discovery event observed within timeout", file=sys.stderr)
        return 1

    with state["lock"]:
        if state["registry_error"] is not None:
            print(
                f"constructor-time callback failed to query the type registry: {state['registry_error']}",
                file=sys.stderr,
            )
            return 1
        if state["seen_participant"] is not pa:
            print(
                "callback received a different participant wrapper than make_domain_participant returned",
                file=sys.stderr,
            )
            return 1
    return 0


def test_survives_reset():
    pa = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.ON
    )
    pb = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )

    lock = threading.Lock()
    events = []
    pa.on_discovered_endpoint(_record_events(lock, events))

    topic_name = "provizio_dds_py_discovery_test_topic_reset"
    pub = provizio_dds.Publisher(pb, topic_name, provizio_dds.StringPubSubType)

    def writer_seen():
        with lock:
            return any(
                d and k == provizio_dds.EndpointKind.DATA_WRITER and t == topic_name
                for (_p, t, ty, k, d) in events
            )

    if not _wait_for(writer_seen, DISCOVERY_TIMEOUT_SEC):
        print("no pre-reset discovery event", file=sys.stderr)
        return 1

    # Drain so a post-reset re-discovery is unambiguous.
    with lock:
        events.clear()

    # Trigger a recovery reset on A. Use the same synthetic-snapshot trick
    # the existing reset tests use — see python_network_recovery_test.py.
    pa._reset_hook(frozenset(), frozenset([("synthetic", "1.2.3.4", 24)]))

    if not _wait_for(writer_seen, DISCOVERY_TIMEOUT_SEC):
        print(
            "no post-reset discovery event — listener was not re-installed?",
            file=sys.stderr,
        )
        return 1
    return 0


def test_unregister():
    # Unregister path: on_discovered_endpoint(None) clears the user callback so it
    # stops firing (the listener itself stays attached to drive the internal
    # match-publisher default); a later re-register restores the callback. A third
    # participant with a live callback (pc) is a positive control — once IT
    # observes the post-unregister publisher we know discovery has propagated in
    # the domain, so asserting the unregistered participant did NOT observe it is
    # meaningful rather than a race that merely checked too early.
    pa = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )
    pb = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )
    pc = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )

    lock = threading.Lock()
    events_a = []  # the participant under test
    events_c = []  # positive-control observer
    pa.on_discovered_endpoint(_record_events(lock, events_a))
    pc.on_discovered_endpoint(_record_events(lock, events_c))

    def a_saw(topic):
        with lock:
            return any(
                d and k == provizio_dds.EndpointKind.DATA_WRITER and t == topic
                for (_p, t, ty, k, d) in events_a
            )

    # Baseline — pa observes the first publisher while registered.
    topic1 = "provizio_dds_py_discovery_test_topic_unreg1"
    pub1 = provizio_dds.Publisher(pb, topic1, provizio_dds.StringPubSubType)
    if not _wait_for(lambda: a_saw(topic1), DISCOVERY_TIMEOUT_SEC):
        print("no pre-unregister discovery event", file=sys.stderr)
        return 1

    # Unregister pa and drain both sinks.
    pa.on_discovered_endpoint(None)
    with lock:
        events_a.clear()
        events_c.clear()

    # A fresh publisher: the control participant must still see it; pa, having
    # unregistered, must not.
    topic2 = "provizio_dds_py_discovery_test_topic_unreg2"
    pub2 = provizio_dds.Publisher(pb, topic2, provizio_dds.StringPubSubType)

    def c_saw_topic2():
        with lock:
            return any(d and t == topic2 for (_p, t, ty, k, d) in events_c)

    if not _wait_for(c_saw_topic2, DISCOVERY_TIMEOUT_SEC):
        print(
            "control participant never saw the post-unregister publisher — test inconclusive",
            file=sys.stderr,
        )
        return 1

    with lock:
        pa_saw_topic2 = any(t == topic2 for (_p, t, ty, k, d) in events_a)
    assert not pa_saw_topic2, (
        "callback fired after on_discovered_endpoint(None) unregistered it"
    )

    # Re-register: a subsequent publisher must reach the callback again.
    pa.on_discovered_endpoint(_record_events(lock, events_a))
    topic3 = "provizio_dds_py_discovery_test_topic_unreg3"
    pub3 = provizio_dds.Publisher(pb, topic3, provizio_dds.StringPubSubType)
    if not _wait_for(lambda: a_saw(topic3), DISCOVERY_TIMEOUT_SEC):
        print("callback did not fire after re-registration", file=sys.stderr)
        return 1
    return 0


_TESTS = {
    "type_registry": test_type_registry,
    "discovers_writer": test_discovers_writer,
    "kind_filter": test_kind_filter,
    "constructor_callback": test_constructor_callback,
    "callback_receives_participant": test_callback_receives_participant,
    "survives_reset": test_survives_reset,
    "unregister": test_unregister,
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
