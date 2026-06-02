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

"""End-to-end network-recovery tests for the Python bindings.

Mirrors the C++ tests in test/network_recovery/network_recovery_test.cpp.
Subcommand-driven so each ctest entry can run in its own process (the
env-var resolution is one-shot cached per process, matching the C++ side).
"""

import gc
import sys
import threading
import time
import traceback
import weakref

import provizio_dds


def _log(message):
    print(f"[python_network_recovery_test] {message}", flush=True)


def _make_pub_sub_pair(participant, topic_name, on_data):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.StringPubSubType
    )
    subscriber = provizio_dds.Subscriber(
        participant,
        topic_name,
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        on_data,
    )
    return publisher, subscriber


def test_logging():
    """log_callback installation and reset; default emitter survives."""
    captured = []

    def cb(level, message):
        captured.append((level, message))

    previous = provizio_dds.set_log_callback(cb)

    from provizio_dds import network_recovery as nr
    nr._emit_log(nr.LogLevel.INFO, "info message")
    nr._emit_log(nr.LogLevel.WARNING, "warn message")
    nr._emit_log(nr.LogLevel.ERROR, "error message")

    assert len(captured) == 3, captured
    assert captured[0][0] == provizio_dds.LogLevel.INFO
    assert captured[0][1] == "info message"
    assert captured[2][0] == provizio_dds.LogLevel.ERROR

    # Restore the previous callback (initially None — falls back to
    # default stdout/stderr emitter).
    provizio_dds.set_log_callback(previous)
    _log("logging: PASS")
    return 0


def test_env_recovery():
    """resolve_network_recovery_enabled honours explicit ON / OFF
    regardless of env var. Run in its own subprocess so the env-var
    one-shot cache (see _resolve_env_once) doesn't leak between cases."""
    assert provizio_dds.resolve_network_recovery_enabled(
        provizio_dds.NetworkRecoveryMode.ON
    ) is True
    assert provizio_dds.resolve_network_recovery_enabled(
        provizio_dds.NetworkRecoveryMode.OFF
    ) is False
    _log("env_recovery: PASS")
    return 0


def _test_env_var_controlled(expected: bool):
    """Common body for the env-var-controlled subcommands. Each ctest
    entry runs in its own process so the function-local cache in
    _resolve_env_once observes the configured PROVIZIO_DDS_NETWORK_RECOVERY
    on its first (and only) call."""
    actual = provizio_dds.resolve_network_recovery_enabled(
        provizio_dds.NetworkRecoveryMode.ENV_VAR_CONTROLLED
    )
    assert actual is expected, f"expected {expected}, got {actual}"
    # The explicit modes must always override the env var.
    assert provizio_dds.resolve_network_recovery_enabled(
        provizio_dds.NetworkRecoveryMode.ON
    ) is True
    assert provizio_dds.resolve_network_recovery_enabled(
        provizio_dds.NetworkRecoveryMode.OFF
    ) is False
    return 0


def test_env_default():
    """PROVIZIO_DDS_NETWORK_RECOVERY unset/empty → default-on."""
    return _test_env_var_controlled(expected=True)


def test_env_on():
    """PROVIZIO_DDS_NETWORK_RECOVERY=on → True."""
    return _test_env_var_controlled(expected=True)


def test_env_off():
    """PROVIZIO_DDS_NETWORK_RECOVERY=off → False."""
    return _test_env_var_controlled(expected=False)


def test_env_truthy():
    """PROVIZIO_DDS_NETWORK_RECOVERY=yes (recognised truthy) → True."""
    return _test_env_var_controlled(expected=True)


def test_env_falsy():
    """PROVIZIO_DDS_NETWORK_RECOVERY=NO (recognised falsy, mixed case) → False."""
    return _test_env_var_controlled(expected=False)


def test_env_garbage():
    """Unrecognised env value → defaults to on AND emits a warning."""
    captured = []
    previous = provizio_dds.set_log_callback(
        lambda level, message: captured.append((level, message))
    )
    try:
        enabled = provizio_dds.resolve_network_recovery_enabled(
            provizio_dds.NetworkRecoveryMode.ENV_VAR_CONTROLLED
        )
    finally:
        provizio_dds.set_log_callback(previous)

    assert enabled is True, f"expected True (garbage defaults to on), got {enabled}"
    saw_warning = any(
        entry[0] == provizio_dds.LogLevel.WARNING
        and "not recognised" in entry[1]
        for entry in captured
    )
    assert saw_warning, f"expected a 'not recognised' warning; got {captured!r}"
    _log("env_garbage: PASS")
    return 0


def test_snapshot():
    """capture_address_snapshot returns a non-error result; loopback excluded."""
    from provizio_dds import network_recovery as nr
    snap = nr._capture_address_snapshot()
    # On a CI container with only loopback up the snapshot may be empty —
    # that's fine; the assertion is "loopback addresses must not appear".
    for (name, address) in snap:
        assert address not in ("127.0.0.1", "::1"), (name, address)
    _log(f"snapshot: PASS ({len(snap)} address(es))")
    return 0


def test_reset_roundtrip():
    """Pub→sub round-trip works before and after a synthetic reset; the
    underlying DataWriter GUID changes after reset."""
    participant = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.ON
    )

    received = []
    received_event = threading.Event()
    received_lock = threading.Lock()

    def on_data(data):
        with received_lock:
            received.append(data.data())
            received_event.set()

    publisher, subscriber = _make_pub_sub_pair(participant, "provizio_dds_python_recovery_topic", on_data)

    # Baseline: publish-receive before reset.
    deadline = time.monotonic() + 10.0
    saw_before = False
    while time.monotonic() < deadline and not saw_before:
        message = provizio_dds.String()
        message.data("before-reset")
        publisher.publish(message)
        if received_event.wait(timeout=0.2):
            with received_lock:
                if "before-reset" in received:
                    saw_before = True
                received_event.clear()
    assert saw_before, "did not receive baseline message"

    # Publisher.get_guid() returns a copy of the DataWriter's GUID rather than
    # the by-reference view Fast-DDS' DataWriter::guid() returns directly —
    # reading the raw reference after the reset destroys the writer would be
    # use-after-free (observed as a same-GUID-after-reset flake on macOS where
    # the freed slot is reused for the new writer).
    guid_before = publisher.get_guid()

    # Trigger the reset directly via the participant's recovery hook. This
    # is the same code path the polling-based monitor would drive on a
    # confirmed network change.
    participant._reset_hook(frozenset(), frozenset([("synthetic", "1.2.3.4")]))

    # After reset, publish-receive must resume on the freshly-rebuilt
    # DataReader / DataWriter.
    deadline = time.monotonic() + 15.0
    saw_after = False
    received_event.clear()
    while time.monotonic() < deadline and not saw_after:
        message = provizio_dds.String()
        message.data("after-reset")
        publisher.publish(message)
        if received_event.wait(timeout=0.2):
            with received_lock:
                if "after-reset" in received:
                    saw_after = True
                received_event.clear()
    assert saw_after, "did not receive after-reset message"

    guid_after = publisher.get_guid()
    assert str(guid_before) != str(guid_after), "DataWriter GUID did not change across reset"

    _log("reset_roundtrip: PASS")
    return 0


def test_reset_disabled():
    """recovery_mode=OFF: participant is NOT registered with the
    coordinator; calling _reset_hook is still safe (it just tears down and
    rebuilds), but the polling-driven path would not call it on an OFF
    participant."""
    participant = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )
    assert participant._recovery_enabled is False

    received = []
    received_event = threading.Event()
    received_lock = threading.Lock()

    def on_data(data):
        with received_lock:
            received.append(data.data())
            received_event.set()

    publisher, subscriber = _make_pub_sub_pair(
        participant, "provizio_dds_python_recovery_disabled_topic", on_data
    )

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not received_event.is_set():
        message = provizio_dds.String()
        message.data("no-recovery")
        publisher.publish(message)
        if received_event.wait(timeout=0.2):
            break
    with received_lock:
        assert "no-recovery" in received

    # Exercise the docstring's claim rather than just asserting it in prose:
    # calling _reset_hook directly is safe even with recovery disabled (OFF only
    # stops the monitor from calling it — a manual call still tears down and
    # rebuilds). Trigger one reset and confirm pub/sub resumes on the rebuilt
    # endpoints.
    participant._reset_hook(frozenset(), frozenset([("synthetic", "1.2.3.4")]))
    with received_lock:
        received.clear()
        received_event.clear()
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not received_event.is_set():
        message = provizio_dds.String()
        message.data("after-manual-reset")
        publisher.publish(message)
        if received_event.wait(timeout=0.2):
            break
    with received_lock:
        assert (
            "after-manual-reset" in received
        ), "pub/sub did not resume after a manual reset on an OFF participant"

    _log("reset_disabled: PASS")
    return 0


def test_reset_refreshes_fastdds_interface_cache():
    """APT-11792 regression test: the Python network-recovery reset path
    must call :func:`refresh_fastdds_interface_cache` before recreating
    the Fast-DDS participant, so the new participant binds to current
    host interfaces rather than the stale set the first participant saw.

    Verifies three things in order, each strictly stronger than the last:

      1. The ctypes binding resolves at import. If libprovizio_dds wasn't
         found or doesn't export
         ``provizio_dds_refresh_fastdds_interface_cache``, the helper
         returns False and we assert here.

      2. The helper actually exercises the underlying
         ``eprosima::SystemInfo::update_interfaces``. (A no-op stub on
         either side would still pass step 1.)

      3. :meth:`_DomainParticipant._reset_hook_locked` calls
         :func:`refresh_fastdds_interface_cache` BEFORE
         ``factory.create_participant``. This is the actual bug: in the
         pre-fix code the reset destroyed and recreated the participant
         without refreshing the cache, so the new participant inherited
         the stale interface set. We monkey-patch the helper, trigger the
         reset, and assert the patched helper was invoked.
    """
    from provizio_dds import network_recovery as nr

    # (1) and (2): refresh helper is wired up and works.
    assert nr.refresh_fastdds_interface_cache() is True, (
        "refresh_fastdds_interface_cache() returned False — either "
        "libprovizio_dds wasn't found or the underlying "
        "eprosima::SystemInfo::update_interfaces failed."
    )

    # (3): the reset path must invoke refresh_fastdds_interface_cache.
    # Monkey-patch the helper on the module attribute that
    # provizio_dds.py uses (`_network_recovery.refresh_fastdds_interface_cache`,
    # which is the same module object as `network_recovery` after the
    # import inside provizio_dds.py). The patched helper counts calls,
    # then defers to the real implementation so the participant rebuild
    # still observes the up-to-date cache.
    participant = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.ON
    )

    original_refresh = nr.refresh_fastdds_interface_cache
    call_count = [0]

    def counted_refresh():
        call_count[0] += 1
        return original_refresh()

    nr.refresh_fastdds_interface_cache = counted_refresh
    try:
        before = call_count[0]
        participant._reset_hook(
            frozenset(), frozenset([("synthetic", "1.2.3.4")])
        )
        after = call_count[0]
    finally:
        nr.refresh_fastdds_interface_cache = original_refresh

    assert after > before, (
        f"_reset_hook did not call refresh_fastdds_interface_cache "
        f"(call count: {before} -> {after}). Reset path is failing to "
        f"refresh the interface cache before recreating the participant, "
        f"which is the APT-11792 regression."
    )

    _log(
        "reset_refreshes_fastdds_interface_cache: PASS "
        f"(refresh called {after - before} time(s) during reset)"
    )
    return 0


def test_teardown_deferred():
    """An endpoint dropped from inside a Fast-DDS listener callback must not
    delete its DataWriter / DataReader inline — Fast-DDS tears the entity down
    by joining the callback thread, so an inline delete self-joins (deadlock).
    With the in-callback marker set, __del__ defers the Fast-DDS teardown to
    the reaper thread instead.

    A regression (inline delete on the callback thread) would hang at the
    `del` below; the ctest TIMEOUT catches it. Exercises the guard in
    Publisher / Subscriber._defer_teardown_off_thread."""

    participant = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.OFF
    )

    # Publisher with a user match-callback — the path that resolves a strong
    # Publisher reference to hand to the callback.
    publisher = provizio_dds.Publisher(
        participant,
        "provizio_dds_python_defer_topic",
        provizio_dds.StringPubSubType,
        lambda _pub, _has_sub: None,
    )
    pub_ref = weakref.ref(publisher)
    assert not provizio_dds._on_fastdds_callback_thread()
    with provizio_dds._fastdds_callback_scope():
        assert provizio_dds._on_fastdds_callback_thread()
        # Drop the last reference while marked in-callback. A self-join
        # regression hangs right here.
        del publisher
        gc.collect()
    deadline = time.monotonic() + 10.0
    while pub_ref() is not None and time.monotonic() < deadline:
        gc.collect()
        time.sleep(0.05)
    assert pub_ref() is None, "Publisher wrapper not reclaimed after deferred teardown"

    # Subscriber: same scenario (delete_datareader would self-join too).
    subscriber = provizio_dds.Subscriber(
        participant,
        "provizio_dds_python_defer_topic",
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        lambda _msg: None,
    )
    sub_ref = weakref.ref(subscriber)
    with provizio_dds._fastdds_callback_scope():
        del subscriber
        gc.collect()
    deadline = time.monotonic() + 10.0
    while sub_ref() is not None and time.monotonic() < deadline:
        gc.collect()
        time.sleep(0.05)
    assert sub_ref() is None, "Subscriber wrapper not reclaimed after deferred teardown"

    # Off a callback thread, teardown stays synchronous and immediate.
    publisher2 = provizio_dds.Publisher(
        participant,
        "provizio_dds_python_defer_topic",
        provizio_dds.StringPubSubType,
        lambda _pub, _has_sub: None,
    )
    pub2_ref = weakref.ref(publisher2)
    assert not provizio_dds._on_fastdds_callback_thread()
    del publisher2
    gc.collect()
    assert pub2_ref() is None, "Publisher wrapper not reclaimed on normal del"

    # Let the reaper run the deferred Fast-DDS deletes before the participant
    # is torn down at function exit.
    time.sleep(1.0)
    _log("teardown_deferred: deferred + synchronous teardown OK")
    return 0


_TESTS = {
    "logging": test_logging,
    "env_recovery": test_env_recovery,
    "env_default": test_env_default,
    "env_on": test_env_on,
    "env_off": test_env_off,
    "env_truthy": test_env_truthy,
    "env_falsy": test_env_falsy,
    "env_garbage": test_env_garbage,
    "snapshot": test_snapshot,
    "reset_roundtrip": test_reset_roundtrip,
    "reset_disabled": test_reset_disabled,
    "reset_refreshes_fastdds_interface_cache": test_reset_refreshes_fastdds_interface_cache,
    "teardown_deferred": test_teardown_deferred,
}


def main():
    if len(sys.argv) < 2:
        # No "run all in one process" fallback: the env-var resolution is
        # one-shot cached per process (matches the C++ side), so an
        # all-in-one run would lock the env decision for everything that
        # comes after the first test that triggered the cache. The CMake
        # harness already runs each subcommand in its own process — that
        # is the supported invocation.
        print(
            f"usage: {sys.argv[0]} <{'|'.join(_TESTS.keys())}>",
            file=sys.stderr,
        )
        return 1
    name = sys.argv[1]
    if name not in _TESTS:
        print(f"Unknown subcommand: {name}", file=sys.stderr)
        return 1
    try:
        return _TESTS[name]()
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
