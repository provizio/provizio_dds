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

import faulthandler
import gc
import os
import sys
import threading
import time
import traceback
import weakref

import provizio_dds


# A case that stops making progress used to reach its ctest timeout and be killed with
# no output and no stacks — an unexplained hang and nothing to diagnose it with (seen as
# a 45 s timeout of reset_roundtrip on macOS while its neighbours took ~1 s). This dumps
# every thread's stack, repeatedly, once a case runs longer than any of them ever should.
# exit=False and repeat=True deliberately: the dump must not decide the case's outcome,
# only describe where each thread is waiting, and repeating distinguishes "wedged" from
# "slow" by showing whether the stacks move.
# Scaled the same way CMake scales the ctest timeouts (provizio_dds_finalize_tests
# multiplies them by PROVIZIO_DDS_TEST_TIMEOUT_SCALE for sanitizer builds), so a
# slow-but-passing Debug/ASan case does not dump every thread's stack just for being slow.
# The same scale on its own, for the cases whose OWN waits have to follow the
# environment the way the ctest timeouts do.
_TIMEOUT_SCALE = float(os.environ.get("PROVIZIO_DDS_TEST_TIMEOUT_SCALE", "1") or "1")
_WATCHDOG_SEC = 20.0 * _TIMEOUT_SCALE


def _arm_watchdog():
    faulthandler.dump_traceback_later(_WATCHDOG_SEC, repeat=True, exit=False)


def _disarm_watchdog():
    """Cancel the watchdog so a case that finished cannot dump stacks while the process
    tears its participants down."""
    faulthandler.cancel_dump_traceback_later()


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
    for (name, address, prefix_length) in snap:
        assert address not in ("127.0.0.1", "::1"), (name, address, prefix_length)
    _log(f"snapshot: PASS ({len(snap)} address(es))")
    return 0


def test_reset_roundtrip():
    """Pub→sub round-trip works before and after a synthetic reset; the
    underlying DataWriter GUID changes after reset."""
    participant = provizio_dds.make_domain_participant(
        0, provizio_dds.NetworkRecoveryMode.ON
    )
    _log("reset_roundtrip: participant created")

    received = []
    received_event = threading.Event()
    received_lock = threading.Lock()

    def on_data(data):
        with received_lock:
            received.append(data.data())
            received_event.set()

    publisher, subscriber = _make_pub_sub_pair(participant, "provizio_dds_python_recovery_topic", on_data)
    _log("reset_roundtrip: endpoints created")

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
    _log("reset_roundtrip: baseline round-trip ok")

    # Publisher.get_guid() returns a copy of the DataWriter's GUID rather than
    # the by-reference view Fast-DDS' DataWriter::guid() returns directly —
    # reading the raw reference after the reset destroys the writer would be
    # use-after-free (observed as a same-GUID-after-reset flake on macOS where
    # the freed slot is reused for the new writer).
    guid_before = publisher.get_guid()

    # Trigger the reset directly via the participant's recovery hook. This
    # is the same code path the polling-based monitor would drive on a
    # confirmed network change.
    _log("reset_roundtrip: triggering reset")
    participant._reset_hook(frozenset(), frozenset([("synthetic", "1.2.3.4", 24)]))
    _log("reset_roundtrip: reset returned")

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
    _log("reset_roundtrip: post-reset round-trip ok")

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
    participant._reset_hook(frozenset(), frozenset([("synthetic", "1.2.3.4", 24)]))
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
        "refresh_fastdds_interface_cache() returned False -- either "
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
            frozenset(), frozenset([("synthetic", "1.2.3.4", 24)])
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


def test_coalescer_resets_on_transient_flap():
    """A transient flap (a DDS-relevant address that leaves and returns within
    the quiet period) nets to an unchanged end-snapshot but must still rebuild —
    the Fast-DDS sockets bound to that address were torn down while it was gone.
    The coordinator's _on_network_event compares the burst-START snapshot too;
    inject_transient_for_test drives that path with a synthetic start snapshot.
    The second half confirms a same-end-state event with NO transient signal is
    still correctly skipped (no spurious reset) — exactly what the old end-only
    logic did for the transient too, which was the bug."""
    from provizio_dds import network_recovery as nr

    # Recovery ON so the coordinator (and monitor) are live (mirrors the C++ test).
    participant = provizio_dds.make_domain_participant(0, provizio_dds.NetworkRecoveryMode.ON)
    assert participant is not None
    coordinator = nr._NetworkRecoveryCoordinator.instance()

    # 1) Transient flap → MUST reset.
    reset_before = coordinator.reset_count
    skipped_before = coordinator.skipped_reset_count
    coordinator.inject_transient_for_test()
    assert coordinator.reset_count == reset_before + 1, (coordinator.reset_count, reset_before)
    assert coordinator.skipped_reset_count == skipped_before, (coordinator.skipped_reset_count, skipped_before)

    # 2) Same end-state but NO transient signal → correctly skipped (no spurious reset).
    snap = nr._capture_address_snapshot()
    reset_before2 = coordinator.reset_count
    skipped_before2 = coordinator.skipped_reset_count
    coordinator.inject_change_for_test(snap, snap)
    assert coordinator.reset_count == reset_before2, (coordinator.reset_count, reset_before2)
    assert coordinator.skipped_reset_count == skipped_before2 + 1, (coordinator.skipped_reset_count, skipped_before2)

    _log("coalescer_resets_on_transient_flap: PASS")
    return 0


def test_no_rebuild_on_address_loss():
    """An address going away is not worth a rebuild: nothing can be bound to what is gone,
    and tearing down endpoints that still work over the remaining interfaces costs every
    in-flight sample for no gain. The rebuild belongs to the moment the address comes BACK,
    which is when it can achieve something — so this drives the pair and asserts exactly
    one rebuild across both halves.

    Mirrors the C++ network_recovery_no_rebuild_on_address_loss."""
    from provizio_dds import network_recovery as nr

    participant = provizio_dds.make_domain_participant(0, provizio_dds.NetworkRecoveryMode.ON)
    assert participant is not None
    coordinator = nr._NetworkRecoveryCoordinator.instance()

    lost = ("provizio_test_lost_if", "203.0.113.9", 24)
    with_address = frozenset(nr._capture_address_snapshot()) | {lost}
    without_address = with_address - {lost}

    # Half one: the address goes away and stays away.
    reset_before = coordinator.reset_count
    skipped_before = coordinator.skipped_reset_count
    coordinator.inject_change_for_test(with_address, without_address)
    assert coordinator.reset_count == reset_before, (coordinator.reset_count, reset_before)
    assert coordinator.skipped_reset_count == skipped_before + 1, (
        coordinator.skipped_reset_count,
        skipped_before,
    )

    # Half two: the same address returns, which is what a rebuild can act on.
    coordinator.inject_change_for_test(without_address, with_address)
    assert coordinator.reset_count == reset_before + 1, (coordinator.reset_count, reset_before)

    _log(f"no_rebuild_on_address_loss: PASS (reset_count {reset_before} -> {coordinator.reset_count})")
    return 0


def test_rebuild_on_address_change():
    """The "rebuild only for what was gained" rule is about snapshot ENTRIES, not interfaces,
    and an entry is (interface name, address, prefix length). So re-addressing an interface that
    never went away is a gain — the old entry leaves and a new one arrives — and so is
    re-subnetting one without changing its address at all, which changes which peers
    Fast-DDS considers on-link. Both must rebuild; only a purely subtractive change must
    not.

    Mirrors the C++ network_recovery_rebuild_on_address_change."""
    from provizio_dds import network_recovery as nr

    participant = provizio_dds.make_domain_participant(0, provizio_dds.NetworkRecoveryMode.ON)
    assert participant is not None
    coordinator = nr._NetworkRecoveryCoordinator.instance()

    before = frozenset({("provizio_test_dhcp_if", "203.0.113.20", 24)})
    readdressed = frozenset({("provizio_test_dhcp_if", "203.0.113.21", 24)})
    resubnetted = frozenset({("provizio_test_dhcp_if", "203.0.113.21", 16)})

    # A new address on an interface that never left.
    reset_before = coordinator.reset_count
    coordinator.inject_change_for_test(before, readdressed)
    assert coordinator.reset_count == reset_before + 1, (coordinator.reset_count, reset_before)

    # Same address, different prefix.
    reset_before_resubnet = coordinator.reset_count
    coordinator.inject_change_for_test(readdressed, resubnetted)
    assert coordinator.reset_count == reset_before_resubnet + 1, (
        coordinator.reset_count,
        reset_before_resubnet,
    )

    _log(f"rebuild_on_address_change: PASS (reset_count {reset_before} -> {coordinator.reset_count})")
    return 0


def test_no_baseline_rebuilds_for_first_readable_list():
    """The interface read at construction can fail just as any later one can, and the monitor's
    baseline is seeded from it. A failure there is not "no addresses" and not "the addresses we
    have" -- it is not knowing, and specifically not knowing what the participants bound to.

    So the first readable list REBUILDS rather than being quietly adopted: an interface can come
    up while the list is unreadable, and adopting it would lose that rebuild permanently, since
    an adopted address is no longer a gain against any later snapshot. An extra rebuild costs one
    reconnect; a missed one costs the interface until the process restarts.

    What this pins, for both backends: a failed seed leaves no baseline, the first readable list
    is reported as all-new exactly once, an empty one is reported as nothing (there is nothing to
    bind), and a real change measured from the baseline is still reported after that.

    Mirrors the C++ network_recovery_no_baseline_rebuilds_for_first_readable_list and
    network_recovery_no_baseline_empty_list_is_not_a_rebuild."""
    from provizio_dds import network_recovery as nr

    snapshot_a = frozenset({("provizio_test_seeded_if", "203.0.113.30", 24)})
    snapshot_b = snapshot_a | {("provizio_test_seeded_if2", "203.0.113.31", 24)}

    def failing():
        raise OSError(12, "getifaddrs failed: Cannot allocate memory")

    def check(make_monitor, label, first_readable):
        events = []
        real_capture = nr._capture_address_snapshot
        nr._capture_address_snapshot = failing
        nr._enumeration_failure_reported = False
        monitor = None
        try:
            # Constructed while the interface list is unreadable: no baseline, NOT an empty one.
            monitor = make_monitor(lambda *args: events.append(args))
            assert monitor.initial_snapshot() is None, (label, monitor.initial_snapshot())

            # Reads start succeeding. With no baseline every visible address counts as new, so a
            # non-empty list is reported once (old is empty, which is what makes the coordinator
            # rebuild) while an empty one has nothing to bind and is reported not at all.
            nr._capture_address_snapshot = lambda: first_readable
            monitor.run_safety_net_tick_for_test()
            expected = 1 if first_readable else 0
            assert len(events) == expected, (label, events)
            if expected:
                old, new, _burst_start = events[0]
                assert old == frozenset(), (label, old)
                assert new == first_readable, (label, new)

            # Seeing the same list again is not a change, which proves the baseline was stored.
            monitor.run_safety_net_tick_for_test()
            assert len(events) == expected, (label, events)

            # A genuine change measured from that baseline is still reported.
            nr._capture_address_snapshot = lambda: snapshot_b
            monitor.run_safety_net_tick_for_test()
            assert len(events) == expected + 1, (label, events)
            old, new, _burst_start = events[-1]
            assert old == first_readable, (label, old)
            assert new == snapshot_b, (label, new)
        finally:
            nr._capture_address_snapshot = real_capture
            nr._enumeration_failure_reported = False
            if monitor is not None:
                monitor.stop()

    # A long interval keeps each monitor's own thread from ticking underneath the test; every
    # tick here is driven explicitly. Each backend is run twice: once where the first readable
    # list has addresses to bind, once where it is empty.
    backends = [(lambda on_event: nr._PollingNetworkMonitor(on_event, 3600.0), "polling")]
    if sys.platform.startswith("linux"):
        backends.append(
            (lambda on_event: nr._NetlinkNetworkMonitor(on_event, 3600.0, None, 0.0), "netlink")
        )
    for make_monitor, label in backends:
        check(make_monitor, f"{label}/non-empty", snapshot_a)
        check(make_monitor, f"{label}/empty", frozenset())

    _log(f"no_baseline_rebuilds_for_first_readable_list: PASS ({len(backends)} backend(s))")
    return 0


def test_unreadable_interfaces_are_not_a_change():
    """Asking the OS for its interfaces can fail — on macOS getifaddrs is a
    sysctl(NET_RT_IFLIST) pair that can lose a race with a routing-table change. Reporting
    that as an empty snapshot is indistinguishable from a host that genuinely has no usable
    address (a container whose only device is a filtered-out veth reads exactly that), so a
    failed read used to present itself as every address disappearing: every participant
    rebuilt for nothing, then rebuilt again when the next read succeeded, with any in-flight
    request/response lost to it.

    Mirrors the C++ network_recovery_unreadable_interfaces_are_not_a_change."""
    from provizio_dds import network_recovery as nr

    def raising_capture():
        raise OSError(12, "getifaddrs failed: Cannot allocate memory")

    captured = []
    previous = provizio_dds.set_log_callback(
        lambda level, message: captured.append((level, message))
    )
    real_capture = nr._capture_address_snapshot
    nr._capture_address_snapshot = raising_capture
    nr._enumeration_failure_reported = False
    try:
        first = nr._try_capture_address_snapshot()
        second = nr._try_capture_address_snapshot()
        warnings_while_failing = [
            message for level, message in captured if level == provizio_dds.LogLevel.WARNING
        ]

        # A successful read ends the streak, so a later failure is reported again rather
        # than being silenced for the life of the process.
        nr._capture_address_snapshot = real_capture
        recovered = nr._try_capture_address_snapshot()
        nr._capture_address_snapshot = raising_capture
        nr._try_capture_address_snapshot()
        warnings_after_recovery = [
            message for level, message in captured if level == provizio_dds.LogLevel.WARNING
        ]
    finally:
        nr._capture_address_snapshot = real_capture
        nr._enumeration_failure_reported = False
        provizio_dds.set_log_callback(previous)

    # A failed read is not a snapshot, so no decision can be made from it.
    assert first is None, first
    assert second is None, second
    # Warned once per run of failures, not once per attempt: a poller asks every few seconds.
    assert len(warnings_while_failing) == 1, warnings_while_failing
    assert recovered is not None, "a readable interface list must produce a snapshot"
    assert len(warnings_after_recovery) == 2, warnings_after_recovery

    _log(
        f"unreadable_interfaces_are_not_a_change: PASS "
        f"({len(warnings_after_recovery)} warning(s) across two failure streaks)"
    )
    return 0


def test_snapshot_prefix_length():
    """The prefix length is part of the snapshot identity, so re-subnetting an
    interface without changing its address counts as a network change (Fast-DDS
    derives its netmask-based locator filtering from the prefix)."""
    from provizio_dds import network_recovery as nr

    snapshot = nr._capture_address_snapshot()

    with_prefix = 0
    for entry in snapshot:
        assert len(entry) == 3, entry
        name, addr, prefix = entry
        assert isinstance(prefix, int), entry
        # Hard invariant: a prefix cannot exceed the address family's width.
        max_prefix = 32 if ":" not in addr else 128
        assert 0 <= prefix <= max_prefix, entry
        if prefix > 0:
            with_prefix += 1
    # Softer: a CI container can legitimately have an empty snapshot, and exotic
    # point-to-point devices report no mask — but if anything was found, at least one
    # entry must carry a real prefix, or we aren't reading netmasks at all.
    if snapshot:
        assert with_prefix > 0, snapshot

    # A netmask-only change must be a *different* snapshot, i.e. it must reach the
    # coordinator as a change rather than being dismissed as "nothing happened".
    # Comparing two hand-built frozensets would be a tautology, so drive the real
    # decision path: same interface and address, different prefix.
    from provizio_dds import network_recovery as nr_mod

    coordinator = nr_mod._NetworkRecoveryCoordinator.instance()
    slash24 = frozenset({("provizio_test_if", "192.0.2.10", 24)})
    slash16 = frozenset({("provizio_test_if", "192.0.2.10", 16)})
    reset_before = coordinator.reset_count
    skipped_before = coordinator.skipped_reset_count
    coordinator.inject_change_for_test(slash24, slash16)
    assert coordinator.reset_count == reset_before + 1, (coordinator.reset_count, reset_before)
    assert coordinator.skipped_reset_count == skipped_before, (coordinator.skipped_reset_count, skipped_before)
    # ... while an identical prefix is correctly judged unchanged.
    coordinator.inject_change_for_test(slash24, slash24)
    assert coordinator.reset_count == reset_before + 1, (coordinator.reset_count, reset_before)
    assert coordinator.skipped_reset_count == skipped_before + 1, (coordinator.skipped_reset_count, skipped_before)

    _log(f"snapshot_prefix_length: PASS ({len(snapshot)} entries, {with_prefix} with a prefix)")
    return 0


def test_netmask_read_is_bounded():
    """Regression: on BSD / macOS ``getifaddrs`` stores each netmask truncated to its
    significant bytes (``sa_len`` omits trailing zeros), so the prefix helper must read
    only the bytes that are actually present. Reading the full fixed-size field and
    clamping afterwards is too late — the out-of-bounds access has already happened,
    reaching into the neighbouring sockaddr of the same buffer or past the allocation
    entirely for the last entry.

    Runs on every platform: the BSD layout is simulated, so Linux CI covers the macOS
    behaviour that cannot otherwise be exercised here."""
    import ctypes

    from provizio_dds import network_recovery as nr

    # The module picks its sockaddr layout at import from the platform, so simulating BSD
    # means swapping both the flag and the struct: there, sa_len is byte 0 and sa_family
    # byte 1, and reading the Linux layout's 16-bit sa_family off a BSD buffer is garbage.
    class BsdSockaddr(ctypes.Structure):
        _fields_ = [
            ("sa_len", ctypes.c_ubyte),
            ("sa_family", ctypes.c_ubyte),
            ("sa_data", ctypes.c_ubyte * 14),
        ]

    buf = BsdSockaddr()
    buf.sa_family = nr._AF_INET
    # Fill everything past the header with 0xFF. Bytes beyond sa_len stand in for the
    # neighbouring sockaddr: if they are read, the prefix comes out as /32 instead of /8.
    for index in range(14):
        buf.sa_data[index] = 0xFF
    pointer = ctypes.cast(ctypes.byref(buf), ctypes.POINTER(BsdSockaddr))

    original_flag, original_struct = nr._IS_BSD_SOCKADDR, nr._Sockaddr
    nr._IS_BSD_SOCKADDR, nr._Sockaddr = True, BsdSockaddr
    try:
        buf.sa_len = 5  # offsetof(sin_addr) == 4, plus ONE significant mask byte
        short_mask = nr._prefix_length_from_netmask(pointer)
        buf.sa_len = 8  # a full four-byte mask
        full_mask = nr._prefix_length_from_netmask(pointer)
        buf.sa_len = 4  # no mask bytes at all
        empty_mask = nr._prefix_length_from_netmask(pointer)
    finally:
        nr._IS_BSD_SOCKADDR, nr._Sockaddr = original_flag, original_struct

    assert short_mask == 8, f"expected /8 from the single present byte, got /{short_mask}"
    assert full_mask == 32, full_mask
    assert empty_mask == 0, empty_mask

    _log(f"netmask_read_is_bounded: PASS (/{short_mask}, /{full_mask}, /{empty_mask})")
    return 0


def test_linked_list_walk_advances() -> int:
    """Regression: the interface walks step through linked lists the OS owns, and a
    ``continue`` that skipped the step stranded the cursor on one node forever — on
    Windows the unicast walk hung the process outright on any adapter carrying a
    tentative, deprecated or duplicate address, which is a state an adapter passes
    through on every DHCP lease. The step therefore belongs to the iterator rather
    than to each branch of the body, so no branch can omit it.

    Runs on every platform: the list is synthetic, so Linux CI covers the structure of
    the Windows walk, which cannot otherwise be exercised here."""
    import ctypes

    from provizio_dds import network_recovery as nr

    class Node(ctypes.Structure):
        pass

    # Self-referential, so the field list is declared after the class exists.
    Node._fields_ = [("next", ctypes.POINTER(Node)), ("value", ctypes.c_int)]

    # The nodes stay owned by this list; the fields hold borrowed pointers into it.
    nodes = [Node() for _ in range(4)]
    for index, node in enumerate(nodes[:-1]):
        node.next = ctypes.pointer(nodes[index + 1])
    for index, node in enumerate(nodes):
        node.value = index

    seen = []
    # Bounded: a walk that fails to advance must fail the case, not hang it.
    for step, node in enumerate(nr._iter_linked_nodes(ctypes.pointer(nodes[0]), "next")):
        assert step < 2 * len(nodes), f"the walk did not advance past {seen}"
        if node.value % 2 == 1:
            continue  # the branch that used to strand the cursor
        seen.append(node.value)
    assert seen == [0, 2], seen

    empty = list(nr._iter_linked_nodes(ctypes.POINTER(Node)(), "next"))
    assert not empty, f"a null head must yield nothing, yielded {len(empty)}"

    _log(f"linked_list_walk_advances: PASS (visited {seen} of {len(nodes)})")
    return 0


def test_extra_interfaces_env():
    """PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES parsing: whitespace trimmed,
    empty entries dropped. Runs in its own process — the value is parsed once."""
    from provizio_dds import network_recovery as nr

    names = nr._force_included_interfaces()
    assert names == frozenset({"docker0", "br-test", "veth9"}), names

    _log(f"extra_interfaces_env: PASS ({len(names)} force-included)")
    return 0


def test_netlink_binds_before_snapshot():
    """Regression: the netlink socket must be bound BEFORE the baseline snapshot is
    captured. In the other order, an address change landing in between is both
    missed by the socket and already reflected in the baseline — so it is never
    noticed at all, which is exactly the boot-time race the monitor exists for."""
    from provizio_dds import network_recovery as nr

    if not sys.platform.startswith("linux"):
        _log("netlink_binds_before_snapshot: SKIP (netlink is Linux-only)")
        return 0

    order = []
    real_bind = nr.socket.socket.bind
    real_capture = nr._capture_address_snapshot

    def tracking_bind(self, address):
        order.append("bind")
        return real_bind(self, address)

    def tracking_capture():
        order.append("capture")
        return real_capture()

    nr.socket.socket.bind = tracking_bind
    nr._capture_address_snapshot = tracking_capture
    try:
        monitor = nr._NetlinkNetworkMonitor(lambda *_: None, 3.0, None, 0.0)
    finally:
        nr.socket.socket.bind = real_bind
        nr._capture_address_snapshot = real_capture
    try:
        assert order[:2] == ["bind", "capture"], order
    finally:
        monitor.stop()

    _log(f"netlink_binds_before_snapshot: PASS (order={order[:2]})")
    return 0


def test_listener_drain_reports_a_stall_until_it_ends():
    """A callback that does not return says so for as long as it does not, and the log
    gets an end as well as a beginning.

    One line cannot say whether a stall is over: the wait is unbounded and the caller
    holds the registration lock throughout, so an operator seeing a single warning could
    not tell a stall that cleared from one still going hours later. Mirrors the C++
    listener_drain cases in test/listener_drain.

    Hermetic: threads, a condition variable and the log callback, with no DDS entity and
    no traffic. The drain's reporting period is an argument for exactly this reason."""
    from provizio_dds import network_recovery as nr

    # Short enough to keep the case under a second, long enough that a loaded runner
    # cannot mistake one slice for two.
    slice_sec = 0.05 * _TIMEOUT_SCALE
    wedged_sec = slice_sec * 8

    captured = []
    captured_lock = threading.Lock()

    def _count(needle):
        with captured_lock:
            return sum(1 for message in captured if needle in message)

    stall_needle = "listener drain has been waiting"
    completion_needle = "listener drain completed after"

    previous = provizio_dds.set_log_callback(
        lambda level, message: captured.append(message)
    )
    try:
        # The ordinary drain -- a callback that has already returned -- says nothing at
        # all. Unconditional, the completion line would put a warning into the log of
        # every reset on a healthy system.
        quiet = nr.ListenerDrain(stall_warning_period=slice_sec)
        with quiet.scope():
            pass
        quiet.detach_and_drain()
        if _count(stall_needle) or _count(completion_needle):
            _log("listener_drain_reports_a_stall_until_it_ends: FAIL (a quiet drain logged)")
            return 1

        drain = nr.ListenerDrain(stall_warning_period=slice_sec)
        entered = threading.Event()
        returned = []

        def _wedged_callback():
            with drain.scope():
                entered.set()
                time.sleep(wedged_sec)
                returned.append(True)

        callback = threading.Thread(target=_wedged_callback)
        callback.start()
        # The drain has nothing to wait for until the callback is in flight, so starting
        # before that would test nothing at all.
        entered.wait(timeout=30 * _TIMEOUT_SCALE)

        drain.detach_and_drain()
        # Read after the drain returned: the callback appends before leaving its scope,
        # and the drain may not return until that scope has been left. A drain that
        # returned early would be a teardown under a running callback in production.
        drained_after_callback = bool(returned)
        callback.join()
    finally:
        provizio_dds.set_log_callback(previous)

    passed = True
    if not drained_after_callback:
        passed = False
        _log("  detach_and_drain returned while a callback was still in flight")
    # At least two, not exactly N: the count is what the scheduler grants in the time the
    # callback is held, and pinning it would make this a timing assertion. Two is what
    # distinguishes a heartbeat from a one-shot report.
    stalls = _count(stall_needle)
    if stalls < 2:
        passed = False
        _log(f"  expected the stall warning to repeat, saw it {stalls} time(s)")
    # Exactly one, and only because a stall was reported: it supersedes the warnings.
    completions = _count(completion_needle)
    if completions != 1:
        passed = False
        _log(f"  expected exactly one completion line, saw {completions}")

    _log(
        f"listener_drain_reports_a_stall_until_it_ends: {'PASS' if passed else 'FAIL'} "
        f"({stalls} stall report(s))"
    )
    return 0 if passed else 1


def test_netlink_kinds_match_the_kernel():
    """The RTM_GETLINK dump actually delivers, checked against what the kernel says
    elsewhere.

    Everything downstream of it degrades silently: a dump whose replies were all discarded
    (the sequence and port-id checks failing, say, on a kernel or netns that does not echo
    the port id the way this assumes) yields an empty kind map, and VPN classification then
    falls back to name prefixes alone with nothing logged -- losing the signal that catches
    a renamed WireGuard device. Nothing pinned that end of it before: the sibling case
    asserts only that the socket is bound before the snapshot runs.

    /sys/class/net/<dev>/uevent carries a DEVTYPE, which OVERLAPS IFLA_INFO_KIND without
    matching it: a physical Wi-Fi NIC reports DEVTYPE=wlan and no netlink kind at all,
    because it is not a virtual link. So only the virtual types both sides name are
    compared, and the same set is what proves the dump delivered -- one of them present in
    sysfs but missing from the map means every reply was discarded. Where the host has no
    such device -- a bare-metal runner whose only NIC is physical -- there is nothing to
    compare and the case says so rather than passing vacuously."""
    import glob

    from provizio_dds import network_recovery as nr

    if sys.platform != "linux":
        _log("netlink_kinds_match_the_kernel: SKIP (netlink is Linux-only)")
        return 0

    # The device types rtnetlink names with an IFLA_INFO_KIND, spelled the same way in
    # both places. Anything outside this set (wlan, and the physical devices that report
    # no DEVTYPE at all) tells us nothing about whether the dump worked.
    virtual_devtypes = {
        "bond",
        "bridge",
        "geneve",
        "macvlan",
        "tun",
        "veth",
        "vlan",
        "vxlan",
        "wireguard",
    }

    from_sysfs = {}
    for uevent in glob.glob("/sys/class/net/*/uevent"):
        device = uevent.split("/")[-2]
        try:
            with open(uevent, "r", encoding="ascii", errors="replace") as handle:
                for line in handle:
                    if line.startswith("DEVTYPE="):
                        devtype = line.strip().split("=", 1)[1]
                        if devtype in virtual_devtypes:
                            from_sysfs[device] = devtype
        except OSError:
            continue

    kinds = nr._fetch_link_kinds_linux()
    if not from_sysfs:
        _log(
            "netlink_kinds_match_the_kernel: SKIP (no virtual device on this host, so "
            f"nothing proves the dump either way; netlink returned {len(kinds)} kind(s))"
        )
        return 0

    # The map is by interface index, so resolve each name the way the walk does.
    import socket as socket_module

    mismatched = []
    for device, devtype in sorted(from_sysfs.items()):
        try:
            index = socket_module.if_nametoindex(device)
        except OSError:
            continue
        reported = kinds.get(index)
        if reported != devtype:
            mismatched.append(f"{device}: netlink={reported!r} sysfs={devtype!r}")

    assert not mismatched, "; ".join(mismatched)
    _log(
        f"netlink_kinds_match_the_kernel: PASS ({len(from_sysfs)} virtual device(s) "
        f"cross-checked, {len(kinds)} kind(s) from netlink)"
    )
    return 0


def test_safety_net_detects_missed_change():
    """The periodic tick must catch a change no kernel event reported — a dropped
    netlink datagram, a transition on a channel we don't subscribe to, or a change
    that raced the monitor's startup. Simulated by making the live capture return a
    set the monitor's last-known value cannot match."""
    from provizio_dds import network_recovery as nr

    participant = provizio_dds.make_domain_participant(0, provizio_dds.NetworkRecoveryMode.ON)
    assert participant is not None
    coordinator = nr._NetworkRecoveryCoordinator.instance()

    reset_before = coordinator.reset_count
    real_capture = nr._capture_address_snapshot
    # TEST-NET-3, RFC 5737 — never routable.
    fake = frozenset({("provizio_test_missing_if", "203.0.113.9", 24)})
    nr._capture_address_snapshot = lambda: fake
    try:
        assert coordinator.run_safety_net_tick_for_test()
        assert coordinator.reset_count == reset_before + 1, (coordinator.reset_count, reset_before)

        # The tick stores what it found, so an immediately following one is a no-op —
        # otherwise every period would rebuild every participant.
        assert coordinator.run_safety_net_tick_for_test()
        assert coordinator.reset_count == reset_before + 1, (coordinator.reset_count, reset_before)
    finally:
        nr._capture_address_snapshot = real_capture

    _log("safety_net_detects_missed_change: PASS")
    return 0


def test_safety_net_reopens_dead_monitor():
    """A monitor that gave up on an unrecoverable channel error used to leave the
    process without auto-recovery for the rest of its life. The periodic tick must
    reopen it."""
    from provizio_dds import network_recovery as nr

    participant = provizio_dds.make_domain_participant(0, provizio_dds.NetworkRecoveryMode.ON)
    assert participant is not None
    coordinator = nr._NetworkRecoveryCoordinator.instance()

    assert coordinator.monitor_alive_for_test()
    if not coordinator.kill_monitor_for_test():
        _log("safety_net_reopens_dead_monitor: SKIP (no killable channel on this backend)")
        return 0
    assert not coordinator.monitor_alive_for_test()

    assert coordinator.run_safety_net_tick_for_test()
    assert coordinator.monitor_alive_for_test()

    _log("safety_net_reopens_dead_monitor: PASS")
    return 0


def test_safety_net_retries_failed_rebuild():
    """A reset that fails part-way leaves the participant inert with its endpoints
    torn down, and no further network event is guaranteed to arrive. The periodic
    tick must retry it — and only it."""
    from provizio_dds import network_recovery as nr

    participant = provizio_dds.make_domain_participant(0, provizio_dds.NetworkRecoveryMode.ON)
    assert participant is not None
    coordinator = nr._NetworkRecoveryCoordinator.instance()

    assert not participant._recovery_retry_needed

    # Simulate the aftermath of a rebuild that could not recreate the participant.
    participant._recovery_retry_needed = True
    reset_before = coordinator.reset_count

    assert coordinator.run_safety_net_tick_for_test()
    # The retry ran a full reset of this participant, which cleared the flag.
    assert not participant._recovery_retry_needed
    assert coordinator.reset_count > reset_before, (coordinator.reset_count, reset_before)

    # With nothing left to retry and no snapshot change, a further tick is a no-op.
    reset_after = coordinator.reset_count
    assert coordinator.run_safety_net_tick_for_test()
    assert coordinator.reset_count == reset_after, (coordinator.reset_count, reset_after)

    _log("safety_net_retries_failed_rebuild: PASS")
    return 0


def test_safety_net_retries_while_interfaces_are_unreadable():
    """An unreadable interface list must not cancel the retry pass.

    The retry re-attempts a rebuild that already failed; it needs to know nothing about
    what changed, and the C++ counterpart runs apply_reset(retry_only) before it reads a
    snapshot at all -- handing participants none, since trigger_network_recovery_reset
    takes none. Gating it on a read, as this side did, meant a failing enumeration
    silently removed the retry: the counter never moved, so the bound was never reached,
    so the exhaustion warning never fired, and a participant left torn down by a failed
    rebuild could stay dead for as long as the reads kept failing with nothing logged.
    macOS' getifaddrs is a sysctl(NET_RT_IFLIST) size-then-fetch pair that fails exactly
    that way.

    Driven through stub participants rather than real ones: what is under test is the
    coordinator's ordering, and a stub makes both the retry and the give-up path
    deterministic without a live monitor or DDS traffic."""
    from provizio_dds import network_recovery as nr

    class _StubParticipant:
        def __init__(self, recovers: bool):
            self._recovery_retry_needed = True
            self._recovers = recovers
            self.resets = 0

        def _reset_hook(self, old_snapshot, new_snapshot):
            self.resets += 1
            # A participant that recovers clears the flag; one that never does keeps it
            # set, which is what drives the pass towards its bound.
            self._recovery_retry_needed = not self._recovers

    coordinator = nr._NetworkRecoveryCoordinator.instance()
    messages = []
    original_capture = nr._try_capture_address_snapshot
    original_emit = nr._emit_log
    nr._emit_log = lambda level, message: messages.append((level, message))
    # Every read fails, for the whole case.
    nr._try_capture_address_snapshot = lambda: None
    try:
        recovering = _StubParticipant(recovers=True)
        coordinator.register_participant(recovering, recovering._reset_hook)
        coordinator._on_safety_net_tick()
        assert recovering.resets == 1, recovering.resets
        assert not recovering._recovery_retry_needed

        # A participant that never comes back still reaches the bound and reports once --
        # the counter moving at all is the part the old ordering skipped entirely. Cleared
        # first because the pass above consumed one, which is the bookkeeping the real code
        # does for itself only once nothing needs retrying at all.
        coordinator._consecutive_retry_passes = 0
        stuck = _StubParticipant(recovers=False)
        coordinator.register_participant(stuck, stuck._reset_hook)
        for _ in range(nr._NetworkRecoveryCoordinator._MAX_CONSECUTIVE_RETRY_PASSES + 2):
            coordinator._on_safety_net_tick()
        assert stuck.resets == nr._NetworkRecoveryCoordinator._MAX_CONSECUTIVE_RETRY_PASSES, stuck.resets
        gave_up = [m for _level, m in messages if "Retrying further" in m]
        assert len(gave_up) == 1, [m for _l, m in messages][-3:]
    finally:
        nr._try_capture_address_snapshot = original_capture
        nr._emit_log = original_emit
        coordinator._consecutive_retry_passes = 0
        coordinator._retry_exhaustion_reported = False

    _log(
        f"safety_net_retries_while_interfaces_are_unreadable: PASS "
        f"({recovering.resets} retry, {stuck.resets} before giving up, every read failing)"
    )
    return 0


def test_safety_net_retry_gives_up_after_bound():
    """A participant whose rebuild NEVER succeeds must stop being retried after
    _MAX_CONSECUTIVE_RETRY_PASSES consecutive passes — one error log, then
    silence — so it cannot churn its healthy siblings once per tick forever. A
    real network change re-arms the retrying. Mirrors the C++
    max_consecutive_retry_passes bound."""
    from provizio_dds import network_recovery as nr

    participant = provizio_dds.make_domain_participant(0, provizio_dds.NetworkRecoveryMode.ON)
    assert participant is not None
    coordinator = nr._NetworkRecoveryCoordinator.instance()

    # Simulate a participant that can never be rebuilt: the retry pass itself
    # succeeds (clearing the flag), so re-raise the flag after every tick as a
    # persistently-failing rebuild would.
    bound = coordinator._MAX_CONSECUTIVE_RETRY_PASSES
    reset_before = coordinator.reset_count
    for expected_pass in range(1, bound + 1):
        participant._recovery_retry_needed = True
        assert coordinator.run_safety_net_tick_for_test()
        assert coordinator.reset_count == reset_before + expected_pass, (
            coordinator.reset_count,
            reset_before,
            expected_pass,
        )

    # The bound is exhausted: further ticks must NOT reset any more.
    participant._recovery_retry_needed = True
    assert coordinator.run_safety_net_tick_for_test()
    assert coordinator.reset_count == reset_before + bound, (coordinator.reset_count, reset_before, bound)

    # A real network change re-arms the retrying: the change itself resets all
    # participants (clearing the flag), after which a fresh failed rebuild is
    # retried by the tick again.
    slash24 = frozenset({("provizio_test_if", "192.0.2.10", 24)})
    slash16 = frozenset({("provizio_test_if", "192.0.2.10", 16)})
    coordinator.inject_change_for_test(slash24, slash16)
    rearmed_reset_count = coordinator.reset_count
    participant._recovery_retry_needed = True
    assert coordinator.run_safety_net_tick_for_test()
    assert coordinator.reset_count == rearmed_reset_count + 1, (coordinator.reset_count, rearmed_reset_count)
    assert not participant._recovery_retry_needed

    _log("safety_net_retry_gives_up_after_bound: PASS")
    return 0


def test_safety_net_env():
    """PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC parses with the cross-language
    integer grammar (mirrors the C++ resolve_safety_net_period), including the
    extremes: fractional / trailing-character / negative / non-ASCII-whitespace /
    beyond-int64 values all fall back to the default, oversized values clamp to a
    day, 0 is accepted (disables the periodic check). The C++ side has equivalent
    CI coverage in test/network_recovery/; this locks the parity claim on the
    Python side."""
    from provizio_dds import network_recovery as nr

    default = nr._DEFAULT_SAFETY_NET_SEC
    cases = (
        ("30", 30.0),
        ("0", 0.0),  # disables the periodic check
        ("+5", 5.0),
        ("0.5", default),  # fractional: rejected, like C++ stoll's full-consume check
        ("30s", default),  # trailing characters
        ("-5", default),  # negative
        ("abc", default),  # non-numeric
        ("99999999999999999999", default),  # > int64: rejected, like C++ stoll's out_of_range
        ("\u00a030", default),  # NBSP: non-ASCII whitespace is rejected, like C++ byte-wise isspace
        ("1" * 5000, default),  # beyond int()'s conversion-length limit (Python 3.11+)
        ("999999999", 86400.0),  # clamped to a day
    )
    for raw, expected in cases:
        os.environ[nr._SAFETY_NET_ENV] = raw
        got = nr._resolve_safety_net_period()
        assert got == expected, (raw, got, expected)
    os.environ.pop(nr._SAFETY_NET_ENV, None)
    assert nr._resolve_safety_net_period() == default

    _log("safety_net_env: PASS")
    return 0


def _tunnel_identity():
    """A synthetic tunnel interface for whatever platform the test runs on: (name, kind)
    on POSIX, (name, friendly, description, if_type) on Windows.

    Synthetic because the host is not a controllable input -- no runner has a tunnel up,
    so every assertion about how one is treated would otherwise be vacuous. The C++ suite
    uses the same identities for the same reason (see vpn_interfaces_test.cpp)."""
    if sys.platform == "win32":
        return ("{00000000-0000-0000-0000-provizio-test}", "Tailscale", "Tailscale Tunnel", 131)
    if sys.platform == "darwin":
        # Every macOS VPN lands on a utunN device -- the prefix that is ALSO in the
        # snapshot's own name-exclusion list, which is what made the override unable to
        # re-admit it.
        return ("utun9", "")
    return ("tailscale0", "wireguard")


def _ordinary_identity():
    if sys.platform == "win32":
        return ("{11111111-1111-1111-1111-provizio-test}", "Ethernet 2",
                "Intel(R) Ethernet Connection I219-LM", 6)
    if sys.platform == "darwin":
        return ("en0", "")
    return ("eth0", "")


def _virtual_identity():
    """Excluded by a heuristic that has nothing to do with VPNs -- container plumbing or a
    hypervisor adapter. The override must not re-admit these with the tunnels."""
    if sys.platform == "win32":
        return ("{22222222-2222-2222-2222-provizio-test}", "vEthernet (WSL)",
                "Hyper-V Virtual Ethernet Adapter", 6)
    if sys.platform == "darwin":
        return ("bridge0", "")
    return ("docker0", "bridge")


def _snapshot_policy_excludes(nr, identity):
    if sys.platform == "win32":
        return nr._snapshot_policy_excludes_windows(*identity)
    return nr._snapshot_policy_excludes_posix(*identity)


def test_snapshot_policy_excludes_tunnel():
    """With no override, an interface the transports refuse to bind is kept out of the
    change-detection snapshot too; an ordinary NIC is not."""
    from provizio_dds import network_recovery as nr

    assert _snapshot_policy_excludes(nr, _tunnel_identity())
    assert not _snapshot_policy_excludes(nr, _ordinary_identity())
    assert _snapshot_policy_excludes(nr, _virtual_identity())
    _log("snapshot_policy_excludes_tunnel: PASS")
    return 0


def test_snapshot_policy_follows_transports():
    """A tunnel stays in the snapshot once a participant reports that the exclusion never
    reached its transports.

    The mirror of the C++ snapshot_policy_follows_transports case, and it exists for the
    same failure: where the caller owns the transport configuration -- their own XML,
    FASTDDS_BUILTIN_TRANSPORTS, descriptors they configured -- DDS binds and announces the
    tunnel after all. Dropping it from change detection then would leave a re-auth or a
    reconnect with a dead locator that no rebuild replaces, which is the one outcome the
    two filters may never produce between them.

    Runs last-ish in its own process: the latch is one-way by design, so everything
    asserted after it must be asserted with it set."""
    from provizio_dds import network_recovery as nr

    # Before: the exclusion is believed to apply, so the tunnel is dropped.
    assert _snapshot_policy_excludes(nr, _tunnel_identity())

    nr.report_vpn_exclusion_not_applied()

    # After: the tunnel is watched like any other interface...
    assert not _snapshot_policy_excludes(nr, _tunnel_identity())
    # ...and nothing else moves with it. Container plumbing has nothing to do with who
    # owns the transports and must still be dropped, or every veth churn on a Docker host
    # would rebuild every participant.
    assert _snapshot_policy_excludes(nr, _virtual_identity())
    assert not _snapshot_policy_excludes(nr, _ordinary_identity())
    _log("snapshot_policy_follows_transports: PASS")
    return 0


def test_snapshot_policy_honours_override():
    """PROVIZIO_DDS_ALLOW_VPN_INTERFACES puts tunnels back into change detection, and
    nothing else with them.

    The variable is set by the ctest registration, not here: it is parsed once per
    process on first use, which is inside the very call being asserted on. Mirror of the
    C++ snapshot_policy_honours_override case, and the regression test for a divergence
    that shipped on macOS and Windows -- the override bypassed the VPN filter and the
    interface was then dropped anyway by the utun name prefix / the adapter-type gate,
    leaving the transports binding an interface change detection ignored."""
    from provizio_dds import network_recovery as nr

    assert not _snapshot_policy_excludes(nr, _tunnel_identity())
    assert not _snapshot_policy_excludes(nr, _ordinary_identity())
    # Still excluded: the override is about tunnels, not about the heuristics.
    assert _snapshot_policy_excludes(nr, _virtual_identity())
    _log("snapshot_policy_honours_override: PASS")
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
    "coalescer_resets_on_transient_flap": test_coalescer_resets_on_transient_flap,
    "no_rebuild_on_address_loss": test_no_rebuild_on_address_loss,
    "unreadable_interfaces_are_not_a_change": test_unreadable_interfaces_are_not_a_change,
    "no_baseline_rebuilds_for_first_readable_list": test_no_baseline_rebuilds_for_first_readable_list,
    "rebuild_on_address_change": test_rebuild_on_address_change,
    "snapshot_prefix_length": test_snapshot_prefix_length,
    "netmask_read_is_bounded": test_netmask_read_is_bounded,
    "linked_list_walk_advances": test_linked_list_walk_advances,
    "extra_interfaces_env": test_extra_interfaces_env,
    "netlink_binds_before_snapshot": test_netlink_binds_before_snapshot,
    "safety_net_detects_missed_change": test_safety_net_detects_missed_change,
    "safety_net_reopens_dead_monitor": test_safety_net_reopens_dead_monitor,
    "safety_net_retries_failed_rebuild": test_safety_net_retries_failed_rebuild,
    "safety_net_retry_gives_up_after_bound": test_safety_net_retry_gives_up_after_bound,
    "safety_net_env": test_safety_net_env,
    "snapshot_policy_excludes_tunnel": test_snapshot_policy_excludes_tunnel,
    "snapshot_policy_follows_transports": test_snapshot_policy_follows_transports,
    "safety_net_retries_while_interfaces_are_unreadable": test_safety_net_retries_while_interfaces_are_unreadable,
    "netlink_kinds_match_the_kernel": test_netlink_kinds_match_the_kernel,
    "listener_drain_reports_a_stall_until_it_ends": test_listener_drain_reports_a_stall_until_it_ends,
    "snapshot_policy_honours_override": test_snapshot_policy_honours_override,
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
    _arm_watchdog()
    try:
        return _TESTS[name]()
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        _disarm_watchdog()


if __name__ == "__main__":
    sys.exit(main())
