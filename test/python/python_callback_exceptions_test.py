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
"""Crash-safety of user callbacks (Python mirror of test/callback_exceptions/).

A user-supplied callback that RAISES must not abort the process — a Python
exception escaping a Fast-DDS director callback becomes a SWIG
DirectorMethodException and std::terminate; a raising Service handler used to kill
its worker thread (silently stopping the service). The exception must instead be
reported through the configurable provizio_dds logging facility
(provizio_dds.set_log_callback), not printed straight to stderr. Each subcommand is
its own ctest entry so per-case failure stays isolated."""

import asyncio
import sys
import threading
import time
import traceback

import provizio_dds

DOMAIN = 0
MATCH_TIMEOUT_SEC = 15.0

# Counts ERROR-level messages routed through the provizio_dds logging facility.
# Installed via set_log_callback, so a passing test also proves the exception is
# reported through the customer-replaceable logger rather than printed directly.
_lock = threading.Lock()
_error_count = [0]


def _recorder(level, _message):
    if level == provizio_dds.LogLevel.ERROR:
        with _lock:
            _error_count[0] += 1


def _install_recorder():
    # Reset the counter so an unrelated ERROR logged before this point can't satisfy the
    # assertions — each case must observe the error raised by its own callback.
    with _lock:
        _error_count[0] = 0
    provizio_dds.set_log_callback(_recorder)


def _restore_recorder():
    provizio_dds.set_log_callback(None)


def _errors():
    with _lock:
        return _error_count[0]


def _make_participant():
    return provizio_dds.make_domain_participant(
        DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
    )


def test_on_data():
    """A raising on-data callback must be logged, not terminate the process."""
    _install_recorder()
    topic = "rt/py_cbexc_on_data"
    sub_participant = _make_participant()
    pub_participant = _make_participant()

    def bad_on_data(_message):
        raise RuntimeError("on_data callback boom")

    # Force an eager RELIABLE reader (not the match-publisher default) so this case stays focused
    # on callback-exception safety and doesn't depend on deferred-build discovery/settling timing.
    subscriber = provizio_dds.Subscriber(
        sub_participant, topic, provizio_dds.StringPubSubType, provizio_dds.String, bad_on_data,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    publisher = provizio_dds.Publisher(
        pub_participant, topic, provizio_dds.StringPubSubType,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    message = provizio_dds.String()
    message.data("x")
    deadline = time.monotonic() + MATCH_TIMEOUT_SEC
    while time.monotonic() < deadline and _errors() == 0:
        publisher.publish(message)
        time.sleep(0.1)

    ok = _errors() > 0 and subscriber.get_num_matched_publishers(MATCH_TIMEOUT_SEC, 0.0) > 0
    print(f"on_data: {'PASS' if ok else 'FAIL'} (errors={_errors()})")
    _restore_recorder()
    return 0 if ok else 1


def test_on_has_publisher_changed():
    """A raising subscriber on-has-publisher-changed callback must be logged, not terminate."""
    _install_recorder()
    topic = "rt/py_cbexc_on_pub_changed"
    sub_participant = _make_participant()
    pub_participant = _make_participant()

    def bad_changed(_matched):
        raise RuntimeError("on_has_publisher_changed callback boom")

    subscriber = provizio_dds.Subscriber(
        sub_participant, topic, provizio_dds.StringPubSubType, provizio_dds.String,
        lambda _message: None,
        on_has_publisher_changed_function=bad_changed,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    publisher = provizio_dds.Publisher(
        pub_participant, topic, provizio_dds.StringPubSubType,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    deadline = time.monotonic() + MATCH_TIMEOUT_SEC
    while time.monotonic() < deadline and _errors() == 0:
        time.sleep(0.1)

    ok = _errors() > 0 and subscriber.get_num_matched_publishers(MATCH_TIMEOUT_SEC, 0.0) > 0
    print(f"on_has_publisher_changed: {'PASS' if ok else 'FAIL'} (errors={_errors()})")
    _restore_recorder()
    return 0 if ok else 1


def test_on_has_subscriber_changed():
    """A raising publisher on-has-subscriber-changed callback must be logged, not terminate."""
    _install_recorder()
    topic = "rt/py_cbexc_on_sub_changed"
    sub_participant = _make_participant()
    pub_participant = _make_participant()

    def bad_changed(_publisher, _matched):
        raise RuntimeError("on_has_subscriber_changed callback boom")

    publisher = provizio_dds.Publisher(
        pub_participant, topic, provizio_dds.StringPubSubType, bad_changed,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    subscriber = provizio_dds.Subscriber(
        sub_participant, topic, provizio_dds.StringPubSubType, provizio_dds.String,
        lambda _message: None,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    deadline = time.monotonic() + MATCH_TIMEOUT_SEC
    while time.monotonic() < deadline and _errors() == 0:
        time.sleep(0.1)

    ok = _errors() > 0 and publisher.get_num_matched_subscribers(MATCH_TIMEOUT_SEC, 0.0) > 0
    print(f"on_has_subscriber_changed: {'PASS' if ok else 'FAIL'} (errors={_errors()})")
    _restore_recorder()
    return 0 if ok else 1


def _run_service_case(handle_request_function, label):
    """Send requests to a service whose handler raises. Before the fix the raising
    handler kills the handler thread / loop (so the error never reaches the logger,
    and for the sync case the service silently stops); after the fix each raise is
    logged through the facility and the handler keeps running."""
    _install_recorder()
    service_name = "provizio_dds_py_cbexc_service"
    service_participant = _make_participant()
    client_participant = _make_participant()

    service = provizio_dds.Service(
        service_participant,
        request_pub_sub_type=provizio_dds.Int32PubSubType,
        request_data_type=provizio_dds.Int32,
        response_pub_sub_type=provizio_dds.Int64PubSubType,
        handle_request_function=handle_request_function,
        service_name=service_name,
    )

    async def send_requests():
        # Wait for >= 2 logged errors (two request cycles), so allow twice the single-
        # match budget: the second cycle is fast once this participant has discovered the
        # service, but the first can be slow in the ROS2-compat Debug containers.
        deadline = time.monotonic() + 2 * MATCH_TIMEOUT_SEC
        # >= 2 logged errors proves the handler survived the first raise (thread /
        # loop still processing). The handler raises, so each request times out.
        while time.monotonic() < deadline and _errors() < 2:
            request_msg = provizio_dds.Int32()
            request_msg.data(1)
            try:
                await asyncio.wait_for(
                    provizio_dds.request(
                        client_participant,
                        provizio_dds.Int32PubSubType,
                        provizio_dds.Int64PubSubType,
                        provizio_dds.Int64,
                        request_msg,
                        service_name=service_name,
                    ),
                    timeout=2.0,
                )
            except asyncio.TimeoutError:
                pass

    asyncio.run(send_requests())

    # Require >= 2 logged errors: the first proves the handler raised without killing the
    # service, the second proves the handler thread/loop SURVIVED the first raise and
    # processed another request (the regression this guards against).
    ok = _errors() >= 2
    print(f"{label}: {'PASS' if ok else 'FAIL'} (errors={_errors()})")
    _restore_recorder()
    return 0 if ok else 1


def test_service_sync():
    """A raising synchronous Service handler."""

    def bad_handler(_request):
        raise RuntimeError("service sync handler boom")

    return _run_service_case(bad_handler, "service_sync")


def test_service_async():
    """A raising asynchronous (coroutine) Service handler."""

    async def bad_handler(_request):
        raise RuntimeError("service async handler boom")

    return _run_service_case(bad_handler, "service_async")


_TESTS = {
    "on_data": test_on_data,
    "on_has_publisher_changed": test_on_has_publisher_changed,
    "on_has_subscriber_changed": test_on_has_subscriber_changed,
    "service_sync": test_service_sync,
    "service_async": test_service_async,
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in _TESTS:
        print(f"usage: {sys.argv[0]} <{'|'.join(_TESTS.keys())}>", file=sys.stderr)
        return 1
    try:
        return _TESTS[sys.argv[1]]()
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
