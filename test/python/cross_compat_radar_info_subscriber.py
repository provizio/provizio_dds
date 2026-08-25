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
#
# Cross-version compat subscriber for provizio::msg::radar_info — see the
# matching cross_compat_radar_info_publisher.py for the schema rationale.
# Asserts that every advertised range field round-trips byte-for-byte
# across the version boundary; a failure here is the early-warning signal
# that the radar_range enum -> uint32 migration broke deployed-fleet
# subscribers.

import sys
import threading
import provizio_dds

CROSS_COMPAT_DOMAIN_ID = 42
CROSS_COMPAT_RADAR_INFO_TOPIC_NAME = "provizio_dds_cross_compat_radar_info_topic"

EXPECTED_SERIAL_NUMBER = "cross_compat_radar_42"
EXPECTED_SUPPORTED_RANGES = [0, 1, 2, 3, 4]
EXPECTED_CURRENT_RANGE = 2  # long_range / LONG_RANGE
WAIT_TIME = 10


received_message = None
had_publishers = False
cv = threading.Condition()


def _ranges_to_list(seq):
    """Normalise the SWIG-wrapped vector returned by `supported_ranges()`
    into a plain Python list. The legacy build returns a vector of
    `radar_range` enum values; SWIG renders these as ints. The 3.x build
    returns a vector of `uint32_t`. Iteration works the same way on both,
    so we materialise via list() and compare as ints downstream."""
    return [int(v) for v in seq]


def on_message(message):
    with cv:
        global received_message
        received_message = {
            "serial_number": message.serial_number(),
            "supported_ranges": _ranges_to_list(message.supported_ranges()),
            "current_range": int(message.current_range()),
        }
        cv.notify()


def on_has_publisher_changed_function(has):
    if has:
        with cv:
            global had_publishers
            had_publishers = True
            cv.notify()


subscriber = provizio_dds.Subscriber(
    provizio_dds.make_domain_participant(CROSS_COMPAT_DOMAIN_ID),
    CROSS_COMPAT_RADAR_INFO_TOPIC_NAME,
    provizio_dds.radar_infoPubSubType,
    provizio_dds.radar_info,
    on_message, on_has_publisher_changed_function)


def _matches_expected(msg):
    return (
        msg is not None
        and msg["serial_number"] == EXPECTED_SERIAL_NUMBER
        and msg["supported_ranges"] == EXPECTED_SUPPORTED_RANGES
        and msg["current_range"] == EXPECTED_CURRENT_RANGE
    )


with cv:
    cv.wait_for(lambda: had_publishers and _matches_expected(received_message),
                WAIT_TIME)
    del subscriber

    if not had_publishers:
        print("cross_compat_radar_info_subscriber: never matched a publisher")
        sys.exit(1)
    if received_message is None:
        print("cross_compat_radar_info_subscriber: matched publisher but received no message")
        sys.exit(1)
    if not _matches_expected(received_message):
        print(
            "cross_compat_radar_info_subscriber: field mismatch -- expected "
            f"serial_number={EXPECTED_SERIAL_NUMBER!r}, "
            f"supported_ranges={EXPECTED_SUPPORTED_RANGES}, "
            f"current_range={EXPECTED_CURRENT_RANGE}; got {received_message}")
        sys.exit(1)
    print("cross_compat_radar_info_subscriber: Success!")
    sys.exit(0)
