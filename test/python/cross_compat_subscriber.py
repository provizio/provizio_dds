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
# Cross-version compat subscriber. Mirrors test/python/python_subscriber.py
# but on a dedicated DDS domain + topic so the test can safely run in
# parallel with the same-version test suite.

import sys
import provizio_dds
import threading

CROSS_COMPAT_DOMAIN_ID = 42
CROSS_COMPAT_TOPIC_NAME = "provizio_dds_cross_compat_pubsub_topic"
TEST_VALUE = "provizio_dds_cross_compat"
WAIT_TIME = 10

received_string = None
had_publishers = False
cv = threading.Condition()


def on_message(message):
    with cv:
        global received_string
        received_string = message.data()
        cv.notify()


def on_has_publisher_changed_function(has):
    if has:
        with cv:
            global had_publishers
            had_publishers = True
            cv.notify()


subscriber = provizio_dds.Subscriber(
    provizio_dds.make_domain_participant(CROSS_COMPAT_DOMAIN_ID),
    CROSS_COMPAT_TOPIC_NAME, provizio_dds.StringPubSubType, provizio_dds.String,
    on_message, on_has_publisher_changed_function)

with cv:
    cv.wait_for(lambda: had_publishers and received_string == TEST_VALUE, WAIT_TIME)
    del subscriber

    if not had_publishers:
        print("cross_compat_subscriber: never matched a publisher")
        sys.exit(1)
    if received_string != TEST_VALUE:
        print(f"cross_compat_subscriber: {TEST_VALUE!r} expected but got {received_string!r}")
        sys.exit(1)
    print("cross_compat_subscriber: Success!")
    sys.exit(0)
