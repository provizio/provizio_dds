#!/usr/bin/env python3

# Copyright 2023 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
"""
Checks stopping a subscriber works as expected
"""
import sys
import provizio_dds
import threading
import time
import os
import signal

TEST_TOPIC_NAME = "provizio_dds_python_stop_subscriber_test"
TEST_VALUE = "test"
MAX_WAIT_TIME = 5
RECEIVE_NUM_TIMES = 10
PUBLISH_EVERY_SEC = 0.05

pid = os.fork()
if pid > 0:
    # Subscriber
    times_received = 0
    had_publishers = False
    cv = threading.Condition()

    def on_message(_):
        """Callback to be invoked on receiving a message"""
        with cv:
            global times_received
            times_received += 1
            cv.notify_all()


    def on_has_publisher_changed_function(has):
        if (has):
            with cv:
                global had_publishers
                had_publishers = True
                cv.notify_all()

    subscriber = provizio_dds.Subscriber(
        provizio_dds.make_domain_participant(), TEST_TOPIC_NAME, provizio_dds.StringPubSubType, provizio_dds.String, on_message, on_has_publisher_changed_function)

    with cv:
        cv.wait_for(
            lambda: had_publishers and times_received >= RECEIVE_NUM_TIMES, MAX_WAIT_TIME)
        times_received_was = times_received

    # Make sure it kept publishing
    time.sleep(RECEIVE_NUM_TIMES * PUBLISH_EVERY_SEC)
    del subscriber  # So we're sure received_string won't be modified anymore

    with cv:
        if times_received_was == times_received:
            print("Stopped publishing too soon")
            sys.exit(1)
        times_received_was = times_received

    # It kept publishing some more, but we're not receiving anything anymore
    time.sleep(RECEIVE_NUM_TIMES * PUBLISH_EVERY_SEC)
    with cv:
        if times_received_was != times_received:
            print("Somehow kept receiving after being deleted!")
            sys.exit(1)

        if (not had_publishers):
            print("subscriber: never matched a publisher")
            sys.exit(1)

        if (times_received < RECEIVE_NUM_TIMES):
            print(
                f"subscriber: received just {times_received} messages while {RECEIVE_NUM_TIMES} was expected!")
            sys.exit(1)

    os.kill(pid, signal.SIGTERM)
    print("Success!")
else:
    # Publisher
    publisher = provizio_dds.Publisher(provizio_dds.make_domain_participant(), TEST_TOPIC_NAME, provizio_dds.StringPubSubType)

    message = provizio_dds.String()
    message.data(TEST_VALUE)

    stop_flag = False
    def stop():
        global stop_flag
        stop_flag = True

    signal.signal(signal.SIGINT, lambda *_: stop())

    while not stop_flag:
        publisher.publish(message)
        time.sleep(PUBLISH_EVERY_SEC)
