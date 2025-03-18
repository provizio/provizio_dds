#!/usr/bin/env python3

# Copyright 2025 Provizio Ltd.
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
Simple test DDS subscriber written in Python
"""
import sys
import provizio_dds
import threading

TEST_TOPIC_NAME_1 = "provizio_dds_test_simplest_pub_sub_topic_1"
TEST_TOPIC_NAME_2 = "provizio_dds_test_simplest_pub_sub_topic_2"
TEST_VALUE = "provizio_dds_test"
WAIT_TIME = 6

received_string_1 = None
had_publishers_1 = False
received_string_2 = None
had_publishers_2 = False
cv = threading.Condition()


def on_message_1(message):
    """Callback to be invoked on receiving a message from topic 1"""
    with cv:
        global received_string_1
        received_string_1 = message.data()
        cv.notify()


def on_message_2(message):
    """Callback to be invoked on receiving a message from topic 2"""
    with cv:
        global received_string_2
        received_string_2 = message.data()
        cv.notify()


def on_has_publisher_changed_function_1(has):
    if (has):
        with cv:
            global had_publishers_1
            had_publishers_1 = True
            cv.notify()


def on_has_publisher_changed_function_2(has):
    if has:
        with cv:
            global had_publishers_2
            had_publishers_2 = True
            cv.notify()


participant = provizio_dds.make_domain_participant()

subscriber_1 = provizio_dds.Subscriber(
    participant,
    TEST_TOPIC_NAME_1,
    provizio_dds.StringPubSubType,
    provizio_dds.String,
    on_message_1,
    on_has_publisher_changed_function_1,
)

subscriber_2 = provizio_dds.Subscriber(
    participant,
    TEST_TOPIC_NAME_2,
    provizio_dds.StringPubSubType,
    provizio_dds.String,
    on_message_2,
    on_has_publisher_changed_function_2,
)

with cv:
    cv.wait_for(
        lambda: had_publishers_1
        and received_string_1 == TEST_VALUE
        and had_publishers_2
        and received_string_2 == TEST_VALUE,
        WAIT_TIME,
    )

    # So we're sure received strings won't be modified anymore
    del subscriber_1
    del subscriber_2

    if (not had_publishers_1):
        print("python_subscriber_1: never matched a publisher")
        sys.exit(1)
    if (not had_publishers_2):
        print("python_subscriber_2: never matched a publisher")
        sys.exit(1)

    if (received_string_1 != TEST_VALUE):
        print(
            f"python_subscriber: {TEST_VALUE} was expected but {received_string_1} was received by subscriber_1!"
        )
        sys.exit(1)
        
    if (received_string_2 != TEST_VALUE):
        print(
            f"python_subscriber: {TEST_VALUE} was expected but {received_string_2} was received by subscriber_2!"
        )
        sys.exit(1)
    
    print("python_subscriber: Success!")
    sys.exit(0)
