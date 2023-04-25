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
Simple test DDS subscriber written in Python
"""
import sys
import provizio_dds
import threading

TEST_TOPIC_NAME = "provizio_dds_test_simplest_pub_sub_topic"
TEST_VALUE = "provizio_dds_test"
WAIT_TIME = 6

received_string = None
had_publishers = False
cv = threading.Condition()


def on_message(message):
    """Callback to be invoked on receiving a message"""
    with cv:
        global received_string
        received_string = message.data()
        cv.notify()


def on_has_publisher_changed_function(has):
    if (has):
        with cv:
            global had_publishers
            had_publishers = True
            cv.notify()


subscriber = provizio_dds.Subscriber(
    provizio_dds.make_domain_participant(), TEST_TOPIC_NAME, provizio_dds.StringPubSubType, provizio_dds.String, on_message, on_has_publisher_changed_function)

with cv:
    cv.wait_for(
        lambda: had_publishers and received_string == TEST_VALUE, WAIT_TIME)
    del subscriber  # So we're sure received_string won't be modified anymore

    if (not had_publishers):
        print("python_subscriber: never matched a publisher")
        sys.exit(1)

    if (received_string != TEST_VALUE):
        print(
            "python_subscriber: {expected} was expected but {received} was received!".format(expected=TEST_VALUE, received=received_string))
        sys.exit(1)
    else:
        print("python_subscriber: Success!")
        sys.exit(0)
