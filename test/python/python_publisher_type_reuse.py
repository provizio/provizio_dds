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
Simple test DDS publisher written in Python
"""
import time
import sys
import provizio_dds

TEST_TOPIC_NAME_1 = "provizio_dds_test_simplest_pub_sub_topic_1"
TEST_TOPIC_NAME_2 = "provizio_dds_test_simplest_pub_sub_topic_2"
TEST_VALUE = "provizio_dds_test"
WAIT_TIME = 0.2
PUBLISH_TIMES = 50

participant = provizio_dds.make_domain_participant()

publisher_1 = provizio_dds.Publisher(
    participant,
    TEST_TOPIC_NAME_1,
    provizio_dds.StringPubSubType,
    lambda _, has_subscriber: print(
        "python_publisher_1: "
        + (
            "first subscriber matched"
            if has_subscriber
            else "all subscribers unmatched"
        )
    ),
)

publisher_2 = provizio_dds.Publisher(
    participant,
    TEST_TOPIC_NAME_2,
    provizio_dds.StringPubSubType,
    lambda _, has_subscriber: print(
        "python_publisher_2: "
        + (
            "first subscriber matched"
            if has_subscriber
            else "all subscribers unmatched"
        )
    ),
)

message = provizio_dds.String()
message.data(TEST_VALUE)
successful_times = 0
for i in range(PUBLISH_TIMES):
    successful_times += 1 if publisher_1.publish(message) else 0
    successful_times += 1 if publisher_2.publish(message) else 0
    time.sleep(WAIT_TIME)

print(f"python_publisher: Successfully published {successful_times} times")

sys.exit(0 if successful_times > 0 else 1)
