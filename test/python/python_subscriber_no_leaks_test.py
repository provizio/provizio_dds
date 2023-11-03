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
Simple test DDS publisher written in Python
"""
import sys
import provizio_dds
import gc

TEST_TOPIC_NAME = "provizio_dds_test_python_subscriber_no_leaks_"
PUBLISH_TIMES = 25

num_objects_after_the_first_subscriber = 0
for i in range(PUBLISH_TIMES):
    subscriber = provizio_dds.Subscriber(
        provizio_dds.make_domain_participant(), TEST_TOPIC_NAME + str(i), provizio_dds.StringPubSubType, provizio_dds.String, lambda data: print(data), lambda has_publisher: print(has_publisher))
    del subscriber
    gc.collect()

    num_objects_now = len(gc.get_objects())
    if i == 0:
        num_objects_after_the_first_subscriber = num_objects_now
    else:
        if num_objects_now > num_objects_after_the_first_subscriber:
            print(
                f"There seems to be a leak: number of objects in gc increased by {num_objects_now - num_objects_after_the_first_subscriber}")
            sys.exit(1)

print("python_subscriber_no_leaks_test: No leaks detected!")
sys.exit(0)
