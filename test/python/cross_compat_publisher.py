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
# Cross-version compat publisher. Mirrors test/python/python_publisher.py
# but on a dedicated DDS domain + topic so the test can safely run in
# parallel with the same-version test suite (which uses domain 0 for
# pub/sub and 14 for request/response).

import os
import time
import sys
import provizio_dds

# The driver (cross_version_compat_test.py) gives every run of this test its own domain and its
# own topic / service names, because CI runs four copies of it on jetson runners that share a
# LAN within seconds of each other, and the halves that are not confined to loopback used to
# meet -- see the comment on _CHILD_ENV there. Falls back to the historical fixed values when
# this script is run by hand.
CROSS_COMPAT_DOMAIN_ID = int(os.environ.get("PROVIZIO_DDS_CROSS_COMPAT_DOMAIN", "42"))
CROSS_COMPAT_NAME_SUFFIX = os.environ.get("PROVIZIO_DDS_CROSS_COMPAT_SUFFIX", "")
CROSS_COMPAT_TOPIC_NAME = "provizio_dds_cross_compat_pubsub_topic" + CROSS_COMPAT_NAME_SUFFIX
TEST_VALUE = "provizio_dds_cross_compat"
WAIT_TIME = 0.2
PUBLISH_TIMES = 50

publisher = provizio_dds.Publisher(
    provizio_dds.make_domain_participant(CROSS_COMPAT_DOMAIN_ID),
    CROSS_COMPAT_TOPIC_NAME, provizio_dds.StringPubSubType,
    lambda _, has_subscriber: print(
        "cross_compat_publisher: " +
        ("first subscriber matched" if has_subscriber else "all subscribers unmatched")))

message = provizio_dds.String()
message.data(TEST_VALUE)
successful_times = 0
for i in range(PUBLISH_TIMES):
    successful_times += 1 if publisher.publish(message) else 0
    time.sleep(WAIT_TIME)

if successful_times == 0:
    print(f"cross_compat_publisher: published NONE of the {PUBLISH_TIMES} messages")
    sys.exit(1)

# Printed on the success path only: the driver (cross_version_compat_test.py) reads a
# "Success" line as proof this side finished the work the pair asserts, so a failure
# must never carry the word. Its failure line is above, and says what went wrong.
print(f"cross_compat_publisher: Successfully published {successful_times} times")
sys.exit(0)
