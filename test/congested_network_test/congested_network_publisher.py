#!/usr/bin/env python3

# Copyright 2023 Provizio Ltd.
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

import provizio_dds
import time
import threading

name = "congested_network_publisher"
topic = "rt/congested_network_test_topic"
done_text = "DONE"
mutex = threading.Lock()
has_been_matched = False
is_matched = False
# Sized to the discovery-retry tail at the harshest profile — see the derivation in
# congested_network_subscriber.py. Kept equal to the subscriber's budget so the
# publisher waits for the same match event instead of publishing into the void (its
# pre-match samples are lost to the volatile late-joining reader anyway), and so the
# DONE retransmission window at the end covers a late-matching subscriber.
max_time_to_match = 360
wait_matched_before_publishing = (
    5  # So the subscriber recognizes matching on their side
)
publish_times = 200
publish_interval = 0.2
time_test_started = time.time()


def test_time():
    now = time.time()
    return f"@{now}[{now - time_test_started}]sec"


def on_matched(_, matched):
    global is_matched
    global has_been_matched
    print(f"{name} {test_time()}: matched = {matched}")
    with mutex:
        is_matched = matched
        has_been_matched |= is_matched


publisher = provizio_dds.Publisher(
    provizio_dds.make_domain_participant(),
    topic,
    provizio_dds.StringPubSubType,
    on_matched,
    history_depth=5,  # KEEP_LAST(5)
    # History no longer implies durability (it was untied), so request TRANSIENT_LOCAL
    # explicitly to keep delivering the last messages to late matchers, improving reliability.
    durability_kind=provizio_dds.TRANSIENT_LOCAL_DURABILITY_QOS,
)

message = provizio_dds.String()

print(f"{name} {test_time()}: Waiting till matched...")
time_started_waiting = time.time()
while True:
    with mutex:
        if has_been_matched:
            break
    if time.time() - time_started_waiting > max_time_to_match:
        print(
            f"{name} {test_time()}: Failed to match on time! Will still try publishing..."
        )
        break
    time.sleep(0.1)

# Time for the subscriber to recognize matching prior to sending anything
print(
    f"{name} {test_time()}: Waiting {wait_matched_before_publishing} sec before publishing..."
)
time.sleep(wait_matched_before_publishing)

print(f"{name} {test_time()}: Publishing...")
times_published = 0
for i in range(publish_times):
    text = f"Hello from {name}: #{i}"
    message.data(text)
    if publisher.publish(message):
        times_published += 1
        print(f"{name}: published successfully: {text}")
    else:
        print(f"{name}: Failed to publish: {text}")
    time.sleep(publish_interval)

print(f"{name} {test_time()}: Publishing {done_text} messages...")
time_started_waiting = time.time()
while True:
    with mutex:
        if not is_matched:
            break

    if time.time() - time_started_waiting > max_time_to_match:
        print(f"{name} {test_time()}: Subscriber hasn't unmatched on time!")
        break

    message.data(done_text)
    publisher.publish(message)
    time.sleep(publish_interval)

print(f"{name} {test_time()}: Done publishing: {times_published} times successfully!")
exit(0 if times_published > 0 else 1)
