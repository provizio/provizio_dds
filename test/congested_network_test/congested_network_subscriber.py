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

name = "congested_network_subscriber"
topic = "rt/congested_network_test_topic"
done_text = "DONE"
mutex = threading.Lock()
has_been_matched = False
is_matched = False
done = False
times_received = 0
max_time_to_match = 90  # As we'll be testing in REALLY congested networks
time_test_started = time.time()


def test_time():
    now = time.time()
    return f"@{now}[{now - time_test_started}]sec"


def on_matched(matched):
    global has_been_matched
    global is_matched
    print(f"{name} {test_time()}: matched = {matched}")
    with mutex:
        if matched:
            has_been_matched = True
        is_matched = matched


def on_message(message):
    global times_received
    global done
    times_received += 1
    print(f"{name} {test_time()}: Received {message.data()}")
    if message.data() == done_text:
        with mutex:
            done = True


subscriber = provizio_dds.Subscriber(
    provizio_dds.make_domain_participant(),
    topic,
    provizio_dds.StringPubSubType,
    provizio_dds.String,
    on_message,
    on_matched,
    # Use an explicit (eager) reader rather than the match-publisher default. This test
    # verifies DATA DELIVERY under heavy congestion, not discovery latency. The
    # match-publisher default defers DataReader creation until the writer is discovered,
    # which serializes discovery into two round-trips (discover writer -> build reader ->
    # match); under extreme congestion (75% load) that tail latency can exceed the match
    # window and flake. An eager reader matches via a single parallel SEDP exchange,
    # keeping this delivery test deterministic. BEST_EFFORT restores the historical
    # default for this test; the match-publisher default is covered by
    # match_publisher_default_test.
    reliability_kind=provizio_dds.BEST_EFFORT_RELIABILITY_QOS,
)

print(f"{name} {test_time()}: Waiting till matched...")
time_started_waiting = time.time()
while True:
    with mutex:
        if has_been_matched:
            break
    if time.time() - time_started_waiting > max_time_to_match:
        print(f"{name} {test_time()}: Failed to match on time!")
        exit(1)
    time.sleep(0.1)

print(f"{name} {test_time()}: Waiting till {done_text} received...")
while not done:
    with mutex:
        if not is_matched:
            break
    time.sleep(0.1)

print(f"{name} {test_time()}: Done receiving: {times_received} times received!")
del subscriber

exit(0 if times_received > 0 else 1)
