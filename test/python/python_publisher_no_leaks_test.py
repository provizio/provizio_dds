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

TEST_TOPIC_NAME = "provizio_dds_test_python_publisher_no_leaks_"
TEST_VALUE = "test message #"
PUBLISH_TIMES = 25

# The check races the process' own background threads: a participant runs the
# network-recovery monitor and coalescer (and Fast-DDS its own threads), any of which can
# allocate a tracked container between the gc.collect() above and the count below. So the
# count is read as a FLOOR rather than as a value: transient allocations only ever push it
# up for an instant, while a leak raises the smallest count the run can reach and keeps
# raising it. Comparing the floor of the last iterations against the floor of the first
# ones therefore keeps the flake fix (observed on jetson (ARM) while four x86 jobs of the
# same commit passed) without the blind spot a flat tolerance leaves -- a leak of one
# object per iteration stays under a 16-object slack for the whole run, but moves the
# floor by that much every iteration.
#
# The windows are the first and last quarters of the run, so each averages out a single
# unlucky iteration, and they never overlap.
FLOOR_WINDOW = max(2, PUBLISH_TIMES // 4)
# Slack on the floor difference, for the one-off allocations a steady state still makes:
# an interned string, a lazily built cache. A leak is monotonic, so it clears this within
# a few iterations; noise does not.
MAX_FLOOR_GROWTH = 4

counts = []
message = provizio_dds.String()
for i in range(PUBLISH_TIMES):
    message.data(TEST_VALUE + str(i))
    publisher = provizio_dds.Publisher(
        provizio_dds.make_domain_participant(), TEST_TOPIC_NAME + str(i), provizio_dds.StringPubSubType, lambda _, has_subscriber: print(has_subscriber))
    del publisher
    gc.collect()

    # The first iteration is warm-up, not a measurement: it is what imports the type
    # support, builds the participant's caches and starts the background threads.
    if i > 0:
        counts.append(len(gc.get_objects()))

baseline_floor = min(counts[:FLOOR_WINDOW])
final_floor = min(counts[-FLOOR_WINDOW:])
growth = final_floor - baseline_floor
if growth > MAX_FLOOR_GROWTH:
    print(
        f"There seems to be a leak: the smallest object count rose by {growth} between the "
        f"first and last {FLOOR_WINDOW} of {len(counts)} measured iteration(s) "
        f"({baseline_floor} -> {final_floor}), beyond the {MAX_FLOOR_GROWTH} allowed for "
        f"one-off steady-state allocations")
    sys.exit(1)

print(f"python_publisher_no_leaks_test: No leaks detected! (object floor {baseline_floor} "
      f"-> {final_floor} over {len(counts)} iterations)")
sys.exit(0)
