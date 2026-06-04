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
# Cross-version compat publisher for sensor_msgs::msg::PointCloud2 — the
# dominant payload type on Provizio's radar topics, so a wire-compat
# regression here would manifest as silent point-cloud loss across mixed-
# version fleets. Builds a deterministic 2-point radar cloud via the
# provizio_dds.point_cloud2 helper (whose API is identical in 1.10.x and
# 3.x) and publishes it repeatedly. The matching subscriber asserts that
# the metadata fields, the field descriptors, and the raw `data` byte
# blob arrive byte-identical on the other side of the version boundary.

import sys
import time
import provizio_dds

CROSS_COMPAT_DOMAIN_ID = 42
CROSS_COMPAT_POINTCLOUD2_TOPIC_NAME = "provizio_dds_cross_compat_pointcloud2_topic"

# Frame metadata locked in so the subscriber can match exactly. The
# (sec, nanosec) pair is chosen as plain integers (no near-overflow
# corner cases) — we're checking wire transport, not Time arithmetic.
HEADER_SEC = 1717000000
HEADER_NANOSEC = 123456789
FRAME_ID = "cross_compat_radar_frame"

# Two radar points with all-finite floats — NaN/Inf would still encode
# fine on the wire but make Python equality awkward on the receiver, and
# the goal here is to verify CDR round-trip, not float-comparison
# semantics. Six fields per point matches make_radar_point_cloud's
# (x, y, z, radar_relative_radial_velocity, signal_to_noise_ratio,
# ground_relative_radial_velocity) layout.
POINTS = [
    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    [1.5, 2.5, 3.5, 4.5, 5.5, 6.5],
]

WAIT_TIME = 0.2
PUBLISH_TIMES = 50


header = provizio_dds.point_cloud2.make_header(HEADER_SEC, HEADER_NANOSEC, FRAME_ID)
cloud = provizio_dds.point_cloud2.make_radar_point_cloud(header, POINTS)

publisher = provizio_dds.Publisher(
    provizio_dds.make_domain_participant(CROSS_COMPAT_DOMAIN_ID),
    CROSS_COMPAT_POINTCLOUD2_TOPIC_NAME,
    provizio_dds.PointCloud2PubSubType,
    lambda _, has_subscriber: print(
        "cross_compat_pointcloud2_publisher: " +
        ("first subscriber matched" if has_subscriber else "all subscribers unmatched")))

successful_times = 0
for _ in range(PUBLISH_TIMES):
    successful_times += 1 if publisher.publish(cloud) else 0
    time.sleep(WAIT_TIME)

print(f"cross_compat_pointcloud2_publisher: Successfully published {successful_times} times")
sys.exit(0 if successful_times > 0 else 1)
