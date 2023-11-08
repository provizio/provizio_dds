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

import sys
import provizio_dds

# Create a PointCloud2
print("Creating a PointCloud2...")
points = [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
          [1.0, 2.0, 3.0, 4.0, 5.0, float('nan')]]
cloud = provizio_dds.point_cloud2.make_radar_point_cloud(
    provizio_dds.point_cloud2.make_header(10, 20, "test_frame"), points)

# Check the cloud
print("Checking the PointCloud2 metadata...")
assert cloud.header().stamp().sec() == 10
assert cloud.header().stamp().nanosec() == 20
assert cloud.header().frame_id() == "test_frame"
assert cloud.height() == 1
assert cloud.width() == 2
assert cloud.point_step() == 24
assert cloud.row_step() == 48
assert cloud.is_bigendian() == (sys.byteorder == "big")
for i in range(6):
    assert cloud.fields()[i].offset() == i * 4
    assert cloud.fields()[i].count() == 1
    assert cloud.fields()[i].datatype() == provizio_dds.FLOAT32
assert cloud.fields()[0].name() == "x"
assert cloud.fields()[1].name() == "y"
assert cloud.fields()[2].name() == "z"
assert cloud.fields()[3].name() == "radar_relative_radial_velocity"
assert cloud.fields()[4].name() == "signal_to_noise_ratio"
assert cloud.fields()[5].name() == "ground_relative_radial_velocity"

# Read it
print("Reading the PointCloud2 and checking its data...")
read_points = provizio_dds.point_cloud2.read_points_list(cloud)
assert str(
    read_points[0]) == "Point(x=0.1, y=0.2, z=0.3, radar_relative_radial_velocity=0.4, signal_to_noise_ratio=0.5, ground_relative_radial_velocity=0.6)", "Got:" + str(read_points[0])
assert str(
    read_points[1]) == "Point(x=1.0, y=2.0, z=3.0, radar_relative_radial_velocity=4.0, signal_to_noise_ratio=5.0, ground_relative_radial_velocity=nan)", "Got:" + str(read_points[1])

print("Success reading PointCloud2 data")


print("Creating a PointCloud2 embedding entities... ")
entities = [[99, 4, 20.5, -2.0, 1.0, 10.2, 25.0, 0, 0, 0, 1, 2, 5, 2, 254, 254],
            [100, 2, 10.0, 2.0, 1.0, -10.0, 0.0, 0, 0, 0, 1, float('nan'), float('nan'), float('nan'), 254, 12]]

entities_cloud = provizio_dds.point_cloud2.make_radar_entities(provizio_dds.point_cloud2.make_header(10, 20, "test_entities"), entities)

# Check the entities cloud
print("Checking the embedded entities in PointCloud2 metadata...")
assert entities_cloud.fields()[0].name() == "track_id"
assert entities_cloud.fields()[1].name() == "entity_class"
assert entities_cloud.fields()[2].name() == "x"
assert entities_cloud.fields()[3].name() == "y"
assert entities_cloud.fields()[4].name() == "z"
assert entities_cloud.fields()[5].name() == "radar_relative_radial_velocity"
assert entities_cloud.fields()[6].name() == "ground_relative_radial_velocity"
assert entities_cloud.fields()[7].name() == "orientation"
assert entities_cloud.fields()[8].name() == "size"
assert entities_cloud.fields()[8].name() == "entity_confidence"
assert entities_cloud.fields()[8].name() == "entity_class_confidence"

# Read it
print("Reading the PointCloud2 entities and checking its data...")

read_entities = provizio_dds.point_cloud2.read_points_list(entities_cloud)
assert str(
    read_entities[0]) == "Point(track_id=99, entity_class=4, x=20.5, y=-2.0, z=1.0, radar_relative_radial_velocity=10.2, ground_relative_radial_velocity=25.0, orientation=[0, 0, 0, 1], size=[2, 5, 2], entity_confidence=254, entity_class_confidence=254)", "Got:" + str(read_entities[0])
