#!/usr/bin/env python3

# Copyright 2024 Provizio Ltd.
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
from math import radians, isclose, sqrt, pi, cos
import time
import threading


def test_accumulate_0_accumulated_point_clouds():
    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        2, snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )

    assert len(accumulator.get_points_local_frame_relative()) == 0
    assert (
        len(
            accumulator.get_points_ego_relative(
                provizio_dds.accumulation.RigidTransform([0, 0, 0], [0, 0, 0])
            )
        )
        == 0
    )


def test_accumulate_extrinsics():
    radar_id = "test_radar"
    identity_transform = provizio_dds.accumulation.RigidTransform([0, 0, 0], [0, 0, 0])

    # 10 meters left, 2 up, looking left
    extrinsics = provizio_dds.accumulation.RigidTransform(
        [0, 10, 2], [0, 0, radians(90)]
    )
    # ego relative [1, 2, 3, 4, 5, 6] as seen by test_radar
    point = [-8, -1, 1, 4, 5, 6]

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        1, snr_threshold=0, point_filter=None, allow_no_extrinsics=False
    )
    accumulator.accumulate(radar_id, [point], identity_transform, extrinsics)
    points_local_frame = accumulator.get_points_local_frame_relative()
    points_ego = accumulator.get_points_ego_relative(identity_transform)

    precision = 0.0001

    assert len(points_local_frame) == 1
    assert len(points_ego) == 1
    assert isclose(1, points_ego[0].position[0], abs_tol=precision)
    assert isclose(2, points_ego[0].position[1], abs_tol=precision)
    assert isclose(3, points_ego[0].position[2], abs_tol=precision)
    for dim in range(3):
        points_local_frame[0].position[dim] == points_ego[0].position[dim]


def test_accumulate_move_no_extrinsics():
    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        2, snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )

    radar_id = "test_radar"
    no_rotation = [0, 0, 0]
    # First PC and appropriate ego position
    point_0 = [1, 2, 3, 4, 5, 6]
    ego_pos_0 = [1, 2, 3]
    # Second PC and appropriate ego position
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]
    ego_pos_1 = [6, 5, 4]
    # Later ego position
    ego_pos_now = [1000, 2000, 3000]

    accumulator.accumulate(
        radar_id,
        [point_0],
        provizio_dds.accumulation.RigidTransform(ego_pos_0, no_rotation),
    )
    accumulator.accumulate(
        radar_id,
        [point_1, point_2],
        provizio_dds.accumulation.RigidTransform(ego_pos_1, no_rotation),
    )

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative(
        provizio_dds.accumulation.RigidTransform(ego_pos_now, no_rotation)
    )
    assert len(accumulated_points_ego_relative) == 3

    input_points = [point_0, point_1, point_2]
    ego_poses = [
        ego_pos_0,
        ego_pos_1,
        ego_pos_1,
    ]  # Last ego_pos_1 is not a typo, as point_1 and point_2 share same ego_pos

    # Check all components of all transformed points are as they should be, in appropriate spaces
    for pt in range(3):
        for dim in range(3):
            # local frame relative
            assert (
                accumulated_points_local_frame_relative[pt].position[dim]
                == input_points[pt][dim] + ego_poses[pt][dim]
            )
            assert accumulated_points_local_frame_relative[pt].radar_id == radar_id
            assert (
                accumulated_points_local_frame_relative[pt].snr == input_points[pt][4]
            )
            assert (
                accumulated_points_local_frame_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )

            # ego relative
            assert (
                accumulated_points_ego_relative[pt].position[dim]
                == input_points[pt][dim] + ego_poses[pt][dim] - ego_pos_now[dim]
            )
            assert accumulated_points_ego_relative[pt].snr == input_points[pt][4]
            assert (
                accumulated_points_ego_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )
            assert accumulated_points_ego_relative[pt].radar_id == radar_id


def test_accumulate_move_simple_extrinsics():
    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        2, snr_threshold=0, point_filter=None, allow_no_extrinsics=False
    )

    no_rotation = [0, 0, 0]
    # First PC, radar and appropriate ego position
    radar_id_0 = "test_radar_0"
    point_0 = [1, 2, 3, 4, 5, 6]
    ego_pos_0 = [1, 2, 3]
    extrinsics_pos_0 = [0.1, 0.2, 0.3]
    # Second PC, radar and appropriate ego position
    radar_id_1 = "test_radar_1"
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]
    ego_pos_1 = [6, 5, 4]
    extrinsics_pos_1 = [-0.1, -0.2, -0.3]
    # Later ego position
    ego_pos_now = [1000, 2000, 3000]

    accumulator.accumulate(
        radar_id_0,
        [point_0],
        provizio_dds.accumulation.RigidTransform(ego_pos_0, no_rotation),
        radar_extrinsics=provizio_dds.accumulation.RigidTransform(
            extrinsics_pos_0, no_rotation
        ),
    )
    accumulator.accumulate(
        radar_id_1,
        [point_1, point_2],
        provizio_dds.accumulation.RigidTransform(ego_pos_1, no_rotation),
        radar_extrinsics=provizio_dds.accumulation.RigidTransform(
            extrinsics_pos_1, no_rotation
        ),
    )

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative(
        provizio_dds.accumulation.RigidTransform(ego_pos_now, no_rotation)
    )
    assert len(accumulated_points_ego_relative) == 3

    input_points = [point_0, point_1, point_2]
    ego_poses = [ego_pos_0, ego_pos_1, ego_pos_1]
    radar_ids = [radar_id_0, radar_id_1, radar_id_1]
    extrinsics = [extrinsics_pos_0, extrinsics_pos_1, extrinsics_pos_1]

    # Check all components of all transformed points are as they should be, in appropriate spaces
    for pt in range(3):
        for dim in range(3):
            # local frame relative
            assert (
                accumulated_points_local_frame_relative[pt].position[dim]
                == input_points[pt][dim] + ego_poses[pt][dim] + extrinsics[pt][dim]
            )
            assert accumulated_points_local_frame_relative[pt].radar_id == radar_ids[pt]
            assert (
                accumulated_points_local_frame_relative[pt].snr == input_points[pt][4]
            )
            assert (
                accumulated_points_local_frame_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )

            # ego relative
            assert (
                accumulated_points_ego_relative[pt].position[dim]
                == input_points[pt][dim]
                + ego_poses[pt][dim]
                + extrinsics[pt][dim]
                - ego_pos_now[dim]
            )
            assert accumulated_points_ego_relative[pt].snr == input_points[pt][4]
            assert (
                accumulated_points_ego_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )
            assert accumulated_points_ego_relative[pt].radar_id == radar_ids[pt]


def test_accumulate_move_simple_extrinsics_snr_filter():
    accumulator_snr_5 = provizio_dds.accumulation.PointCloudsAccumulator(
        2, snr_threshold=5, point_filter=None, allow_no_extrinsics=False
    )
    accumulator_snr_6 = provizio_dds.accumulation.PointCloudsAccumulator(
        2, snr_threshold=6, point_filter=None, allow_no_extrinsics=False
    )
    accumulator_snr_60 = provizio_dds.accumulation.PointCloudsAccumulator(
        2, snr_threshold=60, point_filter=None, allow_no_extrinsics=False
    )
    accumulators = [accumulator_snr_5, accumulator_snr_6, accumulator_snr_60]

    no_rotation = [0, 0, 0]
    # First PC, radar and appropriate ego position
    radar_id_0 = "test_radar_0"
    point_0 = [1, 2, 3, 4, 5, 6]
    ego_pos_0 = [1, 2, 3]
    extrinsics_pos_0 = [0.1, 0.2, 0.3]
    # Second PC, radar and appropriate ego position
    radar_id_1 = "test_radar_1"
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]
    ego_pos_1 = [6, 5, 4]
    extrinsics_pos_1 = [-0.1, -0.2, -0.3]
    # Later ego position
    ego_pos_now = [1000, 2000, 3000]

    for it in accumulators:
        it.accumulate(
            radar_id_0,
            [point_0],
            provizio_dds.accumulation.RigidTransform(ego_pos_0, no_rotation),
            radar_extrinsics=provizio_dds.accumulation.RigidTransform(
                extrinsics_pos_0, no_rotation
            ),
        )
        it.accumulate(
            radar_id_1,
            [point_1, point_2],
            provizio_dds.accumulation.RigidTransform(ego_pos_1, no_rotation),
            radar_extrinsics=provizio_dds.accumulation.RigidTransform(
                extrinsics_pos_1, no_rotation
            ),
        )

    # snr threshold = 5 => point were filtered out, 3 kept
    accumulated_points_snr_5 = accumulator_snr_5.get_points_local_frame_relative()
    assert len(accumulated_points_snr_5) == 3

    # snr threshold = 6 => 1 point were filtered out, 2 kept
    accumulated_points_snr_6 = accumulator_snr_6.get_points_local_frame_relative()
    assert len(accumulated_points_snr_6) == 2

    # snr threshold = 60 => 2 points were filtered out, 1 kept
    accumulated_points_snr_60 = accumulator_snr_60.get_points_local_frame_relative()
    assert len(accumulated_points_snr_60) == 1

    input_points = [point_0, point_1, point_2]
    ego_poses = [ego_pos_0, ego_pos_1, ego_pos_1]
    extrinsics = [extrinsics_pos_0, extrinsics_pos_1, extrinsics_pos_1]

    # Check all
    accumulated_points_all_snrs = [
        accumulated_points_snr_5,
        accumulated_points_snr_6,
        accumulated_points_snr_60,
    ]
    for accumulated_points in accumulated_points_all_snrs:
        for pt in range(len(accumulated_points)):
            for dim in range(3):
                points_filtered_out = 3 - len(accumulated_points)
                assert (
                    accumulated_points[pt].position[dim]
                    == input_points[pt + points_filtered_out][dim]
                    + ego_poses[pt + points_filtered_out][dim]
                    + extrinsics[pt + points_filtered_out][dim]
                )


def test_accumulate_snr_and_velocity_filters():
    radar_id = "test_radar"
    snr_thresholds = [0, 2.5, 10, 100]
    snrs = [1, 2.5, 5, 10, 15]

    velocity_thresholds = [0, 2, 5, 20, 100]
    velocities = [0, 1, 2, 5, 15, 40]
    max_frames_without_filter_variants = [0, 1]

    for snr_threshold in snr_thresholds:
        for velocity_threshold in velocity_thresholds:
            for max_frames_without_filter in max_frames_without_filter_variants:
                accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
                    2,
                    snr_threshold=snr_threshold,
                    max_frames_without_filter=max_frames_without_filter,
                    point_filter=lambda point: point[5] >= velocity_threshold,
                    allow_no_extrinsics=True,
                )

                expected_unfiltered_num_accumulated_points = 0
                expected_filtered_num_accumulated_points = 0
                points = []
                for snr in snrs:
                    for velocity in velocities:
                        points.append([1, 2, 3, 4, snr, velocity])

                        if snr >= snr_threshold:
                            expected_unfiltered_num_accumulated_points += 1
                            if velocity >= velocity_threshold:
                                expected_filtered_num_accumulated_points += 1

                # Empty yet
                assert len(accumulator.get_points_local_frame_relative()) == 0

                # Accumulate now
                accumulator.accumulate(
                    radar_id,
                    points,
                    provizio_dds.accumulation.RigidTransform([5, 6, 7], [8, 9, 10]),
                )

                if max_frames_without_filter > 0:
                    # Nothing is filtered yet
                    assert (
                        len(accumulator.get_points_local_frame_relative())
                        == expected_unfiltered_num_accumulated_points
                    )

                # Accumulate after, till the filter is applied
                for accumulate_after in range(max_frames_without_filter):
                    accumulator.accumulate(
                        radar_id,
                        [],
                        provizio_dds.accumulation.RigidTransform([0, 0, 0], [0, 0, 0]),
                    )

                # Filter has been applied now
                assert (
                    len(accumulator.get_points_local_frame_relative())
                    == expected_filtered_num_accumulated_points
                )


def test_accumulate_radar_filter():
    radar_id_1 = "test_radar_1"
    radar_id_2 = "test_radar_2"
    radar_id_3 = "test_radar_3"

    identity_transform = provizio_dds.accumulation.RigidTransform([0, 0, 0], [0, 0, 0])

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        2,
        snr_threshold=0,
        max_frames_without_filter=0,
        point_filter=None,
        radar_filter=lambda radar_id: "2" not in radar_id,
        allow_no_extrinsics=True,
    )

    accumulator.accumulate(radar_id_1, [[1, 2, 3, 4, 5, 6]], identity_transform)
    accumulator.accumulate(radar_id_2, [[7, 8, 9, 10, 11, 12]], identity_transform)
    accumulator.accumulate(radar_id_3, [[13, 14, 15, 16, 17, 18]], identity_transform)

    accumulated_points = accumulator.get_points_local_frame_relative()

    assert len(accumulated_points) == 2
    assert accumulated_points[0].radar_id == radar_id_1
    assert accumulated_points[1].radar_id == radar_id_3


def test_accumulate_rotation_yaw():
    radar_id = "test_radar"
    no_translation = [0, 0, 0]

    # First PC, radar and appropriate ego orientation
    point_0 = [101, 102, 103, 0, 0, 0]
    ego_orientation_euler_angles_0 = [0, 0, 0]
    # Second PC, radar and appropriate ego orientation
    point_1 = [110, 120, 130, 0, 0, 0]
    ego_orientation_euler_angles_1 = [0, 0, radians(30)]
    # Third PC, radar and appropriate ego orientation
    point_2 = [200, 300, 400, 0, 0, 0]
    ego_orientation_euler_angles_2 = [0, 0, radians(45)]

    # Later ego orientation = same as on accumulating the 3rd point cloud
    ego_orientation_now = ego_orientation_euler_angles_2

    points = [point_0, point_1, point_2]
    orientations = [
        ego_orientation_euler_angles_0,
        ego_orientation_euler_angles_1,
        ego_orientation_euler_angles_2,
    ]

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        len(orientations), snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )

    for i in range(len(orientations)):
        accumulator.accumulate(
            radar_id,
            [points[i]],
            provizio_dds.accumulation.RigidTransform(no_translation, orientations[i]),
        )

    accumulated_points = accumulator.get_points_ego_relative(
        provizio_dds.accumulation.RigidTransform(no_translation, ego_orientation_now)
    )

    precision = 0.0001

    assert len(accumulated_points) == 3
    assert isclose(143.5427, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(0.7071018, accumulated_points[0].position[1], abs_tol=precision)
    assert isclose(103.0, accumulated_points[0].position[2], abs_tol=precision)
    assert isclose(137.31013, accumulated_points[1].position[0], abs_tol=precision)
    assert isclose(87.44099, accumulated_points[1].position[1], abs_tol=precision)
    assert isclose(130.0, accumulated_points[1].position[2], abs_tol=precision)
    assert isclose(200, accumulated_points[2].position[0], abs_tol=precision)
    assert isclose(300, accumulated_points[2].position[1], abs_tol=precision)
    assert isclose(400, accumulated_points[2].position[2], abs_tol=precision)


def test_accumulate_rotation_pitch():
    radar_id = "test_radar"
    no_translation = [0, 0, 0]

    # First PC, radar and appropriate ego orientation
    point_0 = [101, 102, 103, 0, 0, 0]
    ego_orientation_euler_angles_0 = [0, 0, 0]
    # Second PC, radar and appropriate ego orientation
    point_1 = [110, 120, 130, 0, 0, 0]
    ego_orientation_euler_angles_1 = [0, radians(30), 0]
    # Third PC, radar and appropriate ego orientation
    point_2 = [200, 300, 400, 0, 0, 0]
    ego_orientation_euler_angles_2 = [0, radians(45), 0]

    # Later ego orientation = same as on accumulating the 3rd point cloud
    ego_orientation_now = ego_orientation_euler_angles_2

    points = [point_0, point_1, point_2]
    orientations = [
        ego_orientation_euler_angles_0,
        ego_orientation_euler_angles_1,
        ego_orientation_euler_angles_2,
    ]

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        len(orientations), snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )

    for i in range(len(orientations)):
        accumulator.accumulate(
            radar_id,
            [points[i]],
            provizio_dds.accumulation.RigidTransform(no_translation, orientations[i]),
        )

    accumulated_points = accumulator.get_points_ego_relative(
        provizio_dds.accumulation.RigidTransform(no_translation, ego_orientation_now)
    )

    precision = 0.0001

    assert len(accumulated_points) == 3
    assert isclose(-1.414223, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(102.0, accumulated_points[0].position[1], abs_tol=precision)
    assert isclose(144.24978, accumulated_points[0].position[2], abs_tol=precision)
    assert isclose(72.60535, accumulated_points[1].position[0], abs_tol=precision)
    assert isclose(120.0, accumulated_points[1].position[1], abs_tol=precision)
    assert isclose(154.04045, accumulated_points[1].position[2], abs_tol=precision)
    assert isclose(200, accumulated_points[2].position[0], abs_tol=precision)
    assert isclose(300, accumulated_points[2].position[1], abs_tol=precision)
    assert isclose(400, accumulated_points[2].position[2], abs_tol=precision)


def test_accumulate_rotation_roll():
    radar_id = "test_radar"
    no_translation = [0, 0, 0]

    # First PC, radar and appropriate ego orientation
    point_0 = [101, 102, 103, 0, 0, 0]
    ego_orientation_euler_angles_0 = [0, 0, 0]
    # Second PC, radar and appropriate ego orientation
    point_1 = [110, 120, 130, 0, 0, 0]
    ego_orientation_euler_angles_1 = [radians(30), 0, 0]
    # Third PC, radar and appropriate ego orientation
    point_2 = [200, 300, 400, 0, 0, 0]
    ego_orientation_euler_angles_2 = [radians(45), 0, 0]

    # Later ego orientation = same as on accumulating the 3rd point cloud
    ego_orientation_now = ego_orientation_euler_angles_2

    points = [point_0, point_1, point_2]
    orientations = [
        ego_orientation_euler_angles_0,
        ego_orientation_euler_angles_1,
        ego_orientation_euler_angles_2,
    ]

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        len(orientations), snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )

    for i in range(len(orientations)):
        accumulator.accumulate(
            radar_id,
            [points[i]],
            provizio_dds.accumulation.RigidTransform(no_translation, orientations[i]),
        )

    accumulated_points = accumulator.get_points_ego_relative(
        provizio_dds.accumulation.RigidTransform(no_translation, ego_orientation_now)
    )

    precision = 0.0001

    assert len(accumulated_points) == 3
    assert isclose(101.0, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(144.9569, accumulated_points[0].position[1], abs_tol=precision)
    assert isclose(0.7070923, accumulated_points[0].position[2], abs_tol=precision)
    assert isclose(110.0, accumulated_points[1].position[0], abs_tol=precision)
    assert isclose(149.5576, accumulated_points[1].position[1], abs_tol=precision)
    assert isclose(94.51205, accumulated_points[1].position[2], abs_tol=precision)
    assert isclose(200, accumulated_points[2].position[0], abs_tol=precision)
    assert isclose(300, accumulated_points[2].position[1], abs_tol=precision)
    assert isclose(400, accumulated_points[2].position[2], abs_tol=precision)


def test_accumulate_rotation_yaw_with_extrinsics():
    radar_id = "test_radar"
    no_translation = [0, 0, 0]
    extrinsics_rotation = [0, 0, radians(15)]

    # First PC, radar and appropriate ego orientation
    point_0 = [
        123.95805106,
        72.38371073,
        103,
        0,
        0,
        0,
    ]  # [101, 102, 103] rotated by -15 degrees yaw
    ego_orientation_euler_angles_0 = [0, 0, 0]
    # Second PC, radar and appropriate ego orientation
    point_1 = [
        137.3101263,
        87.44100419,
        130,
        0,
        0,
        0,
    ]  # [110, 120, 130] rotated by -15 degrees yaw
    ego_orientation_euler_angles_1 = [0, 0, radians(30)]
    # Third PC, radar and appropriate ego orientation
    point_2 = [
        270.83087879,
        238.01393887,
        400,
        0,
        0,
        0,
    ]  # [200, 300, 400] rotated by -15 degrees yaw
    ego_orientation_euler_angles_2 = [0, 0, radians(45)]

    # Later ego orientation = same as on accumulating the 3rd point cloud
    ego_orientation_now = ego_orientation_euler_angles_2

    points = [point_0, point_1, point_2]
    orientations = [
        ego_orientation_euler_angles_0,
        ego_orientation_euler_angles_1,
        ego_orientation_euler_angles_2,
    ]

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        len(orientations), snr_threshold=0, point_filter=None, allow_no_extrinsics=False
    )

    for i in range(len(orientations)):
        accumulator.accumulate(
            radar_id,
            [points[i]],
            provizio_dds.accumulation.RigidTransform(no_translation, orientations[i]),
            radar_extrinsics=provizio_dds.accumulation.RigidTransform(
                no_translation, extrinsics_rotation
            ),
        )

    accumulated_points = accumulator.get_points_ego_relative(
        provizio_dds.accumulation.RigidTransform(no_translation, ego_orientation_now)
    )

    precision = 0.0001

    assert len(accumulated_points) == 3
    assert isclose(143.5427, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(0.7071018, accumulated_points[0].position[1], abs_tol=precision)
    assert isclose(103.0, accumulated_points[0].position[2], abs_tol=precision)
    assert isclose(137.31013, accumulated_points[1].position[0], abs_tol=precision)
    assert isclose(87.44099, accumulated_points[1].position[1], abs_tol=precision)
    assert isclose(130.0, accumulated_points[1].position[2], abs_tol=precision)
    assert isclose(200, accumulated_points[2].position[0], abs_tol=precision)
    assert isclose(300, accumulated_points[2].position[1], abs_tol=precision)
    assert isclose(400, accumulated_points[2].position[2], abs_tol=precision)


def test_accumulate_rotation_and_move_simple_no_extrinsics():
    radar_id = "test_radar"
    point = [1, 2, 3, 4, 5, 6]
    fix_when_received = provizio_dds.accumulation.RigidTransform(
        [10, 20, 0], [0, 0, radians(135)]
    )
    current_fix = provizio_dds.accumulation.RigidTransform(
        [20, 10, 0], [0, 0, radians(45)]
    )
    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        1, snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )
    accumulator.accumulate(radar_id, [point], fix_when_received)
    accumulated_points = accumulator.get_points_ego_relative(current_fix)

    precision = 0.0001

    assert len(accumulated_points) == 1
    assert isclose(-2, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(
        1 + 10 * sqrt(2), accumulated_points[0].position[1], abs_tol=precision
    )
    assert isclose(3, accumulated_points[0].position[2], abs_tol=precision)


def test_accumulate_rotation_and_move_simple_with_extrinsics():
    radar_id = "test_radar"
    # 10 meters left, 2 up, looking left
    extrinsics = provizio_dds.accumulation.RigidTransform(
        [0, 10, 2], [0, 0, radians(90)]
    )
    # ego relative [1, 2, 3, 4, 5, 6] as seen by test_radar
    point = [-8, -1, 1, 4, 5, 6]
    fix_when_received = provizio_dds.accumulation.RigidTransform(
        [10, 20, 0], [0, 0, radians(135)]
    )
    current_fix = provizio_dds.accumulation.RigidTransform(
        [20, 10, 0], [0, 0, radians(45)]
    )
    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        1, snr_threshold=0, point_filter=None, allow_no_extrinsics=False
    )
    accumulator.accumulate(
        radar_id, [point], fix_when_received, radar_extrinsics=extrinsics
    )
    accumulated_points = accumulator.get_points_ego_relative(current_fix)

    precision = 0.0001

    assert len(accumulated_points) == 1
    assert isclose(-2, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(
        1 + 10 * sqrt(2), accumulated_points[0].position[1], abs_tol=precision
    )
    assert isclose(3, accumulated_points[0].position[2], abs_tol=precision)


def test_accumulate_rotation_and_move_no_extrinsics():
    """
    This test is based on its analogue in provizio_radar_api_core which in turn
    was proven to match APT GUI implementation of accumulation
    """

    radar_id = "test_radar"

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        3, snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )

    # PC 0
    point_0 = [101, 102, 103, 5, 5, 5]
    fix_0 = provizio_dds.accumulation.RigidTransform([879.020, 529.971, 0], [0, 0, 0])
    # PC 1
    point_1 = [110, 120, 130, 5, 5, 5]
    fix_1 = provizio_dds.accumulation.RigidTransform(
        [871.156, 548.981, 0], [0, 0, radians(30)]
    )
    # PC 2
    point_2 = [200, 300, 400, 5, 5, 5]
    fix_2 = provizio_dds.accumulation.RigidTransform(
        [899.447, 562.369, 0], [0, 0, radians(45)]
    )

    current_fix = fix_2

    all_points = [point_0, point_1, point_2]
    all_fixes = [fix_0, fix_1, fix_2]
    for i in range(len(all_fixes)):
        accumulator.accumulate(radar_id, [all_points[i]], all_fixes[i])

    accumulated_points = accumulator.get_points_ego_relative(current_fix)
    assert len(accumulated_points) == len(all_points)

    precision = 0.0001
    assert isclose(106.18976, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(-7.7577, accumulated_points[0].position[1], abs_tol=precision)
    assert isclose(103.0, accumulated_points[0].position[2], abs_tol=precision)
    assert isclose(107.8386, accumulated_points[1].position[0], abs_tol=precision)
    assert isclose(97.979, accumulated_points[1].position[1], abs_tol=precision)
    assert isclose(130.0, accumulated_points[1].position[2], abs_tol=precision)
    assert isclose(200.0, accumulated_points[2].position[0], abs_tol=precision)
    assert isclose(300.0, accumulated_points[2].position[1], abs_tol=precision)
    assert isclose(400.0, accumulated_points[2].position[2], abs_tol=precision)


def test_accumulate_rotation_and_move_with_extrinsics():
    radar_id = "test_radar"

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        3, snr_threshold=0, point_filter=None, allow_no_extrinsics=False
    )

    extrinsics = provizio_dds.accumulation.RigidTransform(
        [-10, -10, -10], [0, 0, radians(180)]
    )

    # PC 0
    point_0 = [-111, -112, 113, 5, 5, 5]  # = [101, 102, 103] ego relative
    fix_0 = provizio_dds.accumulation.RigidTransform([879.020, 529.971, 0], [0, 0, 0])
    # PC 1
    point_1 = [-120, -130, 140, 5, 5, 5]  # = [110, 120, 130] ego relative
    fix_1 = provizio_dds.accumulation.RigidTransform(
        [871.156, 548.981, 0], [0, 0, radians(30)]
    )
    # PC 2
    point_2 = [-210, -310, 410, 5, 5, 5]  # = [200, 300, 400] ego relative
    fix_2 = provizio_dds.accumulation.RigidTransform(
        [899.447, 562.369, 0], [0, 0, radians(45)]
    )

    current_fix = fix_2

    all_points = [point_0, point_1, point_2]
    all_fixes = [fix_0, fix_1, fix_2]
    for i in range(len(all_fixes)):
        accumulator.accumulate(
            radar_id, [all_points[i]], all_fixes[i], radar_extrinsics=extrinsics
        )

    accumulated_points = accumulator.get_points_ego_relative(current_fix)
    assert len(accumulated_points) == len(all_points)

    precision = 0.0001
    assert isclose(106.18976, accumulated_points[0].position[0], abs_tol=precision)
    assert isclose(-7.7577, accumulated_points[0].position[1], abs_tol=precision)
    assert isclose(103.0, accumulated_points[0].position[2], abs_tol=precision)
    assert isclose(107.8386, accumulated_points[1].position[0], abs_tol=precision)
    assert isclose(97.979, accumulated_points[1].position[1], abs_tol=precision)
    assert isclose(130.0, accumulated_points[1].position[2], abs_tol=precision)
    assert isclose(200.0, accumulated_points[2].position[0], abs_tol=precision)
    assert isclose(300.0, accumulated_points[2].position[1], abs_tol=precision)
    assert isclose(400.0, accumulated_points[2].position[2], abs_tol=precision)


def test_accumulate_overflow():
    radar_id_0 = "test_radar_0"
    radar_id_1 = "test_radar_1"
    identity_transform = provizio_dds.accumulation.RigidTransform([0, 0, 0], [0, 0, 0])

    accumulator = provizio_dds.accumulation.PointCloudsAccumulator(
        2, snr_threshold=0, point_filter=None, allow_no_extrinsics=True
    )

    point = [1, 2, 3, 4, 5, 6]

    assert len(accumulator.get_points_local_frame_relative()) == 0

    accumulator.accumulate(radar_id_0, [point], identity_transform)
    assert len(accumulator.get_points_local_frame_relative()) == 1

    accumulator.accumulate(radar_id_0, [point, point], identity_transform)
    assert len(accumulator.get_points_local_frame_relative()) == 3

    # Now the very first point gets dropped
    accumulator.accumulate(radar_id_0, [point, point, point], identity_transform)
    assert len(accumulator.get_points_local_frame_relative()) == 5

    # Another radar, so nothing gets dropped from the first one
    accumulator.accumulate(radar_id_1, [point, point, point, point], identity_transform)
    assert len(accumulator.get_points_local_frame_relative()) == 9

    # 2nd radar again, nothing gets dropped again
    accumulator.accumulate(radar_id_1, [point], identity_transform)
    assert len(accumulator.get_points_local_frame_relative()) == 10

    # Now clear it all by pushing empty pcs twice to the both of radars
    all_radars = [radar_id_0, radar_id_1]
    for radar_id in all_radars:
        for i in range(2):
            accumulator.accumulate(radar_id, [], identity_transform)
    assert len(accumulator.get_points_local_frame_relative()) == 0


def publish_extrinsics(
    publisher,
    rigid_transform: provizio_dds.accumulation.RigidTransform,
    frame_id: str,
    parent_frame_id: str = "test_world",
    timestamp_sec: int = 0,
    timestamp_nanosec: int = 0,
):
    header = provizio_dds.point_cloud2.make_header(
        timestamp_sec, timestamp_nanosec, parent_frame_id
    )

    translation = provizio_dds.Vector3()
    position = rigid_transform.translation()
    translation.x(position[0])
    translation.y(position[1])
    translation.z(position[2])

    rotation = provizio_dds.Quaternion()
    quat = rigid_transform.rotation()
    rotation.w(quat[0])
    rotation.x(quat[1])
    rotation.y(quat[2])
    rotation.z(quat[3])

    transform = provizio_dds.Transform()
    transform.translation(translation)
    transform.rotation(rotation)

    message = provizio_dds.TransformStamped()
    message.header(header)
    message.child_frame_id(frame_id)
    message.transform(transform)

    assert publisher.publish(message), "Failed to publish a TransformStamped message"


def publish_odometry(
    publisher,
    rigid_transform: provizio_dds.accumulation.RigidTransform,
    frame_id: str = "test_odometry",
    parent_frame_id: str = "test_world",
    timestamp_sec: int = 0,
    timestamp_nanosec: int = 0,
):
    header = provizio_dds.point_cloud2.make_header(
        timestamp_sec, timestamp_nanosec, parent_frame_id
    )

    position = provizio_dds.Point()
    translation = rigid_transform.translation()
    position.x(translation[0])
    position.y(translation[1])
    position.z(translation[2])

    orientation = provizio_dds.Quaternion()
    quat = rigid_transform.rotation()
    orientation.w(quat[0])
    orientation.x(quat[1])
    orientation.y(quat[2])
    orientation.z(quat[3])

    pose = provizio_dds.Pose()
    pose.position(position)
    pose.orientation(orientation)

    pose_with_covariance = provizio_dds.PoseWithCovariance()
    pose_with_covariance.pose(pose)

    message = provizio_dds.Odometry()
    message.header(header)
    message.child_frame_id(frame_id)
    message.pose(pose_with_covariance)

    assert publisher.publish(message), "Failed to publish an Odometry message"


def publish_nav_sat_fix(
    publisher,
    lat_lon_alt,
    frame_id: str = "test_nav_sat_fix",
    timestamp_sec: int = 0,
    timestamp_nanosec: int = 0,
):
    header = provizio_dds.point_cloud2.make_header(
        timestamp_sec, timestamp_nanosec, frame_id
    )
    message = provizio_dds.NavSatFix()
    message.header(header)
    status = provizio_dds.NavSatStatus()
    status.status(provizio_dds.STATUS_FIX)
    message.status(status)
    message.latitude(lat_lon_alt[0])
    message.longitude(lat_lon_alt[1])
    message.altitude(lat_lon_alt[2])

    assert publisher.publish(message), "Failed to publish an NavSatFix message"


def publish_pc2(
    publisher,
    points: [[float]],
    radar_id: str,
    timestamp_sec: int = 0,
    timestamp_nanosec: int = 0,
):
    message = provizio_dds.point_cloud2.make_radar_point_cloud(
        provizio_dds.point_cloud2.make_header(
            timestamp_sec, timestamp_nanosec, radar_id
        ),
        points,
    )

    assert publisher.publish(message), "Failed to publish a PointCloud2 message"


def test_accumulate_dds_simple():
    localization_topic = "rt/test_localization_topic"
    pointcloud2_topic = "rt/test_pointcloud2_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant()
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant()

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.Odometry,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=None,  # Ego=radar assumed
        snr_threshold=0,
        point_filter=None,
        dds_domain_participant=dds_domain_participant_subscribers,
    )

    radar_id = "test_radar"
    no_rotation = [0, 0, 0]
    # First PC and appropriate ego position
    point_0 = [1, 2, 3, 4, 5, 6]
    ego_pos_0 = [1, 2, 3]
    # Second PC and appropriate ego position
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]
    ego_pos_1 = [6, 5, 4]
    # Later ego position
    ego_pos_now = [1000, 2000, 3000]

    localization_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        localization_topic,
        provizio_dds.OdometryPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple localization_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple pc2_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    time.sleep(
        0.3
    )  # Make sure there is enough time for publishers and subscribers to match
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(ego_pos_0, no_rotation),
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_0], radar_id)

    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(ego_pos_1, no_rotation),
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_1, point_2], radar_id)

    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(ego_pos_now, no_rotation),
    )

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative()
    assert len(accumulated_points_ego_relative) == 3

    input_points = [point_0, point_1, point_2]
    ego_poses = [
        ego_pos_0,
        ego_pos_1,
        ego_pos_1,
    ]  # Last ego_pos_1 is not a typo, as point_1 and point_2 share same ego_pos

    # Check all components of all transformed points are as they should be, in appropriate spaces
    for pt in range(3):
        for dim in range(3):
            # local frame relative
            assert (
                accumulated_points_local_frame_relative[pt].position[dim]
                == input_points[pt][dim] + ego_poses[pt][dim]
            )
            assert accumulated_points_local_frame_relative[pt].radar_id == radar_id
            assert (
                accumulated_points_local_frame_relative[pt].snr == input_points[pt][4]
            )
            assert (
                accumulated_points_local_frame_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )

            # ego relative
            assert (
                accumulated_points_ego_relative[pt].position[dim]
                == input_points[pt][dim] + ego_poses[pt][dim] - ego_pos_now[dim]
            )
            assert accumulated_points_ego_relative[pt].snr == input_points[pt][4]
            assert (
                accumulated_points_ego_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )
            assert accumulated_points_ego_relative[pt].radar_id == radar_id


def test_accumulate_dds_simple_extrinsics():
    localization_topic = "rt/test_localization_topic"
    pointcloud2_topic = "rt/test_pointcloud2_topic"
    extrinsics_topic = "rt/test_extrinsics"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant()
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant()

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.Odometry,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=[extrinsics_topic],
        snr_threshold=0,
        point_filter=None,
        dds_domain_participant=dds_domain_participant_subscribers,
    )

    radar_id = "test_radar"
    no_rotation = [0, 0, 0]
    radar_extrinsics_pos = [-10, -20, -30]
    # First PC and appropriate ego position
    point_0 = [1, 2, 3, 4, 5, 6]
    ego_pos_0 = [1, 2, 3]
    # Second PC and appropriate ego position
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]
    ego_pos_1 = [6, 5, 4]
    # Later ego position
    ego_pos_now = [1000, 2000, 3000]

    extrinsics_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        extrinsics_topic,
        provizio_dds.TransformStampedPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple_extrinsics extrinsics_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    localization_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        localization_topic,
        provizio_dds.OdometryPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple_extrinsics localization_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple_extrinsics pc2_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    time.sleep(
        0.3
    )  # Make sure there is enough time for publishers and subscribers to match
    publish_extrinsics(
        extrinsics_publisher,
        provizio_dds.accumulation.RigidTransform(radar_extrinsics_pos, no_rotation),
        frame_id=radar_id,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(ego_pos_0, no_rotation),
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_0], radar_id)

    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(ego_pos_1, no_rotation),
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_1, point_2], radar_id)

    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(ego_pos_now, no_rotation),
    )

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative()
    assert len(accumulated_points_ego_relative) == 3

    input_points = [point_0, point_1, point_2]
    ego_poses = [
        ego_pos_0,
        ego_pos_1,
        ego_pos_1,
    ]  # Last ego_pos_1 is not a typo, as point_1 and point_2 share same ego_pos

    # Check all components of all transformed points are as they should be, in appropriate spaces
    for pt in range(3):
        for dim in range(3):
            # local frame relative
            assert (
                accumulated_points_local_frame_relative[pt].position[dim]
                == input_points[pt][dim]
                + ego_poses[pt][dim]
                + radar_extrinsics_pos[dim]
            )
            assert accumulated_points_local_frame_relative[pt].radar_id == radar_id
            assert (
                accumulated_points_local_frame_relative[pt].snr == input_points[pt][4]
            )
            assert (
                accumulated_points_local_frame_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )

            # ego relative
            assert (
                accumulated_points_ego_relative[pt].position[dim]
                == input_points[pt][dim]
                + ego_poses[pt][dim]
                + radar_extrinsics_pos[dim]
                - ego_pos_now[dim]
            )
            assert accumulated_points_ego_relative[pt].snr == input_points[pt][4]
            assert (
                accumulated_points_ego_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )
            assert accumulated_points_ego_relative[pt].radar_id == radar_id


def test_accumulate_dds_simple_extrinsics_including_localization():
    localization_topic = "rt/test_localization_topic"
    pointcloud2_topic = "rt/test_pointcloud2_topic"
    radar_extrinsics_topic = "rt/test_radar_extrinsics"
    localization_extrinsics_topic = "rt/test_localization_extrinsics"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant()
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant()

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.Odometry,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=[radar_extrinsics_topic],
        localization_extrinsics_topic=localization_extrinsics_topic,
        snr_threshold=0,
        point_filter=None,
        dds_domain_participant=dds_domain_participant_subscribers,
    )

    radar_id = "test_radar"
    localization_frame_id = "test_localization"
    localization_extrinsics_pos = [0.3, 0.5, 0.7]
    localization_extrinsics_yaw = 90  # looking left
    radar_extrinsics_pos = [-10, -20, -30]
    radar_extrinsics_yaw = -90  # looking right
    # First PC and appropriate ego position
    point_0_ego_relative = [1, 2, 3, 4, 5, 6]
    point_0_radar_relative = [
        -2 - 20,
        1 + 10,
        3 + 30,
        4,
        5,
        6,
    ]  # ego relative [1, 2, 3] from radar extrinsics point of view
    ego_pos_0 = [1, 2, 3]
    loc_pos_0 = [
        1 + 0.3,
        2 + 0.5,
        3 + 0.7,
    ]  # ego [1, 2, 3] from localization extrinsics point of view
    # Second PC and appropriate ego position
    point_1_ego_relative = [10, 20, 30, 40, 50, 60]
    point_1_radar_relative = [
        -20 - 20,
        10 + 10,
        30 + 30,
        40,
        50,
        60,
    ]  # ego relative [10, 20, 30, 40, 50, 60] from radar extrinsics point of view
    point_2_ego_relative = [100, 200, 300, 400, 500, 600]
    point_2_radar_relative = [
        -200 - 20,
        100 + 10,
        300 + 30,
        400,
        500,
        600,
    ]  # ego relative [100, 200, 300, 400, 500, 600] from radar extrinsics point of view
    ego_pos_1 = [6, 5, 4]
    loc_pos_1 = [
        6 + 0.3,
        5 + 0.5,
        4 + 0.7,
    ]  # ego [6, 5, 4] from localization extrinsics point of view
    # Later ego position
    ego_pos_now = [1000, 2000, 3000]
    loc_pos_now = [
        1000 + 0.3,
        2000 + 0.5,
        3000 + 0.7,
    ]  # ego [1000, 2000, 3000] from localization extrinsics point of view

    radar_extrinsics_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        radar_extrinsics_topic,
        provizio_dds.TransformStampedPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple_extrinsics_including_localization extrinsics_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    localization_extrinsics_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        localization_extrinsics_topic,
        provizio_dds.TransformStampedPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple_extrinsics_including_localization localization_extrinsics_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    localization_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        localization_topic,
        provizio_dds.OdometryPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple_extrinsics_including_localization localization_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_simple_extrinsics_including_localization pc2_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    time.sleep(
        0.3
    )  # Make sure there is enough time for publishers and subscribers to match
    publish_extrinsics(
        radar_extrinsics_publisher,
        provizio_dds.accumulation.RigidTransform(
            radar_extrinsics_pos, [0, 0, radians(radar_extrinsics_yaw)]
        ),
        frame_id=radar_id,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_extrinsics(
        localization_extrinsics_publisher,
        provizio_dds.accumulation.RigidTransform(
            localization_extrinsics_pos, [0, 0, radians(localization_extrinsics_yaw)]
        ),
        frame_id=localization_frame_id,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(
            loc_pos_0, [0, 0, radians(localization_extrinsics_yaw)]
        ),
        frame_id=localization_frame_id,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_0_radar_relative], radar_id)

    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(
            loc_pos_1, [0, 0, radians(localization_extrinsics_yaw)]
        ),
        frame_id=localization_frame_id,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(
        pc2_publisher, [point_1_radar_relative, point_2_radar_relative], radar_id
    )

    time.sleep(0.2)  # to make sure the delivery order
    publish_odometry(
        localization_publisher,
        provizio_dds.accumulation.RigidTransform(
            loc_pos_now, [0, 0, radians(localization_extrinsics_yaw)]
        ),
        frame_id=localization_frame_id,
    )

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative()
    assert len(accumulated_points_ego_relative) == 3

    input_points_ego_relative = [
        point_0_ego_relative,
        point_1_ego_relative,
        point_2_ego_relative,
    ]
    ego_poses = [
        ego_pos_0,
        ego_pos_1,
        ego_pos_1,
    ]  # Last ego_pos_1 is not a typo, as point_1 and point_2 share same ego_pos

    precision = 0.0001

    # Check all components of all transformed points are as they should be, in appropriate spaces
    for pt in range(3):
        for dim in range(3):
            # local frame relative
            assert isclose(
                accumulated_points_local_frame_relative[pt].position[dim],
                input_points_ego_relative[pt][dim] + ego_poses[pt][dim],
                abs_tol=precision,
            )
            assert accumulated_points_local_frame_relative[pt].radar_id == radar_id
            assert (
                accumulated_points_local_frame_relative[pt].snr
                == input_points_ego_relative[pt][4]
            )
            assert (
                accumulated_points_local_frame_relative[pt].ground_relative_velocity
                == input_points_ego_relative[pt][5]
            )

            # ego relative
            assert isclose(
                accumulated_points_ego_relative[pt].position[dim],
                input_points_ego_relative[pt][dim]
                + ego_poses[pt][dim]
                - ego_pos_now[dim],
                abs_tol=precision,
            )
            assert (
                accumulated_points_ego_relative[pt].snr
                == input_points_ego_relative[pt][4]
            )
            assert (
                accumulated_points_ego_relative[pt].ground_relative_velocity
                == input_points_ego_relative[pt][5]
            )
            assert accumulated_points_ego_relative[pt].radar_id == radar_id


def test_accumulate_dds_no_localization_no_extrinsics():
    pointcloud2_topic = "rt/test_pointcloud2_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant()
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant()

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=None,
        localization_topic=None,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        dds_domain_participant=dds_domain_participant_subscribers,
    )

    radar_id = "test_radar"
    # First PC and appropriate ego position
    point_0 = [1, 2, 3, 4, 5, 6]
    # Second PC and appropriate ego position
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]

    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_no_localization_no_extrinsics pc2_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    time.sleep(
        0.3
    )  # Make sure there is enough time for publishers and subscribers to match
    publish_pc2(pc2_publisher, [point_0], radar_id)
    publish_pc2(pc2_publisher, [point_1, point_2], radar_id)

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative()
    assert len(accumulated_points_ego_relative) == 3

    input_points = [point_0, point_1, point_2]

    # Check all components of all transformed points are as they should be, in appropriate spaces
    for pt in range(3):
        for field in range(6):
            for accumulated_points in [
                accumulated_points_local_frame_relative,
                accumulated_points_ego_relative,
            ]:
                if field < 3:
                    value = accumulated_points[pt].position[field]
                elif field == 3:
                    continue  # radar relative velocities are not accumulated
                elif field == 4:
                    value = accumulated_points[pt].snr
                else:  # field == 5
                    value = accumulated_points[pt].ground_relative_velocity

                # local frame relative
                assert value == input_points[pt][field]


def test_accumulate_dds_no_localization_with_extrinsics():
    pointcloud2_topic = "rt/test_pointcloud2_topic"
    extrinsics_topic = "rt/test_extrinsics"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant()
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant()

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=None,
        localization_topic=None,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=[extrinsics_topic],
        snr_threshold=0,
        point_filter=None,
        dds_domain_participant=dds_domain_participant_subscribers,
    )

    radar_id = "test_radar"
    no_rotation = [0, 0, 0]
    radar_extrinsics_pos = [-10, -20, -30]
    # First PC and appropriate ego position
    point_0 = [1, 2, 3, 4, 5, 6]
    # Second PC and appropriate ego position
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]

    extrinsics_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        extrinsics_topic,
        provizio_dds.TransformStampedPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_no_localization_with_extrinsics extrinsics_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_no_localization_with_extrinsics pc2_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    time.sleep(
        0.3
    )  # Make sure there is enough time for publishers and subscribers to match
    publish_extrinsics(
        extrinsics_publisher,
        provizio_dds.accumulation.RigidTransform(radar_extrinsics_pos, no_rotation),
        frame_id=radar_id,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_0], radar_id)
    publish_pc2(pc2_publisher, [point_1, point_2], radar_id)

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative()
    assert len(accumulated_points_ego_relative) == 3

    input_points = [point_0, point_1, point_2]

    # Check all components of all transformed points are as they should be, in appropriate spaces
    for pt in range(3):
        for dim in range(3):
            # local frame relative
            assert (
                accumulated_points_local_frame_relative[pt].position[dim]
                == input_points[pt][dim] + radar_extrinsics_pos[dim]
            )
            assert accumulated_points_local_frame_relative[pt].radar_id == radar_id
            assert (
                accumulated_points_local_frame_relative[pt].snr == input_points[pt][4]
            )
            assert (
                accumulated_points_local_frame_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )

            # ego relative
            assert (
                accumulated_points_ego_relative[pt].position[dim]
                == input_points[pt][dim] + radar_extrinsics_pos[dim]
            )
            assert accumulated_points_ego_relative[pt].snr == input_points[pt][4]
            assert (
                accumulated_points_ego_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )
            assert accumulated_points_ego_relative[pt].radar_id == radar_id


def lat_lon_plus_offset(lat, lon, offset_east_m, offset_north_m):
    earth_radius = 6378137.0
    out_lat = lat + (offset_north_m / earth_radius) * (180 / pi)
    out_lon = lon + (offset_east_m / earth_radius) * (180 / pi) / cos(lat * pi / 180)
    return out_lat, out_lon


def test_accumulate_dds_nav_sat_fix_localization():
    localization_topic = "rt/test_nav_sat_fix_topic"
    pointcloud2_topic = "rt/test_pointcloud2_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant()
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant()

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.NavSatFix,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=None,  # Ego=radar assumed
        snr_threshold=0,
        point_filter=None,
        dds_domain_participant=dds_domain_participant_subscribers,
    )

    radar_id = "test_radar"
    # First PC and appropriate ego position
    point_0 = [1, 2, 3, 4, 5, 6]
    ego_fix_0 = [
        52.705502,
        -8.899619,
        3,
    ]  # Local ENU will be set to [52.705502, -8.899619, 0]
    # Second PC and appropriate ego position (straight east from fix 0)
    point_1 = [10, 20, 30, 40, 50, 60]
    point_2 = [100, 200, 300, 400, 500, 600]
    ego_enu_offset_1 = [5, 0, 4]
    ego_fix_1 = [
        *lat_lon_plus_offset(
            ego_fix_0[0], ego_fix_0[1], ego_enu_offset_1[0], ego_enu_offset_1[1]
        ),
        ego_enu_offset_1[2],
    ]
    # Later ego position (straight north from fix 0)
    ego_enu_offset_now = [0, 10, 20]
    ego_fix_now = [
        *lat_lon_plus_offset(
            ego_fix_0[0], ego_fix_0[1], ego_enu_offset_now[0], ego_enu_offset_now[1]
        ),
        ego_enu_offset_now[2],
    ]

    localization_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        localization_topic,
        provizio_dds.NavSatFixPubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_nav_sat_fix_localization localization_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )
    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_nav_sat_fix_localization pc2_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    time.sleep(
        0.3
    )  # Make sure there is enough time for publishers and subscribers to match
    publish_nav_sat_fix(
        localization_publisher,
        ego_fix_0,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_0], radar_id)

    time.sleep(0.2)  # to make sure the delivery order
    publish_nav_sat_fix(
        localization_publisher,
        ego_fix_1,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_1, point_2], radar_id)

    time.sleep(0.2)  # to make sure the delivery order
    publish_nav_sat_fix(
        localization_publisher,
        ego_fix_now,
    )

    accumulated_points_local_frame_relative = (
        accumulator.get_points_local_frame_relative()
    )
    assert len(accumulated_points_local_frame_relative) == 3

    accumulated_points_ego_relative = accumulator.get_points_ego_relative()
    assert len(accumulated_points_ego_relative) == 3

    input_points = [point_0, point_1, point_2]
    ego_poses_local_enu_space = [
        [0, 0, ego_fix_0[2]],  # As local enu is always set on the Earth spheroid
        ego_enu_offset_1,
        ego_enu_offset_1,  # ego_enu_offset_1 is duplicated as point_1 and point_2 share same fix position
    ]  # Last ego_pos_1 is not a typo, as point_1 and point_2 share same ego_pos

    # Check all components of all transformed points are as they should be, in appropriate spaces
    precision = 0.05  # 5 cm error permitted, due to approximate earth radius (which is actually different in different latitudes) used in lat_lon_plus_offset
    for pt in range(3):
        for dim in range(3):
            # local frame relative
            assert isclose(
                accumulated_points_local_frame_relative[pt].position[dim],
                input_points[pt][dim] + ego_poses_local_enu_space[pt][dim],
                abs_tol=precision,
            )
            assert accumulated_points_local_frame_relative[pt].radar_id == radar_id
            assert (
                accumulated_points_local_frame_relative[pt].snr == input_points[pt][4]
            )
            assert (
                accumulated_points_local_frame_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )

            # ego relative (except position)
            assert accumulated_points_ego_relative[pt].snr == input_points[pt][4]
            assert (
                accumulated_points_ego_relative[pt].ground_relative_velocity
                == input_points[pt][5]
            )
            assert accumulated_points_ego_relative[pt].radar_id == radar_id
        # ego relative position
        assert isclose(
            accumulated_points_ego_relative[pt].position[0],
            input_points[pt][1]
            + ego_poses_local_enu_space[pt][1]
            - ego_enu_offset_now[1],
            abs_tol=precision,
        )
        assert isclose(
            accumulated_points_ego_relative[pt].position[1],
            -input_points[pt][0]
            - ego_poses_local_enu_space[pt][0]
            + ego_enu_offset_now[0],
            abs_tol=precision,
        )
        assert isclose(
            accumulated_points_ego_relative[pt].position[2],
            input_points[pt][2]
            + ego_poses_local_enu_space[pt][2]
            - ego_enu_offset_now[2],
            abs_tol=precision,
        )


def test_accumulate_dds_on_pc2_callback():
    pointcloud2_topic = "rt/test_pointcloud2_topic"
    radar_id = "test_radar"
    point = [1, 2, 3, 4, 5, 6]
    dds_domain_participant_publishers = provizio_dds.make_domain_participant()
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant()
    mutex = threading.Lock()
    num_accumulated = 0

    def do_on_point_cloud(accumulator):
        nonlocal mutex
        nonlocal num_accumulated
        with mutex:
            num_accumulated = len(accumulator.get_points_local_frame_relative())
            print(f"test_accumulate_dds_on_pc2_callback on_point_cloud called, {num_accumulated} points were accumulated")

    dds_accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        1,
        localization_type=None,
        localization_topic=None,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=None,
        snr_threshold=0,
        point_filter=None,
        dds_domain_participant=dds_domain_participant_subscribers,
        on_point_cloud=lambda accumulator: do_on_point_cloud(accumulator),
    )

    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        lambda _, has_subscriber: print(
            f"test_accumulate_dds_on_pc2_callback pc2_publisher has_subscriber={has_subscriber}"
        ),
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    time.sleep(
        0.3
    )  # Make sure there is enough time for publishers and subscribers to match
    assert num_accumulated == 0  # Nothing accumulated
    publish_pc2(pc2_publisher, [point], radar_id)
    time.sleep(0.2)  # Make sure there is enough time to publish
    assert (
        num_accumulated == 1
    )  # A callback was indeed invoked and 1 point was accumulated


test_accumulate_0_accumulated_point_clouds()
test_accumulate_extrinsics()
test_accumulate_move_no_extrinsics()
test_accumulate_move_simple_extrinsics()
test_accumulate_move_simple_extrinsics_snr_filter()
test_accumulate_snr_and_velocity_filters()
test_accumulate_radar_filter()
test_accumulate_rotation_yaw()
test_accumulate_rotation_pitch()
test_accumulate_rotation_roll()
test_accumulate_rotation_yaw_with_extrinsics()
test_accumulate_rotation_and_move_simple_no_extrinsics()
test_accumulate_rotation_and_move_simple_with_extrinsics()
test_accumulate_rotation_and_move_no_extrinsics()
test_accumulate_rotation_and_move_with_extrinsics()
test_accumulate_overflow()
test_accumulate_dds_simple()
test_accumulate_dds_simple_extrinsics()
test_accumulate_dds_simple_extrinsics_including_localization()
test_accumulate_dds_no_localization_no_extrinsics()
test_accumulate_dds_no_localization_with_extrinsics()
test_accumulate_dds_nav_sat_fix_localization()
test_accumulate_dds_on_pc2_callback()
