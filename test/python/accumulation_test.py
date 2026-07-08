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
import random
import time
import threading
from typing import List

# All participants in this test process share this domain, chosen once at random within the DDS-safe range and away
# from 0. These tests integrate every sample they receive on the topics they subscribe to — including the default
# localization-extrinsics topic rt/provizio_extrinsics. Provizio's self-hosted CI includes real radar boards whose
# resident software publishes on that standard topic on the default domain; that extrinsics shifts every accumulated
# ego pose by a constant and corrupts the tests. Loopback confinement cannot exclude a publisher on the same board, so
# a per-process domain is needed to give each test its own discovery space (it also isolates against any concurrent
# run on another host).
TEST_DOMAIN = random.randint(1, 200)  # DDS-safe range, excluding domain 0


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
    # ego query is identity, so ego-relative positions equal the local-frame positions
    for dim in range(3):
        assert isclose(
            points_local_frame[0].position[dim],
            points_ego[0].position[dim],
            abs_tol=precision,
        )


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

    _wait_for_accumulated_points(accumulator, 3)
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

    _wait_for_accumulated_points(accumulator, 3)
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

                expected_unfiltered_accumulated_points = []
                expected_filtered_accumulated_points = []
                points = []
                for snr in snrs:
                    for velocity in velocities:
                        points.append([1, 2, 3, 4, snr, velocity])

                        if snr >= snr_threshold:
                            expected_unfiltered_accumulated_points.append(points[-1])
                            if velocity >= velocity_threshold:
                                expected_filtered_accumulated_points.append(points[-1])

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
                    points_local_frame_relative = (
                        accumulator.get_points_local_frame_relative()
                    )
                    assert len(points_local_frame_relative) == len(
                        expected_unfiltered_accumulated_points
                    )
                    for i in range(len(points_local_frame_relative)):
                        assert (
                            points_local_frame_relative[i].ground_relative_velocity
                            == expected_unfiltered_accumulated_points[i][5]
                        )
                        assert (
                            points_local_frame_relative[i].snr
                            == expected_unfiltered_accumulated_points[i][4]
                        )

                # Accumulate after, till the filter is applied
                for accumulate_after in range(max_frames_without_filter):
                    accumulator.accumulate(
                        radar_id,
                        [],
                        provizio_dds.accumulation.RigidTransform([0, 0, 0], [0, 0, 0]),
                    )

                # Filter has been applied now
                points_local_frame_relative = (
                    accumulator.get_points_local_frame_relative()
                )
                assert len(points_local_frame_relative) == len(
                    expected_filtered_accumulated_points
                )
                for i in range(len(points_local_frame_relative)):
                    assert (
                        points_local_frame_relative[i].ground_relative_velocity
                        == expected_filtered_accumulated_points[i][5]
                    )
                    assert (
                        points_local_frame_relative[i].snr
                        == expected_filtered_accumulated_points[i][4]
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


def test_accumulate_localization_correction():
    # The retroactive localization-extrinsics correction (apply_localization_correction): a frame accumulated
    # under the identity assumption and then corrected for a late-arriving extrinsics E must equal one accumulated
    # with E known from the start — nothing dropped or resampled.
    radar_id = "test_radar"
    radar_extrinsics = provizio_dds.accumulation.RigidTransform([1, 2, 3], [0, 0, radians(30)])
    sensor_localization = provizio_dds.accumulation.RigidTransform([10, 20, 5], [0, 0, radians(45)])
    localization_extrinsics = provizio_dds.accumulation.RigidTransform([0.3, 0.5, 0.7], [0, 0, radians(90)])
    points = [[1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12]]
    precision = 0.0001

    # Reference: the localization extrinsics is known from the start, so the ego pose is sensor * inverse(E).
    reference = provizio_dds.accumulation.PointCloudsAccumulator(
        4, snr_threshold=0, point_filter=None, allow_no_extrinsics=False
    )
    reference.accumulate(
        radar_id,
        points,
        provizio_dds.accumulation.PointCloudsAccumulator.localization_from_sensor_to_ego_frame(
            sensor_localization, localization_extrinsics
        ),
        radar_extrinsics,
    )
    expected = reference.get_points_local_frame_relative()

    # Under test: accumulate assuming identity localization extrinsics, then correct once E becomes known.
    corrected = provizio_dds.accumulation.PointCloudsAccumulator(
        4, snr_threshold=0, point_filter=None, allow_no_extrinsics=False
    )
    corrected.accumulate(
        radar_id,
        points,
        provizio_dds.accumulation.PointCloudsAccumulator.localization_from_sensor_to_ego_frame(
            sensor_localization, provizio_dds.accumulation.RigidTransform([0, 0, 0], [0, 0, 0])
        ),
        radar_extrinsics,
    )
    # delta = E_prev @ inverse(E_new) = identity @ inverse(E) = inverse(E)
    corrected.apply_localization_correction(
        provizio_dds.accumulation.RigidTransform(from_matrix=localization_extrinsics.inversed_matrix())
    )
    actual = corrected.get_points_local_frame_relative()

    assert len(actual) == len(expected) == 2
    for pt in range(2):
        for dim in range(3):
            assert isclose(actual[pt].position[dim], expected[pt].position[dim], abs_tol=precision)


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
    frame_id: str = "provizio_radar_front_center",
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
    points: List[List[float]],
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


def _wait_until_matched(*publishers, timeout_sec=15.0):
    """Block until each publisher has at least one matched subscriber.

    With the match-publisher reader default, the accumulator's subscribers are
    DEFERRED: the DataReader is created only after a writer is discovered, then matches
    ~1 s later. A fixed sleep before a one-shot publish therefore races that match and
    can silently drop early samples — a RELIABLE writer does not deliver a sample to a
    reader that matched after the write (default VOLATILE durability). Waiting for the
    real match keeps these one-shot-publish tests deterministic, mirroring the C++
    tests' get_num_matched / publish_and_wait_for gating."""
    for publisher in publishers:
        assert (
            publisher.get_num_matched_subscribers(timeout_sec, 0.0) > 0
        ), f"accumulation_test: a subscriber failed to match within {timeout_sec}s"


def _wait_for_accumulated_points(accumulator, expected, timeout_sec=10.0):
    """Wait until the accumulator holds at least ``expected`` points.

    In the DDS-fed tests the point-cloud writer is ASYNCHRONOUS and (with the
    match-publisher default) the accumulator's reader is RELIABLE, so a one-shot publish
    is delivered and processed a short time AFTER publish() returns — reading the count
    immediately would race that delivery. Waiting for the count avoids that race; a
    timeout here therefore indicates real sample loss rather than mere latency. For the
    synchronous (non-DDS) tests the points are already accumulated, so this returns at
    once."""
    deadline = time.monotonic() + timeout_sec
    while (
        len(accumulator.get_points_local_frame_relative()) < expected
        and time.monotonic() < deadline
    ):
        time.sleep(0.02)
    # Fail here with a clear cause on timeout (real sample loss, per the docstring) rather
    # than letting it surface later as a less-direct count-mismatch assertion at the caller.
    actual = len(accumulator.get_points_local_frame_relative())
    assert actual >= expected, (
        f"timed out after {timeout_sec}s waiting for {expected} accumulated points, "
        f"got {actual} (likely sample loss)"
    )


def test_accumulate_dds_simple():
    localization_topic = "rt/test_dds_simple_localization_topic"
    pointcloud2_topic = "rt/test_dds_simple_pointcloud2_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant(TEST_DOMAIN)

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.Odometry,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=None,  # Ego=radar assumed
        snr_threshold=0,
        point_filter=None,
        kalman_localization=False,  # Test DDS plumbing; exact-equality assertions require no Kalman drift
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

    # Wait until EVERY publisher in this test actually has a matched subscriber before
    # the one-shot publishes below. With the match-publisher reader default the
    # accumulator's subscribers are DEFERRED (the DataReader is created only once a
    # writer is discovered, then matches ~1 s later), so a fixed sleep would race the
    # match and silently drop early samples. Each publisher is awaited independently:
    # reader match order is NOT publisher creation order, so it is not safe to wait on
    # one and assume the rest matched. Mirrors the C++ tests' get_num_matched gating.
    _wait_until_matched(localization_publisher, pc2_publisher)
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

    _wait_for_accumulated_points(accumulator, 3)
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
    localization_topic = "rt/test_dds_simple_extrinsics_localization_topic"
    pointcloud2_topic = "rt/test_dds_simple_extrinsics_pointcloud2_topic"
    extrinsics_topic = "rt/test_dds_simple_extrinsics_extrinsics_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant(TEST_DOMAIN)

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.Odometry,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=[extrinsics_topic],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=False,  # Test DDS plumbing; exact-equality assertions require no Kalman drift
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

    # Wait until EVERY publisher in this test actually has a matched subscriber before
    # the one-shot publishes below. With the match-publisher reader default the
    # accumulator's subscribers are DEFERRED (the DataReader is created only once a
    # writer is discovered, then matches ~1 s later), so a fixed sleep would race the
    # match and silently drop early samples. Each publisher is awaited independently:
    # reader match order is NOT publisher creation order, so it is not safe to wait on
    # one and assume the rest matched. Mirrors the C++ tests' get_num_matched gating.
    _wait_until_matched(extrinsics_publisher, localization_publisher, pc2_publisher)
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

    _wait_for_accumulated_points(accumulator, 3)
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
    localization_topic = "rt/test_dds_simple_extrinsics_including_localization_localization_topic"
    pointcloud2_topic = "rt/test_dds_simple_extrinsics_including_localization_pointcloud2_topic"
    radar_extrinsics_topic = "rt/test_dds_simple_extrinsics_including_localization_radar_extrinsics"
    localization_extrinsics_topic = "rt/test_dds_simple_extrinsics_including_localization_localization_extrinsics"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant(TEST_DOMAIN)

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.Odometry,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=[radar_extrinsics_topic],
        localization_extrinsics_topic=localization_extrinsics_topic,
        localization_frame_id="test_localization",  # select this source (odometry default is the front-center radar frame)
        snr_threshold=0,
        point_filter=None,
        kalman_localization=False,  # Test DDS plumbing with extrinsics; exact-equality assertions require no Kalman drift
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

    # Wait until EVERY publisher in this test actually has a matched subscriber before
    # the one-shot publishes below. With the match-publisher reader default the
    # accumulator's subscribers are DEFERRED (the DataReader is created only once a
    # writer is discovered, then matches ~1 s later), so a fixed sleep would race the
    # match and silently drop early samples. Each publisher is awaited independently:
    # reader match order is NOT publisher creation order, so it is not safe to wait on
    # one and assume the rest matched. Mirrors the C++ tests' get_num_matched gating.
    _wait_until_matched(radar_extrinsics_publisher, localization_extrinsics_publisher, localization_publisher, pc2_publisher)
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

    _wait_for_accumulated_points(accumulator, 3)
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
    pointcloud2_topic = "rt/test_dds_no_localization_no_extrinsics_pointcloud2_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant(TEST_DOMAIN)

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

    # Wait until EVERY publisher in this test actually has a matched subscriber before
    # the one-shot publishes below. With the match-publisher reader default the
    # accumulator's subscribers are DEFERRED (the DataReader is created only once a
    # writer is discovered, then matches ~1 s later), so a fixed sleep would race the
    # match and silently drop early samples. Each publisher is awaited independently:
    # reader match order is NOT publisher creation order, so it is not safe to wait on
    # one and assume the rest matched. Mirrors the C++ tests' get_num_matched gating.
    _wait_until_matched(pc2_publisher)
    publish_pc2(pc2_publisher, [point_0], radar_id)
    publish_pc2(pc2_publisher, [point_1, point_2], radar_id)

    _wait_for_accumulated_points(accumulator, 3)
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
    pointcloud2_topic = "rt/test_dds_no_localization_with_extrinsics_pointcloud2_topic"
    extrinsics_topic = "rt/test_dds_no_localization_with_extrinsics_extrinsics_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant(TEST_DOMAIN)

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

    # Wait until EVERY publisher in this test actually has a matched subscriber before
    # the one-shot publishes below. With the match-publisher reader default the
    # accumulator's subscribers are DEFERRED (the DataReader is created only once a
    # writer is discovered, then matches ~1 s later), so a fixed sleep would race the
    # match and silently drop early samples. Each publisher is awaited independently:
    # reader match order is NOT publisher creation order, so it is not safe to wait on
    # one and assume the rest matched. Mirrors the C++ tests' get_num_matched gating.
    _wait_until_matched(extrinsics_publisher, pc2_publisher)
    publish_extrinsics(
        extrinsics_publisher,
        provizio_dds.accumulation.RigidTransform(radar_extrinsics_pos, no_rotation),
        frame_id=radar_id,
    )
    time.sleep(0.2)  # to make sure the delivery order
    publish_pc2(pc2_publisher, [point_0], radar_id)
    publish_pc2(pc2_publisher, [point_1, point_2], radar_id)

    _wait_for_accumulated_points(accumulator, 3)
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
    localization_topic = "rt/test_dds_nav_sat_fix_localization_nav_sat_fix_topic"
    pointcloud2_topic = "rt/test_dds_nav_sat_fix_localization_pointcloud2_topic"
    dds_domain_participant_publishers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant(TEST_DOMAIN)

    accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        2,
        localization_type=provizio_dds.NavSatFix,
        localization_topic=localization_topic,
        pointcloud2_topic=pointcloud2_topic,
        extrinsics_topics=None,  # Ego=radar assumed
        snr_threshold=0,
        point_filter=None,
        kalman_localization=False,  # Test DDS + NavSatFix plumbing; deterministic assertions require no Kalman drift
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

    # Wait until EVERY publisher in this test actually has a matched subscriber before
    # the one-shot publishes below. With the match-publisher reader default the
    # accumulator's subscribers are DEFERRED (the DataReader is created only once a
    # writer is discovered, then matches ~1 s later), so a fixed sleep would race the
    # match and silently drop early samples. Each publisher is awaited independently:
    # reader match order is NOT publisher creation order, so it is not safe to wait on
    # one and assume the rest matched. Mirrors the C++ tests' get_num_matched gating.
    _wait_until_matched(localization_publisher, pc2_publisher)
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

    _wait_for_accumulated_points(accumulator, 3)
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
    pointcloud2_topic = "rt/test_dds_on_pc2_callback_pointcloud2_topic"
    radar_id = "test_radar"
    point = [1, 2, 3, 4, 5, 6]
    dds_domain_participant_publishers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    dds_domain_participant_subscribers = provizio_dds.make_domain_participant(TEST_DOMAIN)
    matched = threading.Event()
    callback_fired = threading.Event()
    num_accumulated = 0

    def do_on_point_cloud(accumulator):
        nonlocal num_accumulated
        num_accumulated = len(accumulator.get_points_local_frame_relative())
        print(
            f"test_accumulate_dds_on_pc2_callback on_point_cloud called, {num_accumulated} points were accumulated"
        )
        callback_fired.set()

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

    def on_pub_matched(_, has_subscriber):
        print(
            f"test_accumulate_dds_on_pc2_callback pc2_publisher has_subscriber={has_subscriber}"
        )
        if has_subscriber:
            matched.set()

    pc2_publisher = provizio_dds.Publisher(
        dds_domain_participant_publishers,
        pointcloud2_topic,
        provizio_dds.PointCloud2PubSubType,
        on_pub_matched,
        reliability_kind=provizio_dds.RELIABLE_RELIABILITY_QOS,
    )

    # Wait for the subscriber to actually match before publishing — the
    # old `time.sleep(0.3)` was borderline on slower configs (kilted
    # Debug clang in particular) and would publish before Fast-DDS 3.x's
    # discovery handshake had completed.
    assert matched.wait(timeout=10.0), "publisher never matched subscriber"
    assert num_accumulated == 0, "no point published yet, but accumulator already has data"

    # The publisher's on_matched fires when its *writer-side* discovery
    # observes a compatible reader, but the matched reader's RX path
    # (registering the topic handler, settling SHM/UDP routing) is an
    # independent asynchronous step that can still be in flight for
    # 10s-100s of ms after that point. A single publish in that window
    # is dropped on the floor by the not-quite-ready reader, and because
    # DDSPointCloudsAccumulator's subscriber is BEST_EFFORT (no ACKs,
    # no retransmits) and KEEP_LAST(1) the writer never re-sends it.
    # Publish in a small loop until the callback fires (or we time out),
    # which is robust to that race regardless of platform-specific
    # discovery timing. The accumulator is configured with
    # max_frames_per_radar=1, so duplicate publishes leave
    # num_accumulated == 1 — the assertion semantics are unchanged.
    publish_deadline = time.monotonic() + 5.0
    while not callback_fired.is_set() and time.monotonic() < publish_deadline:
        publish_pc2(pc2_publisher, [point], radar_id)
        callback_fired.wait(timeout=0.5)
    assert callback_fired.is_set(), "on_point_cloud callback never fired"
    assert (
        num_accumulated == 1
    ), f"expected 1 accumulated point, got {num_accumulated}"


test_accumulate_0_accumulated_point_clouds()
test_accumulate_extrinsics()
test_accumulate_move_no_extrinsics()
test_accumulate_move_simple_extrinsics()
test_accumulate_move_simple_extrinsics_snr_filter()
test_accumulate_snr_and_velocity_filters()
test_accumulate_radar_filter()
test_accumulate_localization_correction()
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


def test_localization_filter_cold_start():
    f = provizio_dds.accumulation.LocalizationFilter()
    assert not f.has_estimate()
    f.update(100.0, provizio_dds.accumulation.RigidTransform([5.0, 0.0, 0.0], [0.0, 0.0, 0.0]))
    assert f.has_estimate()
    assert abs(f.predict(100.5).translation()[0] - 5.0) < 1e-9   # rate still 0 -> holds last fix


def test_localization_filter_constant_velocity():
    f = provizio_dds.accumulation.LocalizationFilter()
    v, t = 2.0, 0.0
    for _ in range(25):
        f.update(t, provizio_dds.accumulation.RigidTransform([v * t, 0.0, 0.0], [0.0, 0.0, 0.0]))
        t += 0.1
    assert abs(f.predict(2.45).translation()[0] - v * 2.45) < 0.02   # extrapolates after warm-up


def test_localization_filter_yaw_wrap():
    from transforms3d.euler import quat2euler
    f = provizio_dds.accumulation.LocalizationFilter()
    f.update(0.0, provizio_dds.accumulation.RigidTransform([0.0, 0.0, 0.0], [0.0, 0.0, 3.0]))
    f.update(0.1, provizio_dds.accumulation.RigidTransform([0.0, 0.0, 0.0], [0.0, 0.0, -3.0]))
    yaw = quat2euler(f.predict(0.1).rotation())[2]
    assert abs(yaw) > 2.5   # near the +/-pi seam, not a ~0 mid-range artifact


def test_localization_filter_constant_yaw_rate():
    from math import sin, cos, atan2
    from transforms3d.euler import quat2euler
    f = provizio_dds.accumulation.LocalizationFilter()
    w, t = 0.5, 0.0
    for _ in range(25):
        yaw = atan2(sin(w * t), cos(w * t))   # wrapped constant yaw rate
        f.update(t, provizio_dds.accumulation.RigidTransform([0.0, 0.0, 0.0], [0.0, 0.0, yaw]))
        t += 0.1
    pred_yaw = quat2euler(f.predict(2.45).rotation())[2]
    expected = w * 2.45
    diff = atan2(sin(pred_yaw - expected), cos(pred_yaw - expected))   # wrapped error
    assert abs(diff) < 0.02   # yaw extrapolates at the estimated rate


test_localization_filter_cold_start()
test_localization_filter_constant_velocity()
test_localization_filter_yaw_wrap()
test_localization_filter_constant_yaw_rate()


def _make_odometry_message(
    rigid_transform: provizio_dds.accumulation.RigidTransform,
    child_frame_id: str = "provizio_radar_front_center",
    header_stamp_s: float = 0.0,
):
    """Build a provizio_dds.Odometry message from a RigidTransform.

    The child_frame_id selects the localization source: it must match the accumulator's localization_frame_id
    (which defaults to "provizio_radar_front_center" for odometry) or the fix is dropped. With no
    localization-extrinsics published for that frame, the identity-extrinsics path is exercised.
    header_stamp_s is the sensor-clock timestamp in seconds (split into sec/nanosec).
    """
    stamp_sec = int(header_stamp_s)
    stamp_nanosec = int((header_stamp_s - stamp_sec) * 1e9)
    header = provizio_dds.point_cloud2.make_header(stamp_sec, stamp_nanosec, "world")

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
    message.child_frame_id(child_frame_id)
    message.pose(pose_with_covariance)

    return message


def _make_pc2_message(points: List[List[float]], radar_id: str, header_stamp_s: float = 0.0):
    """Build a provizio_dds PointCloud2 message for the given radar points.

    header_stamp_s is the sensor-clock timestamp in seconds (split into sec/nanosec).
    """
    stamp_sec = int(header_stamp_s)
    stamp_nanosec = int((header_stamp_s - stamp_sec) * 1e9)
    header = provizio_dds.point_cloud2.make_header(stamp_sec, stamp_nanosec, radar_id)
    return provizio_dds.point_cloud2.make_radar_point_cloud(header, points)


def test_kalman_localization_predicts_at_pointcloud_receive_time():
    """Verify that kalman_localization=True extrapolates ego pose to point-cloud receive time.

    Builds an accumulator with an injected deterministic clock and drives the private
    localization and point-cloud callbacks directly (no live DDS). After a long warm-up
    ramp (20 fixes at 10 m/s) the Kalman rate estimate converges, and the predicted x
    at a time 0.5 s past the last fix should exceed the last-fix x by more than 1 m.
    With kalman_localization=False the accumulator uses the stale last fix, which stays
    at last_fix_x.
    """

    def run(kalman):
        clock = {"t": 0.0}
        acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
            max_frames_per_radar=5,
            localization_topic="rt/test_kalman_loc",
            localization_extrinsics_topic=None,
            extrinsics_topics=[],
            snr_threshold=0,
            point_filter=None,
            kalman_localization=kalman,
            time_source=lambda: clock["t"],
        )
        t = 0.0
        for _ in range(20):  # ego x = 10*t, constant 10 m/s; rate converges after warm-up
            clock["t"] = t
            acc._on_odometry(
                _make_odometry_message(
                    provizio_dds.accumulation.RigidTransform([10.0 * t, 0.0, 0.0], [0.0, 0.0, 0.0]),
                    child_frame_id="provizio_radar_front_center",
                )
            )
            t += 0.1
        last_fix_x = 10.0 * (t - 0.1)   # x at the last fed fix
        clock["t"] = (t - 0.1) + 0.5    # point cloud received 0.5 s after the last fix
        acc._on_point_cloud(
            _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar")
        )
        pts = acc.get_points_local_frame_relative()
        assert len(pts) == 1
        return pts[0].position[0], last_fix_x

    kalman_x, last_fix_x = run(kalman=True)
    nokalman_x, _ = run(kalman=False)

    print(f"test_kalman_localization_predicts_at_pointcloud_receive_time: "
          f"kalman_x={kalman_x:.4f}, nokalman_x={nokalman_x:.4f}, last_fix_x={last_fix_x:.4f}")

    assert abs(nokalman_x - last_fix_x) < 0.1, (
        f"kalman=False: expected stale last-fix x ~{last_fix_x:.4f}, got {nokalman_x:.4f}"
    )
    assert kalman_x > last_fix_x + 1.0, (
        f"kalman=True: expected extrapolation > {last_fix_x + 1.0:.4f} m, got {kalman_x:.4f} "
        f"(filter may not have extrapolated — check wiring)"
    )


test_kalman_localization_predicts_at_pointcloud_receive_time()


def test_kalman_localization_get_points_ego_relative_uses_predicted_query_pose():
    """Verify get_points_ego_relative() uses the Kalman-predicted ego pose at query time.

    Builds two accumulators (kalman on and off) with an injected clock, drives 20
    odometry fixes at 10 m/s (ego x = 10*t, constant velocity), feeds one point cloud
    at the radar origin at a known time, then advances the clock well past the last fix
    and calls get_points_ego_relative(). The Kalman-on result must differ from the
    kalman-off (last-fix) result by approximately v * dt_extra, proving the predicted
    current pose is used instead of the stale last fix.
    """
    v = 10.0          # 10 m/s in x
    dt = 0.1          # fix interval
    n_fixes = 20      # warm-up fixes
    dt_extra = 0.5    # clock advance past last fix before calling get_points_ego_relative

    def run(kalman):
        clock = {"t": 0.0}
        acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
            max_frames_per_radar=5,
            localization_topic="rt/test_kalman_query",
            localization_extrinsics_topic=None,
            extrinsics_topics=[],
            snr_threshold=0,
            point_filter=None,
            kalman_localization=kalman,
            time_source=lambda: clock["t"],
        )
        t = 0.0
        for _ in range(n_fixes):
            clock["t"] = t
            acc._on_odometry(
                _make_odometry_message(
                    provizio_dds.accumulation.RigidTransform([v * t, 0.0, 0.0], [0.0, 0.0, 0.0]),
                    child_frame_id="provizio_radar_front_center",
                )
            )
            t += dt

        # feed one point at the radar origin at the time of the last fix
        last_fix_t = t - dt
        last_fix_x = v * last_fix_t
        clock["t"] = last_fix_t
        acc._on_point_cloud(
            _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar")
        )

        # advance the clock past the last fix and query
        clock["t"] = last_fix_t + dt_extra
        pts = acc.get_points_ego_relative()
        assert len(pts) == 1, f"expected 1 point, got {len(pts)}"
        # The single point is at the radar origin; in ego-relative coords:
        # x = local_frame_x_at_accumulation - ego_x_now = last_fix_x - ego_x_now
        # (no rotation, no extrinsics, pure translation)
        return pts[0].position[0], last_fix_x

    kalman_pt_x, last_fix_x = run(kalman=True)
    nokalman_pt_x, _ = run(kalman=False)

    # ego_x at query time for kalman-on is predicted: ~last_fix_x + v*dt_extra
    # ego-relative point x = last_fix_x - ego_x_query
    #   kalman-on:  ~last_fix_x - (last_fix_x + v*dt_extra) = -v*dt_extra  (~-5.0)
    #   kalman-off: ~last_fix_x - last_fix_x = 0.0
    expected_kalman_shift = -v * dt_extra   # ~-5.0

    print(
        f"test_kalman_localization_get_points_ego_relative_uses_predicted_query_pose: "
        f"kalman_pt_x={kalman_pt_x:.4f}, nokalman_pt_x={nokalman_pt_x:.4f}, "
        f"last_fix_x={last_fix_x:.4f}, expected_kalman_shift={expected_kalman_shift:.4f}"
    )

    # kalman=False uses the last-received localization: point should appear near 0 in x
    assert abs(nokalman_pt_x) < 0.5, (
        f"kalman=False: expected ~0 ego-relative x (last-fix ego), got {nokalman_pt_x:.4f}"
    )
    # kalman=True uses predicted ego at query time: x should be shifted by ~-v*dt_extra
    assert kalman_pt_x < nokalman_pt_x - 1.0, (
        f"kalman=True: expected x < {nokalman_pt_x - 1.0:.4f} (shifted by predicted motion), "
        f"got {kalman_pt_x:.4f}; kalman-off was {nokalman_pt_x:.4f}"
    )


test_kalman_localization_get_points_ego_relative_uses_predicted_query_pose()


def _feed_odometry_warm_up(acc, clock, n_fixes=15, v=10.0, base_header=1000.0, dt=0.1):
    """Feed n_fixes odometry messages with ego x = v * header_s, header = base_header + k*dt.

    Returns (H_last, last_fix_x) where H_last is the header stamp of the final fix.
    """
    for k in range(n_fixes):
        header_s = base_header + k * dt
        clock["t"] = header_s  # receive time == header_s (simplifies flush trigger control)
        acc._on_odometry(
            _make_odometry_message(
                provizio_dds.accumulation.RigidTransform([v * header_s, 0.0, 0.0], [0.0, 0.0, 0.0]),
                child_frame_id="provizio_radar_front_center",
                header_stamp_s=header_s,
            )
        )
    H_last = base_header + (n_fixes - 1) * dt
    return H_last, v * H_last


def test_timesync_buffers_until_covering_localization():
    """Verify that timesync_max_delay_seconds > 0 buffers the cloud until covering odometry arrives.

    Warm-up: 15 odometry fixes, ego x = v * header_s (v=10 m/s), headers at 1000+k*0.1 s.
    Cloud arrives with header H_last+0.2 (not yet covered); assert it stays buffered.
    Then feed an odometry fix at H_last+0.2 (x = v*(H_last+0.2)); assert the cloud is
    released at x ~= v*(H_last+0.2) (predict(H_last+0.2) with dt~0).
    """
    v = 10.0
    base_header = 1000.0
    dt = 0.1
    n_fixes = 15

    clock = {"t": 0.0}
    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_timesync_buffer",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=1.0,   # long timeout so it never fires during this test
        time_source=lambda: clock["t"],
    )

    H_last, last_fix_x = _feed_odometry_warm_up(acc, clock, n_fixes=n_fixes, v=v,
                                                  base_header=base_header, dt=dt)

    # Cloud at H_last+0.2 — not yet covered (latest loc header == H_last)
    pc_header_s = H_last + 0.2
    clock["t"] = pc_header_s  # receive time same as header for simplicity
    acc._on_point_cloud(
        _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=pc_header_s)
    )

    # Assert buffered — not yet accumulated
    n_buffered = len(acc.get_points_local_frame_relative())
    print(f"test_timesync_buffers_until_covering_localization: "
          f"after cloud arrive (header={pc_header_s:.1f}): buffered point count={n_buffered}")
    assert n_buffered == 0, (
        f"expected 0 points (cloud buffered), got {n_buffered}"
    )

    # Feed covering odometry at H_last+0.2
    covering_header_s = H_last + 0.2
    expected_x = v * covering_header_s
    clock["t"] = covering_header_s
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([expected_x, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=covering_header_s,
        )
    )

    pts = acc.get_points_local_frame_relative()
    n_after = len(pts)
    placed_x = pts[0].position[0] if pts else float("nan")
    print(f"test_timesync_buffers_until_covering_localization: "
          f"after covering odom (header={covering_header_s:.1f}, expected_x={expected_x:.4f}): "
          f"n_points={n_after}, placed_x={placed_x:.4f}")

    assert n_after == 1, f"expected 1 accumulated point after covering odom, got {n_after}"
    assert abs(placed_x - expected_x) < 0.5, (
        f"cloud placed at x={placed_x:.4f}, expected ~{expected_x:.4f} (capture-time pose)"
    )


test_timesync_buffers_until_covering_localization()


def test_timesync_timeout_releases_with_extrapolation():
    """Verify that on timeout the buffered cloud is released with best-effort extrapolation.

    Warm-up: 15 odometry fixes at v=10 m/s, headers 1000+k*0.1 s.
    Cloud arrives with header H_last+0.2 (not yet covered); stays buffered.
    Advance clock by > timesync_max_delay_seconds without feeding covering odometry.
    Feed a non-covering odometry (header < cloud's), which triggers _flush_timesync_buffer.
    Assert the cloud is released at predict(H_last+0.2) — forward extrapolation beyond last fix.
    """
    v = 10.0
    base_header = 1000.0
    dt = 0.1
    n_fixes = 15
    max_delay = 0.3

    clock = {"t": 0.0}
    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_timesync_timeout",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=max_delay,
        time_source=lambda: clock["t"],
    )

    H_last, last_fix_x = _feed_odometry_warm_up(acc, clock, n_fixes=n_fixes, v=v,
                                                  base_header=base_header, dt=dt)

    # Cloud at H_last+0.2 — not yet covered
    pc_header_s = H_last + 0.2
    pc_receive_s = H_last + 0.2
    clock["t"] = pc_receive_s
    acc._on_point_cloud(
        _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=pc_header_s)
    )
    assert len(acc.get_points_local_frame_relative()) == 0, "cloud should be buffered"

    # Advance clock past the timeout threshold
    clock["t"] = pc_receive_s + max_delay + 0.1   # definitely timed out

    # Feed a non-covering odometry (header < cloud's header) — triggers _flush_timesync_buffer
    non_covering_header_s = H_last + 0.05   # still before the cloud's header
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([v * non_covering_header_s, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=non_covering_header_s,
        )
    )

    pts = acc.get_points_local_frame_relative()
    n_after = len(pts)
    placed_x = pts[0].position[0] if pts else float("nan")

    # Expected: predict(pc_header_s) = forward extrapolation from last fix at H_last
    # With a well-warmed-up filter at v=10, predict at H_last+0.2 should give ≈ v*(H_last+0.2)
    expected_extrap_x = v * pc_header_s

    print(f"test_timesync_timeout_releases_with_extrapolation: "
          f"n_points={n_after}, placed_x={placed_x:.4f}, "
          f"expected_extrap_x={expected_extrap_x:.4f}, last_fix_x={last_fix_x:.4f}")

    assert n_after == 1, f"expected 1 accumulated point after timeout, got {n_after}"
    assert abs(placed_x - expected_extrap_x) < 1.0, (
        f"timed-out cloud placed at x={placed_x:.4f}, expected ~{expected_extrap_x:.4f} "
        f"(forward extrapolation from last fix {last_fix_x:.4f})"
    )


test_timesync_timeout_releases_with_extrapolation()


def test_timesync_off_accumulates_immediately():
    """Verify that timesync_max_delay_seconds=0.0 accumulates point clouds immediately, without buffering.

    Feed odometry warm-up then one cloud; assert the cloud is accumulated right away
    (not deferred to a future localization message).
    """
    v = 10.0
    base_header = 1000.0
    dt = 0.1
    n_fixes = 15

    clock = {"t": 0.0}
    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_timesync_off",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=0.0,
        time_source=lambda: clock["t"],
    )

    H_last, _ = _feed_odometry_warm_up(acc, clock, n_fixes=n_fixes, v=v,
                                         base_header=base_header, dt=dt)

    pc_header_s = H_last + 0.2
    clock["t"] = pc_header_s
    acc._on_point_cloud(
        _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=pc_header_s)
    )

    n_points = len(acc.get_points_local_frame_relative())
    print(f"test_timesync_off_accumulates_immediately: n_points={n_points} (expected 1, immediately)")
    assert n_points == 1, (
        f"timesync_max_delay_seconds=0.0: expected cloud accumulated immediately (1 point), got {n_points}"
    )


test_timesync_off_accumulates_immediately()


def test_timesync_on_point_cloud_callback_fires_at_release():
    """Verify that on_point_cloud fires exactly once per RELEASED (accumulated) cloud, not at buffer time.

    Regression test for the callback-firing parity bug: the old code fired on_point_cloud
    unconditionally once per incoming pc2 message (even when merely buffered), and never
    fired when a localization message triggered the release.

    Scenario:
      1. Warm up the Kalman filter with several odometry fixes.
      2. Feed ONE point cloud whose header is AHEAD of the latest odometry (buffered, not covered).
         Assert: callback fired 0 times; no points accumulated.
      3. Feed a covering odometry (header >= cloud's header).
         Assert: callback fired exactly 1 time; 1 point accumulated.
    """
    v = 10.0
    base_header = 1000.0
    dt = 0.1
    n_fixes = 15

    clock = {"t": 0.0}
    callback_count = [0]

    def on_point_cloud(accumulator):
        callback_count[0] += 1

    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_timesync_callback",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=0.3,   # short but won't expire during the test
        time_source=lambda: clock["t"],
        on_point_cloud=on_point_cloud,
    )

    H_last, _ = _feed_odometry_warm_up(acc, clock, n_fixes=n_fixes, v=v,
                                        base_header=base_header, dt=dt)

    # Feed one point cloud at H_last+0.2 — NOT covered (latest loc header == H_last)
    pc_header_s = H_last + 0.2
    clock["t"] = pc_header_s
    acc._on_point_cloud(
        _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=pc_header_s)
    )

    n_accumulated_after_buffer = len(acc.get_points_local_frame_relative())
    fired_after_buffer = callback_count[0]
    print(f"test_timesync_on_point_cloud_callback_fires_at_release: "
          f"after buffering cloud (header={pc_header_s:.1f}): "
          f"callback_fired={fired_after_buffer}, n_points={n_accumulated_after_buffer}")
    assert fired_after_buffer == 0, (
        f"on_point_cloud must NOT fire when cloud is merely buffered; fired {fired_after_buffer} time(s)"
    )
    assert n_accumulated_after_buffer == 0, (
        f"cloud should be buffered (0 points), got {n_accumulated_after_buffer}"
    )

    # Feed covering odometry at H_last+0.2 — releases the buffered cloud
    covering_header_s = H_last + 0.2
    clock["t"] = covering_header_s
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([v * covering_header_s, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=covering_header_s,
        )
    )

    n_accumulated_after_release = len(acc.get_points_local_frame_relative())
    fired_after_release = callback_count[0]
    print(f"test_timesync_on_point_cloud_callback_fires_at_release: "
          f"after covering odom (header={covering_header_s:.1f}): "
          f"callback_fired={fired_after_release}, n_points={n_accumulated_after_release}")
    assert fired_after_release == 1, (
        f"on_point_cloud must fire exactly once when the buffered cloud is released; fired {fired_after_release} time(s)"
    )
    assert n_accumulated_after_release == 1, (
        f"expected 1 accumulated point after release, got {n_accumulated_after_release}"
    )


test_timesync_on_point_cloud_callback_fires_at_release()


def test_timesync_interpolates_between_fixes():
    """Verify that a cloud whose header falls strictly between two fixes is placed at the
    interpolated midpoint pose — distinct from either fix.

    Setup (ego x = v * header_s, no rotation):
      Fix A:  header = H_A,   x = v * H_A
      Fix B:  header = H_B,   x = v * H_B    (H_B > H_A)
      Cloud:  header = H_mid = (H_A + H_B) / 2

    Expected placed_x = (v*H_A + v*H_B) / 2  (linear midpoint).
    The tolerance is tight (< 0.02 m) because the scenario is fully deterministic and
    the filter plays no role in the covered case — interpolation is exact.
    """
    v = 10.0
    base_header = 1000.0
    dt = 0.1
    n_warm_up = 10   # enough for the Kalman filter to warm up (though not needed for the covered path)

    clock = {"t": 0.0}
    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_interp",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=2.0,   # long timeout — won't fire during this test
        time_source=lambda: clock["t"],
    )

    # Warm-up fixes so the filter has an estimate
    H_last, _ = _feed_odometry_warm_up(acc, clock, n_fixes=n_warm_up, v=v,
                                        base_header=base_header, dt=dt)

    # Fix A: one step past the warm-up range
    H_A = H_last + dt
    x_A = v * H_A
    clock["t"] = H_A
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([x_A, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_A,
        )
    )

    # Cloud at midpoint — buffered because latest_fix.header (H_A) < H_mid
    H_mid = H_A + dt / 2.0   # strictly between H_A and H_B
    clock["t"] = H_mid
    acc._on_point_cloud(
        _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=H_mid)
    )
    assert len(acc.get_points_local_frame_relative()) == 0, "cloud must stay buffered before Fix B"

    # Fix B: covers the cloud
    H_B = H_A + dt
    x_B = v * H_B
    clock["t"] = H_B
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([x_B, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_B,
        )
    )

    pts = acc.get_points_local_frame_relative()
    n_pts = len(pts)
    placed_x = pts[0].position[0] if pts else float("nan")
    expected_x = (x_A + x_B) / 2.0   # exact linear midpoint

    print(
        f"test_timesync_interpolates_between_fixes: "
        f"H_A={H_A:.3f} x_A={x_A:.4f}, H_B={H_B:.3f} x_B={x_B:.4f}, "
        f"H_mid={H_mid:.4f}, placed_x={placed_x:.6f}, expected_x={expected_x:.6f}"
    )

    assert n_pts == 1, f"expected 1 accumulated point, got {n_pts}"
    assert abs(placed_x - expected_x) < 0.02, (
        f"interpolated x={placed_x:.6f} deviates from midpoint {expected_x:.6f} by "
        f"{abs(placed_x - expected_x):.6f} (should be < 0.02 m)"
    )
    # Sanity: result must differ from either fix
    assert abs(placed_x - x_A) > 0.01, (
        f"placed_x={placed_x:.6f} is too close to x_A={x_A:.6f}; interpolation not active"
    )
    assert abs(placed_x - x_B) > 0.01, (
        f"placed_x={placed_x:.6f} is too close to x_B={x_B:.6f}; interpolation not active"
    )


test_timesync_interpolates_between_fixes()


def test_timesync_interpolates_orientation():
    """Verify that the timesync release interpolates ORIENTATION (slerp), not just position.

    Setup (ego stays at the origin for both bracketing fixes, so position interpolation is a
    no-op and only orientation varies):
      Fix A:  header = H_A,   position (0,0,0), yaw = 0
      Cloud:  header = H_mid = (H_A + H_B) / 2, single point at local (x=1, y=0, z=0)
      Fix B:  header = H_B,   position (0,0,0), yaw = +90 deg (pi/2)

    On release frac = 0.5 -> interpolated yaw = 45 deg, so the local point (1,0,0) lands at world
    (cos45, sin45, 0) = (0.7071, 0.7071, 0). Asserting BOTH coords ~ 0.7071 proves the slerp took
    the SHORT way to 45 deg (a sign error / long way would give e.g. (0.707, -0.707) or
    (-0.707, ...)).
    """
    base_header = 1000.0
    dt = 0.1
    n_warm_up = 10   # enough for the Kalman filter to warm up (covered path needs an estimate)

    clock = {"t": 0.0}
    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_interp_orient",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=2.0,   # long timeout — won't fire during this test
        time_source=lambda: clock["t"],
    )

    # Warm-up fixes (origin, yaw 0) so the filter has an estimate
    H_last, _ = _feed_odometry_warm_up(acc, clock, n_fixes=n_warm_up, v=0.0,
                                       base_header=base_header, dt=dt)

    # Fix A: origin, yaw 0
    H_A = H_last + dt
    clock["t"] = H_A
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_A,
        )
    )

    # Cloud at midpoint — buffered because latest_fix.header (H_A) < H_mid. Point at local (1,0,0).
    H_mid = H_A + dt / 2.0   # strictly between H_A and H_B
    clock["t"] = H_mid
    acc._on_point_cloud(
        _make_pc2_message([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=H_mid)
    )
    assert len(acc.get_points_local_frame_relative()) == 0, "cloud must stay buffered before Fix B"

    # Fix B: origin, yaw +90 deg (pi/2) — covers the cloud
    H_B = H_A + dt
    clock["t"] = H_B
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([0.0, 0.0, 0.0], [0.0, 0.0, pi / 2]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_B,
        )
    )

    pts = acc.get_points_local_frame_relative()
    n_pts = len(pts)
    placed_x = pts[0].position[0] if pts else float("nan")
    placed_y = pts[0].position[1] if pts else float("nan")
    expected = sqrt(2.0) / 2.0   # 0.70710678...

    print(
        f"test_timesync_interpolates_orientation: "
        f"placed=({placed_x:.6f}, {placed_y:.6f}), expected=({expected:.6f}, {expected:.6f})"
    )

    assert n_pts == 1, f"expected 1 accumulated point, got {n_pts}"
    assert abs(placed_x - expected) < 1e-2, (
        f"interpolated yaw 45 deg should place x ~ {expected:.6f}, got {placed_x:.6f} "
        f"(short-way slerp)"
    )
    assert abs(placed_y - expected) < 1e-2, (
        f"interpolated yaw 45 deg should place y ~ {expected:.6f}, got {placed_y:.6f} "
        f"(short-way slerp)"
    )


test_timesync_interpolates_orientation()


def test_timesync_drops_out_of_order_fix():
    """Verify that a stale (out-of-order) localization fix is silently dropped.

    Scenario:
      1. Warm up with monotonic fixes (ego x = v * header_s).
      2. Inject a fix at Fix A (first bracket), then a cloud at H_mid strictly between A and B.
      3. Inject a STALE fix (header older than Fix A, wildly wrong x = 99999).
         This must not corrupt fixes or the Kalman filter.
      4. Inject Fix B (second bracket, covers the cloud).
         The cloud must be interpolated at midpoint of (Fix A, Fix B) — the stale fix ignored.
    """
    v = 10.0
    base_header = 1000.0
    dt = 0.1
    n_warm_up = 10

    clock = {"t": 0.0}
    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_stale_drop",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=2.0,
        time_source=lambda: clock["t"],
    )

    H_last, _ = _feed_odometry_warm_up(acc, clock, n_fixes=n_warm_up, v=v,
                                        base_header=base_header, dt=dt)

    # Fix A (lower bracket)
    H_A = H_last + dt
    x_A = v * H_A
    clock["t"] = H_A
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([x_A, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_A,
        )
    )

    # Cloud at midpoint — buffered
    H_mid = H_A + dt / 2.0
    clock["t"] = H_mid
    acc._on_point_cloud(
        _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=H_mid)
    )
    assert len(acc.get_points_local_frame_relative()) == 0, "cloud must be buffered"

    # Stale fix: header older than H_A, wildly wrong pose
    H_stale = H_last - dt   # before H_A, so header_s < latest_fix.header
    clock["t"] = H_stale + 0.001   # receive time (irrelevant — guard fires on header)
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([99999.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_stale,
        )
    )
    # Cloud must still be buffered (stale fix didn't corrupt state)
    assert len(acc.get_points_local_frame_relative()) == 0, \
        "cloud must remain buffered after stale fix injection"

    # Fix B (upper bracket, covers the cloud)
    H_B = H_A + dt
    x_B = v * H_B
    clock["t"] = H_B
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([x_B, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_B,
        )
    )

    pts = acc.get_points_local_frame_relative()
    n_pts = len(pts)
    placed_x = pts[0].position[0] if pts else float("nan")
    expected_x = (x_A + x_B) / 2.0   # midpoint of real fixes A and B

    print(
        f"test_timesync_drops_out_of_order_fix: "
        f"H_A={H_A:.3f} x_A={x_A:.4f}, H_B={H_B:.3f} x_B={x_B:.4f}, "
        f"H_stale={H_stale:.3f} (x=99999), H_mid={H_mid:.4f}, "
        f"placed_x={placed_x:.6f}, expected_x={expected_x:.6f}"
    )

    assert n_pts == 1, f"expected 1 accumulated point after Fix B, got {n_pts}"
    assert abs(placed_x - expected_x) < 0.02, (
        f"interpolated x={placed_x:.6f} should be midpoint {expected_x:.6f}; "
        f"stale fix may have corrupted state (placed_x - expected_x = "
        f"{placed_x - expected_x:.6f})"
    )
    # Stale fix must not have influenced the result
    assert abs(placed_x - 99999.0) > 1000.0, (
        f"placed_x={placed_x:.4f} is suspiciously close to stale fix x=99999"
    )


test_timesync_drops_out_of_order_fix()


def test_timesync_stale_cloud_uses_previous_fix():
    """Verify that a cloud whose header is older than previous_fix is placed at previous_fix's pose.

    "Stale cloud" (badly out-of-order delivery): the cloud arrives late — by the time _on_point_cloud
    runs, TWO consecutive localization fixes are already in the ring.  The cloud's header timestamp
    falls strictly between the warm-up's last fix and Fix A, i.e. older than Fix A (previous_fix).

    Setup (ego x = v * header_s, no rotation):
      Warm-up: fixes up to H_last
      Fix A (will become previous_fix):  header = H_A = H_last + dt,  x = v * H_A
      Fix B (will become latest_fix):    header = H_B = H_last + 2*dt, x = v * H_B
      Cloud header:  H_cloud = H_last + dt/2  (strictly between H_last and H_A)

    Feed order: Fix A → Fix B → Cloud.
    When the cloud arrives after both fixes:
      covered:  latest_fix.header (H_B) >= H_cloud → True
      previous_fix.header (H_A) <= H_cloud? H_A > H_cloud → False
      → use previous_fix.pose → placed_x = x_A = v * H_A

    The tolerance is tight (< 0.02 m) because this is an exact pose lookup — no interpolation.
    """
    v = 10.0
    base_header = 1000.0
    dt = 0.1
    n_warm_up = 5   # minimal warm-up so the filter exists

    clock = {"t": 0.0}
    acc = provizio_dds.accumulation.DDSPointCloudsAccumulator(
        max_frames_per_radar=5,
        localization_topic="rt/test_prev_fix",
        localization_extrinsics_topic=None,
        extrinsics_topics=[],
        snr_threshold=0,
        point_filter=None,
        kalman_localization=True,
        timesync_max_delay_seconds=5.0,   # long timeout — must not fire during setup
        time_source=lambda: clock["t"],
    )

    H_last, _ = _feed_odometry_warm_up(acc, clock, n_fixes=n_warm_up, v=v,
                                        base_header=base_header, dt=dt)

    H_A = H_last + dt           # Fix A header; will be previous_fix after Fix B
    H_B = H_last + 2 * dt       # Fix B header; will be latest_fix
    H_cloud = H_last + dt / 2.0  # strictly between H_last and H_A (older than Fix A)

    x_A = v * H_A
    x_B = v * H_B
    x_cloud_expected = x_A      # cloud placed at previous_fix.pose

    # Fix A: latest_fix becomes (H_A, x_A), previous_fix becomes warm-up's last
    clock["t"] = H_A
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([x_A, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_A,
        )
    )

    # Fix B: latest_fix becomes (H_B, x_B), previous_fix becomes (H_A, x_A)
    clock["t"] = H_B
    acc._on_odometry(
        _make_odometry_message(
            provizio_dds.accumulation.RigidTransform([x_B, 0.0, 0.0], [0.0, 0.0, 0.0]),
            child_frame_id="provizio_radar_front_center",
            header_stamp_s=H_B,
        )
    )
    assert len(acc.get_points_local_frame_relative()) == 0, \
        "no cloud fed yet — accumulator must be empty after Fix A and Fix B"

    # Cloud arrives with header H_cloud < H_A.  When _on_point_cloud calls
    # _flush_timesync_buffer, it immediately finds:
    #   covered = latest_fix.header (H_B) >= H_cloud → True
    #   previous_fix.header (H_A) <= H_cloud → False  (H_A > H_cloud)
    #   → use previous_fix.pose → x_A
    clock["t"] = H_B + 0.01   # receive time shortly after Fix B
    acc._on_point_cloud(
        _make_pc2_message([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], "test_radar",
                          header_stamp_s=H_cloud)
    )

    pts = acc.get_points_local_frame_relative()
    n_pts = len(pts)
    placed_x = pts[0].position[0] if pts else float("nan")

    print(
        f"test_timesync_stale_cloud_uses_previous_fix: "
        f"H_cloud={H_cloud:.4f}, H_A={H_A:.4f} (x_A={x_A:.4f}), H_B={H_B:.4f} (x_B={x_B:.4f}), "
        f"placed_x={placed_x:.6f}, expected_x (prev_fix=x_A)={x_cloud_expected:.6f}"
    )

    assert n_pts == 1, f"expected 1 accumulated point, got {n_pts}"
    assert abs(placed_x - x_cloud_expected) < 0.02, (
        f"stale cloud placed at x={placed_x:.6f}, expected previous_fix x={x_cloud_expected:.6f} "
        f"(difference={placed_x - x_cloud_expected:.6f})"
    )
    # Must NOT be placed at Fix B's x (latest_fix) or at the interpolated midpoint
    assert abs(placed_x - x_B) > 0.01, (
        f"placed_x={placed_x:.6f} must not equal Fix B (latest_fix) x={x_B:.6f}"
    )
    assert abs(placed_x - (x_A + x_B) / 2.0) > 0.01, (
        f"placed_x={placed_x:.6f} must not be the A-B midpoint {(x_A + x_B) / 2.0:.6f}"
    )


test_timesync_stale_cloud_uses_previous_fix()
