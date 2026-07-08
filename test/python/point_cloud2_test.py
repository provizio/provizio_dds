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

# Standalone tests for provizio_dds.point_cloud2 — radar point clouds, entities
# (entities_kind, read_entities, make_entities_from) and the public read_points /
# read_points_list helpers.  Pure message manipulation — no DDS participants involved.

import math
import sys
import types
import provizio_dds


# ---------------------------------------------------------------------------
# Radar point cloud creation / read_points_list
# ---------------------------------------------------------------------------

def test_radar_point_cloud_creation():
    points = [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [1.0, 2.0, 3.0, 4.0, 5.0, float("nan")]]
    cloud = provizio_dds.point_cloud2.make_radar_point_cloud(
        provizio_dds.point_cloud2.make_header(10, 20, "test_frame"), points
    )

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

    read_pts = provizio_dds.point_cloud2.read_points_list(cloud)
    assert (
        str(read_pts[0])
        == "Point(x=0.1, y=0.2, z=0.3, radar_relative_radial_velocity=0.4, signal_to_noise_ratio=0.5, ground_relative_radial_velocity=0.6)"
    ) or (
        str(read_pts[0])
        == "Point(x=np.float32(0.1), y=np.float32(0.2), z=np.float32(0.3), radar_relative_radial_velocity=np.float32(0.4), signal_to_noise_ratio=np.float32(0.5), ground_relative_radial_velocity=np.float32(0.6))"
    ), "Got:" + str(read_pts[0])
    assert (
        str(read_pts[1])
        == "Point(x=1.0, y=2.0, z=3.0, radar_relative_radial_velocity=4.0, signal_to_noise_ratio=5.0, ground_relative_radial_velocity=nan)"
    ) or (
        str(read_pts[1])
        == "Point(x=np.float32(1.0), y=np.float32(2.0), z=np.float32(3.0), radar_relative_radial_velocity=np.float32(4.0), signal_to_noise_ratio=np.float32(5.0), ground_relative_radial_velocity=np.float32(nan))"
    ), "Got:" + str(read_pts[1])


# ---------------------------------------------------------------------------
# Radar entities creation / read_points field layout
# ---------------------------------------------------------------------------

def test_radar_entities_creation():
    entities = [
        [99, 4, 20.5, -2.0, 1.0, 10.2, 25.0, 0, 0, 0, 1, 2, 5, 2, 254, 254],
        [100, 2, 10.0, 2.0, 1.0, -10.0, 0.0, 0, 0, 0, 1, float("nan"), float("nan"), float("nan"), 254, 12],
    ]
    entities_cloud = provizio_dds.point_cloud2.make_radar_entities(
        provizio_dds.point_cloud2.make_header(10, 20, "test_entities"), entities
    )

    assert entities_cloud.fields()[0].name() == "entity_id"
    assert entities_cloud.fields()[1].name() == "entity_class"
    assert entities_cloud.fields()[2].name() == "x"
    assert entities_cloud.fields()[3].name() == "y"
    assert entities_cloud.fields()[4].name() == "z"
    assert entities_cloud.fields()[5].name() == "radar_relative_radial_velocity"
    assert entities_cloud.fields()[6].name() == "ground_relative_radial_velocity"
    assert entities_cloud.fields()[7].name() == "orientation"
    assert entities_cloud.fields()[8].name() == "size"
    assert entities_cloud.fields()[9].name() == "entity_confidence"
    assert entities_cloud.fields()[10].name() == "entity_class_confidence"

    raw = provizio_dds.point_cloud2.read_points(entities_cloud)
    assert (
        str(raw[0])
        == "(99, 4, 20.5, -2., 1., 10.2, 25., 0., 0., 0., 1., 2., 5., 2., 254, 254)"
    ) or (
        str(raw[0])
        == "(99, 4, 20.5, -2.0, 1.0, 10.2, 25.0, 0.0, 0.0, 0.0, 1.0, 2.0, 5.0, 2.0, 254, 254)"
    ), "Got:" + str(raw[0])


# ---------------------------------------------------------------------------
# read_points_list — camera entities and empty cloud
# ---------------------------------------------------------------------------

def test_read_points_list_camera_entities():
    camera_entities = [[1, 2, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10, 11]]
    read_pts = provizio_dds.point_cloud2.read_points_list(
        provizio_dds.point_cloud2.make_camera_entities(
            provizio_dds.point_cloud2.make_header(10, 20, "test_entities"), camera_entities
        ),
        tuple_name="Entity",
    )
    assert len(read_pts) == 1
    assert (
        str(read_pts[0])
        == "Entity(camera_entity_id=1, entity_class=2, x=3., y=4., z=5., camera_bbox_0=6., camera_bbox_1=7., camera_bbox_2=8., camera_bbox_3=9., entity_confidence=10, entity_class_confidence=11)"
    ) or (
        str(read_pts[0])
        == "Entity(camera_entity_id=1, entity_class=2, x=3.0, y=4.0, z=5.0, camera_bbox_0=6.0, camera_bbox_1=7.0, camera_bbox_2=8.0, camera_bbox_3=9.0, entity_confidence=10, entity_class_confidence=11)"
    ) or (
        str(read_pts[0])
        == "Entity(camera_entity_id=np.uint32(1), entity_class=np.uint8(2), x=np.float32(3.0), y=np.float32(4.0), z=np.float32(5.0), camera_bbox_0=np.float32(6.0), camera_bbox_1=np.float32(7.0), camera_bbox_2=np.float32(8.0), camera_bbox_3=np.float32(9.0), entity_confidence=np.uint8(10), entity_class_confidence=np.uint8(11))"
    ), "Got:" + str(read_pts[0])


def test_read_points_list_empty():
    read_pts = provizio_dds.point_cloud2.read_points_list(
        provizio_dds.point_cloud2.make_radar_entities(
            provizio_dds.point_cloud2.make_header(10, 20, "test_entities"), []
        )
    )
    assert len(read_pts) == 0


# ---------------------------------------------------------------------------
# NO_ENTITY_ID sentinel value
# ---------------------------------------------------------------------------

def test_no_entity_id_value():
    assert provizio_dds.point_cloud2.NO_ENTITY_ID == 0xFFFFFFFF


# ---------------------------------------------------------------------------
# entities_kind probes
# ---------------------------------------------------------------------------

def test_entities_kind_radar():
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")
    cloud = provizio_dds.point_cloud2.make_radar_entities(
        header,
        [[42, 3, 1.5, -2.5, 0.5, 10.5, 9.5, 0.1, 0.2, 0.3, 0.4, 4.5, 1.8, 1.5, 90, 80]],
    )
    assert (
        provizio_dds.point_cloud2.entities_kind(cloud) == "radar"
    ), f"Expected 'radar', got {provizio_dds.point_cloud2.entities_kind(cloud)}"


def test_entities_kind_camera():
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")
    cloud = provizio_dds.point_cloud2.make_camera_entities(
        header, [[7, 1, 1.0, 2.0, 3.0, 10, 20, 30, 40, 95, 85]]
    )
    assert (
        provizio_dds.point_cloud2.entities_kind(cloud) == "camera"
    ), f"Expected 'camera', got {provizio_dds.point_cloud2.entities_kind(cloud)}"


def test_entities_kind_fused():
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")
    cloud = provizio_dds.point_cloud2.make_fused_entities(
        header,
        [[42, 7, 2, 1, 2, 3, 4, 5, 0.1, 0.2, 0.3, 0.4, 5, 6, 7, 11, 22, 33, 44, 70, 60]],
    )
    assert (
        provizio_dds.point_cloud2.entities_kind(cloud) == "fused"
    ), f"Expected 'fused', got {provizio_dds.point_cloud2.entities_kind(cloud)}"


def test_entities_kind_none_for_radar_points():
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")
    cloud = provizio_dds.point_cloud2.make_radar_point_cloud(
        header, [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]]
    )
    assert (
        provizio_dds.point_cloud2.entities_kind(cloud) is None
    ), f"Expected None for radar point cloud, got {provizio_dds.point_cloud2.entities_kind(cloud)}"


# ---------------------------------------------------------------------------
# read_entities round-trips — values, NaN absences, NO_ENTITY_ID absences
# ---------------------------------------------------------------------------

def test_read_entities_radar_roundtrip():
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")
    cloud = provizio_dds.point_cloud2.make_radar_entities(
        header,
        [[42, 3, 1.5, -2.5, 0.5, 10.5, 9.5, 0.1, 0.2, 0.3, 0.4, 4.5, 1.8, 1.5, 90, 80]],
    )
    ents = provizio_dds.point_cloud2.read_entities(cloud)

    assert len(ents) == 1, f"Expected 1 entity, got {len(ents)}"
    ent = ents[0]

    assert ent.entity_id == 42, f"entity_id: expected 42, got {ent.entity_id}"
    assert ent.entity_class == 3, f"entity_class: expected 3, got {ent.entity_class}"
    assert abs(ent.x - 1.5) < 1e-6, f"x: expected 1.5, got {ent.x}"
    assert ent.entity_confidence == 90, f"entity_confidence: expected 90, got {ent.entity_confidence}"
    assert ent.entity_class_confidence == 80, f"entity_class_confidence: expected 80, got {ent.entity_class_confidence}"

    assert len(ent.orientation) == 4, f"orientation should be a 4-tuple, got {ent.orientation}"
    assert abs(ent.orientation[0] - 0.1) < 1e-6, f"orientation[0]: expected 0.1, got {ent.orientation[0]}"
    assert abs(ent.orientation[1] - 0.2) < 1e-6, f"orientation[1]: expected 0.2, got {ent.orientation[1]}"
    assert abs(ent.orientation[2] - 0.3) < 1e-6, f"orientation[2]: expected 0.3, got {ent.orientation[2]}"
    assert abs(ent.orientation[3] - 0.4) < 1e-6, f"orientation[3]: expected 0.4, got {ent.orientation[3]}"

    assert len(ent.size) == 3, f"size should be a 3-tuple, got {ent.size}"
    assert abs(ent.size[2] - 1.5) < 1e-6, f"size[2]: expected 1.5, got {ent.size[2]}"

    # camera fields absent: camera_entity_id == NO_ENTITY_ID, camera_bbox NaN-tuple
    assert ent.camera_entity_id == provizio_dds.point_cloud2.NO_ENTITY_ID, (
        f"camera_entity_id should be NO_ENTITY_ID for radar cloud, got {ent.camera_entity_id}"
    )
    assert len(ent.camera_bbox) == 4, f"camera_bbox should be a 4-tuple, got {ent.camera_bbox}"
    assert all(math.isnan(v) for v in ent.camera_bbox), (
        f"camera_bbox elements should all be NaN for radar cloud, got {ent.camera_bbox}"
    )


def test_read_entities_camera_roundtrip():
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")
    cloud = provizio_dds.point_cloud2.make_camera_entities(
        header, [[7, 1, 1.0, 2.0, 3.0, 10, 20, 30, 40, 95, 85]]
    )
    ents = provizio_dds.point_cloud2.read_entities(cloud)

    assert len(ents) == 1, f"Expected 1 entity, got {len(ents)}"
    ent = ents[0]

    assert ent.camera_entity_id == 7, f"camera_entity_id: expected 7, got {ent.camera_entity_id}"

    assert len(ent.camera_bbox) == 4, f"camera_bbox should be a 4-tuple, got {ent.camera_bbox}"
    assert abs(ent.camera_bbox[0] - 10) < 1e-6, f"camera_bbox[0]: expected 10, got {ent.camera_bbox[0]}"
    assert abs(ent.camera_bbox[1] - 20) < 1e-6, f"camera_bbox[1]: expected 20, got {ent.camera_bbox[1]}"
    assert abs(ent.camera_bbox[2] - 30) < 1e-6, f"camera_bbox[2]: expected 30, got {ent.camera_bbox[2]}"
    assert abs(ent.camera_bbox[3] - 40) < 1e-6, f"camera_bbox[3]: expected 40, got {ent.camera_bbox[3]}"

    # radar fields absent: entity_id == NO_ENTITY_ID, radar_relative_radial_velocity NaN, orientation NaN-tuple
    assert ent.entity_id == provizio_dds.point_cloud2.NO_ENTITY_ID, (
        f"entity_id should be NO_ENTITY_ID for camera cloud, got {ent.entity_id}"
    )
    assert math.isnan(ent.radar_relative_radial_velocity), (
        f"radar_relative_radial_velocity should be NaN for camera cloud, got {ent.radar_relative_radial_velocity}"
    )
    assert len(ent.orientation) == 4, f"orientation should be a 4-tuple, got {ent.orientation}"
    assert all(math.isnan(v) for v in ent.orientation), (
        f"orientation elements should all be NaN for camera cloud, got {ent.orientation}"
    )


def test_read_entities_fused_roundtrip():
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")
    cloud = provizio_dds.point_cloud2.make_fused_entities(
        header,
        [[42, 7, 2, 1, 2, 3, 4, 5, 0.1, 0.2, 0.3, 0.4, 5, 6, 7, 11, 22, 33, 44, 70, 60]],
    )
    ents = provizio_dds.point_cloud2.read_entities(cloud)

    assert len(ents) == 1, f"Expected 1 entity, got {len(ents)}"
    ent = ents[0]

    assert ent.entity_id == 42, f"entity_id: expected 42, got {ent.entity_id}"
    assert ent.camera_entity_id == 7, f"camera_entity_id: expected 7, got {ent.camera_entity_id}"
    assert abs(ent.camera_bbox[0] - 11) < 1e-6, f"camera_bbox[0]: expected 11, got {ent.camera_bbox[0]}"
    assert ent.entity_confidence == 70, f"entity_confidence: expected 70, got {ent.entity_confidence}"


def test_read_entities_absent_ids():
    """read_entities: absent id fields return NO_ENTITY_ID; absent float fields return NaN."""
    entities = [
        [99, 4, 20.5, -2.0, 1.0, 10.2, 25.0, 0, 0, 0, 1, 2, 5, 2, 254, 254],
        [100, 2, 10.0, 2.0, 1.0, -10.0, 0.0, 0, 0, 0, 1, float("nan"), float("nan"), float("nan"), 254, 12],
    ]
    entities_cloud = provizio_dds.point_cloud2.make_radar_entities(
        provizio_dds.point_cloud2.make_header(10, 20, "test_entities"), entities
    )
    radar_ents_read = provizio_dds.point_cloud2.read_entities(entities_cloud)
    assert len(radar_ents_read) == 2
    # radar cloud has entity_id present; camera_entity_id is absent → NO_ENTITY_ID
    assert radar_ents_read[0].entity_id == 99, f"entity_id: {radar_ents_read[0].entity_id}"
    assert radar_ents_read[0].camera_entity_id == provizio_dds.point_cloud2.NO_ENTITY_ID, (
        f"camera_entity_id should be NO_ENTITY_ID, got {radar_ents_read[0].camera_entity_id}"
    )

    camera_cloud_single = provizio_dds.point_cloud2.make_camera_entities(
        provizio_dds.point_cloud2.make_header(10, 20, "test_entities"),
        [[1, 2, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10, 11]],
    )
    camera_ents_read = provizio_dds.point_cloud2.read_entities(camera_cloud_single)
    assert len(camera_ents_read) == 1
    assert camera_ents_read[0].camera_entity_id == 1, f"camera_entity_id: {camera_ents_read[0].camera_entity_id}"
    assert camera_ents_read[0].entity_id == provizio_dds.point_cloud2.NO_ENTITY_ID, (
        f"entity_id should be NO_ENTITY_ID, got {camera_ents_read[0].entity_id}"
    )
    assert math.isnan(camera_ents_read[0].radar_relative_radial_velocity), "absent radar_rrv must be NaN"
    assert math.isnan(camera_ents_read[0].orientation[0]), "absent orientation must be NaN"


# ---------------------------------------------------------------------------
# make_entities_from round-trips all 3 kinds + ValueError on bad kind
# ---------------------------------------------------------------------------

def test_make_entities_from_radar():
    r_ent1 = types.SimpleNamespace(
        entity_id=42, camera_entity_id=provizio_dds.point_cloud2.NO_ENTITY_ID,
        entity_class=3, x=1.5, y=-2.5, z=0.5,
        radar_relative_radial_velocity=10.5, ground_relative_radial_velocity=9.5,
        orientation=(0.1, 0.2, 0.3, 0.4), size=(4.5, 1.8, 1.5),
        camera_bbox=(float("nan"),) * 4, entity_confidence=90, entity_class_confidence=80
    )
    r_ent2 = types.SimpleNamespace(
        entity_id=55, camera_entity_id=provizio_dds.point_cloud2.NO_ENTITY_ID,
        entity_class=1, x=-1.0, y=2.0, z=3.0,
        radar_relative_radial_velocity=-5.0, ground_relative_radial_velocity=-4.0,
        orientation=(0.5, 0.6, 0.7, 0.8), size=(2.0, 1.0, 0.5),
        camera_bbox=(float("nan"),) * 4, entity_confidence=70, entity_class_confidence=60
    )
    radar_from_cloud = provizio_dds.point_cloud2.make_entities_from(
        provizio_dds.point_cloud2.make_header(0, 0, "test_from"),
        "radar",
        [r_ent1, r_ent2],
    )
    assert provizio_dds.point_cloud2.entities_kind(radar_from_cloud) == "radar"
    radar_from_ents = provizio_dds.point_cloud2.read_entities(radar_from_cloud)
    assert len(radar_from_ents) == 2
    assert radar_from_ents[0].entity_id == 42
    assert radar_from_ents[0].entity_class == 3
    assert abs(radar_from_ents[0].x - 1.5) < 1e-5
    assert abs(radar_from_ents[0].orientation[3] - 0.4) < 1e-5
    assert radar_from_ents[0].camera_entity_id == provizio_dds.point_cloud2.NO_ENTITY_ID
    assert math.isnan(radar_from_ents[0].camera_bbox[0])
    assert radar_from_ents[1].entity_id == 55


def test_make_entities_from_camera():
    c_ent1 = types.SimpleNamespace(
        entity_id=provizio_dds.point_cloud2.NO_ENTITY_ID, camera_entity_id=7,
        entity_class=1, x=1.0, y=2.0, z=3.0,
        radar_relative_radial_velocity=float("nan"), ground_relative_radial_velocity=float("nan"),
        orientation=(float("nan"),) * 4, size=(float("nan"),) * 3,
        camera_bbox=(10.0, 20.0, 30.0, 40.0), entity_confidence=95, entity_class_confidence=88
    )
    camera_from_cloud = provizio_dds.point_cloud2.make_entities_from(
        provizio_dds.point_cloud2.make_header(0, 0, "test_from"),
        "camera",
        [c_ent1],
    )
    assert provizio_dds.point_cloud2.entities_kind(camera_from_cloud) == "camera"
    camera_from_ents = provizio_dds.point_cloud2.read_entities(camera_from_cloud)
    assert len(camera_from_ents) == 1
    assert camera_from_ents[0].camera_entity_id == 7
    assert abs(camera_from_ents[0].camera_bbox[3] - 40.0) < 1e-5
    assert camera_from_ents[0].entity_id == provizio_dds.point_cloud2.NO_ENTITY_ID
    assert math.isnan(camera_from_ents[0].radar_relative_radial_velocity)


def test_make_entities_from_fused():
    f_ent1 = types.SimpleNamespace(
        entity_id=42, camera_entity_id=7,
        entity_class=2, x=1.0, y=2.0, z=3.0,
        radar_relative_radial_velocity=4.0, ground_relative_radial_velocity=5.0,
        orientation=(0.1, 0.2, 0.3, 0.4), size=(5.0, 6.0, 7.0),
        camera_bbox=(11.0, 22.0, 33.0, 44.0), entity_confidence=70, entity_class_confidence=60
    )
    fused_from_cloud = provizio_dds.point_cloud2.make_entities_from(
        provizio_dds.point_cloud2.make_header(0, 0, "test_from"),
        "fused",
        [f_ent1],
    )
    assert provizio_dds.point_cloud2.entities_kind(fused_from_cloud) == "fused"
    fused_from_ents = provizio_dds.point_cloud2.read_entities(fused_from_cloud)
    assert len(fused_from_ents) == 1
    assert fused_from_ents[0].entity_id == 42
    assert fused_from_ents[0].camera_entity_id == 7
    assert abs(fused_from_ents[0].camera_bbox[0] - 11.0) < 1e-5
    assert abs(fused_from_ents[0].orientation[3] - 0.4) < 1e-5


def test_make_entities_from_bad_kind():
    try:
        provizio_dds.point_cloud2.make_entities_from(
            provizio_dds.point_cloud2.make_header(0, 0, "x"), "bad_kind", []
        )
        assert False, "Expected ValueError for bad kind"
    except ValueError:
        pass


# ---------------------------------------------------------------------------
# Per-entity kind / has_* member checks incl. neither-id → None
# ---------------------------------------------------------------------------

def test_entity_per_entity_kind_methods():
    """Per-entity has_radar_data / has_camera_data / kind methods for radar, camera, fused, and default."""
    header = provizio_dds.point_cloud2.make_header(0, 0, "test")

    # Radar entity
    radar_cloud = provizio_dds.point_cloud2.make_radar_entities(
        header,
        [[42, 3, 1.5, -2.5, 0.5, 10.5, 9.5, 0.1, 0.2, 0.3, 0.4, 4.5, 1.8, 1.5, 90, 80]],
    )
    radar_ents = provizio_dds.point_cloud2.read_entities(radar_cloud)
    ent = radar_ents[0]
    assert ent.has_radar_data(), "radar entity: has_radar_data must be True"
    assert not ent.has_camera_data(), "radar entity: has_camera_data must be False"
    assert ent.kind() == "radar", f"radar entity: kind must be 'radar', got {ent.kind()!r}"

    # Camera entity
    camera_cloud = provizio_dds.point_cloud2.make_camera_entities(
        header, [[7, 1, 1.0, 2.0, 3.0, 10, 20, 30, 40, 95, 85]]
    )
    camera_ents = provizio_dds.point_cloud2.read_entities(camera_cloud)
    ent = camera_ents[0]
    assert not ent.has_radar_data(), "camera entity: has_radar_data must be False"
    assert ent.has_camera_data(), "camera entity: has_camera_data must be True"
    assert ent.kind() == "camera", f"camera entity: kind must be 'camera', got {ent.kind()!r}"

    # Fused entity
    fused_cloud = provizio_dds.point_cloud2.make_fused_entities(
        header,
        [[42, 7, 2, 1, 2, 3, 4, 5, 0.1, 0.2, 0.3, 0.4, 5, 6, 7, 11, 22, 33, 44, 70, 60]],
    )
    fused_ents = provizio_dds.point_cloud2.read_entities(fused_cloud)
    ent = fused_ents[0]
    assert ent.has_radar_data(), "fused entity: has_radar_data must be True"
    assert ent.has_camera_data(), "fused entity: has_camera_data must be True"
    assert ent.kind() == "fused", f"fused entity: kind must be 'fused', got {ent.kind()!r}"

    # Entity carrying neither id (both set to NO_ENTITY_ID)
    NO_ENTITY_ID = provizio_dds.point_cloud2.NO_ENTITY_ID
    Entity = provizio_dds.point_cloud2.Entity
    no_id_ent = Entity(
        entity_id=NO_ENTITY_ID,
        camera_entity_id=NO_ENTITY_ID,
        entity_class=0,
        x=float("nan"),
        y=float("nan"),
        z=float("nan"),
        radar_relative_radial_velocity=float("nan"),
        ground_relative_radial_velocity=float("nan"),
        orientation=(float("nan"),) * 4,
        size=(float("nan"),) * 3,
        camera_bbox=(float("nan"),) * 4,
        entity_confidence=0,
        entity_class_confidence=0,
    )
    assert not no_id_ent.has_radar_data(), "no-id entity: has_radar_data must be False"
    assert not no_id_ent.has_camera_data(), "no-id entity: has_camera_data must be False"
    assert no_id_ent.kind() is None, f"no-id entity: kind must be None, got {no_id_ent.kind()!r}"


# ---------------------------------------------------------------------------
# Run all tests
# ---------------------------------------------------------------------------

test_radar_point_cloud_creation()
test_radar_entities_creation()
test_read_points_list_camera_entities()
test_read_points_list_empty()
test_no_entity_id_value()
test_entities_kind_radar()
test_entities_kind_camera()
test_entities_kind_fused()
test_entities_kind_none_for_radar_points()
test_read_entities_radar_roundtrip()
test_read_entities_camera_roundtrip()
test_read_entities_fused_roundtrip()
test_read_entities_absent_ids()
test_make_entities_from_radar()
test_make_entities_from_camera()
test_make_entities_from_fused()
test_make_entities_from_bad_kind()
test_entity_per_entity_kind_methods()

print("point_cloud2_test.py PASSED")
