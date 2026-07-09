# Copyright 2008 Willow Garage, Inc.
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
#
# It's a modified version of
# https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs_py/sensor_msgs_py/point_cloud2.py
# which is also licensed under Apache Licence 2.0:
# https://github.com/ros2/common_interfaces/blob/humble/LICENSE
# and includes the following copyright notice:
#
# Copyright 2008 Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Serialization of provizio_dds.PointCloud2 messages.

import array
import ctypes
import math
import sys
from collections import namedtuple
from typing import Iterable, List, NamedTuple, Optional, Sequence
import numpy as np
from numpy.lib.recfunctions import (
    structured_to_unstructured,
    unstructured_to_structured,
)
from provizio_dds_python_types import *


_DATATYPES = {}
_DATATYPES[INT8] = np.dtype(np.int8)
_DATATYPES[UINT8] = np.dtype(np.uint8)
_DATATYPES[INT16] = np.dtype(np.int16)
_DATATYPES[UINT16] = np.dtype(np.uint16)
_DATATYPES[INT32] = np.dtype(np.int32)
_DATATYPES[UINT32] = np.dtype(np.uint32)
_DATATYPES[FLOAT32] = np.dtype(np.float32)
_DATATYPES[FLOAT64] = np.dtype(np.float64)

_DATATYPES_SIZES = {}
_DATATYPES_SIZES[INT8] = 1
_DATATYPES_SIZES[UINT8] = 1
_DATATYPES_SIZES[INT16] = 2
_DATATYPES_SIZES[UINT16] = 2
_DATATYPES_SIZES[INT32] = 4
_DATATYPES_SIZES[UINT32] = 4
_DATATYPES_SIZES[FLOAT32] = 4
_DATATYPES_SIZES[FLOAT64] = 8


DUMMY_FIELD_PREFIX = "unnamed_field"

NO_ENTITY_ID = 0xFFFFFFFF
"""The value of entity_id/camera_entity_id meaning the id is not present in the source cloud (see entities_kind)."""


class Entity(NamedTuple):
    """A single Provizio entity of any kind (radar / camera / fused), as read by read_entities. Fields absent from
    the source cloud read as NaN (float and multi-value fields such as orientation / size / camera_bbox),
    NO_ENTITY_ID (entity_id / camera_entity_id), or 0 (the integral entity_class and confidence fields).
    orientation is a quaternion in (x, y, z, w) order — the PointCloud2 wire contract (note: Python accumulation
    uses (w, x, y, z))."""

    entity_id: int
    camera_entity_id: int
    entity_class: int
    x: float
    y: float
    z: float
    radar_relative_radial_velocity: float
    ground_relative_radial_velocity: float
    orientation: tuple
    size: tuple
    camera_bbox: tuple
    entity_confidence: int
    entity_class_confidence: int

    def has_radar_data(self) -> bool:
        """Returns True when this entity carries radar-sourced data (its entity_id is present)."""
        return self.entity_id != NO_ENTITY_ID

    def has_camera_data(self) -> bool:
        """Returns True when this entity carries camera-sourced data (its camera_entity_id is present)."""
        return self.camera_entity_id != NO_ENTITY_ID

    def kind(self) -> Optional[str]:
        """Returns the kind of this single entity: "fused"/"radar"/"camera", or None when it carries neither id.
        The whole-cloud counterpart is entities_kind."""
        if self.has_radar_data() and self.has_camera_data():
            return "fused"
        if self.has_radar_data():
            return "radar"
        if self.has_camera_data():
            return "camera"
        return None


def read_points(
    cloud: PointCloud2,
    field_names: Optional[List[str]] = None,
    skip_nans: bool = False,
    uvs: Optional[Iterable] = None,
    reshape_organized_cloud: bool = False,
) -> np.ndarray:
    """
    Read points from a provizio_dds.PointCloud2 message.

    :param cloud: The point cloud to read from provizio_dds.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_cloud: Returns the array as an 2D organized point cloud if set.
    :return: Structured NumPy array containing all points.
    """
    assert isinstance(cloud, PointCloud2), "Cloud is not a provizio_dds.PointCloud2"

    dtype = dtype_from_fields(cloud.fields(), point_step=cloud.point_step())

    if cloud.data() is None or cloud.data().size() == 0:
        return np.empty(0, dtype=dtype)

    # Cast bytes to numpy array
    points = np.ndarray(
        shape=(cloud.width() * cloud.height(),),
        dtype=dtype,
        buffer=bytearray(
            ctypes.cast(int(cloud.data().get_buffer()), ctypes.POINTER(ctypes.c_uint8))[
                : cloud.data().size()
            ]
        ),
    )

    # Keep only the requested fields
    if field_names is not None:
        assert all(
            field_name in points.dtype.names for field_name in field_names
        ), "Requests field is not in the fields of the PointCloud!"
        # Mask fields
        points = points[list(field_names)]

    # Swap array if byte order does not match
    if bool(sys.byteorder != "little") != bool(cloud.is_bigendian()):
        points = points.byteswap(inplace=True)

    # Check if we want to drop points with nan values
    if skip_nans and not cloud.is_dense():
        # Init mask which selects all points
        not_nan_mask = np.ones(len(points), dtype=bool)
        for field_name in points.dtype.names:
            # Only keep points without any non values in the mask
            not_nan_mask = np.logical_and(not_nan_mask, ~np.isnan(points[field_name]))
        # Select these points
        points = points[not_nan_mask]

    # Select points indexed by the uvs field
    if uvs is not None:
        # Don't convert to numpy array if it is already one
        if not isinstance(uvs, np.ndarray):
            uvs = np.fromiter(uvs, int)
        # Index requested points
        points = points[uvs]

    # Cast into 2d array if cloud is 'organized'
    if reshape_organized_cloud and cloud.height() > 1:
        points = points.reshape(cloud.width(), cloud.height())

    return points


def read_points_numpy(
    cloud: PointCloud2,
    field_names: Optional[List[str]] = None,
    skip_nans: bool = False,
    uvs: Optional[Iterable] = None,
    reshape_organized_cloud: bool = False,
) -> np.ndarray:
    """
    Read equally typed fields from provizio_dds.PointCloud2 message as a unstructured numpy array.

    This method is better suited if one wants to perform math operations
    on e.g. all x,y,z fields.
    But it is limited to fields with the same dtype as unstructured numpy arrays
    only contain one dtype.

    :param cloud: The point cloud to read from provizio_dds.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_cloud: Returns the array as an 2D organized point cloud if set.
    :return: Numpy array containing all points.
    """
    assert all(
        cloud.fields()[0].datatype() == field.datatype() for field in cloud.fields()[1:]
    ), "All fields need to have the same datatype. Use `read_points()` otherwise."
    structured_numpy_array = read_points(
        cloud, field_names, skip_nans, uvs, reshape_organized_cloud
    )
    return structured_to_unstructured(structured_numpy_array)


def read_points_list(
    cloud: PointCloud2,
    field_names: Optional[List[str]] = None,
    skip_nans: bool = False,
    uvs: Optional[Iterable] = None,
    tuple_name: str = "Point",
) -> List[NamedTuple]:
    """
    Read points from a provizio_dds.PointCloud2 message.

    This function returns a list of namedtuples. It operates on top of the
    read_points method. For more efficient access use read_points directly.

    :param cloud: The point cloud to read from. (Type: provizio_dds.PointCloud2)
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
                coordinates. (Type: Iterable, Default: None]
    :return: List of namedtuples containing the values for each point
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a provizio_dds.PointCloud2"

    if field_names is None:
        field_names = dtype_from_fields(
            cloud.fields(), point_step=cloud.point_step()
        ).names

    Point = namedtuple(tuple_name, field_names)

    return [Point._make(p) for p in read_points(cloud, field_names, skip_nans, uvs)]


def dtype_from_fields(fields: Sequence, point_step: Optional[int] = None) -> np.dtype:
    """
    Convert a Iterable of provizio_dds.PointField messages to a np.dtype.

    :param fields: The point cloud fields. (Type: Sequence of provizio_dds.PointField)
    :param point_step: Point step size in bytes. Calculated from the given fields by default.
                       (Type: optional of integer)
    :returns: NumPy datatype
    """
    # Create a lists containing the names, offsets and datatypes of all fields
    field_names = []
    field_offsets = []
    field_datatypes = []
    for i in range(len(fields)):
        # Datatype as numpy datatype
        datatype = _DATATYPES[fields[i].datatype()]
        # Name field
        if fields[i].name() == "":
            name = f"{DUMMY_FIELD_PREFIX}_{i}"
        else:
            name = fields[i].name()
        # Handle fields with count > 1 by creating subfields with a suffix consiting
        # of "_" followed by the subfield counter [0 -> (count - 1)]
        field_count = fields[i].count()
        assert field_count > 0, "Can't process fields with count = 0."
        for a in range(field_count):
            # Add suffix if we have multiple subfields
            if field_count > 1:
                subfield_name = f"{name}_{a}"
            else:
                subfield_name = name
            assert (
                subfield_name not in field_names
            ), "Duplicate field names are not allowed!"
            field_names.append(subfield_name)
            # Create new offset that includes subfields
            field_offsets.append(
                fields[i].offset() + a * _DATATYPES_SIZES[fields[i].datatype()]
            )
            field_datatypes.append(datatype.str)

    # Create dtype
    dtype_dict = {
        "names": field_names,
        "formats": field_datatypes,
        "offsets": field_offsets,
    }
    if point_step is not None:
        dtype_dict["itemsize"] = point_step
    return np.dtype(dtype_dict)


def create_cloud(
    header: Header, fields: Sequence, points: Iterable, is_dense: bool = True
) -> PointCloud2:
    """
    Create a provizio_dds.PointCloud2 message.

    :param header: The point cloud header. (Type: provizio_dds.Header)
    :param fields: The point cloud fields. (Type: Sequence of provizio_dds.PointField)
    :param points: The point cloud points. List of iterables, i.e. one iterable
                   for each point, with the elements of each iterable being the
                   values of the fields for that point (in the same order as
                   the fields parameter)
    :param is_dense: True if there are no invalid points
    :return: The point cloud as provizio_dds.PointCloud2
    """
    # Check if input is numpy array
    if isinstance(points, np.ndarray):
        # Check if this is an unstructured array
        if points.dtype.names is None:
            assert all(
                fields[0].datatype == field.datatype for field in fields[1:]
            ), "All fields need to have the same datatype. Pass a structured NumPy array \
                    with multiple dtypes otherwise."
            # Convert unstructured to structured array
            points = unstructured_to_structured(points, dtype=dtype_from_fields(fields))
        else:
            assert points.dtype == dtype_from_fields(
                fields
            ), "PointFields and structured NumPy array dtype do not match for all fields! \
                    Check their field order, names and types."
    else:
        # Cast python objects to structured NumPy array (slow)
        points = np.array(
            # Points need to be tuples in the structured array
            list(map(tuple, points)),
            dtype=dtype_from_fields(fields),
        )

    # Handle organized clouds
    assert (
        len(points.shape) <= 2
    ), "Too many dimensions for organized cloud! \
            Points can only be organized in max. two dimensional space"
    height = 1
    width = points.shape[0]
    # Check if input points are an organized cloud (2D array of points)
    if len(points.shape) == 2:
        height = points.shape[1]

    # Convert numpy points to array.array
    memory_view = memoryview(points)
    casted = memory_view.cast("B")
    array_array = array.array("B")
    array_array.frombytes(casted)

    # Put everything together
    cloud = PointCloud2()
    cloud.header(header)
    cloud.height(height)
    cloud.width(width)
    cloud.is_dense(is_dense)
    cloud.is_bigendian(sys.byteorder != "little")
    cloud.fields(fields)
    cloud.point_step(points.dtype.itemsize)
    cloud.row_step(points.dtype.itemsize * width)
    cloud.data(array_array)

    return cloud


def make_radar_point_cloud(
    header: Header, points: Iterable, is_dense: bool = True
) -> PointCloud2:
    """
    Create a provizio_dds.PointCloud2 message with
    (x, y, z, radar_relative_radial_velocity, signal_to_noise_ratio, ground_relative_radial_velocity) fields.

    :param header: The point cloud header. (Type: provizio_dds.Header)
    :param points: The point cloud points. (Type: Iterable)
    :param is_dense: True if there are no invalid points
    :return: The point cloud as provizio_dds.PointCloud2.
    """
    fields = [None] * 6
    for i in range(6):
        fields[i] = PointField()
        fields[i].offset(i * 4)
        fields[i].count(1)
        fields[i].datatype(FLOAT32)
    fields[0].name("x")
    fields[1].name("y")
    fields[2].name("z")
    fields[3].name("radar_relative_radial_velocity")
    fields[4].name("signal_to_noise_ratio")
    fields[5].name("ground_relative_radial_velocity")

    return create_cloud(header, fields, points, is_dense)


def make_entities(
    header: Header, has_radar_data: bool, has_camera_data: bool, entities: Iterable
) -> PointCloud2:
    """
    Create a PointCloud2 containing entities (radar, camera or fused).

    :param header: The point cloud header. (Type: Header)
    :param entities: The entities. List of iterables, i.e. one iterable
                   for entity, with the elements of each iterable being the
                   values of the fields for that entity.
    :return: The point cloud containing entities as PointCloud2.
    """
    num_fields = 6
    if has_radar_data:
        num_fields += 5
    if has_camera_data:
        num_fields += 2

    fields = [None] * num_fields
    index = 0
    offset = 0

    # entity_id
    if has_radar_data:
        fields[index] = PointField()
        fields[index].offset(offset)
        fields[index].count(1)
        fields[index].datatype(UINT32)
        fields[index].name("entity_id")
        index += 1
        offset += 4

    # camera_entity_id
    if has_camera_data:
        fields[index] = PointField()
        fields[index].offset(offset)
        fields[index].count(1)
        fields[index].datatype(UINT32)
        fields[index].name("camera_entity_id")
        index += 1
        offset += 4

    # entity_class
    fields[index] = PointField()
    fields[index].offset(offset)
    fields[index].count(1)
    fields[index].datatype(UINT8)
    fields[index].name("entity_class")
    index += 1
    offset += 1

    # x
    fields[index] = PointField()
    fields[index].offset(offset)
    fields[index].count(1)
    fields[index].datatype(FLOAT32)
    fields[index].name("x")
    index += 1
    offset += 4

    # y
    fields[index] = PointField()
    fields[index].offset(offset)
    fields[index].count(1)
    fields[index].datatype(FLOAT32)
    fields[index].name("y")
    index += 1
    offset += 4

    # z
    fields[index] = PointField()
    fields[index].offset(offset)
    fields[index].count(1)
    fields[index].datatype(FLOAT32)
    fields[index].name("z")
    index += 1
    offset += 4

    # radar_relative_radial_velocity
    if has_radar_data:
        fields[index] = PointField()
        fields[index].offset(offset)
        fields[index].count(1)
        fields[index].datatype(FLOAT32)
        fields[index].name("radar_relative_radial_velocity")
        index += 1
        offset += 4

    # ground_relative_radial_velocity
    if has_radar_data:
        fields[index] = PointField()
        fields[index].offset(offset)
        fields[index].count(1)
        fields[index].datatype(FLOAT32)
        fields[index].name("ground_relative_radial_velocity")
        index += 1
        offset += 4

    # orientation (x, y, z, w)
    if has_radar_data:
        fields[index] = PointField()
        fields[index].offset(offset)
        fields[index].count(4)
        fields[index].datatype(FLOAT32)
        fields[index].name("orientation")
        index += 1
        offset += 4 * 4

    # size (x, y, z)
    if has_radar_data:
        fields[index] = PointField()
        fields[index].offset(offset)
        fields[index].count(3)
        fields[index].datatype(FLOAT32)
        fields[index].name("size")
        index += 1
        offset += 3 * 4

    # camera_bbox (left, top, right, bottom)
    if has_camera_data:
        fields[index] = PointField()
        fields[index].offset(offset)
        fields[index].count(4)
        fields[index].datatype(FLOAT32)
        fields[index].name("camera_bbox")
        index += 1
        offset += 4 * 4

    # entity_confidence
    fields[index] = PointField()
    fields[index].offset(offset)
    fields[index].count(1)
    fields[index].datatype(UINT8)
    fields[index].name("entity_confidence")
    index += 1
    offset += 1

    # entity_class_confidence
    fields[index] = PointField()
    fields[index].offset(offset)
    fields[index].count(1)
    fields[index].datatype(UINT8)
    fields[index].name("entity_class_confidence")
    index += 1
    offset += 1

    return create_cloud(header, fields, entities)


def make_radar_entities(header: Header, entities: Iterable) -> PointCloud2:
    """
    Create a PointCloud2 containing entities
    (entity_id, entity_class, x, y, z, radar_relative_radial_velocity, ground_relative_radial_velocity, orientation(x, y, z, w), size(x, y, z), entity_confidence, entity_class_confidence) fields.

    :param header: The point cloud header. (Type: Header)
    :param entities: The entities. List of iterables, i.e. one iterable
                   for entity, with the elements of each iterable being the
                   values of the fields for that entity.
    :return: The point cloud containing entities as PointCloud2.
    """
    return make_entities(header, True, False, entities)


def make_camera_entities(header: Header, entities: Iterable) -> PointCloud2:
    """
    Create a PointCloud2 containing entities
    (camera_entity_id, entity_class, x, y, z, camera_bbox(left, top, right, bottom), entity_confidence, entity_class_confidence) fields.

    :param header: The point cloud header. (Type: Header)
    :param entities: The entities. List of iterables, i.e. one iterable
                   for entity, with the elements of each iterable being the
                   values of the fields for that entity.
    :return: The point cloud containing entities as PointCloud2.
    """
    return make_entities(header, False, True, entities)


def make_fused_entities(header: Header, entities: Iterable) -> PointCloud2:
    """
    Create a PointCloud2 containing entities
    (entity_id, camera_entity_id, entity_class, x, y, z, radar_relative_radial_velocity, ground_relative_radial_velocity, orientation(x, y, z, w), size(x, y, z), camera_bbox(left, top, right, bottom), entity_confidence, entity_class_confidence) fields.

    :param header: The point cloud header. (Type: Header)
    :param entities: The entities. List of iterables, i.e. one iterable
                   for entity, with the elements of each iterable being the
                   values of the fields for that entity.
    :return: The point cloud containing entities as PointCloud2.
    """
    return make_entities(header, True, True, entities)


def entities_kind(cloud: PointCloud2) -> Optional[str]:
    """
    Detects the kind of a Provizio entities cloud by which entity id fields it carries.

    :param cloud: The cloud to probe. (Type: provizio_dds.PointCloud2)
    :return: "fused" when both entity_id and camera_entity_id fields are present, "radar" or "camera" when only the
             respective id field is, None when neither is (not a Provizio entities cloud).
    """
    field_names = {f.name() for f in cloud.fields()}
    has_entity_id = "entity_id" in field_names
    has_camera_entity_id = "camera_entity_id" in field_names
    if has_entity_id and has_camera_entity_id:
        return "fused"
    if has_entity_id:
        return "radar"
    if has_camera_entity_id:
        return "camera"
    return None


def read_entities(cloud: PointCloud2, skip_nans: bool = False) -> List[Entity]:
    """
    Reads all entities of a Provizio entities PointCloud2 (radar, camera or fused - see entities_kind) into Entity
    instances with a unified field set: entity_id, camera_entity_id, entity_class, x, y, z,
    radar_relative_radial_velocity, ground_relative_radial_velocity, orientation (4-tuple), size (3-tuple),
    camera_bbox (4-tuple), entity_confidence, entity_class_confidence. Fields absent from the cloud read as NaN
    (float / multi-value fields as tuples of NaN), NO_ENTITY_ID (the id fields), or 0 (the integral entity_class
    and confidence fields) - matching the C++ read_entities defaults.

    :param cloud: The entities cloud to read. (Type: provizio_dds.PointCloud2)
    :param skip_nans: If True, then don't return any entity with a NaN value in a present field. (Type: bool,
                      Default: False)
                      Note: skip_nans only has effect when the cloud declares is_dense=False (read_points
                      semantics); this library sets is_dense=True by default.
    :return: List of Entity instances, one per entity.
    """
    arr = read_points(cloud, skip_nans=skip_nans)
    names = set(arr.dtype.names) if arr.dtype.names else set()

    _nan = math.nan

    # Pre-resolve suffixed sub-field name lists for multi-component fields once, outside the row loop.
    _orientation_names = [f"orientation_{i}" for i in range(4)]
    _size_names = [f"size_{i}" for i in range(3)]
    _camera_bbox_names = [f"camera_bbox_{i}" for i in range(4)]
    _has_orientation = all(n in names for n in _orientation_names)
    _has_size = all(n in names for n in _size_names)
    _has_camera_bbox = all(n in names for n in _camera_bbox_names)

    def _multi(row, base_names, has_all):
        if has_all:
            return tuple(float(row[n]) for n in base_names)
        return tuple(_nan for _ in base_names)

    result = []
    for row in arr:

        result.append(Entity(
            entity_id=int(row["entity_id"]) if "entity_id" in names else NO_ENTITY_ID,
            camera_entity_id=int(row["camera_entity_id"]) if "camera_entity_id" in names else NO_ENTITY_ID,
            entity_class=int(row["entity_class"]) if "entity_class" in names else 0,
            x=float(row["x"]) if "x" in names else _nan,
            y=float(row["y"]) if "y" in names else _nan,
            z=float(row["z"]) if "z" in names else _nan,
            radar_relative_radial_velocity=float(row["radar_relative_radial_velocity"])
            if "radar_relative_radial_velocity" in names
            else _nan,
            ground_relative_radial_velocity=float(row["ground_relative_radial_velocity"])
            if "ground_relative_radial_velocity" in names
            else _nan,
            orientation=_multi(row, _orientation_names, _has_orientation),
            size=_multi(row, _size_names, _has_size),
            camera_bbox=_multi(row, _camera_bbox_names, _has_camera_bbox),
            entity_confidence=int(row["entity_confidence"]) if "entity_confidence" in names else 0,
            entity_class_confidence=int(row["entity_class_confidence"])
            if "entity_class_confidence" in names
            else 0,
        ))
    return result


def make_entities_from(header: Header, kind: str, entities: Iterable) -> PointCloud2:
    """
    Creates a Provizio entities PointCloud2 of the given kind ("radar", "camera" or "fused" - the same values
    entities_kind returns) from any iterable of entity-like objects exposing the unified attributes returned by
    read_entities (entity_id, camera_entity_id, entity_class, x, y, z, radar_relative_radial_velocity,
    ground_relative_radial_velocity, orientation, size, camera_bbox, entity_confidence, entity_class_confidence).
    Only the field groups of the requested kind are written (e.g. camera_bbox of an entity is ignored when kind is
    "radar"). read_entities reads such a cloud back losslessly for the written groups.

    :param header: The point cloud header. (Type: Header)
    :param kind: The kind of the entities cloud to create: "radar", "camera" or "fused".
    :param entities: Any iterable of entity-like objects with the unified attributes.
    :return: The entities PointCloud2.
    :raises ValueError: When kind is not "radar", "camera" or "fused".
    """
    if kind not in ("radar", "camera", "fused"):
        raise ValueError(f"kind must be 'radar', 'camera' or 'fused', got {kind!r}")

    has_radar = kind != "camera"
    has_camera = kind != "radar"

    rows = []
    for ent in entities:
        row = []
        if has_radar:
            row.append(ent.entity_id)
        if has_camera:
            row.append(ent.camera_entity_id)
        row.append(ent.entity_class)
        row.append(ent.x)
        row.append(ent.y)
        row.append(ent.z)
        if has_radar:
            row.append(ent.radar_relative_radial_velocity)
            row.append(ent.ground_relative_radial_velocity)
            # orientation: 4 scalars
            if len(ent.orientation) != 4:
                raise ValueError(f"orientation must have 4 elements, got {len(ent.orientation)}")
            row.extend(ent.orientation)
            # size: 3 scalars
            if len(ent.size) != 3:
                raise ValueError(f"size must have 3 elements, got {len(ent.size)}")
            row.extend(ent.size)
        if has_camera:
            # camera_bbox: 4 scalars
            if len(ent.camera_bbox) != 4:
                raise ValueError(f"camera_bbox must have 4 elements, got {len(ent.camera_bbox)}")
            row.extend(ent.camera_bbox)
        row.append(ent.entity_confidence)
        row.append(ent.entity_class_confidence)
        rows.append(row)

    return make_entities(header, has_radar, has_camera, rows)


def make_header(timestamp_sec: int, timestamp_nanosec: int, frame_id: str) -> Header:
    """
    Create a provizio_dds.Header

    :param timestamp_sec: Seconds component of timestamp. (Type: int)
    :param timestamp_nanosec: Nanoseconds component of timestamp, valid in the range [0, 999999999]. (Type: int)
    :param frame_id: Frame this data is associated with
    """
    header = Header()
    header.stamp().sec(timestamp_sec)
    header.stamp().nanosec(timestamp_nanosec)
    header.frame_id(frame_id)
    return header
