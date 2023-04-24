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
from collections import namedtuple
import sys
from typing import Iterable, List, NamedTuple, Optional
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
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


DUMMY_FIELD_PREFIX = 'unnamed_field'


def read_points(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_cloud: bool = False) -> np.ndarray:
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
    assert isinstance(cloud, PointCloud2), \
        'Cloud is not a provizio_dds.PointCloud2'

    # Cast bytes to numpy array
    points = np.ndarray(
        shape=(cloud.width() * cloud.height(), ),
        dtype=dtype_from_fields(cloud.fields(), point_step=cloud.point_step()),
        buffer=bytearray(cloud.data()))

    # Keep only the requested fields
    if field_names is not None:
        assert all(field_name in points.dtype.names for field_name in field_names), \
            'Requests field is not in the fields of the PointCloud!'
        # Mask fields
        points = points[list(field_names)]

    # Swap array if byte order does not match
    if bool(sys.byteorder != 'little') != bool(cloud.is_bigendian()):
        points = points.byteswap(inplace=True)

    # Check if we want to drop points with nan values
    if skip_nans and not cloud.is_dense():
        # Init mask which selects all points
        not_nan_mask = np.ones(len(points), dtype=bool)
        for field_name in points.dtype.names:
            # Only keep points without any non values in the mask
            not_nan_mask = np.logical_and(
                not_nan_mask, ~np.isnan(points[field_name]))
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
        reshape_organized_cloud: bool = False) -> np.ndarray:
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
    assert all(cloud.fields()[0].datatype() == field.datatype() for field in cloud.fields()[1:]), \
        'All fields need to have the same datatype. Use `read_points()` otherwise.'
    structured_numpy_array = read_points(
        cloud, field_names, skip_nans, uvs, reshape_organized_cloud)
    return structured_to_unstructured(structured_numpy_array)


def read_points_list(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None) -> List[NamedTuple]:
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
    assert isinstance(cloud, PointCloud2), \
        'cloud is not a provizio_dds.PointCloud2'
    
    if field_names is None:
        fields = cloud.fields()
        field_names = [fields[i].name() for i in range(len(fields))]

    Point = namedtuple('Point', field_names)

    return [Point._make(p) for p in read_points(cloud, field_names,
                                                skip_nans, uvs)]


def dtype_from_fields(fields: Iterable[PointField], point_step: Optional[int] = None) -> np.dtype:
    """
    Convert a Iterable of provizio_dds.PointField messages to a np.dtype.

    :param fields: The point cloud fields.
                   (Type: iterable of provizio_dds.PointField)
    :param point_step: Point step size in bytes. Calculated from the given fields by default.
                       (Type: optional of integer)
    :returns: NumPy datatype
    """
    # Create a lists containing the names, offsets and datatypes of all fields
    field_names = []
    field_offsets = []
    field_datatypes = []
    for i, field in enumerate(fields):
        # Datatype as numpy datatype
        datatype = _DATATYPES[field.datatype()]
        # Name field
        if field.name == '':
            name = f'{DUMMY_FIELD_PREFIX}_{i}'
        else:
            name = field.name()
        # Handle fields with count > 1 by creating subfields with a suffix consiting
        # of "_" followed by the subfield counter [0 -> (count - 1)]
        field_count = field.count()
        assert field_count > 0, "Can't process fields with count = 0."
        for a in range(field_count):
            # Add suffix if we have multiple subfields
            if field_count > 1:
                subfield_name = f'{name}_{a}'
            else:
                subfield_name = name
            assert subfield_name not in field_names, 'Duplicate field names are not allowed!'
            field_names.append(subfield_name)
            # Create new offset that includes subfields
            field_offsets.append(field.offset() + a *
                                 _DATATYPES_SIZES[field.datatype()])
            field_datatypes.append(datatype.str)

    # Create dtype
    dtype_dict = {
        'names': field_names,
        'formats': field_datatypes,
        'offsets': field_offsets
    }
    if point_step is not None:
        dtype_dict['itemsize'] = point_step
    return np.dtype(dtype_dict)


def create_cloud(
        header: Header,
        fields: Iterable[PointField],
        points: Iterable,
        is_dense: bool = True) -> PointCloud2:
    """
    Create a provizio_dds.PointCloud2 message.

    :param header: The point cloud header. (Type: provizio_dds.Header)
    :param fields: The point cloud fields.
                   (Type: iterable of provizio_dds.PointField)
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
            assert all(fields[0].datatype == field.datatype for field in fields[1:]), \
                'All fields need to have the same datatype. Pass a structured NumPy array \
                    with multiple dtypes otherwise.'
            # Convert unstructured to structured array
            points = unstructured_to_structured(
                points,
                dtype=dtype_from_fields(fields))
        else:
            assert points.dtype == dtype_from_fields(fields), \
                'PointFields and structured NumPy array dtype do not match for all fields! \
                    Check their field order, names and types.'
    else:
        # Cast python objects to structured NumPy array (slow)
        points = np.array(
            # Points need to be tuples in the structured array
            list(map(tuple, points)),
            dtype=dtype_from_fields(fields))

    # Handle organized clouds
    assert len(points.shape) <= 2, \
        'Too many dimensions for organized cloud! \
            Points can only be organized in max. two dimensional space'
    height = 1
    width = points.shape[0]
    # Check if input points are an organized cloud (2D array of points)
    if len(points.shape) == 2:
        height = points.shape[1]

    # Convert numpy points to array.array
    memory_view = memoryview(points)
    casted = memory_view.cast('B')
    array_array = array.array('B')
    array_array.frombytes(casted)

    # Put everything together
    cloud = PointCloud2()
    cloud.header(header)
    cloud.height(height)
    cloud.width(width)
    cloud.is_dense(is_dense)
    cloud.is_bigendian(sys.byteorder != 'little')
    cloud.fields(fields)
    cloud.point_step(points.dtype.itemsize)
    cloud.row_step(points.dtype.itemsize * width)
    cloud.data(array_array)

    return cloud


def make_radar_point_cloud(header: Header, points: Iterable, is_dense: bool = True) -> PointCloud2:
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

    return create_cloud(header, fields, points)


def make_header(timestamp_sec, timestamp_nanosec, frame_id: str) -> Header:
    """
    Create a provizio_dds.Header

    :param timestamp_sec: Seconds component of timestamp
    :param timestamp_nanosec: Nanoseconds component of timestamp, valid in the range [0, 10e9)
    :param frame_id: Frame this data is associated with
    """
    header = Header()
    header.stamp().sec(timestamp_sec)
    header.stamp().nanosec(timestamp_nanosec)
    header.frame_id(frame_id)
    return header
