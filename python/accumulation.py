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

"""
Provides Provizio radar point clouds accumulation and multi-radar fusion functionality.
See PointCloudsAccumulator below for more details.
"""

import collections
from provizio_dds_python_types import *
from typing import Callable, List
import numpy as np
from threading import Lock
from transforms3d.euler import quat2euler
from transforms3d._gohlketransforms import (
    compose_matrix,
    identity_matrix,
    inverse_matrix,
    concatenate_matrices,
    translation_from_matrix,
    quaternion_from_matrix,
)
from math import isnan, atan2

if __package__ or "." in __name__:
    from . import point_cloud2
    from . import provizio_dds
    from . import gps_utils
else:
    import point_cloud2
    import provizio_dds
    import gps_utils


def is_point_static(point: [float]) -> bool:
    """Standard accumulation filter that only keeps points with ground relative radial velocities around 0.

    Args:
        point (float]): a point to filter, as a list of float values matching the format used in provizio_dds.point_cloud2.

    Returns:
        bool: True for points detected as static.
    """
    ground_relative_radial_velocity_index = 5
    dynamic_velocity_threshold_m_s = 2

    return (
        abs(point[ground_relative_radial_velocity_index])
        < dynamic_velocity_threshold_m_s
    )


def _make_vector(x, y, z) -> np.array:
    return np.array([x, y, z, 1]).reshape((4, 1))


class RigidTransform:
    """
    Represents a combination of rotation and translation in Euclidean space.
    """

    def __init__(
        self, position: [float] = None, rotation: [float] = None, from_matrix=None
    ) -> None:
        """Constructs a RigidTransform. If no arguments specified, makes an identity transform (no translation, no rotation).

        Args:
            position ([float], optional): When specified, defines the translation as 3-components array (x, y, z) and from_matrix must be None. Defaults to None.
            rotation ([float], optional): When specified, defines the rotation as either 3 euler angles (roll, pitch, yaw) in radians or a 4-component quaternion (w, x, y, z) and from_matrix must be None. Defaults to None.
            from_matrix (optional): When specified, defines the whole 4x4 transformation matrix and both position and rotation must be None. Defaults to None.
        """
        assert (position is None and rotation is None) or (
            position is not None and rotation is not None and from_matrix is None
        ), "Either both position and rotation are specified and not from_matrix, or neither of position or rotation should be specified"
        assert (
            position is None or len(position) == 3
        ), "position must be a 3-component iteratable (x, y, z)"
        assert (
            rotation is None or len(rotation) == 3 or len(rotation) == 4
        ), "rotation must be either a 3-component iteratable Euler angles (roll, pitch, yaw) or a 4-component iteratable quaternion (w, x, y, z)"

        if position is None and from_matrix is None:
            self._inversed_matrix = self._matrix = identity_matrix()
        else:
            self._matrix = (
                from_matrix
                if from_matrix is not None
                else compose_matrix(
                    angles=(rotation if len(rotation) == 3 else quat2euler(rotation)),
                    translate=position,
                )
            )
            self._inversed_matrix = None  # Lazy calculation, only if required

    def matrix(self):
        """Returns the 4x4 transformation matrix combining both translation and rotation.

        Returns:
            A 4x4 transformation matrix.
        """
        return self._matrix

    def inversed_matrix(self):
        """Returns the inversed 4x4 transformation matrix combining both translation and rotation.

        Returns:
            An inversion of self.matrix().
        """
        if self._inversed_matrix is None:
            self._inversed_matrix = inverse_matrix(self._matrix)
        return self._inversed_matrix

    def translation(self):
        """Returns the translation component of the transformation as a 3-component float list (x, y, z).

        Returns:
            The translation component of the transformation as a 3-component float list (x, y, z).
        """
        return translation_from_matrix(self.matrix())

    def rotation(self):
        """Returns the rotation component of the transformation as a 4-component quaternion float list (w, x, y, z).

        Returns:
            The rotation component of the transformation as a 4-component quaternion float list (w, x, y, z).
        """
        return quaternion_from_matrix(self.matrix())


_IDENTITY_TRANSFORM = RigidTransform()


class TransformedPoint:
    """
    Represents a point transformed in a target coordinate space.

    Fields:
        position_np: 4d 1-extended vertical numpy vector.
        position: 3d horizontal array.
        ground_relative_velocity: Radial ground relative velocity, as m/s.
        snr: Signal-to-noise ratio.
        radar_id: Radar position id, f.e. "provizio_radar_front_center".
    """

    def __init__(self, position_np, ground_relative_velocity, snr, radar_id):
        self.position_np = position_np  # 4d 1-extended vertical vector
        self.position = self.position_np[:-1].flatten()  # 3d horizontal array
        self.ground_relative_velocity = ground_relative_velocity
        self.snr = snr
        self.radar_id = radar_id
        pass


class PointCloudsAccumulator:
    """
    Implements the core accumulation functionality, relying on point clouds, localization (if present) and extrinsics (if present) provided by the object customer.
    See also DDSPointCloudsAccumulator.
    """

    _default_snr_threshold: float = 2.5
    _default_max_frames_without_filter: int = 3
    _default_point_filter = is_point_static
    _x_index_in_point = 0
    _y_index_in_point = 1
    _z_index_in_point = 2
    _snr_index_in_point = 4
    _ground_relative_velocity_index_in_point = 5

    def __init__(
        self,
        max_frames_per_radar: int,
        snr_threshold: float = _default_snr_threshold,
        max_frames_without_filter: int = _default_max_frames_without_filter,
        point_filter: Callable[[List[float]], bool] = _default_point_filter,
        radar_filter: Callable[[str], bool] = None,
        allow_no_extrinsics: bool = False,
    ):
        """Constructs a PointCloudsAccumulator.

        Args:
            max_frames_per_radar (int): Max number of radar frames per radar that can be accumulated. On exceeding the number, the oldest frame gets dropped
            snr_threshold (float, optional): Signal to noise ratio threshold. Points with snr below it will be dropped. Defaults to _default_snr_threshold.
            max_frames_without_filter (int, optional): Number of frames every new point cloud remains unfiltered, except the SnR threshold filter. When > 0 it can be used to temporarily accumulate points of moving objects to improve dynamic objects detection. Defaults to _default_max_frames_without_filter.
            point_filter (Callable[[List[float]], bool], optional): A point filter to apply after max_frames_without_filter. Defaults to _default_point_filter, which only accumulates static points.
            radar_filter (Callable[[str], bool], optional): If specified, lets keeping only points from specific radars. Defaults to None, which stands for keeping points from all radars.
            allow_no_extrinsics (bool, optional): Allows accumulation when radar extrinsics are not specified (then assumes the ego coordinate frame to be same as the radar coordinate frame). Defaults to False.
        """
        assert max_frames_per_radar > 0, "max_frames_per_radar can't be <= 0"

        self.max_frames_per_radar = max_frames_per_radar
        self.snr_threshold = snr_threshold
        self.max_frames_without_filter = max_frames_without_filter
        self.point_filter = (
            point_filter if max_frames_without_filter < max_frames_per_radar else None
        )
        self.radar_filter = radar_filter
        self.allow_no_extrinsics = allow_no_extrinsics

        self._buffers = dict()
        self._extrinsics = dict()

    def accumulate(
        self,
        radar_position_id: str,
        points: [[float]],
        ego_localization_when_received: RigidTransform,
        radar_extrinsics: RigidTransform = None,
    ):
        """Accumulates next radar point cloud.

        Args:
            radar_position_id (str): Radar position id the point cloud originates from, such as "provizio_radar_front_center".
            points ([[float]]): A list of radar points to accumulate, each being a list of floats matching the format used in provizio_dds.point_cloud2.
            ego_localization_when_received (RigidTransform): RigidTransform representing the ego position and orientation in some local Euclidean reference frame, usually ENU (see https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates).
            radar_extrinsics (RigidTransform, optional): When provided specifies the radar position and orientation relative to the ego coordinate frame. Defaults to None, which stands for keeping previous extrinsics of that radar if any, or radar frame = ego frame otherwise.

        Raises:
            ValueError: In case no extrinsics were ever provided for this radar while allow_no_extrinsics was specified as False when constructing the PointCloudsAccumulator.
        """
        assert isinstance(ego_localization_when_received, RigidTransform)
        assert radar_extrinsics is None or isinstance(radar_extrinsics, RigidTransform)

        if self.radar_filter and not self.radar_filter(radar_position_id):
            # Simply ignore this radar
            return

        if radar_position_id not in self._buffers:
            self._buffers[radar_position_id] = collections.deque(
                maxlen=self.max_frames_per_radar
            )

        buffer = self._buffers[radar_position_id]

        if self.snr_threshold > 0:
            points = list(
                filter(
                    lambda point: (
                        point[PointCloudsAccumulator._snr_index_in_point]
                        >= self.snr_threshold
                    ),
                    points,
                )
            )

        if self.point_filter:
            if self.max_frames_without_filter <= 0:
                points = list(filter(self.point_filter, points))
            elif len(buffer) >= self.max_frames_without_filter:
                buffer[-self.max_frames_without_filter].points = filter(
                    self.point_filter, buffer[-self.max_frames_without_filter].points
                )

        buffer.append(
            PointCloudsAccumulator._AccumulatedPointCloud(
                points, ego_localization_when_received
            )
        )

        if radar_extrinsics:
            self._extrinsics[radar_position_id] = radar_extrinsics
        elif not self.allow_no_extrinsics and radar_position_id not in self._extrinsics:
            raise ValueError(
                "allow_no_extrinsics is False so radar_extrinsics must be specified!"
            )

    def get_points_local_frame_relative(self) -> [TransformedPoint]:
        """Returns all currently accumulated points with positions relative to the same coordinate frame as localization uses (usually, local ENU).

        Returns:
            [TransformedPoint]: A list of TransformedPoint.
        """
        return_points = []

        for radar_id, accumulated_pcs in self._buffers.items():
            radar_extrinsics_matrix = (
                self._extrinsics[radar_id].matrix()
                if radar_id in self._extrinsics
                else identity_matrix()
            )
            for accumulated_pc in accumulated_pcs:
                to_local_frame_matrix = concatenate_matrices(
                    accumulated_pc.ego_localization_when_received.matrix(),
                    radar_extrinsics_matrix,
                )
                # TODO: try vectorizing the operation maybe to optimize this slow loop?
                for point in accumulated_pc.points:
                    transformed_position = to_local_frame_matrix @ _make_vector(
                        point[PointCloudsAccumulator._x_index_in_point],
                        point[PointCloudsAccumulator._y_index_in_point],
                        point[PointCloudsAccumulator._z_index_in_point],
                    )

                    return_points.append(
                        TransformedPoint(
                            transformed_position,
                            point[
                                PointCloudsAccumulator._ground_relative_velocity_index_in_point
                            ],
                            point[PointCloudsAccumulator._snr_index_in_point],
                            radar_id,
                        )
                    )

        return return_points

    def get_points_ego_relative(
        self, ego_localization_now: RigidTransform
    ) -> [TransformedPoint]:
        """Returns all currently accumulated points with positions relative to ego_localization_now.

        Args:
            ego_localization_now (RigidTransform): A RigidTransform in the same coordinate frame as localization uses (usually, local ENU).

        Returns:
            [TransformedPoint]: A list of TransformedPoint.
        """
        return_points = self.get_points_local_frame_relative()
        to_ego_space_matrix = ego_localization_now.inversed_matrix()
        # TODO: try vectorizing the operation maybe to optimize this slow loop?
        for point in return_points:
            point.position_np = to_ego_space_matrix @ point.position_np
            point.position = point.position_np[:-1].flatten()

        return return_points

    @staticmethod
    def localization_from_sensor_to_ego_frame(
        sensor_localization: RigidTransform, sensor_extrinsics: RigidTransform
    ) -> RigidTransform:
        """Converts a rigid transform of the localization sensor reading (i.e. a position and orientation of localization sensor) to the rigid transform of the ego vehicle (i.e. a position and orientation of the ego)

        Args:
            sensor_localization (RigidTransform): A RigidTransform localization as detected by the localization sensor, i.e. where that sensor is.
            sensor_extrinsics (RigidTransform): Position and orientation of the localization sensor in the ego coordinate frame.

        Returns:
            RigidTransform: A RigidTransform localization of the ego coordinate frame in the same coordinate frame as localization uses (usually, local ENU).
        """
        return RigidTransform(
            from_matrix=concatenate_matrices(
                sensor_localization.matrix(), sensor_extrinsics.inversed_matrix()
            )
        )

    class _AccumulatedPointCloud:
        def __init__(
            self, points, ego_localization_when_received: RigidTransform
        ) -> None:
            assert isinstance(ego_localization_when_received, RigidTransform)

            self.points = points
            self.ego_localization_when_received = ego_localization_when_received
            self._position = None


class DDSPointCloudsAccumulator:
    """
    Implements accumulation functionality with all input data (radar point clouds, localization, extrinsics) received from apppropriate DDS topics.
    """

    def __init__(
        self,
        max_frames_per_radar: int,
        localization_type: type = Odometry,  # Either None, provizio_dds.Odometry or provizio_dds.NavSatFix
        localization_topic: str = "rt/provizio_radar_odometry",  # Use None with localization_type=None for always static use case
        localization_frame_id: str = None,  # Use None to accept any on localization_topic, or with localization_topic=None
        localization_extrinsics_topic: str = None,  # Use None with localization_type=None or to assume localization frame = ego frame
        pointcloud2_topic: str = "rt/provizio_radar_point_cloud",
        extrinsics_topics: [str] = [
            "rt/provizio_extrinsics"
        ],  # If None/empty, radar-relative coordinate frame is assumed instead of vehicle-relative
        snr_threshold: float = PointCloudsAccumulator._default_snr_threshold,
        max_frames_without_filter: int = PointCloudsAccumulator._default_max_frames_without_filter,
        point_filter: Callable[
            [object], bool
        ] = PointCloudsAccumulator._default_point_filter,
        radar_filter: Callable[[str], bool] = None,
        on_point_cloud: Callable[
            [object], None
        ] = None,  # Optional callback to be invoked on receiving and accumulating a new point cloud, takes DDSPointCloudsAccumulator as the only argument
        dds_domain_participant=None,  # DDS Domain participant to reuse if any, otherwise a new one will be created
    ) -> None:
        """Constructs a DDSPointCloudsAccumulator.

        Args:
            max_frames_per_radar (int): Max number of radar frames per radar that can be accumulated. On exceeding the number, the oldest frame gets dropped
            localization_type (type, optional): Type of localization data inputs, either None, provizio_dds.Odometry or provizio_dds.NavSatFix. Defaults to provizio_dds.Odometry.
            localization_topic (str, optional): DDS topic of localization data (message type of localization_type), if any. Defaults to "rt/provizio_radar_odometry".
            localization_frame_id (str, optional): Localization data frame name or None to accept any on localization_topic, or with localization_topic=None. Defaults to None.
            localization_extrinsics_topic (str, optional): DDS Topic of localization extrinsics (with message type of provizio_dds.TransformStamped) or None with localization_type=None or to assume localization frame = ego frame. Defaults to None.
            pointcloud2_topic (str, optional): DDS Topic of radar provizio.PointCloud2 input data. Defaults to "rt/provizio_radar_point_cloud".
            extrinsics_topics ([str], optional): List of DDS Topics of radar (and optionally of localization) extrinsics data (with message type of provizio_dds.TransformStamped). Defaults to ["rt/provizio_extrinsics"].
            snr_threshold (float, optional): Signal to noise ratio threshold. Points with snr below it will be dropped. Defaults to PointCloudsAccumulator._default_snr_threshold.
            max_frames_without_filter (int, optional): Number of frames every new point cloud remains unfiltered, except the SnR threshold filter. When > 0 it can be used to temporarily accumulate points of moving objects to improve dynamic objects detection. Defaults to PointCloudsAccumulator._default_max_frames_without_filter.
            point_filter (Callable[[List[float]], bool], optional): A point filter to apply after max_frames_without_filter. Defaults to PointCloudsAccumulator._default_point_filter, which only accumulates static points.
            radar_filter (Callable[[str], bool], optional): If specified, lets keeping only points from specific radars. Defaults to None, which stands for keeping points from all radars.
            on_point_cloud (Callable[[object], None], optional): A callback, if any, to be invoked on accumulating every radar point cloud. Defaults to None.
            dds_domain_participant (optional): If specified, existing DDS domain participant to use (otherwise, a new one will be automatically created). Defaults to None.
        """
        assert max_frames_per_radar > 0, "max_frames_per_radar can't be <= 0"

        assert (
            localization_type is None
            or localization_type == Odometry
            or localization_type == NavSatFix
        ), "localization_type must be either None, provizio_dds.Odometry or provizio_dds.NavSatFix"

        assert (localization_type is not None and localization_topic) or (
            localization_type is None
            and not localization_topic
            and not localization_frame_id
            and not localization_extrinsics_topic
        ), "If either localization_type or localization_topic is not present, then all related arguments have to be None/empty too"

        self._mutex = Lock()

        self.no_localization = localization_type is None
        if self.no_localization:
            # Always same localization at (0, 0, 0), (0, 0, 0)
            self._latest_ego_localization = _IDENTITY_TRANSFORM
        else:
            self._latest_ego_localization = None

        self._extrinsics = dict()
        self.no_extrinsics = extrinsics_topics is None or len(extrinsics_topics) == 0

        self._accumulator = PointCloudsAccumulator(
            max_frames_per_radar,
            snr_threshold=snr_threshold,
            max_frames_without_filter=max_frames_without_filter,
            point_filter=point_filter,
            radar_filter=radar_filter,
            allow_no_extrinsics=self.no_extrinsics,
        )

        self.on_point_cloud = on_point_cloud

        self._participant = (
            dds_domain_participant
            if dds_domain_participant
            else provizio_dds.make_domain_participant()
        )

        self.localization_extrinsics_topic = localization_extrinsics_topic
        self.localization_frame_id = localization_frame_id
        if self.no_localization:
            self._localization_subscriber = None
        else:
            if localization_type == Odometry:
                self._localization_subscriber = provizio_dds.Subscriber(
                    self._participant,
                    localization_topic,
                    OdometryPubSubType,
                    Odometry,
                    lambda odometry: self._on_odometry(odometry),
                )
            else:
                assert localization_type == NavSatFix
                self._gps_utils = None
                # Used to estimate orientation, which NavSatFix doesn't provide
                self._positions_history = collections.deque(maxlen=3)
                self._last_yaw = 0
                self._localization_subscriber = provizio_dds.Subscriber(
                    self._participant,
                    localization_topic,
                    NavSatFixPubSubType,
                    NavSatFix,
                    lambda nav_sat_fix: self._on_nav_sat_fix(nav_sat_fix),
                )

        if extrinsics_topics is None:
            extrinsics_topics = []
        if (
            self.localization_extrinsics_topic
            and self.localization_extrinsics_topic not in extrinsics_topics
        ):
            extrinsics_topics.append(self.localization_extrinsics_topic)

        self._extrinsics_subscribers = []
        for topic in extrinsics_topics:
            self._extrinsics_subscribers.append(
                provizio_dds.Subscriber(
                    self._participant,
                    topic,
                    TransformStampedPubSubType,
                    TransformStamped,
                    lambda transform_stamped, topic=topic: self._on_extrinsics(
                        topic, transform_stamped
                    ),
                )
            )

        self._pc2_subscriber = provizio_dds.Subscriber(
            self._participant,
            pointcloud2_topic,
            PointCloud2PubSubType,
            PointCloud2,
            lambda point_cloud: self._on_point_cloud(point_cloud),
        )

    def __del__(self):
        with self._mutex:
            del self._pc2_subscriber
            for it in self._extrinsics_subscribers:
                del it
            if self._localization_subscriber is not None:
                del self._localization_subscriber

    def get_points_local_frame_relative(self) -> [TransformedPoint]:
        """Returns all currently accumulated points with positions relative to the same coordinate frame as localization uses (usually, local ENU).

        Returns:
            [TransformedPoint]: A list of TransformedPoint.
        """

        with self._mutex:
            return self._accumulator.get_points_local_frame_relative()

    def get_points_ego_relative(self) -> [TransformedPoint]:
        """Returns all currently accumulated points with positions relative to the current ego position/orientation.

        Returns:
            [TransformedPoint]: A list of TransformedPoint.
        """

        with self._mutex:
            ego_localization = self._get_current_ego_localization()
            if ego_localization is None:
                if self.no_localization:
                    # Always same position & orientation is assumed with no_localization
                    ego_localization = _IDENTITY_TRANSFORM
                else:
                    # Ego localization is required but not received yet
                    # TODO: If no localization for too long, report an error
                    return []
            return self._accumulator.get_points_ego_relative(ego_localization)

    def _on_odometry(self, odometry: Odometry):
        assert not self.no_localization

        with self._mutex:
            extrinsics = self._get_localization_extrinsics(odometry.child_frame_id())
            if extrinsics is None:
                # Extrinsics not received yet
                return

            pose = odometry.pose().pose()
            position = pose.position()
            orientation = pose.orientation()

            self._latest_ego_localization = (
                PointCloudsAccumulator.localization_from_sensor_to_ego_frame(
                    RigidTransform(
                        [position.x(), position.y(), position.z()],
                        [
                            orientation.w(),
                            orientation.x(),
                            orientation.y(),
                            orientation.z(),
                        ],
                    ),
                    extrinsics,
                )
            )

    def _on_nav_sat_fix(self, nav_sat_fix: NavSatFix):
        assert not self.no_localization

        with self._mutex:
            extrinsics = self._get_localization_extrinsics(
                nav_sat_fix.header().frame_id()
            )
            if extrinsics is None:
                # Extrinsics not received yet
                return

            lat = nav_sat_fix.latitude()
            lon = nav_sat_fix.longitude()
            alt = nav_sat_fix.altitude() if not isnan(nav_sat_fix.altitude()) else 0

            if self._gps_utils is None:
                self._gps_utils = gps_utils.GPS_utils()
                self._gps_utils.setENUorigin(lat, lon, 0)

            enu_fix = self._gps_utils.geo2enu(lat, lon, 0)
            ego_localization = (
                PointCloudsAccumulator.localization_from_sensor_to_ego_frame(
                    RigidTransform(
                        [enu_fix.item(0), enu_fix.item(1), alt],
                        [
                            0,
                            0,
                            0,
                        ],  # Orientation is estimated later by the history of positions
                    ),
                    extrinsics,
                )
            )
            ego_position = ego_localization.translation()
            # yaw is stimated by the history of positions as NavSatFix doesn't provide headings
            yaw = self._estimate_yaw(ego_position)
            self._latest_ego_localization = RigidTransform(ego_position, [0, 0, yaw])

    def _on_extrinsics(self, topic_name, transform_stamped: TransformStamped):
        with self._mutex:
            frame_id = transform_stamped.child_frame_id()
            if (
                not self.no_localization
                and not self.localization_frame_id
                and topic_name == self.localization_extrinsics_topic
            ):
                # Great, now we know the localization frame id
                self.localization_frame_id = frame_id

            translation = transform_stamped.transform().translation()
            rotation = transform_stamped.transform().rotation()
            extrinsics = RigidTransform(
                [translation.x(), translation.y(), translation.z()],
                [rotation.w(), rotation.x(), rotation.y(), rotation.z()],
            )

            self._extrinsics[frame_id] = extrinsics

    def _on_point_cloud(self, point_cloud: PointCloud2):
        with self._mutex:
            ego_localization = self._get_current_ego_localization()
            if ego_localization is None:
                # No localization yet, skip till there is localization info
                # TODO: If no localization for too long, report an error
                return

            frame_id = point_cloud.header().frame_id()
            if not self.no_extrinsics and frame_id not in self._extrinsics:
                # No extrinsics yet, skip till there is extrinsics info
                # TODO: If no extrinsics for too long, report an error
                return

            extrinsics = None if self.no_extrinsics else self._extrinsics[frame_id]

            self._accumulator.accumulate(
                frame_id,
                point_cloud2.read_points(point_cloud),
                ego_localization,
                radar_extrinsics=extrinsics,
            )

        # Out of the mutex lock on purpose, so
        # get_points_local_frame_relative/get_points_ego_relative can be called
        # in the callback
        if self.on_point_cloud:
            self.on_point_cloud(self)

    def _get_current_ego_localization(self):
        # TODO: in the future we might like to extrapolate localization based
        # on history of readings instead of just using the latest reading
        return self._latest_ego_localization

    def _get_localization_extrinsics(self, frame_id: str) -> RigidTransform:
        if not self.localization_frame_id:
            # Great, now we know the localization frame id
            self.localization_frame_id = frame_id

        if self.localization_extrinsics_topic:
            assert (
                self.localization_frame_id == frame_id
            ), f"localization_frame_id is expected to be {self.localization_frame_id} but {frame_id} received"

            if not self.localization_frame_id in self._extrinsics:
                # Localization extrinsics expected, but not received yet
                # TODO: If no extrinsics for too long, report an error
                return None
            return self._extrinsics[frame_id]
        else:
            return _IDENTITY_TRANSFORM

    def _estimate_yaw(self, current_ego_enu_position: [float]):
        assert (
            len(current_ego_enu_position) == 3
        ), "3-component array of floats expected for current_ego_position as [east, north, up]"

        if len(self._positions_history) > 0:
            past_position = self._positions_history[0]
        else:
            past_position = current_ego_enu_position
            self._positions_history.append(current_ego_enu_position)

        moved = [
            current_ego_enu_position[0] - past_position[0],
            current_ego_enu_position[1] - past_position[1],
        ]  # up ignored

        if moved[0] * moved[0] + moved[1] * moved[1] > 0.05:
            # Moved far enough to estimate orientation
            yaw = atan2(moved[1], moved[0])
            self._last_yaw = yaw
            self._positions_history.append(current_ego_enu_position)
        else:
            # Moved too little to estimate orientation
            yaw = self._last_yaw

        return yaw
