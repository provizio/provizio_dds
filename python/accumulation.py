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
from typing import Callable, List, Optional, Sequence
import numpy as np
from threading import Lock
import time
import weakref
from transforms3d.euler import quat2euler
from transforms3d._gohlketransforms import (
    compose_matrix,
    identity_matrix,
    inverse_matrix,
    concatenate_matrices,
    translation_from_matrix,
    quaternion_from_matrix,
)
from math import isnan, atan2, pi, fmod, acos, sin, sqrt

if __package__ or "." in __name__:
    from . import point_cloud2
    from . import provizio_dds
    from . import gps_utils
else:
    import point_cloud2
    import provizio_dds
    import gps_utils


def is_point_static(point: List[float]) -> bool:
    """Standard accumulation filter that only keeps points with ground relative radial velocities around 0.

    Args:
        point: a point to filter, as a list of float values matching the format used in provizio_dds.point_cloud2.

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
        self,
        position: List[float] = None,
        rotation: List[float] = None,
        from_matrix=None,
    ) -> None:
        """Constructs a RigidTransform. If no arguments specified, makes an identity transform (no translation, no rotation).

        Args:
            position (optional): When specified, defines the translation as 3-components array (x, y, z) and from_matrix must be None. Defaults to None.
            rotation (optional): When specified, defines the rotation as either 3 euler angles (roll, pitch, yaw) in radians or a 4-component quaternion (w, x, y, z) and from_matrix must be None. Defaults to None.
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


class LocalizationFilter:
    """Decoupled constant-velocity Kalman filter over an ego pose (x, y, z, roll, pitch, yaw).

    Estimates the ego localization at a point cloud's receive time from the lower-rate, jittery localization
    stream. Six independent [value, rate] channels; each channel's rate (velocity) is estimated internally
    from successive fixes. roll/pitch/yaw are angle channels (wrapped to [-pi, pi)). Times are an opaque
    monotonic clock in seconds; only differences are used. With fewer than two updates the rates are zero, so
    predict() returns ~the last fix. Not internally synchronized: the caller holds the accumulator's lock.
    """

    _r = [0.1 * 0.1] * 3 + [0.02 * 0.02] * 3   # measurement variances: position (m^2), angle (rad^2)
    _q = [1.0] * 3 + [0.5] * 3                 # process spectral densities: position, angle
    _p0 = 1.0                                  # initial covariance

    def __init__(self):
        self._initialized = False
        self._last_time = 0.0
        self._value = [0.0] * 6
        self._rate = [0.0] * 6
        self._cov = [[[self._p0, 0.0], [0.0, self._p0]] for _ in range(6)]

    @staticmethod
    def _is_angle(i):
        return i >= 3

    @staticmethod
    def _wrap(angle):
        w = fmod(angle + pi, 2 * pi)
        return (w + 2 * pi if w < 0 else w) - pi

    @staticmethod
    def _pose_to_vector(pose: "RigidTransform"):
        t = pose.translation()
        roll, pitch, yaw = quat2euler(pose.rotation())   # (w, x, y, z) -> sxyz roll/pitch/yaw
        return [t[0], t[1], t[2], roll, pitch, yaw]

    def has_estimate(self) -> bool:
        """Returns True once at least one update has been fed to the filter.

        Returns:
            bool: True if the filter has been initialized with at least one measurement.
        """
        return self._initialized

    def reset(self):
        """Discards the estimate, returning the filter to its freshly-constructed state (has_estimate() is False;
        the next update() re-initializes from that fix). Used when the localization extrinsics changes: the
        estimated rates were built from poses in the old extrinsics, so they are dropped rather than mapped (a
        rigid right-multiply does not transform the decomposed per-channel pose and rate cleanly), and the filter
        re-converges from subsequent fixes.
        """
        self._initialized = False

    def update(self, time_seconds: float, pose: "RigidTransform"):
        """Feed a new localization measurement into the filter.

        Args:
            time_seconds: Monotonic clock time of the measurement in seconds.
            pose: Ego pose at the given time as a RigidTransform.
        """
        z = self._pose_to_vector(pose)
        if any(isnan(v) for v in z):
            # A NaN pose (e.g. from a malformed odometry message) would poison value/covariance for the rest of
            # the session: _initialized never clears, so has_estimate() would keep returning a NaN estimate.
            # Drop the update and keep the prior state intact.
            return
        if not self._initialized:
            for i in range(6):
                self._value[i] = z[i]
                self._rate[i] = 0.0
                self._cov[i] = [[self._p0, 0.0], [0.0, self._p0]]
            self._last_time = time_seconds
            self._initialized = True
            return

        dt = max(0.0, time_seconds - self._last_time)   # monotonic clock expected
        dt2 = dt * dt
        dt3 = dt2 * dt
        for i in range(6):
            # Predict: value advances by its rate; P = F P F^T + Q, F = [[1, dt],[0, 1]]
            self._value[i] += self._rate[i] * dt
            if self._is_angle(i):
                self._value[i] = self._wrap(self._value[i])
            p = self._cov[i]
            p00 = p[0][0] + dt * (p[1][0] + p[0][1]) + dt2 * p[1][1]
            p01 = p[0][1] + dt * p[1][1]
            p10 = p[1][0] + dt * p[1][1]
            p11 = p[1][1]
            q = self._q[i]
            p[0][0] = p00 + q * dt3 / 3.0
            p[0][1] = p01 + q * dt2 / 2.0
            p[1][0] = p10 + q * dt2 / 2.0
            p[1][1] = p11 + q * dt
            # Update with measurement z[i], H = [1, 0]
            innovation = z[i] - self._value[i]
            if self._is_angle(i):
                innovation = self._wrap(innovation)
            s = p[0][0] + self._r[i]
            k0 = p[0][0] / s
            k1 = p[1][0] / s
            self._value[i] += k0 * innovation
            self._rate[i] += k1 * innovation
            if self._is_angle(i):
                self._value[i] = self._wrap(self._value[i])
            n00 = (1 - k0) * p[0][0]
            n01 = (1 - k0) * p[0][1]
            n10 = p[1][0] - k1 * p[0][0]
            n11 = p[1][1] - k1 * p[0][1]
            p[0][0], p[0][1], p[1][0], p[1][1] = n00, n01, n10, n11
        # Only ever advance the clock; an out-of-order (already dt-clamped) fix must not regress _last_time,
        # or the next update would compute an inflated dt from the older timestamp.
        if time_seconds > self._last_time:
            self._last_time = time_seconds

    def predict(self, time_seconds: float) -> "RigidTransform":
        """Return the predicted ego pose at the given time.

        Args:
            time_seconds: Monotonic clock time at which to evaluate the prediction in seconds.

        Returns:
            RigidTransform: Predicted ego pose at time_seconds. Returns the identity transform
                if the filter has not yet been initialized.
        """
        if not self._initialized:
            return RigidTransform()
        dt = max(0.0, time_seconds - self._last_time)
        v = [0.0] * 6
        for i in range(6):
            vi = self._value[i] + self._rate[i] * dt
            v[i] = self._wrap(vi) if self._is_angle(i) else vi
        return RigidTransform([v[0], v[1], v[2]], [v[3], v[4], v[5]])


_IDENTITY_TRANSFORM = RigidTransform()


_LocalizationFix = collections.namedtuple("_LocalizationFix", ("header", "pose"))


def _slerp_quaternion(q_a, q_b, frac):
    """Shortest-path quaternion slerp between two unit quaternions at fraction frac in [0, 1].

    Takes the short way around by negating q_b when dot(q_a, q_b) < 0. Falls back to
    normalized lerp when the quaternions are nearly parallel (sin(theta) ~ 0) to avoid
    division by near-zero. Both q_a and q_b are (w, x, y, z) tuples/arrays.

    Args:
        q_a: First unit quaternion (w, x, y, z).
        q_b: Second unit quaternion (w, x, y, z).
        frac: Interpolation fraction in [0, 1]; 0 returns q_a, 1 returns q_b.

    Returns:
        Interpolated unit quaternion as a numpy array (w, x, y, z).
    """
    q_b = np.array(q_b, dtype=float)
    q_a = np.array(q_a, dtype=float)
    dot = q_a[0] * q_b[0] + q_a[1] * q_b[1] + q_a[2] * q_b[2] + q_a[3] * q_b[3]
    if dot < 0.0:
        # Take the short way around
        q_b = -q_b
        dot = -dot
    # Clamp for numerical safety before acos
    if dot > 1.0:
        dot = 1.0
    theta = acos(dot)
    sin_theta = sin(theta)
    if sin_theta > 1e-10:
        wa = sin((1.0 - frac) * theta) / sin_theta
        wb = sin(frac * theta) / sin_theta
    else:
        # Nearly parallel: normalized lerp
        wa = 1.0 - frac
        wb = frac
    r = wa * q_a + wb * q_b
    norm = sqrt(float(np.dot(r, r)))
    if norm > 1e-12:
        return r / norm
    # Zero-norm fallback: identity quaternion
    return np.array([1.0, 0.0, 0.0, 0.0])


def _interpolate_pose(fix_a, fix_b, t):
    """Interpolate between two localization fixes at time t.

    Position is linearly interpolated; orientation is slerped along the shortest arc.
    When fix_b.header == fix_a.header the interpolation fraction is 0 (returns fix_a pose).
    fix_a and fix_b are (header_seconds, RigidTransform) named-tuples or plain 2-tuples.

    Args:
        fix_a: Earlier fix (header_seconds, pose).
        fix_b: Later fix (header_seconds, pose).
        t: Target time in seconds.

    Returns:
        RigidTransform at time t.
    """
    t_a, pose_a = fix_a
    t_b, pose_b = fix_b
    if t_b > t_a:
        frac = max(0.0, min(1.0, (t - t_a) / (t_b - t_a)))
    else:
        frac = 0.0
    pos_a = np.array(pose_a.translation())
    pos_b = np.array(pose_b.translation())
    pos = pos_a + frac * (pos_b - pos_a)
    rot = _slerp_quaternion(pose_a.rotation(), pose_b.rotation(), frac)
    return RigidTransform(list(pos), list(rot))


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
        points: List[List[float]],
        ego_localization_when_received: RigidTransform,
        radar_extrinsics: RigidTransform = None,
    ):
        """Accumulates next radar point cloud.

        Args:
            radar_position_id (str): Radar position id the point cloud originates from, such as "provizio_radar_front_center".
            points (List[List[float]]): A list of radar points to accumulate, each being a list of floats matching the format used in provizio_dds.point_cloud2.
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

        # Validate BEFORE creating or touching any buffer: when accumulate raises, the accumulator state stays
        # untouched — a rejected call must not create an empty buffer for a new radar (which would also fix that
        # radar's position in the getters' first-accumulation iteration order).
        if (
            not radar_extrinsics
            and not self.allow_no_extrinsics
            and radar_position_id not in self._extrinsics
        ):
            raise ValueError(
                "allow_no_extrinsics is False so radar_extrinsics must be specified!"
            )

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
                buffer[-self.max_frames_without_filter].points = list(
                    filter(
                        self.point_filter,
                        buffer[-self.max_frames_without_filter].points,
                    )
                )

        buffer.append(
            PointCloudsAccumulator._AccumulatedPointCloud(
                points, ego_localization_when_received
            )
        )

        if radar_extrinsics:
            self._extrinsics[radar_position_id] = radar_extrinsics

    def apply_localization_correction(self, ego_pose_delta: RigidTransform):
        """Retroactively re-places every accumulated frame for a change in the localization extrinsics, without
        resampling or dropping any points. Each frame stores raw points plus the ego pose used, and the world
        placement is ego_pose @ radar_extrinsics @ point computed at read time, so a change in the localization
        extrinsics (which only enters through the ego pose) is corrected by right-multiplying every stored ego
        pose by a single delta. Lets a late-arriving (or updated) localization extrinsics correct
        already-accumulated data instead of forcing a reset.

        Args:
            ego_pose_delta (RigidTransform): Right-multiplied onto every stored ego pose: E_prev @ inverse(E_new),
                where E_prev is the previously-applied and E_new the new localization extrinsics (identity leaves
                everything unchanged).
        """
        delta = ego_pose_delta.matrix()
        for accumulated_pcs in self._buffers.values():
            for accumulated_pc in accumulated_pcs:
                accumulated_pc.ego_localization_when_received = RigidTransform(
                    from_matrix=concatenate_matrices(
                        accumulated_pc.ego_localization_when_received.matrix(), delta
                    )
                )

    def get_points_local_frame_relative(self) -> List[TransformedPoint]:
        """Returns all currently accumulated points with positions relative to the same coordinate frame as localization uses (usually, local ENU).

        Returns:
            List[TransformedPoint]: A list of TransformedPoint.
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
    ) -> List[TransformedPoint]:
        """Returns all currently accumulated points with positions relative to ego_localization_now.

        Args:
            ego_localization_now (RigidTransform): A RigidTransform in the same coordinate frame as localization uses (usually, local ENU).

        Returns:
            List[TransformedPoint]: A list of TransformedPoint.
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
        """Container for a single accumulated point cloud and the ego pose at reception time."""

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
        localization_frame_id: str = None,  # None selects the per-source default: "provizio_radar_front_center" for Odometry, or "any" (learned from the first fix) for NavSatFix
        localization_extrinsics_topic: str = None,  # None selects "rt/provizio_extrinsics" (shared with radar extrinsics) for Odometry/NavSatFix; assumed identity until that frame's transform arrives, then accumulated data is retroactively re-placed
        pointcloud2_topic: str = "rt/provizio_radar_point_cloud",
        extrinsics_topics: Optional[Sequence[str]] = (
            "rt/provizio_extrinsics",
        ),  # A single topic str, a sequence of them, or None/empty (radar-relative frame assumed instead of vehicle-relative)
        snr_threshold: float = PointCloudsAccumulator._default_snr_threshold,
        max_frames_without_filter: int = PointCloudsAccumulator._default_max_frames_without_filter,
        point_filter: Callable[
            [object], bool
        ] = PointCloudsAccumulator._default_point_filter,
        radar_filter: Callable[[str], bool] = None,
        on_point_cloud: Callable[
            [object], None
        ] = None,  # Optional callback to be invoked on receiving and accumulating a new point cloud, takes DDSPointCloudsAccumulator as the only argument
        kalman_localization: bool = True,  # Estimate localization at point-cloud receive time via a Kalman filter
        timesync_max_delay_seconds: float = 0.0,  # Max wait (s) for covering localization; > 0 enables timesync, 0 (default) disables it
        time_source: Callable[[], float] = None,  # Monotonic receive-time clock (seconds); default time.monotonic
        dds_domain_participant=None,  # DDS Domain participant to reuse if any, otherwise a new one will be created
    ) -> None:
        """Constructs a DDSPointCloudsAccumulator.

        Args:
            max_frames_per_radar (int): Max number of radar frames per radar that can be accumulated. On exceeding the number, the oldest frame gets dropped
            localization_type (type, optional): Type of localization data inputs, either None, provizio_dds.Odometry or provizio_dds.NavSatFix. Defaults to provizio_dds.Odometry.
            localization_topic (str, optional): DDS topic of localization data (message type of localization_type), if any. Defaults to "rt/provizio_radar_odometry".
            localization_frame_id (str, optional): Localization data frame name. Selects which source on localization_topic is used (others, identified by a different child_frame_id / header frame_id, are dropped). None selects the per-source default: "provizio_radar_front_center" for Odometry, or "any" (learned from the first fix) for NavSatFix. Defaults to None.
            localization_extrinsics_topic (str, optional): DDS Topic of localization extrinsics (message type provizio_dds.TransformStamped). None selects "rt/provizio_extrinsics" (shared with the radar extrinsics) for Odometry/NavSatFix. The localization extrinsics is assumed identity until the localization frame's transform is received; when it arrives (or later changes) the already-accumulated frames are retroactively re-placed, so none are lost. A GNSS frame that never receives extrinsics simply stays at identity (localization frame = ego frame). Must be None with localization_type=None. Defaults to None.
            pointcloud2_topic (str, optional): DDS Topic of radar provizio.PointCloud2 input data. Defaults to "rt/provizio_radar_point_cloud".
            extrinsics_topics (Sequence[str] | str | None, optional): DDS Topics of radar (and optionally localization) extrinsics data (message type provizio_dds.TransformStamped). Accepts a sequence of topic names, a single topic name (str), or None/empty — in the latter case a radar-relative coordinate frame is assumed instead of a vehicle-relative one. Defaults to ("rt/provizio_extrinsics",).
            snr_threshold (float, optional): Signal to noise ratio threshold. Points with snr below it will be dropped. Defaults to PointCloudsAccumulator._default_snr_threshold.
            max_frames_without_filter (int, optional): Number of frames every new point cloud remains unfiltered, except the SnR threshold filter. When > 0 it can be used to temporarily accumulate points of moving objects to improve dynamic objects detection. Defaults to PointCloudsAccumulator._default_max_frames_without_filter.
            point_filter (Callable[[List[float]], bool], optional): A point filter to apply after max_frames_without_filter. Defaults to PointCloudsAccumulator._default_point_filter, which only accumulates static points.
            radar_filter (Callable[[str], bool], optional): If specified, lets keeping only points from specific radars. Defaults to None, which stands for keeping points from all radars.
            on_point_cloud (Callable[[object], None], optional): A callback, if any, to be invoked on accumulating every radar point cloud. Defaults to None.
            kalman_localization (bool, optional): When True, estimates ego localization at point-cloud receive time via a constant-velocity Kalman filter fed from the localization stream. When False the last-received localization is used, except under timesync (timesync_max_delay_seconds > 0), which always places clouds via the filter's capture-time prediction regardless of this flag. Has no effect when localization_type is None. Defaults to True.
            timesync_max_delay_seconds (float, optional): Maximum time a point cloud is buffered waiting for the localization message that covers its header (sensor-clock) timestamp, so the cloud can be placed at its exact capture-time localization instead of an extrapolated one. 0 (default) disables timesync entirely (clouds are accumulated immediately at the receive-time estimate). ~0.3 (300 ms) is a reasonable value to enable it — Provizio radar odometry is derived from the radar cloud and lags ~200-300 ms. A buffered cloud that exceeds this delay (e.g. the covering localization never arrives) is released anyway using the best-effort forward-extrapolated estimate when the next message arrives. No effect when localization_type is None.
            time_source (Callable[[], float], optional): Monotonic clock returning time in seconds, used to timestamp localization fixes and query predictions at point-cloud receive time. Defaults to time.monotonic.
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

        self._time_source = time_source if time_source is not None else time.monotonic
        self._kalman_localization = kalman_localization and not self.no_localization
        self._timesync = timesync_max_delay_seconds > 0.0 and not self.no_localization
        self._timesync_max_delay_seconds = timesync_max_delay_seconds
        self._use_filter = (kalman_localization or self._timesync) and not self.no_localization
        self._localization_header_offset = 0.0
        self._localization_header_offset_valid = False
        # Two most-recent localization fixes retained for bracketing interpolation (timesync only).
        # Each is a _LocalizationFix(header_seconds, pose) or None before the first fix arrives.
        self._previous_fix = None  # type: _LocalizationFix | None
        self._latest_fix = None    # type: _LocalizationFix | None
        self._timesync_pending = []
        # Match the C++ implementation: only allocate the localization filter when it will actually be used.
        self._localization_filter = LocalizationFilter() if self._use_filter else None

        self._extrinsics = dict()
        if isinstance(extrinsics_topics, str):
            # A bare string is itself a Sequence[str]; treat it as a single topic, not a sequence of characters.
            extrinsics_topics = [extrinsics_topics]
        extrinsics_topics = list(extrinsics_topics) if extrinsics_topics else []
        self.no_extrinsics = len(extrinsics_topics) == 0

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

        # Per-source defaults (mirrors the C++ validate()): localization extrinsics share the radar extrinsics
        # topic, so a frame id used by both a radar and the localization source resolves the same radar->ego
        # transform; odometry defaults to the front-center radar frame, while nav_sat_fix keeps "any".
        if not self.no_localization:
            if not localization_extrinsics_topic:
                localization_extrinsics_topic = "rt/provizio_extrinsics"
            if localization_type == Odometry and not localization_frame_id:
                localization_frame_id = "provizio_radar_front_center"
        self.localization_extrinsics_topic = localization_extrinsics_topic
        self.localization_frame_id = localization_frame_id
        # Localization extrinsics currently baked into the ego poses (assumed identity until the localization
        # frame's transform is received); a change is reconciled by _sync_localization_extrinsics, which
        # retroactively re-places already-accumulated data so nothing is lost.
        self._applied_localization_extrinsics = _IDENTITY_TRANSFORM

        # Use weak references in subscriber callbacks to break reference cycles
        # (Subscriber -> callback -> self -> Subscriber) that would prevent
        # deterministic cleanup of DDS objects before process exit.
        weak_self = weakref.ref(self)

        def _on_odometry_cb(odometry):
            obj = weak_self()
            if obj is not None:
                obj._on_odometry(odometry)

        def _on_nav_sat_fix_cb(nav_sat_fix):
            obj = weak_self()
            if obj is not None:
                obj._on_nav_sat_fix(nav_sat_fix)

        def _make_extrinsics_cb(topic):
            def _on_extrinsics_cb(transform_stamped):
                obj = weak_self()
                if obj is not None:
                    obj._on_extrinsics(topic, transform_stamped)
            return _on_extrinsics_cb

        def _on_point_cloud_cb(point_cloud):
            obj = weak_self()
            if obj is not None:
                obj._on_point_cloud(point_cloud)

        if self.no_localization:
            self._localization_subscriber = None
        else:
            if localization_type == Odometry:
                self._localization_subscriber = provizio_dds.Subscriber(
                    self._participant,
                    localization_topic,
                    OdometryPubSubType,
                    Odometry,
                    _on_odometry_cb,
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
                    _on_nav_sat_fix_cb,
                )

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
                    _make_extrinsics_cb(topic),
                )
            )

        self._pc2_subscriber = provizio_dds.Subscriber(
            self._participant,
            pointcloud2_topic,
            PointCloud2PubSubType,
            PointCloud2,
            _on_point_cloud_cb,
        )

    def __del__(self):
        with self._mutex:
            del self._pc2_subscriber
            for it in self._extrinsics_subscribers:
                del it
            if self._localization_subscriber is not None:
                del self._localization_subscriber

    def get_points_local_frame_relative(self) -> List[TransformedPoint]:
        """Returns all currently accumulated points with positions relative to the same coordinate frame as localization uses (usually, local ENU).

        Returns:
            List[TransformedPoint]: A list of TransformedPoint.
        """

        with self._mutex:
            return self._accumulator.get_points_local_frame_relative()

    def get_points_ego_relative(self) -> List[TransformedPoint]:
        """Returns all currently accumulated points with positions relative to the current ego position/orientation.

        Returns:
            List[TransformedPoint]: A list of TransformedPoint.
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
            if self._use_filter and self._localization_filter.has_estimate():
                if self._timesync and self._localization_header_offset_valid:
                    ego_localization = self._localization_filter.predict(
                        self._time_source() + self._localization_header_offset)
                elif self._kalman_localization:
                    ego_localization = self._localization_filter.predict(self._time_source())
            return self._accumulator.get_points_ego_relative(ego_localization)

    def _on_odometry(self, odometry: Odometry):
        """Processes an incoming `Odometry` message to update ego localization.

        Args:
            odometry: Incoming odometry sample.
        """
        assert not self.no_localization

        released = 0
        with self._mutex:
            # Multiple odometry sources can share one topic; select the desired one by child_frame_id (the moving
            # frame the odometry describes) and drop the rest (localization_frame_id is preset for odometry).
            if odometry.child_frame_id() != self.localization_frame_id:
                return

            # Under timesync: compute the header time up front so the staleness guard runs before
            # any state is mutated. The same header_s is reused in the filter-update block below.
            if self._timesync:
                header_s = odometry.header().stamp().sec() + \
                           odometry.header().stamp().nanosec() * 1e-9
                if self._latest_fix is not None and header_s < self._latest_fix.header:
                    # Out-of-order fix (older than the latest received) — ignore to keep the fix
                    # stream monotonic. Must run before ego localization or fixes are mutated.
                    return

            # Reconcile the localization extrinsics (assumed identity until the localization frame's transform is
            # received); a change retroactively re-places already-accumulated data so nothing is lost.
            self._sync_localization_extrinsics()

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
                    self._applied_localization_extrinsics,
                )
            )
            if self._use_filter:
                now = self._time_source()
                if self._timesync:
                    self._localization_header_offset = header_s - now
                    self._localization_header_offset_valid = True
                    self._localization_filter.update(header_s, self._latest_ego_localization)
                    self._previous_fix = self._latest_fix
                    self._latest_fix = _LocalizationFix(header_s, self._latest_ego_localization)
                    released = self._flush_timesync_buffer(now)
                else:
                    self._localization_filter.update(now, self._latest_ego_localization)

        # Fire on_point_cloud outside the lock for every cloud released from the timesync buffer
        if self.on_point_cloud:
            for _ in range(released):
                self.on_point_cloud(self)

    def _on_nav_sat_fix(self, nav_sat_fix: NavSatFix):
        """Processes an incoming `NavSatFix` message to update ego localization.

        Orientation (yaw) is estimated from recent positions history.

        Args:
            nav_sat_fix: Incoming NavSatFix sample.
        """
        assert not self.no_localization

        released = 0
        with self._mutex:
            # Reject a garbage fix (e.g. a GPS sample emitted before fix-lock) BEFORE anything else: a NaN
            # fix must never learn/lock localization_frame_id — that would drop every later valid fix from the
            # intended source — nor poison the ENU origin (set once, from the first valid fix).
            lat = nav_sat_fix.latitude()
            lon = nav_sat_fix.longitude()
            if isnan(lat) or isnan(lon):
                return

            # nav_sat_fix learns its localization frame from the first (valid) fix (default "any"); multiple GNSS
            # sources can share one topic, so fixes from any other frame are dropped.
            if not self.localization_frame_id:
                self.localization_frame_id = nav_sat_fix.header().frame_id()
            if nav_sat_fix.header().frame_id() != self.localization_frame_id:
                return

            # Under timesync: compute the header time up front so the staleness guard runs before
            # any state is mutated. The same header_s is reused in the filter-update block below.
            if self._timesync:
                header_s = nav_sat_fix.header().stamp().sec() + \
                           nav_sat_fix.header().stamp().nanosec() * 1e-9
                if self._latest_fix is not None and header_s < self._latest_fix.header:
                    # Out-of-order fix (older than the latest received) — ignore to keep the fix
                    # stream monotonic. Must run before ego localization or fixes are mutated.
                    return

            alt = nav_sat_fix.altitude() if not isnan(nav_sat_fix.altitude()) else 0

            if self._gps_utils is None:
                self._gps_utils = gps_utils.GPS_utils()
                self._gps_utils.setENUorigin(lat, lon, 0)

            # Reconcile the localization extrinsics (assumed identity until the localization frame's transform is
            # received — a GNSS frame typically has none, so it stays at identity / localization frame = ego
            # frame). Run before _estimate_yaw, which re-seeds the (possibly just-cleared) position history.
            self._sync_localization_extrinsics()

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
                    self._applied_localization_extrinsics,
                )
            )
            ego_position = ego_localization.translation()
            # yaw is estimated by the history of positions as NavSatFix doesn't provide headings
            yaw = self._estimate_yaw(ego_position)
            self._latest_ego_localization = RigidTransform(ego_position, [0, 0, yaw])
            if self._use_filter:
                now = self._time_source()
                if self._timesync:
                    self._localization_header_offset = header_s - now
                    self._localization_header_offset_valid = True
                    self._localization_filter.update(header_s, self._latest_ego_localization)
                    self._previous_fix = self._latest_fix
                    self._latest_fix = _LocalizationFix(header_s, self._latest_ego_localization)
                    released = self._flush_timesync_buffer(now)
                else:
                    self._localization_filter.update(now, self._latest_ego_localization)

        # Fire on_point_cloud outside the lock for every cloud released from the timesync buffer
        if self.on_point_cloud:
            for _ in range(released):
                self.on_point_cloud(self)

    def _on_extrinsics(self, topic_name, transform_stamped: TransformStamped):
        """Processes an incoming `TransformStamped` with extrinsics for radars or localization sensor.

        Args:
            topic_name: DDS topic the extrinsics have been published to.
            transform_stamped: Incoming TransformStamped sample.
        """
        # Every extrinsics topic feeds the shared map; the localization frame is identified by
        # localization_frame_id (learned from localization messages, not from extrinsics).
        del topic_name
        with self._mutex:
            frame_id = transform_stamped.child_frame_id()
            translation = transform_stamped.transform().translation()
            rotation = transform_stamped.transform().rotation()
            self._extrinsics[frame_id] = RigidTransform(
                [translation.x(), translation.y(), translation.z()],
                [rotation.w(), rotation.x(), rotation.y(), rotation.z()],
            )

            # The newly-received transform may be the localization frame's: reconcile and retroactively re-place
            # the already-accumulated data if the localization extrinsics just changed. No-op for other frames.
            self._sync_localization_extrinsics()

    def _on_point_cloud(self, point_cloud: PointCloud2):
        """Processes an incoming radar `PointCloud2` and accumulates its points if prerequisites are met.

        Args:
            point_cloud: Incoming PointCloud2 sample.
        """
        released = 0
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

            if self._timesync:
                now = self._time_source()
                pc_header_s = point_cloud.header().stamp().sec() + \
                              point_cloud.header().stamp().nanosec() * 1e-9
                points = point_cloud2.read_points(point_cloud)
                self._timesync_pending.append((frame_id, points, extrinsics, pc_header_s, now))
                released = self._flush_timesync_buffer(now)
            else:
                if self._use_filter and self._localization_filter.has_estimate():
                    now = self._time_source()
                    if self._kalman_localization:
                        ego_localization = self._localization_filter.predict(now)

                self._accumulator.accumulate(
                    frame_id,
                    point_cloud2.read_points(point_cloud),
                    ego_localization,
                    radar_extrinsics=extrinsics,
                )
                released = 1

        # Out of the mutex lock on purpose, so
        # get_points_local_frame_relative/get_points_ego_relative can be called
        # in the callback. Fire once per released cloud.
        if self.on_point_cloud:
            for _ in range(released):
                self.on_point_cloud(self)

    def _flush_timesync_buffer(self, now):
        """Release buffered clouds that are either covered by localization or timed out.

        Must be called with self._mutex already held — does not re-acquire the lock.

        A cloud is COVERED when self._latest_fix is not None and
        self._latest_fix.header >= the cloud's own header timestamp.
        A cloud TIMES OUT when its receive-time age exceeds self._timesync_max_delay_seconds.

        Pose determination for a released cloud at time T (priority order):
          1. Covered (latest_fix.header >= T):
               - previous_fix present and previous_fix.header <= T:
                   interpolate between (previous_fix, latest_fix) at T — collapses to the
                   exact fix pose when a fix sits at T (frac == 0 or 1).
               - previous_fix present but previous_fix.header > T:
                   cloud is older than the last-but-one fix (badly out-of-order delivery);
                   use previous_fix (the closer, older of the two retained fixes).
               - only one fix ever received:
                   use latest_fix directly (startup edge case).
          2. Not covered, Kalman filter has an estimate:
               predict(T) — forward-extrapolation beyond the latest known fix.
          3. Latest raw ego localization: use it as a last resort.
          4. No localization at all: drop the cloud.

        Args:
            now: Current monotonic clock time in seconds.
        """
        kept = []
        released = 0
        for frame_id, points, extrinsics, header_s, receive_s in self._timesync_pending:
            covered = (self._latest_fix is not None
                       and self._latest_fix.header >= header_s)
            timed_out = (now - receive_s) >= self._timesync_max_delay_seconds
            if not covered and not timed_out:
                kept.append((frame_id, points, extrinsics, header_s, receive_s))
                continue

            T = header_s
            ego_localization = None

            if covered:
                # latest_fix is the upper bracket.
                if self._previous_fix is not None and self._previous_fix.header <= T:
                    # Both brackets present: interpolate (collapses to exact fix when a fix sits at T).
                    ego_localization = _interpolate_pose(self._previous_fix, self._latest_fix, T)
                elif self._previous_fix is not None:
                    # Cloud is older than the last-but-one fix (badly out-of-order delivery):
                    # use previous_fix, the closer (older) of the two retained fixes.
                    ego_localization = self._previous_fix.pose
                else:
                    # Startup edge: only one fix received so far; use it directly.
                    ego_localization = self._latest_fix.pose
            elif self._localization_filter.has_estimate():
                # Beyond the latest fix (timeout release): Kalman forward-extrapolation to T.
                ego_localization = self._localization_filter.predict(T)
            elif self._latest_ego_localization is not None:
                ego_localization = self._latest_ego_localization
            # else: no localization at all — drop (matches the pre-localization skip in _on_point_cloud)

            if ego_localization is not None:
                self._accumulator.accumulate(
                    frame_id,
                    points,
                    ego_localization,
                    radar_extrinsics=extrinsics,
                )
                released += 1
        self._timesync_pending = kept
        return released

    def _get_current_ego_localization(self):
        """Returns the latest known ego localization transform or None if not available."""
        # TODO: in the future we might like to extrapolate localization based
        # on history of readings instead of just using the latest reading
        return self._latest_ego_localization

    def _sync_localization_extrinsics(self):
        """Reconciles the localization extrinsics known for localization_frame_id (assumed identity until that
        frame's transform is received) with the one already applied to accumulated data. On a change (notably the
        first non-identity extrinsics arriving after frames were accumulated under the identity assumption) it
        retroactively re-places every accumulated frame and the retained localization poses, and resets the
        transient localization estimators (Kalman filter + yaw history). No-op while localization_frame_id is
        unknown (nav_sat_fix before its first fix). Called under self._mutex.
        """
        if not self.localization_frame_id:
            return

        current = self._extrinsics.get(
            self.localization_frame_id, _IDENTITY_TRANSFORM
        )
        if (current.matrix() == self._applied_localization_extrinsics.matrix()).all():
            # Unchanged (still identity, or the same transform re-published): nothing to re-place.
            return

        # The ego pose computed under the previously-applied extrinsics is sensor_pose @ inverse(E_prev); under
        # the new one it is sensor_pose @ inverse(E_new). Mapping every already-placed ego pose from the former to
        # the latter is a single right-multiply by delta = E_prev @ inverse(E_new).
        delta_matrix = concatenate_matrices(
            self._applied_localization_extrinsics.matrix(), current.inversed_matrix()
        )
        delta = RigidTransform(from_matrix=delta_matrix)

        # Retroactively re-place everything already accumulated (no points dropped or resampled) ...
        self._accumulator.apply_localization_correction(delta)
        # ... and the retained ego poses used by the cloud-placement and timesync-interpolation paths.
        if self._latest_ego_localization is not None:
            self._latest_ego_localization = RigidTransform(
                from_matrix=concatenate_matrices(
                    self._latest_ego_localization.matrix(), delta_matrix
                )
            )
        if self._latest_fix is not None:
            self._latest_fix = _LocalizationFix(
                self._latest_fix.header,
                RigidTransform(
                    from_matrix=concatenate_matrices(self._latest_fix.pose.matrix(), delta_matrix)
                ),
            )
        if self._previous_fix is not None:
            self._previous_fix = _LocalizationFix(
                self._previous_fix.header,
                RigidTransform(
                    from_matrix=concatenate_matrices(self._previous_fix.pose.matrix(), delta_matrix)
                ),
            )

        # The transient estimators were built from poses in the old extrinsics and do not transform cleanly under
        # a rigid right-multiply (the decomposed per-channel Kalman pose+rate, and the yaw-from-position history),
        # so they are dropped and re-converge from subsequent fixes. The accumulated data is corrected above, so
        # nothing accumulated is lost.
        if self._localization_filter is not None:
            self._localization_filter.reset()
        positions_history = getattr(self, "_positions_history", None)
        if positions_history is not None:
            positions_history.clear()
            self._last_yaw = 0

        self._applied_localization_extrinsics = current

    def _estimate_yaw(self, current_ego_enu_position: List[float]):
        """Estimates yaw from displacement between current and past ENU positions.

        Args:
            current_ego_enu_position: [east, north, up]
        Returns:
            float: Estimated yaw in radians.
        """
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
