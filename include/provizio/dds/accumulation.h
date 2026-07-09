// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DDS_ACCUMULATION
#define DDS_ACCUMULATION

/**
 * @file accumulation.h
 * @brief Point clouds accumulation and multi-radar fusion.
 *
 * Non-template, Eigen-independent code (rigid_transform, the accumulate/bookkeeping paths, the DDS wiring) is
 * compiled into libprovizio_dds (src/accumulation.cpp). The accumulated-points getters (the get_points* ->
 * math::transform_points call path) deliberately stay header-inline: the Eigen/no-Eigen maths choice binds in the
 * CONSUMER's translation unit (see detail/accumulation_math.h), keeping libprovizio_dds and the bin cache
 * Eigen-agnostic.
 */

#include <array>
#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "provizio/dds/common.h"
#include "provizio/dds/point_cloud2.h"

// The DDS-fed accumulator's method bodies live in src/accumulation.cpp, so the header only needs these types
// DECLARED (member declarations through std::shared_ptr work with incomplete types because the destructor is
// defined in the .cpp as well); sensor_msgs::msg::PointCloud2 is complete via point_cloud2.h.
namespace geometry_msgs::msg
{
    class TransformStamped;
    class TransformStampedPubSubType;
}  // namespace geometry_msgs::msg
namespace nav_msgs::msg
{
    class Odometry;
    class OdometryPubSubType;
}  // namespace nav_msgs::msg
namespace sensor_msgs::msg
{
    class NavSatFix;
    class NavSatFixPubSubType;
    class PointCloud2PubSubType;
}  // namespace sensor_msgs::msg
namespace provizio::dds::accumulation::detail
{
    class gps_utils;
    class localization_filter;
}  // namespace provizio::dds::accumulation::detail
namespace provizio::dds
{
    class domain_participant;
    template <typename data_pub_sub_type> class subscriber_handle;
}  // namespace provizio::dds

namespace provizio::dds::accumulation
{
    /**
     * @brief A 4x4 row-major transformation matrix, double precision (all matrix composing/folding/inversion maths
     * is computed in double — large local-ENU translations would suffer cancellation in float32; only the per-point
     * batch transform runs in float32, matching the wire format and vectorizing wider — see
     * detail/accumulation_math.h).
     */
    using matrix4x4 = std::array<std::array<double, 4>, 4>;

    /**
     * @brief A rotation quaternion (w, x, y, z).
     *
     * @note Deliberately NOT an aggregate (explicit 4-argument constructor): a braced 3-element list like {0, 0, 0}
     * must never be a viable quaternion, so rigid_transform{{x, y, z}, {roll, pitch, yaw}} unambiguously selects the
     * Euler constructor while rigid_transform{{x, y, z}, {w, x, y, z}} unambiguously selects the quaternion one.
     */
    struct quaternion_wxyz
    {
        /**
         * @brief Constructs the quaternion from its 4 components.
         */
        quaternion_wxyz(const double w, const double x, const double y, const double z) noexcept
            : w(w), x(x), y(y), z(z)
        {
        }

        double w; /**< @brief The scalar (real) component */
        double x; /**< @brief The i component */
        double y; /**< @brief The j component */
        double z; /**< @brief The k component */
    };

    /**
     * @brief Represents a combination of rotation and translation in Euclidean space.
     *
     * @note Methods don't mutate and don't cache: concurrent const access is safe.
     */
    class rigid_transform
    {
      public:
        /**
         * @brief Constructs an identity transform (no translation, no rotation).
         */
        PROVIZIO_DDS_API rigid_transform() noexcept;

        /**
         * @brief Constructs a transform from a translation and Euler angles.
         *
         * @param position Translation as (x, y, z).
         * @param rotation_euler_roll_pitch_yaw Rotation as Euler angles (roll, pitch, yaw) in radians, static-frame
         * 'sxyz' convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
         */
        PROVIZIO_DDS_API rigid_transform(const std::array<double, 3> &position,
                                         const std::array<double, 3> &rotation_euler_roll_pitch_yaw) noexcept;

        /**
         * @brief Constructs a transform from a translation and a rotation quaternion.
         *
         * @param position Translation as (x, y, z).
         * @param rotation Rotation as a quaternion (w, x, y, z); normalized internally.
         * @note A zero-norm quaternion yields the identity rotation (translation preserved).
         */
        PROVIZIO_DDS_API rigid_transform(const std::array<double, 3> &position,
                                         const quaternion_wxyz &rotation) noexcept;

        /**
         * @brief Constructs a transform from a whole 4x4 matrix.
         *
         * @param from_matrix The transformation matrix; expected to be a RIGID transform (orthonormal rotation +
         * translation) — translation()/rotation()/inversed_matrix() assume rigidity.
         */
        PROVIZIO_DDS_API explicit rigid_transform(const matrix4x4 &from_matrix) noexcept;

        /**
         * @brief Returns the 4x4 transformation matrix combining both translation and rotation.
         */
        PROVIZIO_DDS_API const matrix4x4 &matrix() const noexcept;

        /**
         * @brief Returns the inversion of matrix(). Computed on demand (cheap for rigid transforms).
         */
        PROVIZIO_DDS_API matrix4x4 inversed_matrix() const noexcept;

        /**
         * @brief Returns the translation component as (x, y, z).
         */
        PROVIZIO_DDS_API std::array<double, 3> translation() const noexcept;

        /**
         * @brief Returns the rotation component as a quaternion (w, x, y, z).
         */
        PROVIZIO_DDS_API quaternion_wxyz rotation() const noexcept;

      private:
        matrix4x4 the_matrix;
    };

    /**
     * @brief A point as returned by the accumulator: position transformed into the requested coordinate frame plus
     * the point's accumulated metadata.
     */
    struct accumulated_point
    {
        std::array<float, 3> position{};          /**< @brief Position (x, y, z) in the requested frame, meters */
        float ground_relative_radial_velocity{0}; /**< @brief Radial ground relative velocity, m/s */
        float signal_to_noise_ratio{0};           /**< @brief Signal-to-noise ratio */
        std::shared_ptr<const std::string> radar_position_id{}; /**< @brief Radar position id the point originates
            from, f.e. "provizio_radar_front_center"; shared by all points of the same radar */

        /**
         * @brief Convenience accessor: the radar position id as a string (empty when unset).
         */
        const std::string &radar_id() const noexcept
        {
            static const std::string empty;
            return radar_position_id ? *radar_position_id : empty;
        }
    };

    /**
     * @brief A point filter: returns true for points to keep. Receives the original (radar-frame) radar_point.
     */
    using point_filter_function = std::function<bool(const point_cloud2::radar_point &)>;

    /**
     * @brief A radar filter: returns true for radars whose point clouds are to be accumulated.
     */
    using radar_filter_function = std::function<bool(const std::string &radar_position_id)>;

    /**
     * @brief The standard accumulation point filter: keeps only points with ground-relative radial velocities
     * around 0, i.e. points of static objects.
     *
     * @param point The point to filter.
     * @return true for points detected as static; false otherwise (incl. NaN ground_relative_radial_velocity,
     * f.e. when the source cloud doesn't provide that optional field).
     */
    PROVIZIO_DDS_API bool is_point_static(const point_cloud2::radar_point &point);

    /**
     * @brief Configuration of a point_clouds_accumulator (named-field configuration: default-construct it and set
     * only the fields you need).
     */
    struct accumulation_options
    {
        /** @brief Signal-to-noise-ratio threshold: points with snr below it are dropped at accumulation time
         * (<= 0 disables) */
        float snr_threshold = 2.5F;

        /** @brief Number of frames every new point cloud remains unfiltered (except the SnR threshold). When > 0,
         * points of moving objects stay briefly accumulated, improving dynamic object detection. When >=
         * max_frames_per_radar, point_filter is disabled entirely */
        std::size_t max_frames_without_filter = 3;

        /** @brief The point filter applied after max_frames_without_filter; empty = no filtering.
         * Defaults to is_point_static.
         * @note Must not throw: the deferred re-filtering path applies it to already-stored frames, and a throwing
         * filter would leave such a frame partially filtered
         * @note When driven by dds_point_clouds_accumulator, the filter runs under its internal lock: do NOT call
         * that accumulator's getters from the filter (self-deadlock). */
        point_filter_function point_filter = is_point_static;

        /** @brief When set, only radars it returns true for are accumulated; empty = all radars
         * @note When driven by dds_point_clouds_accumulator, the filter runs under its internal lock: do NOT call
         * that accumulator's getters from the filter (self-deadlock). */
        radar_filter_function radar_filter = {};

        /** @brief Allows accumulation when radar extrinsics are not specified (the ego coordinate frame is then
         * assumed to be the radar coordinate frame) */
        bool allow_no_extrinsics = false;
    };

    /**
     * @brief Implements the core points accumulation functionality, relying on point clouds, localization and
     * extrinsics provided by the caller. See also dds_point_clouds_accumulator for the DDS-fed counterpart.
     *
     * @note NOT thread-safe by itself: synchronize externally when sharing across threads.
     * dds_point_clouds_accumulator does precisely that.
     */
    class point_clouds_accumulator
    {
      public:
        /**
         * @brief Constructs a point_clouds_accumulator.
         *
         * @param max_frames_per_radar Max number of radar frames per radar that can be accumulated; on exceeding it
         * the oldest frame of that radar is dropped. Must be > 0.
         * @param options Accumulation options (filters, thresholds); see accumulation_options.
         * @throws std::invalid_argument When max_frames_per_radar is 0.
         */
        PROVIZIO_DDS_API explicit point_clouds_accumulator(std::size_t max_frames_per_radar,
                                                           accumulation_options options = {});

        /**
         * @brief Accumulates the next radar point cloud.
         *
         * @param radar_position_id Radar position id the point cloud originates from, f.e.
         * "provizio_radar_front_center".
         * @param points The radar points to accumulate (f.e. parsed by point_cloud2::read_radar_points); taken by
         * value and moved into the internal frame storage.
         * @param ego_localization_when_received The ego position and orientation in a local Euclidean reference
         * frame (usually ENU) at the moment the cloud was received.
         * @param radar_extrinsics The radar position and orientation relative to the ego coordinate frame, when
         * known. std::nullopt keeps the previously provided extrinsics of that radar if any, or means radar frame =
         * ego frame when constructed with allow_no_extrinsics.
         * @throws std::invalid_argument When no extrinsics were ever provided for this radar while
         * allow_no_extrinsics is false. Nothing is buffered in that case: the accumulator state stays untouched.
         */
        PROVIZIO_DDS_API void accumulate(const std::string &radar_position_id,
                                         std::vector<point_cloud2::radar_point> points,
                                         const rigid_transform &ego_localization_when_received,
                                         const std::optional<rigid_transform> &radar_extrinsics = std::nullopt);

        /**
         * @brief Returns all currently accumulated points with positions relative to the same coordinate frame as
         * localization uses (usually, local ENU).
         *
         * @return Accumulated points; per-radar in first-accumulation order, frames oldest to newest.
         * @note Header-inline by design (NOT compiled into libprovizio_dds): it is the maths path, see the @file doc.
         */
        std::vector<accumulated_point> get_points_local_frame_relative() const;

        /**
         * @brief Returns all currently accumulated points with positions relative to ego_localization_now.
         *
         * @param ego_localization_now The current ego localization, in the same coordinate frame the accumulated
         * ego localizations use (usually, local ENU).
         * @return Accumulated points; per-radar in first-accumulation order, frames oldest to newest.
         * @note Header-inline by design (NOT compiled into libprovizio_dds): it is the maths path, see the @file doc.
         */
        std::vector<accumulated_point> get_points_ego_relative(const rigid_transform &ego_localization_now) const;

        /**
         * @brief Retroactively re-places every accumulated frame for a change in the localization extrinsics,
         * without resampling or dropping any points. Each frame stores raw points plus the ego pose used, and the
         * world placement is ego_pose * radar_extrinsics * point computed at read time, so a change in the
         * localization extrinsics (which only enters through the ego pose) is corrected by right-multiplying every
         * stored ego pose by a single delta. Lets a late-arriving (or updated) localization extrinsics correct
         * already-accumulated data instead of forcing a reset. Thread-safety is the caller's responsibility.
         *
         * @param ego_pose_delta Right-multiplied onto every stored ego pose: E_prev * E_new^-1, where E_prev is the
         * previously-applied and E_new the new localization extrinsics (identity leaves everything unchanged).
         */
        PROVIZIO_DDS_API void apply_localization_correction(const rigid_transform &ego_pose_delta);

        /**
         * @brief Converts a localization sensor reading (the position and orientation OF the localization sensor)
         * to the localization of the ego frame, given the sensor's extrinsics.
         *
         * @param sensor_localization The localization as detected by the localization sensor, i.e. where that
         * sensor is.
         * @param sensor_extrinsics Position and orientation of the localization sensor in the ego coordinate frame.
         * @return The localization of the ego coordinate frame, in the localization's reference frame (usually ENU).
         */
        PROVIZIO_DDS_API static rigid_transform localization_from_sensor_to_ego_frame(
            const rigid_transform &sensor_localization, const rigid_transform &sensor_extrinsics);

      private:
        /// A single accumulated point cloud and the ego pose at reception time
        struct accumulated_frame
        {
            std::vector<point_cloud2::radar_point> points;
            rigid_transform ego_localization_when_received;
        };

        /// All state of one radar: its shared id string, the frames ring and the latest known extrinsics
        struct radar_buffer
        {
            std::shared_ptr<const std::string> radar_position_id;
            std::deque<accumulated_frame> frames;
            std::optional<rigid_transform> extrinsics;
        };

        /// Non-mutating lookup of an existing radar buffer (nullptr if this radar has none yet). Lets
        /// accumulate() validate before buffer_for() creates any state, so a rejected call leaves the
        /// accumulator — and the first-accumulation ordering below — untouched. In src/accumulation.cpp.
        radar_buffer *find_buffer(const std::string &radar_position_id);
        /// In src/accumulation.cpp; no export needed: only called from accumulate (the same translation unit)
        radar_buffer &buffer_for(const std::string &radar_position_id);
        /// Header-inline by design: the maths path shared by both public getters (see the @file doc)
        std::vector<accumulated_point> get_points(const matrix4x4 *to_ego_matrix) const;

        std::size_t max_frames_per_radar;
        accumulation_options options;
        // Insertion-ordered: accumulated points are returned grouped by radar in first-accumulation order (a
        // documented guarantee of the getters); radar counts are small (typically <= 6 per vehicle), so linear
        // lookup beats any map
        std::vector<radar_buffer> buffers;
    };

    /**
     * @brief The source of ego localization data for dds_point_clouds_accumulator.
     */
    enum class localization_source
    {
        none,       /**< @brief No localization: a permanently-identity ego pose (static ego / single-frame
                         multi-radar fusion use case) */
        odometry,   /**< @brief nav_msgs::msg::Odometry messages (position + orientation) */
        nav_sat_fix /**< @brief sensor_msgs::msg::NavSatFix GPS messages (positions converted to local ENU;
                         orientation estimated from the position history) */
    };

    class dds_point_clouds_accumulator;

    /**
     * @brief Configuration of a dds_point_clouds_accumulator (named-field configuration: default-construct it and
     * set only the fields you need).
     */
    struct dds_accumulation_options
    {
        /** @brief The localization data source kind. With localization_source::none, localization_topic,
         * localization_frame_id and localization_extrinsics_topic must all be empty */
        localization_source localization = localization_source::odometry;

        /** @brief DDS topic of localization data; must be non-empty unless localization_source::none */
        std::string localization_topic = "rt/provizio_radar_odometry";

        /** @brief Localization data frame id; empty selects the per-source default applied at construction:
         * "provizio_radar_front_center" for odometry (radar-based odometry shares the front-center radar's frame,
         * hence its extrinsics), or "any" for nav_sat_fix (learned from the first fix — the GNSS sensor frame
         * differs from the radar frames). */
        std::string localization_frame_id = {};

        /** @brief DDS topic of localization extrinsics (TransformStamped). Empty selects the default
         * "rt/provizio_extrinsics" (shared with the radar extrinsics topic) for odometry / nav_sat_fix. The
         * localization extrinsics is assumed identity until the localization frame's transform is received on this
         * topic; when it arrives (or later changes) the already-accumulated frames are retroactively re-placed, so
         * none are lost. A GNSS frame that never receives extrinsics simply stays at identity (localization frame =
         * ego frame). */
        std::string localization_extrinsics_topic = {};

        /** @brief DDS topic of radar PointCloud2 input data */
        std::string pointcloud2_topic = "rt/provizio_radar_point_cloud";

        /** @brief DDS topics of radar (and optionally localization) extrinsics (TransformStamped). Empty list =
         * no extrinsics: the radar coordinate frame is assumed to be the ego coordinate frame */
        std::vector<std::string> extrinsics_topics = {"rt/provizio_extrinsics"};

        /** @brief Core accumulation options (thresholds and filters). NOTE: accumulation.allow_no_extrinsics is
         * ignored here — it is derived from extrinsics_topics being empty */
        accumulation_options accumulation = {};

        /** @brief Optional callback invoked on receiving and accumulating every radar point cloud. Invoked OUTSIDE
         * the internal lock, so get_points_local_frame_relative / get_points_ego_relative may be called from it.
         * Invoked on a Fast-DDS thread: treat it as a hot callback and never block in it
         * @note The callback runs inside a DDS data callback: it must NOT create or destroy DDS endpoints (no
         * make_publisher / make_subscriber / new participants) — see the callback rules in subscriber.h. Calling
         * this accumulator's get_points_* getters is explicitly safe. */
        std::function<void(dds_point_clouds_accumulator &)> on_point_cloud = {};

        /** @brief When true (default), the ego localization used to place each incoming point cloud is estimated
         * at the cloud's receive time by a constant-velocity Kalman filter over the localization stream, instead of
         * using the last-received (often lower-rate, out-of-sync) localization. When false, the last-received
         * localization is used (the pre-filter behaviour) — except under timesync (timesync_max_delay_seconds > 0),
         * which always places clouds via the filter's capture-time prediction regardless of this flag, since exact
         * timestamp alignment requires it. No effect with localization_source::none. */
        bool kalman_localization = true;

        /** @brief Source of the monotonic "receive time" (seconds) the Kalman filter runs on, captured at the start
         * of each localization / point-cloud callback. Empty (default) installs a std::chrono::steady_clock source.
         * Injectable for reproducible tests and future clock policies; it must be monotonic and is only ever read
         * for time differences. */
        std::function<double()> time_source = {};

        /** @brief Maximum time a point cloud is buffered waiting for the localization message that covers its
         * header (sensor-clock) timestamp, so the cloud can be placed at its exact capture-time localization
         * instead of an extrapolated one. 0 (default) disables timesync entirely (clouds are accumulated
         * immediately at the receive-time estimate). ~0.3 (300 ms) is a reasonable value to enable it —
         * Provizio radar odometry is derived from the radar cloud and lags ~200-300 ms. A buffered cloud that
         * exceeds this delay (e.g. the covering localization never arrives) is released anyway using the
         * best-effort forward-extrapolated estimate when the next message arrives. No effect with
         * localization_source::none. */
        double timesync_max_delay_seconds = 0.0;
    };

    /**
     * @brief Implements points accumulation with all input data (radar point clouds, localization, extrinsics)
     * received from the appropriate DDS topics.
     *
     * Thread-safe. Non-copyable and non-movable (DDS callbacks capture `this`).
     */
    class dds_point_clouds_accumulator
    {
      public:
        /**
         * @brief Constructs a dds_point_clouds_accumulator and subscribes to the configured topics.
         *
         * @param max_frames_per_radar Max number of radar frames per radar that can be accumulated; on exceeding it
         * the oldest frame of that radar is dropped. Must be > 0.
         * @param options DDS and accumulation configuration; see dds_accumulation_options.
         * @param participant The DDS domain participant to reuse, or nullptr (default) to create a new one.
         * @throws std::invalid_argument When options are inconsistent (localization_source::none with non-empty
         * localization_topic / localization_frame_id / localization_extrinsics_topic), when localization_topic is
         * empty for a non-none localization source, or when max_frames_per_radar == 0.
         */
        PROVIZIO_DDS_API explicit dds_point_clouds_accumulator(
            std::size_t max_frames_per_radar, dds_accumulation_options options = {},
            std::shared_ptr<domain_participant> participant = nullptr);

        // Destruction is safe without any custom code: the subscribers are the LAST members, so they are destroyed
        // FIRST, and a provizio_dds subscriber destructor drains in-flight callbacks — the `this` captured by the
        // callbacks can never touch destroyed state. Defined (= default) in src/accumulation.cpp, where the
        // subscriber types are complete — the header only forward-declares them.
        PROVIZIO_DDS_API ~dds_point_clouds_accumulator();

        dds_point_clouds_accumulator(const dds_point_clouds_accumulator &) = delete;
        dds_point_clouds_accumulator &operator=(const dds_point_clouds_accumulator &) = delete;
        dds_point_clouds_accumulator(dds_point_clouds_accumulator &&) = delete;
        dds_point_clouds_accumulator &operator=(dds_point_clouds_accumulator &&) = delete;

        /**
         * @brief Returns all currently accumulated points with positions relative to the same coordinate frame as
         * localization uses (usually, local ENU). Thread-safe.
         * @note Header-inline by design (NOT compiled into libprovizio_dds): it is the maths path, see the @file doc.
         */
        std::vector<accumulated_point> get_points_local_frame_relative() const;

        /**
         * @brief Returns all currently accumulated points with positions relative to the current (latest received)
         * ego localization. Thread-safe.
         *
         * @return Accumulated points; empty when localization is required but hasn't been received yet.
         * @note Header-inline by design (NOT compiled into libprovizio_dds): it is the maths path, see the @file doc.
         */
        std::vector<accumulated_point> get_points_ego_relative() const;

      private:
        // The private methods below are compiled into libprovizio_dds (src/accumulation.cpp); no export needed:
        // they are only called from that same translation unit
        void on_pc2_message(const sensor_msgs::msg::PointCloud2 &cloud);
        void on_extrinsics_message(const geometry_msgs::msg::TransformStamped &transform);
        void on_odometry_message(const nav_msgs::msg::Odometry &odometry);
        void on_nav_sat_fix_message(const sensor_msgs::msg::NavSatFix &fix);
        /// Releases every READY entry from timesync_buffer (covered by localization OR timed out), in FIFO order,
        /// keeping the rest. Returns the number of clouds actually passed to accumulate(). Must be called with
        /// the mutex already held.
        std::size_t flush_timesync_buffer(double now);
        /// Estimates yaw from the displacement between the current and the oldest remembered ENU positions
        /// (NavSatFix carries no heading): returns the previous yaw while the squared displacement is <= 0.05 m^2,
        /// and only then appends to the (3-deep) position history. Must be called under the mutex.
        double estimate_yaw(const std::array<double, 3> &current_ego_enu_position);
        /// Reconciles the localization extrinsics currently known for localization_frame_id (assumed identity until
        /// that frame's transform is received) with the one already applied to accumulated data. On a change
        /// (notably the first non-identity extrinsics arriving after frames were accumulated under the identity
        /// assumption) it retroactively re-places every accumulated frame and the retained localization poses, and
        /// resets the transient localization estimators (Kalman filter + yaw history). No-op while
        /// localization_frame_id is unknown (nav_sat_fix before its first fix). Must be called under the mutex.
        void sync_localization_extrinsics();
        /// The ego localization to use for an ego-relative query: the Kalman estimate predicted to the current
        /// time when kalman_localization is enabled and the filter has an estimate, otherwise the last-received
        /// localization. Must be called under the mutex. Compiled into libprovizio_dds (no maths -> no Eigen).
        /// Exported because the header-inline get_points_ego_relative() calls it, so the symbol must be visible
        /// to consumer translation units (MSVC needs the explicit export; gcc/clang export it by default).
        PROVIZIO_DDS_API std::optional<rigid_transform> ego_localization_for_query() const;

        /// Validates and normalizes the options BEFORE any DDS resource is created (fail fast: no participant or
        /// subscribers are constructed for invalid configurations). Returns the validated options.
        static dds_accumulation_options validate(dds_accumulation_options options);

        dds_accumulation_options options;
        const bool no_localization;
        const bool no_extrinsics;

        mutable std::mutex mutex;
        point_clouds_accumulator accumulator;
        std::map<std::string, rigid_transform> extrinsics;
        std::optional<rigid_transform> latest_ego_localization;
        std::string localization_frame_id;
        // The localization extrinsics currently baked into latest_ego_localization, the accumulated frames and the
        // retained fixes. Starts identity (assumed until the localization frame's transform is received on
        // localization_extrinsics_topic); a change is reconciled by sync_localization_extrinsics, which
        // retroactively re-places already-accumulated data so nothing is lost.
        rigid_transform applied_localization_extrinsics;

        // NavSatFix localization state (all under the mutex): ENU conversion + yaw estimation from the position
        // history
        std::unique_ptr<detail::gps_utils> gps;
        std::deque<std::array<double, 3>> positions_history;
        double last_yaw{0};

        // Estimates the localization at point-cloud receive time (accessed under the mutex). Null when unused
        // (both kalman_localization and timesync_max_delay_seconds==0, or localization_source::none).
        std::unique_ptr<detail::localization_filter> localization_filter;

        // Timesync buffer entry: everything accumulate() needs, deferred until the covering localization arrives
        struct timesync_buffer_entry
        {
            std::string frame_id;
            std::vector<point_cloud2::radar_point> points;
            std::optional<rigid_transform> radar_extrinsics;
            double header_seconds{0};   ///< Cloud's own header (sensor-clock) timestamp in seconds
            double receive_seconds{0};  ///< Receive-time clock (time_source()) when the cloud arrived
        };

        // FIFO of pending clouds waiting for their covering localization (timesync active only; under the mutex)
        std::deque<timesync_buffer_entry> timesync_buffer;

        // The two most-recent localization fixes retained for bracketing interpolation (timesync active only;
        // under the mutex). A buffered cloud is released the instant a covering fix arrives, so the interpolation
        // bracket is always (previous_fix, latest_fix): the covering fix is latest_fix, and the prior fix
        // (header < cloud header) is previous_fix.
        struct localization_history_entry
        {
            double header_seconds{0};
            rigid_transform pose;
        };
        std::optional<localization_history_entry> previous_fix;
        std::optional<localization_history_entry> latest_fix;

        // Header-clock state (under the mutex).
        // localization_header_offset: last localization's header stamp minus its receive time; used by
        // ego_localization_for_query() to translate a receive-time "now" to the header clock domain. Valid after
        // the first localization arrives when timesync is active (and also when kalman_localization is enabled
        // for the query path).
        double localization_header_offset{0};
        bool localization_header_offset_valid{false};

        std::shared_ptr<domain_participant> participant;

        // Subscribers are the LAST members: destroyed first, and their destructors drain in-flight callbacks
        // (a provizio_dds subscriber guarantee), so the raw `this` captures can never touch destroyed state
        std::vector<std::shared_ptr<subscriber_handle<geometry_msgs::msg::TransformStampedPubSubType>>>
            extrinsics_subscribers;
        std::shared_ptr<subscriber_handle<nav_msgs::msg::OdometryPubSubType>> odometry_subscriber;
        std::shared_ptr<subscriber_handle<sensor_msgs::msg::NavSatFixPubSubType>> nav_sat_fix_subscriber;
        std::shared_ptr<subscriber_handle<sensor_msgs::msg::PointCloud2PubSubType>> pc2_subscriber;
    };
}  // namespace provizio::dds::accumulation

// The maths detail header requires the types declared above (its #error guard checks DDS_ACCUMULATION is already
// defined — i.e. it must be reached through this header); the inline definitions below need the maths. Hence this
// include sits between declarations and definitions by design.
#include "provizio/dds/detail/accumulation_math.h"

// Only the accumulated-points getters are defined below — everything else is compiled into libprovizio_dds
// (src/accumulation.cpp). These getters MUST stay header-inline: they are the call path into
// math::transform_points, whose Eigen/no-Eigen implementation choice binds in the consumer's translation unit.
namespace provizio::dds::accumulation
{
    inline std::vector<accumulated_point> point_clouds_accumulator::get_points(const matrix4x4 *to_ego_matrix) const
    {
        std::size_t total_points = 0;
        for (const auto &buffer : buffers)
        {
            for (const auto &frame : buffer.frames)
            {
                total_points += frame.points.size();
            }
        }

        std::vector<accumulated_point> result{total_points};
        std::size_t at = 0;
        for (const auto &buffer : buffers)
        {
            const matrix4x4 extrinsics_matrix = buffer.extrinsics ? buffer.extrinsics->matrix() : math::identity();
            for (const auto &frame : buffer.frames)
            {
                if (frame.points.empty())
                {
                    continue;
                }
                // ONE folded matrix per frame: [inv(ego_now)] * ego_at_receipt * extrinsics — folding keeps it a
                // single batch pass over the frame's contiguous points
                matrix4x4 to_target = math::multiply(frame.ego_localization_when_received.matrix(), extrinsics_matrix);
                if (to_ego_matrix != nullptr)
                {
                    to_target = math::multiply(*to_ego_matrix, to_target);
                }
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                math::transform_points(to_target, frame.points, result.data() + at);
                for (std::size_t i = 0; i < frame.points.size(); ++i)
                {
                    auto &point = result[at + i];
                    point.ground_relative_radial_velocity = frame.points[i].ground_relative_radial_velocity;
                    point.signal_to_noise_ratio = frame.points[i].signal_to_noise_ratio;
                    point.radar_position_id = buffer.radar_position_id;
                }
                at += frame.points.size();
            }
        }
        return result;
    }

    inline std::vector<accumulated_point> point_clouds_accumulator::get_points_local_frame_relative() const
    {
        return get_points(nullptr);
    }

    inline std::vector<accumulated_point> point_clouds_accumulator::get_points_ego_relative(
        const rigid_transform &ego_localization_now) const
    {
        const matrix4x4 to_ego_matrix = ego_localization_now.inversed_matrix();
        return get_points(&to_ego_matrix);
    }

    inline std::vector<accumulated_point> dds_point_clouds_accumulator::get_points_local_frame_relative() const
    {
        const std::lock_guard<std::mutex> lock{mutex};
        return accumulator.get_points_local_frame_relative();
    }

    inline std::vector<accumulated_point> dds_point_clouds_accumulator::get_points_ego_relative() const
    {
        const std::lock_guard<std::mutex> lock{mutex};
        const auto ego = ego_localization_for_query();
        if (!ego)
        {
            // Ego localization is required but not received yet
            return {};
        }
        return accumulator.get_points_ego_relative(*ego);
    }
}  // namespace provizio::dds::accumulation

#endif  // DDS_ACCUMULATION
