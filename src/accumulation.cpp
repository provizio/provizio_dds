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

// Non-template, Eigen-independent part of provizio/dds/accumulation.h. Deliberately does NOT call the accumulated
// points getters or math::transform_points: that maths path stays header-inline so the Eigen/no-Eigen choice binds
// in the CONSUMER's translation unit (see provizio/dds/detail/accumulation_math.h) and libprovizio_dds plus the bin
// cache stay Eigen-agnostic.

#include "provizio/dds/accumulation.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/TransformStampedPubSubTypes.hpp>
#include <nav_msgs/msg/OdometryPubSubTypes.hpp>
#include <sensor_msgs/msg/NavSatFixPubSubTypes.hpp>
#include <sensor_msgs/msg/PointCloud2PubSubTypes.hpp>

#include "detail/gps_utils.h"
#include "provizio/dds/detail/localization_filter.h"

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/subscriber.h"

namespace
{
    // Quaternion slerp: shortest-path interpolation between two unit quaternions at fraction frac in [0, 1].
    // Takes the short-way-around by negating q_b when dot(q_a, q_b) < 0. Falls back to normalized lerp when
    // the quaternions are nearly parallel (sin(theta) ~ 0) to avoid division by near-zero.
    provizio::dds::accumulation::quaternion_wxyz slerp_quaternion(
        const provizio::dds::accumulation::quaternion_wxyz &q_a, provizio::dds::accumulation::quaternion_wxyz q_b,
        const double frac) noexcept
    {
        using provizio::dds::accumulation::quaternion_wxyz;
        double dot = q_a.w * q_b.w + q_a.x * q_b.x + q_a.y * q_b.y + q_a.z * q_b.z;
        if (dot < 0.0)
        {
            // Take the short way around
            q_b.w = -q_b.w;
            q_b.x = -q_b.x;
            q_b.y = -q_b.y;
            q_b.z = -q_b.z;
            dot = -dot;
        }
        // Clamp for numerical safety before acos
        if (dot > 1.0)
        {
            dot = 1.0;
        }
        const double theta = std::acos(dot);
        const double sin_theta = std::sin(theta);
        // Below this sin(theta) the quaternions are treated as parallel and slerp degrades to normalized lerp
        constexpr double slerp_sin_epsilon = 1e-10;
        double weight_a = 0.0;
        double weight_b = 0.0;
        if (sin_theta > slerp_sin_epsilon)
        {
            weight_a = std::sin((1.0 - frac) * theta) / sin_theta;
            weight_b = std::sin(frac * theta) / sin_theta;
        }
        else
        {
            // Nearly parallel: normalized lerp
            weight_a = 1.0 - frac;
            weight_b = frac;
        }
        const double result_w = weight_a * q_a.w + weight_b * q_b.w;
        const double result_x = weight_a * q_a.x + weight_b * q_b.x;
        const double result_y = weight_a * q_a.y + weight_b * q_b.y;
        const double result_z = weight_a * q_a.z + weight_b * q_b.z;
        // Normalize the result
        const double norm =
            std::sqrt(result_w * result_w + result_x * result_x + result_y * result_y + result_z * result_z);
        // Below this norm the result is degenerate and slerp falls back to the identity quaternion
        constexpr double quaternion_norm_epsilon = 1e-12;
        if (norm > quaternion_norm_epsilon)
        {
            return quaternion_wxyz{result_w / norm, result_x / norm, result_y / norm, result_z / norm};
        }
        // Zero-norm fallback: identity
        return quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
    }

    // Linear interpolation between two ego poses at time target_time, bracketed by (t_a, pose_a) and (t_b, pose_b).
    // Position is lerped; orientation is slerped (shortest-path quaternion). When t_b == t_a, frac = 0 →
    // returns pose_a exactly.
    provizio::dds::accumulation::rigid_transform interpolate_pose(
        const double t_a, const provizio::dds::accumulation::rigid_transform &pose_a, const double t_b,
        const provizio::dds::accumulation::rigid_transform &pose_b, const double target_time) noexcept
    {
        using provizio::dds::accumulation::rigid_transform;
        const double frac = (t_b > t_a) ? std::max(0.0, std::min(1.0, (target_time - t_a) / (t_b - t_a))) : 0.0;
        const auto pos_a = pose_a.translation();
        const auto pos_b = pose_b.translation();
        const std::array<double, 3> pos{pos_a[0] + frac * (pos_b[0] - pos_a[0]),
                                        pos_a[1] + frac * (pos_b[1] - pos_a[1]),
                                        pos_a[2] + frac * (pos_b[2] - pos_a[2])};
        const auto rot = slerp_quaternion(pose_a.rotation(), pose_b.rotation(), frac);
        return rigid_transform{pos, rot};
    }
}  // namespace

namespace provizio::dds::accumulation
{
    rigid_transform::rigid_transform() noexcept : the_matrix(math::identity())
    {
    }

    rigid_transform::rigid_transform(const std::array<double, 3> &position,
                                     const std::array<double, 3> &rotation_euler_roll_pitch_yaw) noexcept
        : the_matrix(math::compose(position, rotation_euler_roll_pitch_yaw))
    {
    }

    rigid_transform::rigid_transform(const std::array<double, 3> &position, const quaternion_wxyz &rotation) noexcept
        : the_matrix(math::compose_quaternion(position, rotation))
    {
    }

    rigid_transform::rigid_transform(const matrix4x4 &from_matrix) noexcept : the_matrix(from_matrix)
    {
    }

    const matrix4x4 &rigid_transform::matrix() const noexcept
    {
        return the_matrix;
    }

    matrix4x4 rigid_transform::inversed_matrix() const noexcept
    {
        return math::rigid_inverse(the_matrix);
    }

    std::array<double, 3> rigid_transform::translation() const noexcept
    {
        return {the_matrix[0][3], the_matrix[1][3], the_matrix[2][3]};
    }

    quaternion_wxyz rigid_transform::rotation() const noexcept
    {
        return math::quaternion_from_matrix(the_matrix);
    }

    bool is_point_static(const point_cloud2::radar_point &point)
    {
        constexpr float dynamic_velocity_threshold_m_s = 2.0F;
        // NaN compares false => NaN ground velocity (optional field absent in the source cloud) means "drop"
        return std::abs(point.ground_relative_radial_velocity) < dynamic_velocity_threshold_m_s;
    }

    point_clouds_accumulator::point_clouds_accumulator(const std::size_t max_frames_per_radar,
                                                       accumulation_options options)
        : max_frames_per_radar(max_frames_per_radar), options(std::move(options))
    {
        if (max_frames_per_radar == 0)
        {
            throw std::invalid_argument{"point_clouds_accumulator: max_frames_per_radar can't be 0"};
        }
        if (this->options.max_frames_without_filter >= max_frames_per_radar)
        {
            // Every frame would be dropped before its filter is ever applied => the filter is a no-op; disable it
            // once here instead of re-checking per accumulate
            this->options.point_filter = {};
        }
    }

    point_clouds_accumulator::radar_buffer &point_clouds_accumulator::buffer_for(const std::string &radar_position_id)
    {
        const auto iter =
            std::find_if(buffers.begin(), buffers.end(), [&radar_position_id](const radar_buffer &buffer) {
                return *buffer.radar_position_id == radar_position_id;
            });
        if (iter != buffers.end())
        {
            return *iter;
        }
        buffers.push_back(radar_buffer{std::make_shared<const std::string>(radar_position_id), {}, std::nullopt});
        return buffers.back();
    }

    void point_clouds_accumulator::accumulate(const std::string &radar_position_id,
                                              std::vector<point_cloud2::radar_point> points,
                                              const rigid_transform &ego_localization_when_received,
                                              const std::optional<rigid_transform> &radar_extrinsics)
    {
        if (options.radar_filter && !options.radar_filter(radar_position_id))
        {
            // This radar is filtered out entirely
            return;
        }

        auto &buffer = buffer_for(radar_position_id);

        // Validate BEFORE buffering the frame: when accumulate throws, the accumulator state is untouched
        if (!radar_extrinsics && !options.allow_no_extrinsics && !buffer.extrinsics)
        {
            throw std::invalid_argument{
                "point_clouds_accumulator: allow_no_extrinsics is false so radar_extrinsics must be specified!"};
        }

        if (options.snr_threshold > 0)
        {
            points.erase(std::remove_if(points.begin(), points.end(),
                                        [this](const point_cloud2::radar_point &point) {
                                            return point.signal_to_noise_ratio < options.snr_threshold;
                                        }),
                         points.end());
        }

        if (options.point_filter)
        {
            const auto apply_point_filter = [this](std::vector<point_cloud2::radar_point> &filtered_points) {
                filtered_points.erase(std::remove_if(filtered_points.begin(), filtered_points.end(),
                                                     [this](const point_cloud2::radar_point &point) {
                                                         return !options.point_filter(point);
                                                     }),
                                      filtered_points.end());
            };
            if (options.max_frames_without_filter == 0)
            {
                // Filter immediately
                apply_point_filter(points);
            }
            else if (buffer.frames.size() >= options.max_frames_without_filter)
            {
                // The frame that now becomes max_frames_without_filter old gets re-filtered in place, while the new
                // incoming frame is intentionally left unfiltered until it ages to max_frames_without_filter frames
                // old — the deferred-filter scheme that briefly keeps moving objects accumulated
                apply_point_filter(buffer.frames[buffer.frames.size() - options.max_frames_without_filter].points);
            }
        }

        buffer.frames.push_back(accumulated_frame{std::move(points), ego_localization_when_received});
        if (buffer.frames.size() > max_frames_per_radar)
        {
            buffer.frames.pop_front();
        }

        if (radar_extrinsics)
        {
            buffer.extrinsics = *radar_extrinsics;
        }
    }

    rigid_transform point_clouds_accumulator::localization_from_sensor_to_ego_frame(
        const rigid_transform &sensor_localization, const rigid_transform &sensor_extrinsics)
    {
        return rigid_transform{math::multiply(sensor_localization.matrix(), sensor_extrinsics.inversed_matrix())};
    }

    void point_clouds_accumulator::apply_localization_correction(const rigid_transform &ego_pose_delta)
    {
        const auto &delta = ego_pose_delta.matrix();
        for (auto &buffer : buffers)
        {
            for (auto &frame : buffer.frames)
            {
                frame.ego_localization_when_received =
                    rigid_transform{math::multiply(frame.ego_localization_when_received.matrix(), delta)};
            }
        }
    }

    dds_accumulation_options dds_point_clouds_accumulator::validate(dds_accumulation_options options)
    {
        const bool is_none = options.localization == localization_source::none;
        if (is_none)
        {
            if (!options.localization_topic.empty() || !options.localization_frame_id.empty() ||
                !options.localization_extrinsics_topic.empty())
            {
                throw std::invalid_argument{
                    "dds_point_clouds_accumulator: with localization_source::none, localization_topic, "
                    "localization_frame_id and localization_extrinsics_topic must all be empty"};
            }
        }
        else
        {
            if (options.localization_topic.empty())
            {
                throw std::invalid_argument{
                    "dds_point_clouds_accumulator: localization_topic must be non-empty (or use "
                    "localization_source::none)"};
            }
            if (options.localization_extrinsics_topic.empty())
            {
                // Default: localization extrinsics share the radar extrinsics topic, so a frame id used by both a
                // radar and the localization source resolves the same radar->ego transform. Until that transform
                // is received the localization extrinsics is assumed identity (see sync_localization_extrinsics).
                options.localization_extrinsics_topic = "rt/provizio_extrinsics";
            }
            if (options.localization == localization_source::odometry && options.localization_frame_id.empty())
            {
                // Default: radar-based odometry is published in the front-center radar's frame, so it shares that
                // radar's extrinsics. nav_sat_fix keeps "any" (empty): its sensor frame differs from the radar
                // frames and is learned from the first fix.
                options.localization_frame_id = "provizio_radar_front_center";
            }
        }
        if (!options.time_source)
        {
            options.time_source = [] {
                return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            };
        }
        return options;
    }

    dds_point_clouds_accumulator::dds_point_clouds_accumulator(const std::size_t max_frames_per_radar,
                                                               dds_accumulation_options in_options,
                                                               std::shared_ptr<domain_participant> in_participant)
        : options(validate(std::move(in_options))), no_localization(options.localization == localization_source::none),
          no_extrinsics(options.extrinsics_topics.empty()),
          accumulator(max_frames_per_radar,
                      [this] {
                          // allow_no_extrinsics is DERIVED, not user-set: no extrinsics topics
                          // means the radar frame is taken to be the ego frame
                          auto accumulation_options_copy = options.accumulation;
                          accumulation_options_copy.allow_no_extrinsics = no_extrinsics;
                          return accumulation_options_copy;
                      }()),
          participant(in_participant != nullptr ? std::move(in_participant) : make_domain_participant())
    {
        if (no_localization)
        {
            // Always the same localization at (0, 0, 0), (0, 0, 0)
            latest_ego_localization = rigid_transform{};
        }

        localization_frame_id = options.localization_frame_id;

        if ((options.kalman_localization || options.timesync_max_delay_seconds > 0.0) && !no_localization)
        {
            localization_filter = std::make_unique<detail::localization_filter>();
        }

        // Subscribe: extrinsics first, then localization, then point clouds last — a cloud arriving before its
        // prerequisites is simply skipped by on_pc2_message
        auto extrinsics_topics = options.extrinsics_topics;
        if (!options.localization_extrinsics_topic.empty() &&
            std::find(extrinsics_topics.begin(), extrinsics_topics.end(), options.localization_extrinsics_topic) ==
                extrinsics_topics.end())
        {
            extrinsics_topics.push_back(options.localization_extrinsics_topic);
        }
        extrinsics_subscribers.reserve(extrinsics_topics.size());
        for (const auto &topic : extrinsics_topics)
        {
            extrinsics_subscribers.push_back(make_subscriber<geometry_msgs::msg::TransformStampedPubSubType>(
                participant, topic,
                [this](const geometry_msgs::msg::TransformStamped &transform) { on_extrinsics_message(transform); }));
        }

        switch (options.localization)
        {
        case localization_source::odometry:
            odometry_subscriber = make_subscriber<nav_msgs::msg::OdometryPubSubType>(
                participant, options.localization_topic,
                [this](const nav_msgs::msg::Odometry &odometry) { on_odometry_message(odometry); });
            break;
        case localization_source::nav_sat_fix:
            nav_sat_fix_subscriber = make_subscriber<sensor_msgs::msg::NavSatFixPubSubType>(
                participant, options.localization_topic,
                [this](const sensor_msgs::msg::NavSatFix &fix) { on_nav_sat_fix_message(fix); });
            break;
        case localization_source::none:
            break;
        }

        pc2_subscriber = make_subscriber<sensor_msgs::msg::PointCloud2PubSubType>(
            participant, options.pointcloud2_topic,
            [this](const sensor_msgs::msg::PointCloud2 &cloud) { on_pc2_message(cloud); });
    }

    dds_point_clouds_accumulator::~dds_point_clouds_accumulator() = default;

    void dds_point_clouds_accumulator::on_pc2_message(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        std::size_t released_count = 0;
        {
            const std::lock_guard<std::mutex> lock{mutex};
            if (!latest_ego_localization)
            {
                // No localization yet: skip until there is localization info
                return;
            }
            const std::string &frame_id = cloud.header().frame_id();
            std::optional<rigid_transform> radar_extrinsics;
            if (!no_extrinsics)
            {
                const auto found = extrinsics.find(frame_id);
                if (found == extrinsics.end())
                {
                    // No extrinsics yet for this radar: skip until there is extrinsics info
                    return;
                }
                radar_extrinsics = found->second;
            }
            if (options.timesync_max_delay_seconds > 0.0)
            {
                // Buffer the cloud; flush releases it once its covering localization arrives (or on timeout)
                const double now = options.time_source();
                const double header_s = cloud.header().stamp().sec() + cloud.header().stamp().nanosec() * 1e-9;
                timesync_buffer.push_back(
                    {frame_id, point_cloud2::read_radar_points(cloud), radar_extrinsics, header_s, now});
                released_count = flush_timesync_buffer(now);
            }
            else
            {
                rigid_transform localization = *latest_ego_localization;
                if (localization_filter && localization_filter->has_estimate())
                {
                    const double now = options.time_source();
                    if (options.kalman_localization)
                    {
                        localization = localization_filter->predict(now);
                    }
                }
                accumulator.accumulate(frame_id, point_cloud2::read_radar_points(cloud), localization,
                                       radar_extrinsics);
                released_count = 1;
            }
        }
        // Outside the lock on purpose, so get_points_local_frame_relative / get_points_ego_relative can be
        // called in the callback. Fire once per released cloud.
        if (options.on_point_cloud)
        {
            for (std::size_t i = 0; i < released_count; ++i)
            {
                options.on_point_cloud(*this);
            }
        }
    }

    void dds_point_clouds_accumulator::on_extrinsics_message(const geometry_msgs::msg::TransformStamped &transform)
    {
        const std::lock_guard<std::mutex> lock{mutex};
        const std::string &frame_id = transform.child_frame_id();
        const auto &translation = transform.transform().translation();
        const auto &rotation = transform.transform().rotation();
        extrinsics.insert_or_assign(frame_id,
                                    rigid_transform{{translation.x(), translation.y(), translation.z()},
                                                    {rotation.w(), rotation.x(), rotation.y(), rotation.z()}});
        // The newly-received transform may be the localization frame's: reconcile and retroactively re-place the
        // already-accumulated data if the localization extrinsics just changed (e.g. arrived after being assumed
        // identity). No-op for any other (radar-only) frame.
        sync_localization_extrinsics();
    }

    void dds_point_clouds_accumulator::on_odometry_message(const nav_msgs::msg::Odometry &odometry)
    {
        std::size_t released_count = 0;
        {
            const std::lock_guard<std::mutex> lock{mutex};

            // Multiple odometry sources can share one topic; the desired one is selected by its child_frame_id
            // (the moving frame the odometry describes). Drop any fix from a different frame than the configured
            // localization frame (preset for odometry — see validate).
            if (odometry.child_frame_id() != localization_frame_id)
            {
                return;
            }

            // Under timesync: compute the message's header time up front so the staleness guard runs
            // before any state is mutated. The same header_s is reused in the filter-update block below.
            const double header_s = (options.timesync_max_delay_seconds > 0.0)
                                        ? odometry.header().stamp().sec() + odometry.header().stamp().nanosec() * 1e-9
                                        : 0.0;
            if (options.timesync_max_delay_seconds > 0.0 && latest_fix && header_s < latest_fix->header_seconds)
            {
                // Out-of-order fix (older than the latest received) — ignore to keep the fix stream monotonic.
                return;
            }

            // Reconcile the localization extrinsics (assumed identity until the localization frame's transform is
            // received); a change retroactively re-places already-accumulated data so nothing is lost.
            sync_localization_extrinsics();

            const auto &pose = odometry.pose().pose();
            const auto &position = pose.position();
            const auto &orientation = pose.orientation();
            latest_ego_localization = point_clouds_accumulator::localization_from_sensor_to_ego_frame(
                rigid_transform{{position.x(), position.y(), position.z()},
                                {orientation.w(), orientation.x(), orientation.y(), orientation.z()}},
                applied_localization_extrinsics);
            if (localization_filter)
            {
                const double now = options.time_source();
                if (options.timesync_max_delay_seconds > 0.0)
                {
                    localization_header_offset = header_s - now;
                    localization_header_offset_valid = true;
                    localization_filter->update(header_s, *latest_ego_localization);
                    previous_fix = latest_fix;
                    latest_fix = localization_history_entry{header_s, *latest_ego_localization};
                    released_count = flush_timesync_buffer(now);
                }
                else
                {
                    localization_filter->update(now, *latest_ego_localization);
                }
            }
        }
        // Fire on_point_cloud outside the lock for every cloud released from the timesync buffer
        if (options.on_point_cloud)
        {
            for (std::size_t i = 0; i < released_count; ++i)
            {
                options.on_point_cloud(*this);
            }
        }
    }

    void dds_point_clouds_accumulator::on_nav_sat_fix_message(const sensor_msgs::msg::NavSatFix &fix)
    {
        std::size_t released_count = 0;
        {
            const std::lock_guard<std::mutex> lock{mutex};

            // nav_sat_fix learns its localization frame from the first fix (default "any"); multiple GNSS sources
            // can share one topic, so fixes from any other frame are dropped.
            if (localization_frame_id.empty())
            {
                localization_frame_id = fix.header().frame_id();
            }
            if (fix.header().frame_id() != localization_frame_id)
            {
                return;
            }

            // Under timesync: compute the message's header time up front so the staleness guard runs
            // before any state is mutated. The same header_s is reused in the filter-update block below.
            const double header_s = (options.timesync_max_delay_seconds > 0.0)
                                        ? fix.header().stamp().sec() + fix.header().stamp().nanosec() * 1e-9
                                        : 0.0;
            if (options.timesync_max_delay_seconds > 0.0 && latest_fix && header_s < latest_fix->header_seconds)
            {
                // Out-of-order fix (older than the latest received) — ignore to keep the fix stream monotonic.
                return;
            }

            const double latitude = fix.latitude();
            const double longitude = fix.longitude();
            if (std::isnan(latitude) || std::isnan(longitude))
            {
                // A NaN fix (e.g. a GPS sample emitted before fix-lock) would permanently poison the ENU origin
                // (set once, from the first fix) and every geo_to_enu thereafter — drop it, keep the prior estimate.
                return;
            }
            const double altitude = std::isnan(fix.altitude()) ? 0.0 : fix.altitude();

            if (!gps)
            {
                gps = std::make_unique<detail::gps_utils>();
                // The ENU origin is set on the Earth spheroid (height 0); altitude is applied to the position
                // separately
                gps->set_enu_origin(latitude, longitude, 0);
            }

            // Reconcile the localization extrinsics (assumed identity until the localization frame's transform is
            // received — a GNSS frame typically has none, so it stays at identity / localization frame = ego
            // frame). Run before estimate_yaw, which re-seeds the (possibly just-cleared) position history.
            sync_localization_extrinsics();

            const auto enu = gps->geo_to_enu(latitude, longitude, 0);
            const auto ego_localization = point_clouds_accumulator::localization_from_sensor_to_ego_frame(
                // Orientation is estimated below from the history of positions (NavSatFix has no heading)
                rigid_transform{{enu[0], enu[1], altitude}, std::array<double, 3>{0, 0, 0}},
                applied_localization_extrinsics);
            const auto ego_position = ego_localization.translation();
            const double yaw = estimate_yaw(ego_position);
            latest_ego_localization = rigid_transform{ego_position, std::array<double, 3>{0, 0, yaw}};
            if (localization_filter)
            {
                const double now = options.time_source();
                if (options.timesync_max_delay_seconds > 0.0)
                {
                    localization_header_offset = header_s - now;
                    localization_header_offset_valid = true;
                    localization_filter->update(header_s, *latest_ego_localization);
                    previous_fix = latest_fix;
                    latest_fix = localization_history_entry{header_s, *latest_ego_localization};
                    released_count = flush_timesync_buffer(now);
                }
                else
                {
                    localization_filter->update(now, *latest_ego_localization);
                }
            }
        }
        // Fire on_point_cloud outside the lock for every cloud released from the timesync buffer
        if (options.on_point_cloud)
        {
            for (std::size_t i = 0; i < released_count; ++i)
            {
                options.on_point_cloud(*this);
            }
        }
    }

    std::size_t dds_point_clouds_accumulator::flush_timesync_buffer(const double now)
    {
        // Mutex is already held by the caller. Iterate the whole buffer front-to-back so out-of-order entries
        // (rare: same radar, late delivery) are not blocked by an earlier un-covered entry.
        std::size_t released = 0;
        for (auto iter = timesync_buffer.begin(); iter != timesync_buffer.end();)
        {
            const bool covered = localization_filter && localization_filter->has_estimate() && latest_fix &&
                                 latest_fix->header_seconds >= iter->header_seconds;
            const bool timed_out = (now - iter->receive_seconds) >= options.timesync_max_delay_seconds;

            if (!covered && !timed_out)
            {
                ++iter;
                continue;
            }

            // Move the heavy points vector out of the entry before erasing (avoids a copy).
            timesync_buffer_entry released_entry = std::move(*iter);
            iter = timesync_buffer.erase(iter);

            // Determine the ego pose at the cloud's capture time (released_entry.header_seconds = cloud_time).
            // Priority:
            //   1. Covered (latest_fix->header_seconds >= cloud_time): interpolate when cloud_time falls between
            //      previous_fix and latest_fix; use previous_fix when the cloud is older than the last-but-one kept
            //      fix (badly out-of-order delivery); fall back to latest_fix only at startup (single fix received).
            //   2. Kalman forward-extrapolation when beyond the latest known fix (timeout release).
            //   3. Latest raw localization as a last resort.
            const double cloud_time = released_entry.header_seconds;

            std::optional<rigid_transform> localization;
            if (latest_fix && latest_fix->header_seconds >= cloud_time)
            {
                // Covered: latest_fix is the upper bracket.
                if (previous_fix && previous_fix->header_seconds <= cloud_time)
                {
                    // Both brackets present: interpolate (collapses to exact fix when a fix sits at cloud_time).
                    localization = interpolate_pose(previous_fix->header_seconds, previous_fix->pose,
                                                    latest_fix->header_seconds, latest_fix->pose, cloud_time);
                }
                else if (previous_fix)
                {
                    // Cloud is older than the last-but-one fix (badly out-of-order delivery): use
                    // previous_fix, the closer (older) of the two retained fixes.
                    localization = previous_fix->pose;
                }
                else
                {
                    // Startup edge: only one fix has been received so far; use it directly.
                    localization = latest_fix->pose;
                }
            }
            else if (localization_filter && localization_filter->has_estimate())
            {
                // Beyond the latest fix (timeout release): Kalman forward-extrapolation to cloud_time.
                localization = localization_filter->predict(cloud_time);
            }
            else if (latest_ego_localization)
            {
                localization = latest_ego_localization;
            }
            // else: no localization available — drop (matches the pre-localization skip in on_pc2_message)

            if (localization)
            {
                accumulator.accumulate(released_entry.frame_id, std::move(released_entry.points), *localization,
                                       released_entry.radar_extrinsics);
                ++released;
            }
        }

        return released;
    }

    double dds_point_clouds_accumulator::estimate_yaw(const std::array<double, 3> &current_ego_enu_position)
    {
        constexpr std::size_t history_depth = 3;
        constexpr double moved_far_enough_squared = 0.05;

        std::array<double, 3> past_position{};
        if (!positions_history.empty())
        {
            past_position = positions_history.front();
        }
        else
        {
            past_position = current_ego_enu_position;
            positions_history.push_back(current_ego_enu_position);
        }

        const double moved_east = current_ego_enu_position[0] - past_position[0];
        const double moved_north = current_ego_enu_position[1] - past_position[1];  // up ignored

        if (moved_east * moved_east + moved_north * moved_north > moved_far_enough_squared)
        {
            // Moved far enough to estimate orientation
            last_yaw = std::atan2(moved_north, moved_east);
            positions_history.push_back(current_ego_enu_position);
            while (positions_history.size() > history_depth)
            {
                positions_history.pop_front();
            }
        }
        return last_yaw;
    }

    std::optional<rigid_transform> dds_point_clouds_accumulator::ego_localization_for_query() const
    {
        if (localization_filter && localization_filter->has_estimate())
        {
            if (options.timesync_max_delay_seconds > 0.0 && localization_header_offset_valid)
            {
                return localization_filter->predict(options.time_source() + localization_header_offset);
            }
            if (options.kalman_localization)
            {
                return localization_filter->predict(options.time_source());
            }
        }
        return latest_ego_localization;
    }

    void dds_point_clouds_accumulator::sync_localization_extrinsics()
    {
        if (localization_frame_id.empty())
        {
            // nav_sat_fix before its first fix: the localization frame is not known yet.
            return;
        }

        const auto found = extrinsics.find(localization_frame_id);
        // Assumed identity until the localization frame's transform is received.
        const rigid_transform current = (found != extrinsics.end()) ? found->second : rigid_transform{};
        if (current.matrix() == applied_localization_extrinsics.matrix())
        {
            // Unchanged (still identity, or the same transform re-published): nothing to re-place.
            return;
        }

        // The ego pose computed under the previously-applied extrinsics is sensor_pose * E_prev^-1; under the new
        // one it is sensor_pose * E_new^-1. So mapping every already-placed ego pose from the former to the latter
        // is a single right-multiply by delta = E_prev * E_new^-1.
        const auto delta_matrix = math::multiply(applied_localization_extrinsics.matrix(), current.inversed_matrix());
        const rigid_transform delta{delta_matrix};

        // Retroactively re-place everything already accumulated (no points dropped or resampled) ...
        accumulator.apply_localization_correction(delta);
        // ... and the retained ego poses used by the cloud-placement and timesync-interpolation paths.
        if (latest_ego_localization)
        {
            latest_ego_localization = rigid_transform{math::multiply(latest_ego_localization->matrix(), delta_matrix)};
        }
        if (latest_fix)
        {
            latest_fix->pose = rigid_transform{math::multiply(latest_fix->pose.matrix(), delta_matrix)};
        }
        if (previous_fix)
        {
            previous_fix->pose = rigid_transform{math::multiply(previous_fix->pose.matrix(), delta_matrix)};
        }

        // The transient estimators were built from poses in the old extrinsics and do not transform cleanly under a
        // rigid right-multiply (the decomposed per-channel Kalman pose+rate, and the yaw-from-position history), so
        // they are dropped and re-converge from subsequent fixes. The accumulated data is corrected above, so
        // nothing accumulated is lost.
        if (localization_filter)
        {
            localization_filter->reset();
        }
        positions_history.clear();
        last_yaw = 0;

        applied_localization_extrinsics = current;
    }
}  // namespace provizio::dds::accumulation
