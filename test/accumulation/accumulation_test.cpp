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

// Subcommand-driven tests for provizio::dds::accumulation — a C++ mirror of
// test/python/accumulation_test.py whose accumulation golden constants trace
// back to provizio_radar_api_core (proven to match the APT GUI implementation);
// the rigid-transform goldens are computed with transforms3d, the library the
// Python implementation uses. Each ctest entry runs one subcommand so
// per-case failure stays isolated, mirroring the match_publisher_default test.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "provizio/dds/accumulation.h"

// Src-private detail header — the gps maths is deliberately NOT supported public API (not even installed), but its
// correctness is golden-tested directly
#include "../../src/detail/gps_utils.h"

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/publisher.h"

#include <geometry_msgs/msg/TransformStampedPubSubTypes.hpp>
#include <nav_msgs/msg/OdometryPubSubTypes.hpp>
#include <sensor_msgs/msg/NavSatFixPubSubTypes.hpp>
#include <sensor_msgs/msg/PointCloud2PubSubTypes.hpp>

namespace
{
    namespace acc = provizio::dds::accumulation;
    namespace pc2 = provizio::dds::point_cloud2;

    constexpr double pi = 3.14159265358979323846;
    // The Python tests use 0.0001 with float64 maths throughout; the C++ per-point batch deliberately runs in
    // float32 (see detail/accumulation_math.h), so at the synthetic test coordinate magnitudes (up to ~3 km of
    // ego translation) representation error alone reaches ~5e-4. 1e-3 keeps the checks meaningful (real radar
    // ranges of <= 300 m stay sub-millimetre) while not flaking on float32 rounding.
    constexpr double default_precision = 0.001;

    // All participants in one test process share this domain, chosen once at random within the DDS-safe range and
    // away from 0. These tests integrate every sample they receive on the topics they subscribe to — including the
    // default localization-extrinsics topic rt/provizio_extrinsics. Provizio's self-hosted CI includes real radar
    // boards whose resident software publishes on that standard topic on the default domain; that extrinsics shifts
    // every accumulated ego pose by a constant and corrupts the tests. Loopback confinement cannot exclude a
    // publisher on the same board, so a per-process domain is needed to give each test its own discovery space
    // (it also isolates against any concurrent run on another host).
    const auto test_domain = [] {
        std::random_device random_device;
        std::uniform_int_distribution<int> distribution(1, 200);  // DDS-safe range, excluding domain 0
        return distribution(random_device);
    }();

    double radians(const double degrees)
    {
        return degrees * pi / 180.0;
    }

    void check(const bool condition, const std::string &message)
    {
        if (!condition)
        {
            throw std::runtime_error{message};
        }
    }

    void check_near(const double actual, const double expected, const double tolerance, const std::string &message)
    {
        if (!(std::abs(actual - expected) <= tolerance))
        {
            std::ostringstream stream;
            stream << message << ": expected " << expected << ", got " << actual << " (tolerance " << tolerance << ")";
            throw std::runtime_error{stream.str()};
        }
    }

    void check_matrix_near(const acc::matrix4x4 &actual, const acc::matrix4x4 &expected, const double tolerance,
                           const std::string &message)
    {
        for (std::size_t row = 0; row < 4; ++row)
        {
            for (std::size_t column = 0; column < 4; ++column)
            {
                check_near(actual[row][column], expected[row][column], tolerance,
                           message + " [" + std::to_string(row) + "][" + std::to_string(column) + "]");
            }
        }
    }

    void test_rigid_transform_golden()
    {
        constexpr double tolerance = 1e-9;

        // Identity by default
        const acc::rigid_transform identity_transform;
        check_matrix_near(identity_transform.matrix(),
                          acc::matrix4x4{{{{1, 0, 0, 0}}, {{0, 1, 0, 0}}, {{0, 0, 1, 0}}, {{0, 0, 0, 1}}}}, 0,
                          "rigid_transform: identity");

        // Golden values computed with Python transforms3d (the library the Python implementation uses):
        // compose_matrix(angles=(0.1, 0.2, 0.3), translate=(1, 2, 3)) — 'sxyz' convention
        acc::matrix4x4 expected{};
        expected[0] = {0.936293363584, -0.275095847318, 0.218350663146, 1.0};
        expected[1] = {0.289629477626, 0.956425085849, -0.036957013525, 2.0};
        expected[2] = {-0.198669330795, 0.097843395007, 0.975170327202, 3.0};
        expected[3] = {0.0, 0.0, 0.0, 1.0};

        const acc::rigid_transform euler_transform{{1, 2, 3}, {0.1, 0.2, 0.3}};
        check_matrix_near(euler_transform.matrix(), expected, tolerance, "rigid_transform: euler compose");

        // The same rotation as a quaternion (w, x, y, z) = transforms3d euler2quat(0.1, 0.2, 0.3)
        const acc::rigid_transform quaternion_transform{
            {1, 2, 3}, {0.983347443256, 0.03427079855, 0.106020511062, 0.143572175027}};
        check_matrix_near(quaternion_transform.matrix(), expected, 1e-8, "rigid_transform: quaternion compose");

        // rotation() recovers the quaternion; translation() the translation
        const auto rotation = euler_transform.rotation();
        check_near(rotation.w, 0.983347443256, 1e-9, "rigid_transform: rotation w");
        check_near(rotation.x, 0.03427079855, 1e-9, "rigid_transform: rotation x");
        check_near(rotation.y, 0.106020511062, 1e-9, "rigid_transform: rotation y");
        check_near(rotation.z, 0.143572175027, 1e-9, "rigid_transform: rotation z");
        const auto translation = euler_transform.translation();
        check(translation[0] == 1.0 && translation[1] == 2.0 && translation[2] == 3.0, "rigid_transform: translation");

        // Inverse golden (transforms3d inverse_matrix of the matrix above)
        acc::matrix4x4 expected_inverse{};
        expected_inverse[0] = {0.936293363584, 0.289629477626, -0.198669330795, -0.91954432645};
        expected_inverse[1] = {-0.275095847318, 0.956425085849, 0.097843395007, -1.931284509402};
        expected_inverse[2] = {0.218350663146, -0.036957013525, 0.975170327202, -3.069947617703};
        expected_inverse[3] = {0.0, 0.0, 0.0, 1.0};
        check_matrix_near(euler_transform.inversed_matrix(), expected_inverse, tolerance, "rigid_transform: inverse");

        // from_matrix roundtrip
        const acc::rigid_transform from_matrix_transform{expected};
        check_matrix_near(from_matrix_transform.matrix(), expected, 0, "rigid_transform: from_matrix");

        // Near-180-degree rotations exercise all four Shepperd branches of quaternion_from_matrix: rotation about
        // x/y/z by ~pi each plus a composite. Each must roundtrip matrix -> quaternion -> matrix exactly.
        const std::array<std::array<double, 3>, 4> near_pi_rotations{
            {{pi - 0.001, 0, 0}, {0, pi - 0.001, 0}, {0, 0, pi - 0.001}, {pi - 0.001, 0.5, pi - 0.5}}};
        for (const auto &rotation_roll_pitch_yaw : near_pi_rotations)
        {
            const acc::rigid_transform original{{0, 0, 0}, rotation_roll_pitch_yaw};
            const auto quaternion = original.rotation();
            const acc::rigid_transform roundtripped{{0, 0, 0}, quaternion};
            check_matrix_near(roundtripped.matrix(), original.matrix(), 1e-12,
                              "rigid_transform: Shepperd roundtrip for near-pi rotation");
        }
    }

    using radar_points = std::vector<pc2::radar_point>;

    const acc::rigid_transform identity_transform{};

    /// Verifies an accumulated point's metadata (snr/velocity/radar id) against its input point
    void check_point_meta(const acc::accumulated_point &accumulated, const pc2::radar_point &input,
                          const std::string &radar_id, const std::string &message)
    {
        check(accumulated.signal_to_noise_ratio == input.signal_to_noise_ratio, message + ": snr");
        check(accumulated.ground_relative_radial_velocity == input.ground_relative_radial_velocity,
              message + ": ground_relative_radial_velocity");
        check(accumulated.radar_id() == radar_id, message + ": radar_id");
    }

    acc::accumulation_options no_filters_options(const float snr_threshold = 0, const bool allow_no_extrinsics = true)
    {
        acc::accumulation_options options;
        options.snr_threshold = snr_threshold;
        options.point_filter = {};
        options.allow_no_extrinsics = allow_no_extrinsics;
        return options;
    }

    // Mirrors Python test_accumulate_0_accumulated_point_clouds
    void test_zero_accumulated()
    {
        acc::point_clouds_accumulator accumulator{2, no_filters_options()};
        check(accumulator.get_points_local_frame_relative().empty(), "zero: local frame not empty");
        check(accumulator.get_points_ego_relative(acc::rigid_transform{{0, 0, 0}, {0, 0, 0}}).empty(),
              "zero: ego relative not empty");
    }

    // Mirrors Python test_accumulate_extrinsics
    void test_extrinsics()
    {
        const std::string radar_id{"test_radar"};
        // 10 meters left, 2 up, looking left
        const acc::rigid_transform extrinsics{{0, 10, 2}, {0, 0, radians(90)}};
        // ego relative [1, 2, 3, 4, 5, 6] as seen by test_radar
        const pc2::radar_point point{-8, -1, 1, 4, 5, 6};

        acc::point_clouds_accumulator accumulator{1, no_filters_options(0, false)};
        accumulator.accumulate(radar_id, radar_points{point}, identity_transform, extrinsics);
        const auto points_local_frame = accumulator.get_points_local_frame_relative();
        const auto points_ego = accumulator.get_points_ego_relative(identity_transform);

        check(points_local_frame.size() == 1, "extrinsics: local size");
        check(points_ego.size() == 1, "extrinsics: ego size");
        check_near(points_ego[0].position[0], 1, default_precision, "extrinsics: x");
        check_near(points_ego[0].position[1], 2, default_precision, "extrinsics: y");
        check_near(points_ego[0].position[2], 3, default_precision, "extrinsics: z");
        for (std::size_t dim = 0; dim < 3; ++dim)
        {
            check(points_local_frame[0].position[dim] == points_ego[0].position[dim],
                  "extrinsics: local == ego with identity localization");
        }
    }

    // Mirrors Python test_accumulate_move_no_extrinsics
    void test_move_no_extrinsics()
    {
        acc::point_clouds_accumulator accumulator{2, no_filters_options()};

        const std::string radar_id{"test_radar"};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        const std::array<double, 3> ego_pos_0{1, 2, 3};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};
        const std::array<double, 3> ego_pos_1{6, 5, 4};
        const std::array<double, 3> ego_pos_now{1000, 2000, 3000};

        accumulator.accumulate(radar_id, radar_points{point_0}, acc::rigid_transform{ego_pos_0, {0, 0, 0}});
        accumulator.accumulate(radar_id, radar_points{point_1, point_2}, acc::rigid_transform{ego_pos_1, {0, 0, 0}});

        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "move_no_extrinsics: local size");
        const auto ego = accumulator.get_points_ego_relative(acc::rigid_transform{ego_pos_now, {0, 0, 0}});
        check(ego.size() == 3, "move_no_extrinsics: ego size");

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        // Last ego_pos_1 is not a typo: point_1 and point_2 share the same ego pose
        const std::array<std::array<double, 3>, 3> ego_poses{ego_pos_0, ego_pos_1, ego_pos_1};

        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points[pt].x, input_points[pt].y, input_points[pt].z};
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(local[pt].position[dim], xyz[dim] + ego_poses[pt][dim], default_precision,
                           "move_no_extrinsics: local position");
                check_near(ego[pt].position[dim], xyz[dim] + ego_poses[pt][dim] - ego_pos_now[dim], default_precision,
                           "move_no_extrinsics: ego position");
            }
            check_point_meta(local[pt], input_points[pt], radar_id, "move_no_extrinsics local");
            check_point_meta(ego[pt], input_points[pt], radar_id, "move_no_extrinsics ego");
        }
    }

    // Mirrors Python test_accumulate_move_simple_extrinsics (two radars, translation-only extrinsics)
    void test_move_simple_extrinsics()
    {
        acc::point_clouds_accumulator accumulator{2, no_filters_options(0, false)};

        const std::string radar_id_0{"test_radar_0"};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        const std::array<double, 3> ego_pos_0{1, 2, 3};
        const std::array<double, 3> extrinsics_pos_0{0.1, 0.2, 0.3};
        const std::string radar_id_1{"test_radar_1"};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};
        const std::array<double, 3> ego_pos_1{6, 5, 4};
        const std::array<double, 3> extrinsics_pos_1{-0.1, -0.2, -0.3};
        const std::array<double, 3> ego_pos_now{1000, 2000, 3000};

        accumulator.accumulate(radar_id_0, radar_points{point_0}, acc::rigid_transform{ego_pos_0, {0, 0, 0}},
                               acc::rigid_transform{extrinsics_pos_0, {0, 0, 0}});
        accumulator.accumulate(radar_id_1, radar_points{point_1, point_2}, acc::rigid_transform{ego_pos_1, {0, 0, 0}},
                               acc::rigid_transform{extrinsics_pos_1, {0, 0, 0}});

        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "move_simple_extrinsics: local size");
        const auto ego = accumulator.get_points_ego_relative(acc::rigid_transform{ego_pos_now, {0, 0, 0}});
        check(ego.size() == 3, "move_simple_extrinsics: ego size");

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        const std::array<std::array<double, 3>, 3> ego_poses{ego_pos_0, ego_pos_1, ego_pos_1};
        const std::array<std::string, 3> radar_ids{radar_id_0, radar_id_1, radar_id_1};
        const std::array<std::array<double, 3>, 3> extrinsics{extrinsics_pos_0, extrinsics_pos_1, extrinsics_pos_1};

        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points[pt].x, input_points[pt].y, input_points[pt].z};
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(local[pt].position[dim], xyz[dim] + ego_poses[pt][dim] + extrinsics[pt][dim],
                           default_precision, "move_simple_extrinsics: local position");
                check_near(ego[pt].position[dim],
                           xyz[dim] + ego_poses[pt][dim] + extrinsics[pt][dim] - ego_pos_now[dim], default_precision,
                           "move_simple_extrinsics: ego position");
            }
            check_point_meta(local[pt], input_points[pt], radar_ids[pt], "move_simple_extrinsics local");
            check_point_meta(ego[pt], input_points[pt], radar_ids[pt], "move_simple_extrinsics ego");
        }

        // Extrinsics required but never provided => throws BEFORE any buffer is created, so the accumulator
        // (and the getters' first-accumulation ordering) stays untouched. Same in Python.
        acc::point_clouds_accumulator strict_accumulator{2, no_filters_options(0, false)};
        bool threw = false;
        try
        {
            strict_accumulator.accumulate("no_extrinsics_radar", radar_points{point_0}, identity_transform);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "move_simple_extrinsics: missing extrinsics did not throw");
        check(strict_accumulator.get_points_local_frame_relative().empty(),
              "move_simple_extrinsics: throwing accumulate must not buffer the frame");
        // The rejected call must NOT have registered a buffer for "no_extrinsics_radar". Accumulate a
        // different radar, then "no_extrinsics_radar" WITH extrinsics: the getters return radars in
        // first-accumulation order, so "later_radar" must come first — it would come second if the earlier
        // failed call had already created the "no_extrinsics_radar" buffer.
        const acc::rigid_transform zero_extrinsics{{0, 0, 0}, {0, 0, 0}};
        strict_accumulator.accumulate("later_radar", radar_points{point_0}, identity_transform, zero_extrinsics);
        strict_accumulator.accumulate("no_extrinsics_radar", radar_points{point_0}, identity_transform,
                                      zero_extrinsics);
        const auto ordered = strict_accumulator.get_points_local_frame_relative();
        check(ordered.size() == 2 && ordered.front().radar_position_id != nullptr &&
                  *ordered.front().radar_position_id == "later_radar",
              "move_simple_extrinsics: a rejected accumulate must not create a buffer that reorders later radars");
    }

    // Mirrors Python test_accumulate_overflow
    void test_overflow()
    {
        const std::string radar_id_0{"test_radar_0"};
        const std::string radar_id_1{"test_radar_1"};

        acc::point_clouds_accumulator accumulator{2, no_filters_options()};
        const pc2::radar_point point{1, 2, 3, 4, 5, 6};

        check(accumulator.get_points_local_frame_relative().empty(), "overflow: initially non-empty");

        accumulator.accumulate(radar_id_0, radar_points{point}, identity_transform);
        check(accumulator.get_points_local_frame_relative().size() == 1, "overflow: != 1");
        accumulator.accumulate(radar_id_0, radar_points{point, point}, identity_transform);
        check(accumulator.get_points_local_frame_relative().size() == 3, "overflow: != 3");
        // Now the very first frame gets dropped
        accumulator.accumulate(radar_id_0, radar_points{point, point, point}, identity_transform);
        check(accumulator.get_points_local_frame_relative().size() == 5, "overflow: != 5");
        // Another radar, so nothing gets dropped from the first one
        accumulator.accumulate(radar_id_1, radar_points{point, point, point, point}, identity_transform);
        check(accumulator.get_points_local_frame_relative().size() == 9, "overflow: != 9");
        accumulator.accumulate(radar_id_1, radar_points{point}, identity_transform);
        check(accumulator.get_points_local_frame_relative().size() == 10, "overflow: != 10");
        // Clear all by pushing empty frames twice to both radars
        for (const auto &radar_id : {radar_id_0, radar_id_1})
        {
            for (int i = 0; i < 2; ++i)
            {
                accumulator.accumulate(radar_id, radar_points{}, identity_transform);
            }
        }
        check(accumulator.get_points_local_frame_relative().empty(), "overflow: not empty after clearing");

        // max_frames_per_radar == 0 must throw (Python asserts)
        bool threw = false;
        try
        {
            const acc::point_clouds_accumulator invalid_accumulator{0, no_filters_options()};
            (void)invalid_accumulator;
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "overflow: max_frames_per_radar == 0 did not throw");
    }

    // Mirrors Python test_accumulate_move_simple_extrinsics_snr_filter
    void test_move_simple_extrinsics_snr_filter()
    {
        const std::array<float, 3> snr_thresholds{5, 6, 60};
        // For each threshold: how many of the 3 input points (snr 5, 50, 500) survive
        const std::array<std::size_t, 3> expected_kept{3, 2, 1};

        const std::string radar_id_0{"test_radar_0"};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        const std::array<double, 3> ego_pos_0{1, 2, 3};
        const std::array<double, 3> extrinsics_pos_0{0.1, 0.2, 0.3};
        const std::string radar_id_1{"test_radar_1"};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};
        const std::array<double, 3> ego_pos_1{6, 5, 4};
        const std::array<double, 3> extrinsics_pos_1{-0.1, -0.2, -0.3};

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        const std::array<std::array<double, 3>, 3> ego_poses{ego_pos_0, ego_pos_1, ego_pos_1};
        const std::array<std::array<double, 3>, 3> extrinsics{extrinsics_pos_0, extrinsics_pos_1, extrinsics_pos_1};

        for (std::size_t variant = 0; variant < snr_thresholds.size(); ++variant)
        {
            acc::accumulation_options options;
            options.snr_threshold = snr_thresholds[variant];
            options.point_filter = {};
            options.allow_no_extrinsics = false;
            acc::point_clouds_accumulator accumulator{2, options};

            accumulator.accumulate(radar_id_0, radar_points{point_0}, acc::rigid_transform{ego_pos_0, {0, 0, 0}},
                                   acc::rigid_transform{extrinsics_pos_0, {0, 0, 0}});
            accumulator.accumulate(radar_id_1, radar_points{point_1, point_2},
                                   acc::rigid_transform{ego_pos_1, {0, 0, 0}},
                                   acc::rigid_transform{extrinsics_pos_1, {0, 0, 0}});

            const auto accumulated = accumulator.get_points_local_frame_relative();
            check(accumulated.size() == expected_kept[variant],
                  "snr_filter: wrong number of points kept for threshold " + std::to_string(snr_thresholds[variant]));

            // SNR filtering removes the lowest-SNR points first here because the lowest-SNR point belongs to
            // radar_0, which accumulates before radar_1 (output is in first-accumulation radar order)
            const std::size_t filtered_out = 3 - accumulated.size();
            for (std::size_t pt = 0; pt < accumulated.size(); ++pt)
            {
                const auto &input = input_points[pt + filtered_out];
                const std::array<float, 3> xyz{input.x, input.y, input.z};
                for (std::size_t dim = 0; dim < 3; ++dim)
                {
                    check_near(accumulated[pt].position[dim],
                               xyz[dim] + ego_poses[pt + filtered_out][dim] + extrinsics[pt + filtered_out][dim],
                               default_precision, "snr_filter: position");
                }
            }
        }
    }

    // Mirrors Python test_accumulate_snr_and_velocity_filters
    void test_snr_and_velocity_filters()
    {
        const std::string radar_id{"test_radar"};
        const std::array<float, 4> snr_thresholds{0, 2.5F, 10, 100};
        const std::array<float, 5> snrs{1, 2.5F, 5, 10, 15};
        const std::array<float, 5> velocity_thresholds{0, 2, 5, 20, 100};
        const std::array<float, 6> velocities{0, 1, 2, 5, 15, 40};
        const std::array<std::size_t, 2> max_frames_without_filter_variants{0, 1};

        for (const float snr_threshold : snr_thresholds)
        {
            for (const float velocity_threshold : velocity_thresholds)
            {
                for (const std::size_t max_frames_without_filter : max_frames_without_filter_variants)
                {
                    acc::accumulation_options options;
                    options.snr_threshold = snr_threshold;
                    options.max_frames_without_filter = max_frames_without_filter;
                    options.point_filter = [velocity_threshold](const pc2::radar_point &point) {
                        return point.ground_relative_radial_velocity >= velocity_threshold;
                    };
                    options.allow_no_extrinsics = true;
                    acc::point_clouds_accumulator accumulator{2, options};

                    radar_points points;
                    radar_points expected_unfiltered;
                    radar_points expected_filtered;
                    for (const float snr : snrs)
                    {
                        for (const float velocity : velocities)
                        {
                            points.push_back(pc2::radar_point{1, 2, 3, 4, snr, velocity});
                            if (snr >= snr_threshold)
                            {
                                expected_unfiltered.push_back(points.back());
                                if (velocity >= velocity_threshold)
                                {
                                    expected_filtered.push_back(points.back());
                                }
                            }
                        }
                    }

                    check(accumulator.get_points_local_frame_relative().empty(), "filters: not initially empty");

                    accumulator.accumulate(radar_id, points, acc::rigid_transform{{5, 6, 7}, {8, 9, 10}});

                    if (max_frames_without_filter > 0)
                    {
                        // Nothing is point-filtered yet
                        const auto unfiltered = accumulator.get_points_local_frame_relative();
                        const std::string combination =
                            " (snr_threshold=" + std::to_string(snr_threshold) +
                            ", velocity_threshold=" + std::to_string(velocity_threshold) +
                            ", max_frames_without_filter=" + std::to_string(max_frames_without_filter) + ")";
                        check(unfiltered.size() == expected_unfiltered.size(),
                              "filters: unfiltered size" + combination);
                        for (std::size_t i = 0; i < unfiltered.size(); ++i)
                        {
                            check(unfiltered[i].ground_relative_radial_velocity ==
                                      expected_unfiltered[i].ground_relative_radial_velocity,
                                  "filters: unfiltered velocity");
                            check(unfiltered[i].signal_to_noise_ratio == expected_unfiltered[i].signal_to_noise_ratio,
                                  "filters: unfiltered snr");
                        }
                    }

                    // Accumulate empty frames until the filter is applied to the original frame
                    for (std::size_t after = 0; after < max_frames_without_filter; ++after)
                    {
                        accumulator.accumulate(radar_id, radar_points{}, identity_transform);
                    }

                    const auto filtered = accumulator.get_points_local_frame_relative();
                    const std::string combination =
                        " (snr_threshold=" + std::to_string(snr_threshold) +
                        ", velocity_threshold=" + std::to_string(velocity_threshold) +
                        ", max_frames_without_filter=" + std::to_string(max_frames_without_filter) + ")";
                    check(filtered.size() == expected_filtered.size(), "filters: filtered size" + combination);
                    for (std::size_t i = 0; i < filtered.size(); ++i)
                    {
                        check(filtered[i].ground_relative_radial_velocity ==
                                  expected_filtered[i].ground_relative_radial_velocity,
                              "filters: filtered velocity");
                        check(filtered[i].signal_to_noise_ratio == expected_filtered[i].signal_to_noise_ratio,
                              "filters: filtered snr");
                    }
                }
            }
        }
    }

    // Mirrors Python test_accumulate_radar_filter
    void test_radar_filter()
    {
        const std::string radar_id_1{"test_radar_1"};
        const std::string radar_id_2{"test_radar_2"};
        const std::string radar_id_3{"test_radar_3"};

        acc::accumulation_options options;
        options.snr_threshold = 0;
        options.max_frames_without_filter = 0;
        options.point_filter = {};
        options.radar_filter = [](const std::string &radar_id) { return radar_id.find('2') == std::string::npos; };
        options.allow_no_extrinsics = true;
        acc::point_clouds_accumulator accumulator{2, options};

        accumulator.accumulate(radar_id_1, radar_points{{1, 2, 3, 4, 5, 6}}, identity_transform);
        accumulator.accumulate(radar_id_2, radar_points{{7, 8, 9, 10, 11, 12}}, identity_transform);
        accumulator.accumulate(radar_id_3, radar_points{{13, 14, 15, 16, 17, 18}}, identity_transform);

        const auto accumulated = accumulator.get_points_local_frame_relative();
        check(accumulated.size() == 2, "radar_filter: size != 2");
        check(accumulated[0].radar_id() == radar_id_1, "radar_filter: [0] not radar_1");
        check(accumulated[1].radar_id() == radar_id_3, "radar_filter: [1] not radar_3");
    }

    // C++-only direct test of the static helper (Python covers it only indirectly via DDS tests)
    void test_localization_from_sensor_to_ego()
    {
        // Sensor mounted 0.3/0.5/0.7 from the ego origin, looking left (90 deg)
        const acc::rigid_transform sensor_extrinsics{{0.3, 0.5, 0.7}, {0, 0, radians(90)}};
        // Sensor's own localization reading
        const acc::rigid_transform sensor_localization{{10, 20, 30}, {0, 0, radians(90)}};
        const auto ego = acc::point_clouds_accumulator::localization_from_sensor_to_ego_frame(sensor_localization,
                                                                                              sensor_extrinsics);
        // ego = sensor_localization * inverse(sensor_extrinsics):
        // inverse(extrinsics) rotates by -90 and translates accordingly; the composition keeps yaw 0... Verify by
        // transforming the ego origin: ego.translation must equal sensor_localization applied to the inverse
        // extrinsics translation.
        const auto translation = ego.translation();
        // inverse(extrinsics).translation = -R^T*t = -(Rz(-90)*[0.3,0.5,0.7]) = [-0.5, 0.3, -0.7]
        // ego.translation = sensor_localization * [-0.5, 0.3, -0.7, 1] = [10,20,30] + Rz(90)*[-0.5,0.3,-0.7]
        //                 = [10 - 0.3, 20 - 0.5, 30 - 0.7] = [9.7, 19.5, 29.3]
        check_near(translation[0], 9.7, 1e-9, "localization_from_sensor_to_ego: x");
        check_near(translation[1], 19.5, 1e-9, "localization_from_sensor_to_ego: y");
        check_near(translation[2], 29.3, 1e-9, "localization_from_sensor_to_ego: z");
        // Rotation: Rz(90) * Rz(-90) = identity => quaternion w == 1
        check_near(ego.rotation().w, 1.0, 1e-9, "localization_from_sensor_to_ego: rotation w");
    }

    // The retroactive localization-extrinsics correction (apply_localization_correction): a frame accumulated
    // under the identity assumption and then corrected for a late-arriving extrinsics E must end up identical to
    // one accumulated with E known from the start — nothing dropped or resampled.
    void test_localization_correction()
    {
        const std::string radar_id{"test_radar"};
        const acc::rigid_transform radar_extrinsics{{1, 2, 3}, {0, 0, radians(30)}};
        const acc::rigid_transform sensor_localization{{10, 20, 5}, {0, 0, radians(45)}};
        const acc::rigid_transform localization_extrinsics{{0.3, 0.5, 0.7}, {0, 0, radians(90)}};
        const radar_points points{{1, 2, 3, 4, 5, 6}, {7, 8, 9, 10, 11, 12}};

        // Reference: the localization extrinsics is known from the start, so the ego pose is sensor * E^-1.
        acc::point_clouds_accumulator reference{4, no_filters_options()};
        reference.accumulate(radar_id, points,
                             acc::point_clouds_accumulator::localization_from_sensor_to_ego_frame(
                                 sensor_localization, localization_extrinsics),
                             radar_extrinsics);
        const auto expected = reference.get_points_local_frame_relative();

        // Under test: accumulate assuming identity localization extrinsics, then correct once E becomes known.
        acc::point_clouds_accumulator corrected{4, no_filters_options()};
        corrected.accumulate(radar_id, points,
                             acc::point_clouds_accumulator::localization_from_sensor_to_ego_frame(
                                 sensor_localization, acc::rigid_transform{}),
                             radar_extrinsics);
        // delta = E_prev * E_new^-1 = identity * E^-1 = E^-1
        corrected.apply_localization_correction(acc::rigid_transform{localization_extrinsics.inversed_matrix()});
        const auto actual = corrected.get_points_local_frame_relative();

        check(actual.size() == expected.size() && actual.size() == 2, "localization_correction: size");
        for (std::size_t pt = 0; pt < expected.size(); ++pt)
        {
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(actual[pt].position[dim], expected[pt].position[dim], default_precision,
                           "localization_correction: position matches extrinsics-known-from-start");
            }
        }
    }

    struct rotation_case
    {
        std::array<pc2::radar_point, 3> points;
        std::array<std::array<double, 3>, 3> orientations_roll_pitch_yaw;  // per-frame ego orientation
        std::optional<acc::rigid_transform> extrinsics;
        std::array<std::array<double, 3>, 3> expected_ego_relative;  // golden, for ego at
                                                                     // orientations_roll_pitch_yaw[2]
    };

    void run_rotation_case(const rotation_case &test_case, const std::string &name)
    {
        const std::string radar_id{"test_radar"};
        const std::array<double, 3> no_translation{0, 0, 0};

        acc::point_clouds_accumulator accumulator{3, no_filters_options(0, !test_case.extrinsics.has_value())};
        for (std::size_t i = 0; i < 3; ++i)
        {
            accumulator.accumulate(radar_id, radar_points{test_case.points[i]},
                                   acc::rigid_transform{no_translation, test_case.orientations_roll_pitch_yaw[i]},
                                   test_case.extrinsics);
        }

        const auto accumulated = accumulator.get_points_ego_relative(
            acc::rigid_transform{no_translation, test_case.orientations_roll_pitch_yaw[2]});
        check(accumulated.size() == 3, name + ": size != 3");
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(accumulated[pt].position[dim], test_case.expected_ego_relative[pt][dim], default_precision,
                           name + ": point " + std::to_string(pt));
            }
        }
    }

    // Mirrors Python test_accumulate_rotation_yaw
    void test_rotation_yaw()
    {
        run_rotation_case(
            rotation_case{{{{101, 102, 103, 0, 0, 0}, {110, 120, 130, 0, 0, 0}, {200, 300, 400, 0, 0, 0}}},
                          {{{0, 0, 0}, {0, 0, radians(30)}, {0, 0, radians(45)}}},
                          std::nullopt,
                          {{{143.5427, 0.7071018, 103.0}, {137.31013, 87.44099, 130.0}, {200, 300, 400}}}},
            "rotation_yaw");
    }

    // Mirrors Python test_accumulate_rotation_pitch
    void test_rotation_pitch()
    {
        run_rotation_case(
            rotation_case{{{{101, 102, 103, 0, 0, 0}, {110, 120, 130, 0, 0, 0}, {200, 300, 400, 0, 0, 0}}},
                          {{{0, 0, 0}, {0, radians(30), 0}, {0, radians(45), 0}}},
                          std::nullopt,
                          {{{-1.414223, 102.0, 144.24978}, {72.60535, 120.0, 154.04045}, {200, 300, 400}}}},
            "rotation_pitch");
    }

    // Mirrors Python test_accumulate_rotation_roll: orientations are (0, 0, 0), (radians(30), 0, 0),
    // (radians(45), 0, 0) — frame 0 has NO rotation, exactly as in the Python original
    void test_rotation_roll()
    {
        run_rotation_case(
            rotation_case{{{{101, 102, 103, 0, 0, 0}, {110, 120, 130, 0, 0, 0}, {200, 300, 400, 0, 0, 0}}},
                          {{{0, 0, 0}, {radians(30), 0, 0}, {radians(45), 0, 0}}},
                          std::nullopt,
                          {{{101.0, 144.9569, 0.7070923}, {110.0, 149.5576, 94.51205}, {200, 300, 400}}}},
            "rotation_roll");
    }

    // Mirrors Python test_accumulate_rotation_yaw_with_extrinsics: same expected output as rotation_yaw, but the
    // input points are pre-rotated by -15 deg yaw and the radar extrinsics rotate by +15 deg yaw
    void test_rotation_yaw_with_extrinsics()
    {
        run_rotation_case(
            rotation_case{{{{123.95805106F, 72.38371073F, 103, 0, 0, 0},
                            {137.3101263F, 87.44100419F, 130, 0, 0, 0},
                            {270.83087879F, 238.01393887F, 400, 0, 0, 0}}},
                          {{{0, 0, 0}, {0, 0, radians(30)}, {0, 0, radians(45)}}},
                          acc::rigid_transform{{0, 0, 0}, {0, 0, radians(15)}},
                          {{{143.5427, 0.7071018, 103.0}, {137.31013, 87.44099, 130.0}, {200, 300, 400}}}},
            "rotation_yaw_with_extrinsics");
    }

    // Mirrors Python test_accumulate_rotation_and_move_simple_no_extrinsics
    void test_rotation_and_move_simple_no_extrinsics()
    {
        const std::string radar_id{"test_radar"};
        const pc2::radar_point point{1, 2, 3, 4, 5, 6};
        const acc::rigid_transform fix_when_received{{10, 20, 0}, {0, 0, radians(135)}};
        const acc::rigid_transform current_fix{{20, 10, 0}, {0, 0, radians(45)}};
        acc::point_clouds_accumulator accumulator{1, no_filters_options()};
        accumulator.accumulate(radar_id, radar_points{point}, fix_when_received);
        const auto accumulated = accumulator.get_points_ego_relative(current_fix);

        check(accumulated.size() == 1, "rotation_and_move_simple_no_extrinsics: size");
        check_near(accumulated[0].position[0], -2, default_precision, "rotation_and_move_simple_no_extrinsics: x");
        check_near(accumulated[0].position[1], 1 + 10 * std::sqrt(2.0), default_precision,
                   "rotation_and_move_simple_no_extrinsics: y");
        check_near(accumulated[0].position[2], 3, default_precision, "rotation_and_move_simple_no_extrinsics: z");
    }

    // Mirrors Python test_accumulate_rotation_and_move_simple_with_extrinsics
    void test_rotation_and_move_simple_with_extrinsics()
    {
        const std::string radar_id{"test_radar"};
        // 10 meters left, 2 up, looking left
        const acc::rigid_transform extrinsics{{0, 10, 2}, {0, 0, radians(90)}};
        // ego relative [1, 2, 3, 4, 5, 6] as seen by test_radar
        const pc2::radar_point point{-8, -1, 1, 4, 5, 6};
        const acc::rigid_transform fix_when_received{{10, 20, 0}, {0, 0, radians(135)}};
        const acc::rigid_transform current_fix{{20, 10, 0}, {0, 0, radians(45)}};
        acc::point_clouds_accumulator accumulator{1, no_filters_options(0, false)};
        accumulator.accumulate(radar_id, radar_points{point}, fix_when_received, extrinsics);
        const auto accumulated = accumulator.get_points_ego_relative(current_fix);

        check(accumulated.size() == 1, "rotation_and_move_simple_with_extrinsics: size");
        check_near(accumulated[0].position[0], -2, default_precision, "rotation_and_move_simple_with_extrinsics: x");
        check_near(accumulated[0].position[1], 1 + 10 * std::sqrt(2.0), default_precision,
                   "rotation_and_move_simple_with_extrinsics: y");
        check_near(accumulated[0].position[2], 3, default_precision, "rotation_and_move_simple_with_extrinsics: z");
    }

    // Mirrors Python test_accumulate_rotation_and_move_no_extrinsics: based on its analogue in
    // provizio_radar_api_core, which was proven to match the APT GUI implementation of accumulation
    void test_rotation_and_move_no_extrinsics()
    {
        const std::string radar_id{"test_radar"};
        acc::point_clouds_accumulator accumulator{3, no_filters_options()};

        const pc2::radar_point point_0{101, 102, 103, 5, 5, 5};
        const acc::rigid_transform fix_0{{879.020, 529.971, 0}, {0, 0, 0}};
        const pc2::radar_point point_1{110, 120, 130, 5, 5, 5};
        const acc::rigid_transform fix_1{{871.156, 548.981, 0}, {0, 0, radians(30)}};
        const pc2::radar_point point_2{200, 300, 400, 5, 5, 5};
        const acc::rigid_transform fix_2{{899.447, 562.369, 0}, {0, 0, radians(45)}};

        accumulator.accumulate(radar_id, radar_points{point_0}, fix_0);
        accumulator.accumulate(radar_id, radar_points{point_1}, fix_1);
        accumulator.accumulate(radar_id, radar_points{point_2}, fix_2);

        const auto accumulated = accumulator.get_points_ego_relative(fix_2);
        check(accumulated.size() == 3, "rotation_and_move_no_extrinsics: size");
        check_near(accumulated[0].position[0], 106.18976, default_precision, "ram_ne: p0 x");
        check_near(accumulated[0].position[1], -7.7577, default_precision, "ram_ne: p0 y");
        check_near(accumulated[0].position[2], 103.0, default_precision, "ram_ne: p0 z");
        check_near(accumulated[1].position[0], 107.8386, default_precision, "ram_ne: p1 x");
        check_near(accumulated[1].position[1], 97.979, default_precision, "ram_ne: p1 y");
        check_near(accumulated[1].position[2], 130.0, default_precision, "ram_ne: p1 z");
        check_near(accumulated[2].position[0], 200.0, default_precision, "ram_ne: p2 x");
        check_near(accumulated[2].position[1], 300.0, default_precision, "ram_ne: p2 y");
        check_near(accumulated[2].position[2], 400.0, default_precision, "ram_ne: p2 z");
    }

    // Mirrors Python test_accumulate_rotation_and_move_with_extrinsics: same golden output as above, with the
    // input points expressed in a radar frame offset by [-10,-10,-10] and rotated 180 deg yaw
    void test_rotation_and_move_with_extrinsics()
    {
        const std::string radar_id{"test_radar"};
        acc::point_clouds_accumulator accumulator{3, no_filters_options(0, false)};
        const acc::rigid_transform extrinsics{{-10, -10, -10}, {0, 0, radians(180)}};

        const pc2::radar_point point_0{-111, -112, 113, 5, 5, 5};  // = [101, 102, 103] ego relative
        const acc::rigid_transform fix_0{{879.020, 529.971, 0}, {0, 0, 0}};
        const pc2::radar_point point_1{-120, -130, 140, 5, 5, 5};  // = [110, 120, 130] ego relative
        const acc::rigid_transform fix_1{{871.156, 548.981, 0}, {0, 0, radians(30)}};
        const pc2::radar_point point_2{-210, -310, 410, 5, 5, 5};  // = [200, 300, 400] ego relative
        const acc::rigid_transform fix_2{{899.447, 562.369, 0}, {0, 0, radians(45)}};

        accumulator.accumulate(radar_id, radar_points{point_0}, fix_0, extrinsics);
        accumulator.accumulate(radar_id, radar_points{point_1}, fix_1, extrinsics);
        accumulator.accumulate(radar_id, radar_points{point_2}, fix_2, extrinsics);

        const auto accumulated = accumulator.get_points_ego_relative(fix_2);
        check(accumulated.size() == 3, "rotation_and_move_with_extrinsics: size");
        check_near(accumulated[0].position[0], 106.18976, default_precision, "ram_we: p0 x");
        check_near(accumulated[0].position[1], -7.7577, default_precision, "ram_we: p0 y");
        check_near(accumulated[0].position[2], 103.0, default_precision, "ram_we: p0 z");
        check_near(accumulated[1].position[0], 107.8386, default_precision, "ram_we: p1 x");
        check_near(accumulated[1].position[1], 97.979, default_precision, "ram_we: p1 y");
        check_near(accumulated[1].position[2], 130.0, default_precision, "ram_we: p1 z");
        check_near(accumulated[2].position[0], 200.0, default_precision, "ram_we: p2 x");
        check_near(accumulated[2].position[1], 300.0, default_precision, "ram_we: p2 y");
        check_near(accumulated[2].position[2], 400.0, default_precision, "ram_we: p2 z");
    }

    void test_gps_utils_golden()
    {
        provizio::dds::accumulation::detail::gps_utils gps;
        gps.set_enu_origin(52.705502, -8.899619, 0);

        const auto at_origin = gps.geo_to_enu(52.705502, -8.899619, 0);
        check_near(at_origin[0], 0, 1e-9, "gps: origin east");
        check_near(at_origin[1], 0, 1e-9, "gps: origin north");
        check_near(at_origin[2], 0, 1e-9, "gps: origin up");

        const auto near_point = gps.geo_to_enu(52.707298, -8.898137, 5.5);
        check_near(near_point[0], 100.169027, 1e-5, "gps: near east");
        check_near(near_point[1], 199.861401, 1e-5, "gps: near north");
        check_near(near_point[2], 5.496083, 1e-5, "gps: near up");

        const auto far_point = gps.geo_to_enu(52.715502, -8.909619, 100);
        check_near(far_point[0], -675.787585, 1e-5, "gps: far east");
        check_near(far_point[1], 1112.872476, 1e-5, "gps: far north");
        check_near(far_point[2], 99.867155, 1e-5, "gps: far up");
    }

    using namespace std::chrono_literals;

    // Discovery/matching is asynchronous; generous ceiling for cross-participant matches on busy runners —
    // mirrors the Python tests' _wait_until_matched(timeout_sec=15)
    // Scaled by the environment, as the ctest TIMEOUT wrapped around this case already is. An
    // unscaled deadline inside a test is the defect this branch has now fixed three times
    // elsewhere: under a sanitizer everything it waits for takes several times longer while
    // this budget stays put, so the case reports a discovery failure that is really a slow
    // machine. Discovery here is genuinely quick -- the sibling cases match in well under a
    // second -- so the bound is never what ends the wait on a healthy runner.
    constexpr auto match_timeout = std::chrono::milliseconds{15000 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};

    /// Block until each publisher has at least one matched subscriber. With the match-publisher reader default the
    /// accumulator's subscribers are DEFERRED (the DataReader is created only once a writer is discovered), so a
    /// fixed sleep would race the match and silently drop early samples. Each publisher is awaited independently:
    /// reader match order is NOT publisher creation order. Mirrors the Python tests' _wait_until_matched.
    template <typename... publisher_handle_types> void wait_until_matched(const publisher_handle_types &...publishers)
    {
        const auto wait_one = [](const auto &publisher) {
            check(publisher->get_num_matched_subscribers(match_timeout, std::chrono::milliseconds{0}) > 0,
                  "accumulation_test: a subscriber failed to match in time");
        };
        (wait_one(publishers), ...);
    }

    /// Wait until the accumulator holds at least `expected` points: the point-cloud delivery happens a short time
    /// AFTER publish() returns, so reading the count immediately would race it. Mirrors the Python tests'
    /// _wait_for_accumulated_points.
    void wait_for_accumulated_points(const acc::dds_point_clouds_accumulator &accumulator, const std::size_t expected,
                                     const std::chrono::milliseconds timeout = std::chrono::milliseconds{10000})
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (accumulator.get_points_local_frame_relative().size() < expected &&
               std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(20ms);
        }
        const auto actual = accumulator.get_points_local_frame_relative().size();
        check(actual >= expected, "accumulation_test: timed out waiting for " + std::to_string(expected) +
                                      " accumulated points, got " + std::to_string(actual) + " (likely sample loss)");
    }

    void delivery_order_pause()
    {
        std::this_thread::sleep_for(200ms);  // to make sure the delivery order, as in the Python tests
    }

    void publish_extrinsics(provizio::dds::data_publisher<geometry_msgs::msg::TransformStampedPubSubType> &publisher,
                            const acc::rigid_transform &transform, const std::string &frame_id,
                            const std::string &parent_frame_id = "test_world")
    {
        geometry_msgs::msg::TransformStamped message;
        message.header(pc2::make_header(0, 0U, parent_frame_id));
        message.child_frame_id(frame_id);
        const auto translation = transform.translation();
        message.transform().translation().x(translation[0]);
        message.transform().translation().y(translation[1]);
        message.transform().translation().z(translation[2]);
        const auto rotation = transform.rotation();
        message.transform().rotation().w(rotation.w);
        message.transform().rotation().x(rotation.x);
        message.transform().rotation().y(rotation.y);
        message.transform().rotation().z(rotation.z);
        check(publisher.publish(message), "Failed to publish a TransformStamped message");
    }

    void publish_odometry(provizio::dds::data_publisher<nav_msgs::msg::OdometryPubSubType> &publisher,
                          const acc::rigid_transform &transform,
                          const std::string &frame_id = "provizio_radar_front_center",
                          const std::string &parent_frame_id = "test_world", const double header_stamp_seconds = 0.0)
    {
        nav_msgs::msg::Odometry message;
        const auto hdr_sec = static_cast<std::int32_t>(std::floor(header_stamp_seconds));
        const auto hdr_nsec =
            static_cast<std::uint32_t>(std::llround((header_stamp_seconds - static_cast<double>(hdr_sec)) * 1e9));
        message.header(pc2::make_header(hdr_sec, hdr_nsec, parent_frame_id));
        message.child_frame_id(frame_id);
        const auto translation = transform.translation();
        message.pose().pose().position().x(translation[0]);
        message.pose().pose().position().y(translation[1]);
        message.pose().pose().position().z(translation[2]);
        const auto rotation = transform.rotation();
        message.pose().pose().orientation().w(rotation.w);
        message.pose().pose().orientation().x(rotation.x);
        message.pose().pose().orientation().y(rotation.y);
        message.pose().pose().orientation().z(rotation.z);
        check(publisher.publish(message), "Failed to publish an Odometry message");
    }

    void publish_pc2(provizio::dds::data_publisher<sensor_msgs::msg::PointCloud2PubSubType> &publisher,
                     const radar_points &points, const std::string &radar_id, const double header_stamp_seconds = 0.0)
    {
        const auto hdr_sec = static_cast<std::int32_t>(std::floor(header_stamp_seconds));
        const auto hdr_nsec =
            static_cast<std::uint32_t>(std::llround((header_stamp_seconds - static_cast<double>(hdr_sec)) * 1e9));
        auto message = pc2::make_radar_point_cloud(pc2::make_header(hdr_sec, hdr_nsec, radar_id), points);
        check(publisher.publish(message), "Failed to publish a PointCloud2 message");
    }

    // Mirrors Python test_accumulate_dds_simple
    void test_dds_simple()
    {
        const std::string localization_topic{"rt/test_dds_simple_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_dds_simple_pointcloud2_topic"};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // Ego = radar assumed
        options.accumulation = no_filters_options();
        // Disable the Kalman filter so this test exercises DDS delivery plumbing, not prediction;
        // the filter is unit-tested in localization_filter_test
        options.kalman_localization = false;
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        const std::array<double, 3> ego_pos_0{1, 2, 3};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};
        const std::array<double, 3> ego_pos_1{6, 5, 4};
        const std::array<double, 3> ego_pos_now{1000, 2000, 3000};

        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);

        wait_until_matched(localization_publisher, pc2_publisher);
        publish_odometry(*localization_publisher, acc::rigid_transform{ego_pos_0, {0, 0, 0}});
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_0}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher, acc::rigid_transform{ego_pos_1, {0, 0, 0}});
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_1, point_2}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher, acc::rigid_transform{ego_pos_now, {0, 0, 0}});

        wait_for_accumulated_points(accumulator, 3);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "dds_simple: local size");
        const auto ego = accumulator.get_points_ego_relative();
        check(ego.size() == 3, "dds_simple: ego size");

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        // Last ego_pos_1 is not a typo: point_1 and point_2 share the same ego pose
        const std::array<std::array<double, 3>, 3> ego_poses{ego_pos_0, ego_pos_1, ego_pos_1};
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points[pt].x, input_points[pt].y, input_points[pt].z};
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(local[pt].position[dim], xyz[dim] + ego_poses[pt][dim], default_precision,
                           "dds_simple: local position");
                check_near(ego[pt].position[dim], xyz[dim] + ego_poses[pt][dim] - ego_pos_now[dim], default_precision,
                           "dds_simple: ego position");
            }
            check_point_meta(local[pt], input_points[pt], radar_id, "dds_simple local");
            check_point_meta(ego[pt], input_points[pt], radar_id, "dds_simple ego");
        }
    }

    // Mirrors Python test_accumulate_dds_simple_extrinsics
    void test_dds_simple_extrinsics()
    {
        const std::string localization_topic{"rt/test_dds_simple_extrinsics_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_dds_simple_extrinsics_pointcloud2_topic"};
        const std::string extrinsics_topic{"rt/test_dds_simple_extrinsics_extrinsics_topic"};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics = {extrinsics_topic};
        options.accumulation = no_filters_options();
        // Disable the Kalman filter so this test exercises DDS delivery plumbing, not prediction
        options.kalman_localization = false;
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const std::array<double, 3> radar_extrinsics_pos{-10, -20, -30};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        const std::array<double, 3> ego_pos_0{1, 2, 3};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};
        const std::array<double, 3> ego_pos_1{6, 5, 4};
        const std::array<double, 3> ego_pos_now{1000, 2000, 3000};

        const auto extrinsics_publisher = provizio::dds::make_publisher<geometry_msgs::msg::TransformStampedPubSubType>(
            publishers_participant, extrinsics_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);

        wait_until_matched(extrinsics_publisher, localization_publisher, pc2_publisher);
        publish_extrinsics(*extrinsics_publisher, acc::rigid_transform{radar_extrinsics_pos, {0, 0, 0}}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher, acc::rigid_transform{ego_pos_0, {0, 0, 0}});
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_0}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher, acc::rigid_transform{ego_pos_1, {0, 0, 0}});
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_1, point_2}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher, acc::rigid_transform{ego_pos_now, {0, 0, 0}});

        wait_for_accumulated_points(accumulator, 3);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "dds_simple_extrinsics: local size");
        const auto ego = accumulator.get_points_ego_relative();
        check(ego.size() == 3, "dds_simple_extrinsics: ego size");

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        const std::array<std::array<double, 3>, 3> ego_poses{ego_pos_0, ego_pos_1, ego_pos_1};
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points[pt].x, input_points[pt].y, input_points[pt].z};
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(local[pt].position[dim], xyz[dim] + ego_poses[pt][dim] + radar_extrinsics_pos[dim],
                           default_precision, "dds_simple_extrinsics: local position");
                check_near(ego[pt].position[dim],
                           xyz[dim] + ego_poses[pt][dim] + radar_extrinsics_pos[dim] - ego_pos_now[dim],
                           default_precision, "dds_simple_extrinsics: ego position");
            }
            check_point_meta(local[pt], input_points[pt], radar_id, "dds_simple_extrinsics local");
            check_point_meta(ego[pt], input_points[pt], radar_id, "dds_simple_extrinsics ego");
        }
    }

    // Mirrors Python test_accumulate_dds_simple_extrinsics_including_localization
    void test_dds_simple_extrinsics_including_localization()
    {
        const std::string localization_topic{"rt/test_dds_simple_extrinsics_including_localization_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_dds_simple_extrinsics_including_localization_pointcloud2_topic"};
        const std::string radar_extrinsics_topic{
            "rt/test_dds_simple_extrinsics_including_localization_radar_extrinsics"};
        const std::string localization_extrinsics_topic{
            "rt/test_dds_simple_extrinsics_including_localization_localization_extrinsics"};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics = {radar_extrinsics_topic};
        options.localization_extrinsics_topic = localization_extrinsics_topic;
        // Select this localization source by its frame id (odometry's default is the front-center radar frame).
        options.localization_frame_id = "test_localization";
        options.accumulation = no_filters_options();
        // Disable the Kalman filter so this test exercises DDS delivery plumbing, not prediction
        options.kalman_localization = false;
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const std::string localization_frame_id{"test_localization"};
        const std::array<double, 3> localization_extrinsics_pos{0.3, 0.5, 0.7};
        const double localization_extrinsics_yaw = 90;  // looking left
        const std::array<double, 3> radar_extrinsics_pos{-10, -20, -30};
        const double radar_extrinsics_yaw = -90;  // looking right

        // Points are given radar-relative; their ego-relative equivalents drive the expected values
        const pc2::radar_point point_0_ego{1, 2, 3, 4, 5, 6};
        const pc2::radar_point point_0_radar{-2 - 20, 1 + 10, 3 + 30, 4, 5, 6};
        const std::array<double, 3> ego_pos_0{1, 2, 3};
        const std::array<double, 3> loc_pos_0{1 + 0.3, 2 + 0.5, 3 + 0.7};
        const pc2::radar_point point_1_ego{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_1_radar{-20 - 20, 10 + 10, 30 + 30, 40, 50, 60};
        const pc2::radar_point point_2_ego{100, 200, 300, 400, 500, 600};
        const pc2::radar_point point_2_radar{-200 - 20, 100 + 10, 300 + 30, 400, 500, 600};
        const std::array<double, 3> ego_pos_1{6, 5, 4};
        const std::array<double, 3> loc_pos_1{6 + 0.3, 5 + 0.5, 4 + 0.7};
        const std::array<double, 3> ego_pos_now{1000, 2000, 3000};
        const std::array<double, 3> loc_pos_now{1000 + 0.3, 2000 + 0.5, 3000 + 0.7};

        const auto radar_extrinsics_publisher =
            provizio::dds::make_publisher<geometry_msgs::msg::TransformStampedPubSubType>(
                publishers_participant, radar_extrinsics_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto localization_extrinsics_publisher =
            provizio::dds::make_publisher<geometry_msgs::msg::TransformStampedPubSubType>(
                publishers_participant, localization_extrinsics_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);

        wait_until_matched(radar_extrinsics_publisher, localization_extrinsics_publisher, localization_publisher,
                           pc2_publisher);
        publish_extrinsics(*radar_extrinsics_publisher,
                           acc::rigid_transform{radar_extrinsics_pos, {0, 0, radians(radar_extrinsics_yaw)}}, radar_id);
        delivery_order_pause();
        publish_extrinsics(
            *localization_extrinsics_publisher,
            acc::rigid_transform{localization_extrinsics_pos, {0, 0, radians(localization_extrinsics_yaw)}},
            localization_frame_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher,
                         acc::rigid_transform{loc_pos_0, {0, 0, radians(localization_extrinsics_yaw)}},
                         localization_frame_id);
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_0_radar}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher,
                         acc::rigid_transform{loc_pos_1, {0, 0, radians(localization_extrinsics_yaw)}},
                         localization_frame_id);
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_1_radar, point_2_radar}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher,
                         acc::rigid_transform{loc_pos_now, {0, 0, radians(localization_extrinsics_yaw)}},
                         localization_frame_id);

        wait_for_accumulated_points(accumulator, 3);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "dds_extr_incl_loc: local size");
        const auto ego = accumulator.get_points_ego_relative();
        check(ego.size() == 3, "dds_extr_incl_loc: ego size");

        const std::array<pc2::radar_point, 3> input_points_ego{point_0_ego, point_1_ego, point_2_ego};
        const std::array<std::array<double, 3>, 3> ego_poses{ego_pos_0, ego_pos_1, ego_pos_1};
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points_ego[pt].x, input_points_ego[pt].y, input_points_ego[pt].z};
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(local[pt].position[dim], xyz[dim] + ego_poses[pt][dim], default_precision,
                           "dds_extr_incl_loc: local position");
                check_near(ego[pt].position[dim], xyz[dim] + ego_poses[pt][dim] - ego_pos_now[dim], default_precision,
                           "dds_extr_incl_loc: ego position");
            }
            check_point_meta(local[pt], input_points_ego[pt], radar_id, "dds_extr_incl_loc local");
            check_point_meta(ego[pt], input_points_ego[pt], radar_id, "dds_extr_incl_loc ego");
        }
    }

    // Late localization extrinsics: the same geometry as dds_simple_extrinsics_including_localization, but the
    // localization extrinsics arrives AFTER frames have already been accumulated under the identity assumption.
    // They must be retroactively re-placed to the same result, with none lost.
    void test_dds_localization_extrinsics_late()
    {
        const std::string localization_topic{"rt/test_dds_localization_extrinsics_late_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_dds_localization_extrinsics_late_pointcloud2_topic"};
        const std::string radar_extrinsics_topic{"rt/test_dds_localization_extrinsics_late_radar_extrinsics"};
        const std::string localization_extrinsics_topic{
            "rt/test_dds_localization_extrinsics_late_localization_extrinsics"};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics = {radar_extrinsics_topic};
        options.localization_extrinsics_topic = localization_extrinsics_topic;
        options.localization_frame_id = "test_localization";
        options.accumulation = no_filters_options();
        options.kalman_localization = false;
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const std::array<double, 3> localization_extrinsics_pos{0.3, 0.5, 0.7};
        const double localization_extrinsics_yaw = 90;  // looking left
        const std::array<double, 3> radar_extrinsics_pos{-10, -20, -30};
        const double radar_extrinsics_yaw = -90;  // looking right

        const pc2::radar_point point_0_ego{1, 2, 3, 4, 5, 6};
        const pc2::radar_point point_0_radar{-2 - 20, 1 + 10, 3 + 30, 4, 5, 6};
        const std::array<double, 3> ego_pos_0{1, 2, 3};
        const std::array<double, 3> loc_pos_0{1 + 0.3, 2 + 0.5, 3 + 0.7};
        const pc2::radar_point point_1_ego{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_1_radar{-20 - 20, 10 + 10, 30 + 30, 40, 50, 60};
        const pc2::radar_point point_2_ego{100, 200, 300, 400, 500, 600};
        const pc2::radar_point point_2_radar{-200 - 20, 100 + 10, 300 + 30, 400, 500, 600};
        const std::array<double, 3> ego_pos_1{6, 5, 4};
        const std::array<double, 3> loc_pos_1{6 + 0.3, 5 + 0.5, 4 + 0.7};

        const auto radar_extrinsics_publisher =
            provizio::dds::make_publisher<geometry_msgs::msg::TransformStampedPubSubType>(
                publishers_participant, radar_extrinsics_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto localization_extrinsics_publisher =
            provizio::dds::make_publisher<geometry_msgs::msg::TransformStampedPubSubType>(
                publishers_participant, localization_extrinsics_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);

        wait_until_matched(radar_extrinsics_publisher, localization_extrinsics_publisher, localization_publisher,
                           pc2_publisher);
        // Radar extrinsics + odometry + clouds first, but NOT the localization extrinsics: the frames accumulate
        // with the localization extrinsics assumed identity (ego pose = the raw odometry pose).
        publish_extrinsics(*radar_extrinsics_publisher,
                           acc::rigid_transform{radar_extrinsics_pos, {0, 0, radians(radar_extrinsics_yaw)}}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher,
                         acc::rigid_transform{loc_pos_0, {0, 0, radians(localization_extrinsics_yaw)}},
                         "test_localization");
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_0_radar}, radar_id);
        delivery_order_pause();
        publish_odometry(*localization_publisher,
                         acc::rigid_transform{loc_pos_1, {0, 0, radians(localization_extrinsics_yaw)}},
                         "test_localization");
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_1_radar, point_2_radar}, radar_id);

        // All three accumulate under the identity assumption ...
        wait_for_accumulated_points(accumulator, 3);

        // ... then the real localization extrinsics arrives and must retroactively re-place them (RELIABLE QoS
        // guarantees delivery).
        publish_extrinsics(
            *localization_extrinsics_publisher,
            acc::rigid_transform{localization_extrinsics_pos, {0, 0, radians(localization_extrinsics_yaw)}},
            "test_localization");

        // Poll until the correction has landed (the first point moves from its identity placement to the
        // extrinsics-corrected one).
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{10000};
        auto local = accumulator.get_points_local_frame_relative();
        while (std::chrono::steady_clock::now() < deadline &&
               (local.size() < 3 || std::abs(static_cast<double>(local[0].position[0]) -
                                             (point_0_ego.x + ego_pos_0[0])) > default_precision))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
            local = accumulator.get_points_local_frame_relative();
        }
        check(local.size() == 3, "dds_loc_extr_late: local size");

        const std::array<pc2::radar_point, 3> input_points_ego{point_0_ego, point_1_ego, point_2_ego};
        const std::array<std::array<double, 3>, 3> ego_poses{ego_pos_0, ego_pos_1, ego_pos_1};
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points_ego[pt].x, input_points_ego[pt].y, input_points_ego[pt].z};
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(local[pt].position[dim], xyz[dim] + ego_poses[pt][dim], default_precision,
                           "dds_loc_extr_late: retroactively corrected local position");
            }
            check_point_meta(local[pt], input_points_ego[pt], radar_id, "dds_loc_extr_late");
        }
    }

    // Mirrors Python test_accumulate_dds_no_localization_no_extrinsics
    void test_dds_no_localization_no_extrinsics()
    {
        const std::string pointcloud2_topic{"rt/test_dds_no_localization_no_extrinsics_pointcloud2_topic"};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::none;
        options.localization_topic.clear();
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();
        options.accumulation = no_filters_options();
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};

        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);

        wait_until_matched(pc2_publisher);
        publish_pc2(*pc2_publisher, radar_points{point_0}, radar_id);
        publish_pc2(*pc2_publisher, radar_points{point_1, point_2}, radar_id);

        wait_for_accumulated_points(accumulator, 3);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "dds_no_loc_no_extr: local size");
        const auto ego = accumulator.get_points_ego_relative();
        check(ego.size() == 3, "dds_no_loc_no_extr: ego size");

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points[pt].x, input_points[pt].y, input_points[pt].z};
            for (const auto *accumulated : {&local, &ego})
            {
                for (std::size_t dim = 0; dim < 3; ++dim)
                {
                    check((*accumulated)[pt].position[dim] == xyz[dim], "dds_no_loc_no_extr: position passthrough");
                }
                check_point_meta((*accumulated)[pt], input_points[pt], radar_id, "dds_no_loc_no_extr");
            }
        }
    }

    // Mirrors Python test_accumulate_dds_no_localization_with_extrinsics
    void test_dds_no_localization_with_extrinsics()
    {
        const std::string pointcloud2_topic{"rt/test_dds_no_localization_with_extrinsics_pointcloud2_topic"};
        const std::string extrinsics_topic{"rt/test_dds_no_localization_with_extrinsics_extrinsics_topic"};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::none;
        options.localization_topic.clear();
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics = {extrinsics_topic};
        options.accumulation = no_filters_options();
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const std::array<double, 3> radar_extrinsics_pos{-10, -20, -30};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};

        const auto extrinsics_publisher = provizio::dds::make_publisher<geometry_msgs::msg::TransformStampedPubSubType>(
            publishers_participant, extrinsics_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);

        wait_until_matched(extrinsics_publisher, pc2_publisher);
        publish_extrinsics(*extrinsics_publisher, acc::rigid_transform{radar_extrinsics_pos, {0, 0, 0}}, radar_id);
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_0}, radar_id);
        publish_pc2(*pc2_publisher, radar_points{point_1, point_2}, radar_id);

        wait_for_accumulated_points(accumulator, 3);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "dds_no_loc_with_extr: local size");
        const auto ego = accumulator.get_points_ego_relative();
        check(ego.size() == 3, "dds_no_loc_with_extr: ego size");

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points[pt].x, input_points[pt].y, input_points[pt].z};
            for (const auto *accumulated : {&local, &ego})
            {
                for (std::size_t dim = 0; dim < 3; ++dim)
                {
                    check_near((*accumulated)[pt].position[dim], xyz[dim] + radar_extrinsics_pos[dim],
                               default_precision, "dds_no_loc_with_extr: position");
                }
                check_point_meta((*accumulated)[pt], input_points[pt], radar_id, "dds_no_loc_with_extr");
            }
        }
    }

    /// Approximate-earth lat/lon offsetting, mirroring the Python test's lat_lon_plus_offset
    std::array<double, 2> lat_lon_plus_offset(const double lat, const double lon, const double offset_east_m,
                                              const double offset_north_m)
    {
        constexpr double earth_radius = 6378137.0;
        const double out_lat = lat + (offset_north_m / earth_radius) * (180.0 / pi);
        const double out_lon = lon + (offset_east_m / earth_radius) * (180.0 / pi) / std::cos(lat * pi / 180.0);
        return {out_lat, out_lon};
    }

    void publish_nav_sat_fix(provizio::dds::data_publisher<sensor_msgs::msg::NavSatFixPubSubType> &publisher,
                             const std::array<double, 3> &lat_lon_alt, const std::string &frame_id = "test_nav_sat_fix")
    {
        sensor_msgs::msg::NavSatFix message;
        message.header(pc2::make_header(0, 0U, frame_id));
        message.status().status(sensor_msgs::msg::NavSatStatus_Constants::STATUS_FIX);
        message.latitude(lat_lon_alt[0]);
        message.longitude(lat_lon_alt[1]);
        message.altitude(lat_lon_alt[2]);
        check(publisher.publish(message), "Failed to publish a NavSatFix message");
    }

    // Mirrors Python test_accumulate_dds_nav_sat_fix_localization
    void test_dds_nav_sat_fix_localization()
    {
        const std::string localization_topic{"rt/test_dds_nav_sat_fix_localization_nav_sat_fix_topic"};
        const std::string pointcloud2_topic{"rt/test_dds_nav_sat_fix_localization_pointcloud2_topic"};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::nav_sat_fix;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // Ego = radar assumed
        options.accumulation = no_filters_options();
        // Disable the Kalman filter so this test exercises DDS delivery plumbing, not prediction
        options.kalman_localization = false;
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const pc2::radar_point point_0{1, 2, 3, 4, 5, 6};
        // Local ENU origin will be set at (52.705502, -8.899619, 0)
        const std::array<double, 3> ego_fix_0{52.705502, -8.899619, 3};
        const pc2::radar_point point_1{10, 20, 30, 40, 50, 60};
        const pc2::radar_point point_2{100, 200, 300, 400, 500, 600};
        const std::array<double, 3> ego_enu_offset_1{5, 0, 4};  // straight east from fix 0
        const auto fix_1_lat_lon =
            lat_lon_plus_offset(ego_fix_0[0], ego_fix_0[1], ego_enu_offset_1[0], ego_enu_offset_1[1]);
        const std::array<double, 3> ego_fix_1{fix_1_lat_lon[0], fix_1_lat_lon[1], ego_enu_offset_1[2]};
        const std::array<double, 3> ego_enu_offset_now{0, 10, 20};  // straight north from fix 0
        const auto fix_now_lat_lon =
            lat_lon_plus_offset(ego_fix_0[0], ego_fix_0[1], ego_enu_offset_now[0], ego_enu_offset_now[1]);
        const std::array<double, 3> ego_fix_now{fix_now_lat_lon[0], fix_now_lat_lon[1], ego_enu_offset_now[2]};

        const auto localization_publisher = provizio::dds::make_publisher<sensor_msgs::msg::NavSatFixPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);

        wait_until_matched(localization_publisher, pc2_publisher);
        publish_nav_sat_fix(*localization_publisher, ego_fix_0);
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_0}, radar_id);
        delivery_order_pause();
        publish_nav_sat_fix(*localization_publisher, ego_fix_1);
        delivery_order_pause();
        publish_pc2(*pc2_publisher, radar_points{point_1, point_2}, radar_id);
        delivery_order_pause();
        publish_nav_sat_fix(*localization_publisher, ego_fix_now);

        wait_for_accumulated_points(accumulator, 3);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 3, "dds_nav_sat_fix: local size");
        const auto ego = accumulator.get_points_ego_relative();
        check(ego.size() == 3, "dds_nav_sat_fix: ego size");

        const std::array<pc2::radar_point, 3> input_points{point_0, point_1, point_2};
        // As the local ENU origin is always on the Earth spheroid, fix 0 contributes only its altitude;
        // ego_enu_offset_1 is duplicated because point_1 and point_2 share the same fix position
        const std::array<std::array<double, 3>, 3> ego_poses_enu{std::array<double, 3>{0, 0, ego_fix_0[2]},
                                                                 ego_enu_offset_1, ego_enu_offset_1};

        // 5 cm error permitted: lat_lon_plus_offset uses an approximate (latitude-independent) earth radius
        const double precision = 0.05;
        for (std::size_t pt = 0; pt < 3; ++pt)
        {
            const std::array<float, 3> xyz{input_points[pt].x, input_points[pt].y, input_points[pt].z};
            for (std::size_t dim = 0; dim < 3; ++dim)
            {
                check_near(local[pt].position[dim], xyz[dim] + ego_poses_enu[pt][dim], precision,
                           "dds_nav_sat_fix: local position");
            }
            check_point_meta(local[pt], input_points[pt], radar_id, "dds_nav_sat_fix local");
            check_point_meta(ego[pt], input_points[pt], radar_id, "dds_nav_sat_fix ego");

            // Ego-relative positions are rotated: by the time of the last fix the estimated yaw is 90 deg
            // (the ego moved straight north relative to the position history start)
            check_near(ego[pt].position[0], xyz[1] + ego_poses_enu[pt][1] - ego_enu_offset_now[1], precision,
                       "dds_nav_sat_fix: ego x");
            check_near(ego[pt].position[1], -xyz[0] - ego_poses_enu[pt][0] + ego_enu_offset_now[0], precision,
                       "dds_nav_sat_fix: ego y");
            check_near(ego[pt].position[2], xyz[2] + ego_poses_enu[pt][2] - ego_enu_offset_now[2], precision,
                       "dds_nav_sat_fix: ego z");
        }
    }

    // Mirrors Python test_accumulate_dds_on_pc2_callback
    void test_dds_on_pc2_callback()
    {
        const std::string pointcloud2_topic{"rt/test_dds_on_pc2_callback_pointcloud2_topic"};
        const std::string radar_id{"test_radar"};
        const pc2::radar_point point{1, 2, 3, 4, 5, 6};
        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::mutex callback_mutex;
        std::condition_variable callback_cv;
        bool callback_fired = false;
        std::size_t num_accumulated = 0;

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::none;
        options.localization_topic.clear();
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();
        options.accumulation = no_filters_options();
        // get_points_* are deliberately callable from the callback: it fires outside the internal lock
        options.on_point_cloud = [&](acc::dds_point_clouds_accumulator &callback_accumulator) {
            const auto count = callback_accumulator.get_points_local_frame_relative().size();
            {
                const std::lock_guard<std::mutex> lock{callback_mutex};
                callback_fired = true;
                num_accumulated = count;
            }
            callback_cv.notify_all();
        };
        const acc::dds_point_clouds_accumulator accumulator{1, options, subscribers_participant};

        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(pc2_publisher);
        {
            const std::lock_guard<std::mutex> lock{callback_mutex};
            check(!callback_fired, "dds_on_pc2_callback: callback fired before anything was published");
        }

        // The matched reader's RX path can still be settling for a short while after the writer-side match;
        // publish in a small loop until the callback fires (max_frames_per_radar=1 keeps the count at 1 regardless
        // of duplicates), mirroring the robustness note in the Python test.
        const auto publish_deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
        std::unique_lock<std::mutex> lock{callback_mutex};
        while (!callback_fired && std::chrono::steady_clock::now() < publish_deadline)
        {
            lock.unlock();
            publish_pc2(*pc2_publisher, radar_points{point}, radar_id);
            lock.lock();
            callback_cv.wait_for(lock, 500ms, [&callback_fired] { return callback_fired; });
        }
        check(callback_fired, "dds_on_pc2_callback: on_point_cloud never fired");
        check(num_accumulated == 1,
              "dds_on_pc2_callback: expected 1 accumulated point, got " + std::to_string(num_accumulated));
    }
    // Drives the Kalman-predicted localization path end-to-end over DDS with a deterministic
    // injected clock. The clock is sequenced by the time_source call count: each processed
    // odometry fix and the point cloud call time_source exactly once, so the test advances the
    // clock per call and waits on the call counter (not on sleeps), keeping it reproducible. A
    // constant-velocity ramp of fixes lets the filter's rate converge; the cloud then arrives
    // 0.6 s after the last fix, so its point lands at the EXTRAPOLATED ego pose, well past the
    // stale last fix. (The kalman_localization=false / last-fix path is covered by test_dds_simple.)
    void test_dds_kalman_localization()
    {
        const std::string localization_topic{"rt/test_kalman_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_kalman_pointcloud2_topic"};
        const std::size_t num_fixes = 20;

        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::atomic<std::size_t> calls{0};
        std::vector<double> times(num_fixes + 1);
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            times[k] = 0.1 * static_cast<double>(k);  // fix k at t = 0.1*k
        }
        times[num_fixes] = times[num_fixes - 1] + 0.6;  // cloud 0.6 s after the last fix

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // ego == radar frame
        options.accumulation = no_filters_options();
        options.kalman_localization = true;
        options.time_source = [&calls, &times] {
            const std::size_t i = calls.fetch_add(1);
            return times[std::min(i, times.size() - 1)];
        };
        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_radar"};
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(localization_publisher, pc2_publisher);

        // Constant-velocity ramp: ego x = 10 * t = 10 * (0.1*k) = k, so fix k is at x = k.
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            publish_odometry(*localization_publisher, acc::rigid_transform{{static_cast<double>(k), 0, 0}, {0, 0, 0}});
        }
        // Wait until all fixes are processed (each calls time_source once) BEFORE the cloud, so the
        // cloud deterministically gets times[num_fixes]. Bounded so a delivery failure can't hang.
        for (int waited = 0; calls.load() < num_fixes && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes, "dds_kalman: all fixes processed before the cloud");

        publish_pc2(*pc2_publisher, radar_points{pc2::radar_point{0, 0, 0, 0, 100, 0}}, radar_id);
        wait_for_accumulated_points(accumulator, 1);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 1, "dds_kalman: one accumulated point");

        const double last_fix_x = static_cast<double>(num_fixes - 1);  // 19
        // Enabled Kalman extrapolates ~0.6 s forward at ~10 m/s -> ~25, well past the last fix (19).
        check(local[0].position[0] > last_fix_x + 1.0,
              "dds_kalman: cloud placed at the extrapolated receive-time ego pose, not the stale last fix");
    }

    // Verifies end-to-end timesync buffering and exact placement at covering localization.
    //
    // Scenario:
    //   - N=20 odometry fixes, header stamps h_k = 0.1*k, ego x = 10*h_k = k.
    //   - Cloud published with header T_pc = 0.1*N = 2.0, which is AHEAD of the latest localization
    //     header h_{N-1} = 1.9 — so on_pc2 buffers the cloud rather than accumulating it.
    //   - on_point_cloud must NOT fire when the cloud is merely buffered.
    //   - A covering odometry fix arrives with header T_pc = 2.0 and ego x = 20; the buffer is released,
    //     and the cloud is placed at predict(2.0) — effectively the exact covering pose, x ≈ 20.
    //   - on_point_cloud fires exactly once, at release.
    //
    // timesync_max_delay_seconds = 100.0 (timeout never fires during the test).
    void test_dds_timesync_buffered_then_covered()
    {
        const std::string localization_topic{"rt/test_timesync_buffered_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_timesync_buffered_pointcloud2_topic"};
        const std::size_t num_fixes = 20;
        // Cloud header stamp — one step ahead of the last fix so the buffer is triggered.
        const double t_pc = 0.1 * static_cast<double>(num_fixes);  // 2.0

        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::atomic<std::size_t> calls{0};
        std::atomic<std::size_t> callback_count{0};

        // time_source is sequenced by call index:
        //   calls 0..N-1  : odometry fixes   → times 0..N-1 (0.1*k, k in [0,N))
        //   call  N       : on_pc2 (buffer)  → receive time for the cloud (2.0, well inside the 100s timeout)
        //   call  N+1     : covering odometry → triggers flush
        // All times are << 100s apart from receive, so the timeout never fires.
        std::vector<double> times(num_fixes + 2);
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            times[k] = 0.1 * static_cast<double>(k);
        }
        times[num_fixes] = t_pc;            // cloud receive time (same as header; timeout = 100s)
        times[num_fixes + 1] = t_pc + 0.1;  // covering odometry receive time (barely after cloud)

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // ego frame == radar frame
        options.accumulation = no_filters_options();
        options.kalman_localization = true;
        options.timesync_max_delay_seconds = 100.0;
        options.time_source = [&calls, &times] {
            const std::size_t i = calls.fetch_add(1);
            return times[std::min(i, times.size() - 1)];
        };
        options.on_point_cloud = [&callback_count](acc::dds_point_clouds_accumulator &) { ++callback_count; };

        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_ts_radar"};
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(localization_publisher, pc2_publisher);

        // Constant-velocity ramp: ego x = 10 * h_k = k; header stamp = h_k = 0.1*k.
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            const double h_k = 0.1 * static_cast<double>(k);
            publish_odometry(*localization_publisher, acc::rigid_transform{{10.0 * h_k, 0, 0}, {0, 0, 0}},
                             "provizio_radar_front_center", "test_world", h_k);
        }
        // Wait until all fixes are processed (each calls time_source once) before publishing the cloud.
        for (int waited = 0; calls.load() < num_fixes && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes, "dds_timesync_buffered: all fixes processed before the cloud");

        // Publish a cloud with header AHEAD of the latest localization header (1.9) — on_pc2 buffers it.
        publish_pc2(*pc2_publisher, radar_points{pc2::radar_point{0, 0, 0, 0, 100, 0}}, radar_id, t_pc);

        // Wait until on_pc2 has executed (calls bumped to N+1).
        for (int waited = 0; calls.load() < num_fixes + 1 && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes + 1, "dds_timesync_buffered: on_pc2 callback executed");

        // Cloud must be buffered: no accumulated points and no on_point_cloud fire.
        check(accumulator.get_points_local_frame_relative().empty(),
              "dds_timesync_buffered: cloud must be buffered, not accumulated immediately");
        check(callback_count.load() == 0,
              "dds_timesync_buffered: on_point_cloud must not fire while cloud is buffered");

        // Covering odometry: header = T_pc = 2.0, ego x = 10 * T_pc = 20.
        // This fix covers the cloud's header stamp, releasing it from the buffer.
        publish_odometry(*localization_publisher, acc::rigid_transform{{10.0 * t_pc, 0, 0}, {0, 0, 0}},
                         "provizio_radar_front_center", "test_world", t_pc);

        // Wait for accumulation: the buffered cloud is placed at predict(T_pc) — the exact covering pose.
        wait_for_accumulated_points(accumulator, 1);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 1, "dds_timesync_buffered: one accumulated point after covering fix");

        // The cloud point is at the origin of the radar frame; with ego == radar, position == ego pose.
        // predict(T_pc) with the covering fix at T_pc and ego x = 20 → position[0] ≈ 20.
        check_near(local[0].position[0], 20.0, 1e-2,
                   "dds_timesync_buffered: point placed at the exact covering localization, x ≈ 20");

        // on_point_cloud must have fired exactly once (at buffer release, not at buffer insertion).
        check(callback_count.load() == 1, "dds_timesync_buffered: on_point_cloud fired exactly once at release");
    }

    // Verifies that a buffered cloud whose header falls strictly between two known fixes is placed at
    // the linearly-interpolated pose rather than snapping to the covering (later) fix.
    //
    // Scenario:
    //   - N=20 odometry fixes, header stamps h_k = 0.1*k, ego x = 10*(0.1*k) = k.
    //   - Cloud with header T_pc = 1.95 (strictly between h_{19}=1.9 and the next fix): buffered.
    //   - Covering fix at header 2.0, ego x = 20.
    //   - Bracket: t0=(1.9, x=19), t1=(2.0, x=20), T=1.95 → frac=0.5 → interpolated x ≈ 19.5.
    //   - Old behaviour (snap to covering fix) would yield x ≈ 20; interpolation yields x ≈ 19.5.
    //
    // timesync_max_delay_seconds = 100.0 (timeout never fires during the test).
    void test_dds_timesync_interpolates_between_fixes()
    {
        const std::string localization_topic{"rt/test_timesync_interp_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_timesync_interp_pointcloud2_topic"};
        const std::size_t num_fixes = 20;
        // Cloud header strictly between h_{N-1}=1.9 and the covering fix at 2.0.
        const double t_pc = 1.95;

        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::atomic<std::size_t> calls{0};
        std::atomic<std::size_t> callback_count{0};

        // time_source sequencing:
        //   calls 0..N-1 : ramp fixes      → receive times = header times (0.1*k)
        //   call  N      : on_pc2 (buffer) → receive time = t_pc (well inside the 100s timeout)
        //   call  N+1    : covering fix    → receive time = t_pc + 0.1 (barely after cloud)
        std::vector<double> times(num_fixes + 2);
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            times[k] = 0.1 * static_cast<double>(k);
        }
        times[num_fixes] = t_pc;
        times[num_fixes + 1] = t_pc + 0.1;

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // ego frame == radar frame
        options.accumulation = no_filters_options();
        options.kalman_localization = true;
        options.timesync_max_delay_seconds = 100.0;
        options.time_source = [&calls, &times] {
            const std::size_t i = calls.fetch_add(1);
            return times[std::min(i, times.size() - 1)];
        };
        options.on_point_cloud = [&callback_count](acc::dds_point_clouds_accumulator &) { ++callback_count; };

        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_ts_interp_radar"};
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(localization_publisher, pc2_publisher);

        // Constant-velocity ramp: ego x = 10 * h_k = k; header stamp h_k = 0.1*k.
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            const double h_k = 0.1 * static_cast<double>(k);
            publish_odometry(*localization_publisher, acc::rigid_transform{{10.0 * h_k, 0, 0}, {0, 0, 0}},
                             "provizio_radar_front_center", "test_world", h_k);
        }
        // Wait until all ramp fixes are processed before the cloud.
        for (int waited = 0; calls.load() < num_fixes && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes, "dds_timesync_interp: all ramp fixes processed before the cloud");

        // Cloud header 1.95 is ahead of the latest localization header 1.9 — buffered.
        publish_pc2(*pc2_publisher, radar_points{pc2::radar_point{0, 0, 0, 0, 100, 0}}, radar_id, t_pc);

        // Wait until on_pc2 has executed (calls bumped to N+1).
        for (int waited = 0; calls.load() < num_fixes + 1 && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes + 1, "dds_timesync_interp: on_pc2 callback executed");

        // Cloud must be buffered: no accumulated points yet.
        check(accumulator.get_points_local_frame_relative().empty(),
              "dds_timesync_interp: cloud must be buffered, not accumulated immediately");
        check(callback_count.load() == 0, "dds_timesync_interp: on_point_cloud must not fire while cloud is buffered");

        // Covering fix at header 2.0, ego x = 20. This releases the cloud with t0=(1.9, x=19), t1=(2.0, x=20).
        // frac = (1.95 - 1.9) / (2.0 - 1.9) = 0.5 → interpolated x = 19.5.
        const double covering_header = 2.0;
        const double covering_x = 20.0;
        publish_odometry(*localization_publisher, acc::rigid_transform{{covering_x, 0, 0}, {0, 0, 0}},
                         "provizio_radar_front_center", "test_world", covering_header);

        wait_for_accumulated_points(accumulator, 1);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 1, "dds_timesync_interp: one accumulated point after covering fix");

        // Interpolated x = 19.5; old snap-to-covering would give 20.0; stale-last-fix would give 19.0.
        check_near(local[0].position[0], 19.5, 1e-2,
                   "dds_timesync_interp: point placed at interpolated pose, x ≈ 19.5");

        check(callback_count.load() == 1, "dds_timesync_interp: on_point_cloud fired exactly once at release");
    }

    // Verifies that the timesync release interpolates ORIENTATION (slerp), not just position.
    //
    // Scenario:
    //   - Ego stays at the origin (0,0,0) for BOTH bracketing fixes, so position interpolation is a no-op
    //     and only orientation varies.
    //   - Fix A: header H_A = 1.0, yaw = 0.
    //   - Cloud: header at the midpoint 1.5, a single radar point at local (x=1, y=0, z=0); buffered.
    //   - Fix B (covering): header H_B = 2.0, yaw = +90° (pi/2).
    //   - On release frac = 0.5 → interpolated yaw = 45°, so the local point (1,0,0) lands at world
    //     (cos45°, sin45°, 0) = (0.7071, 0.7071, 0). Asserting BOTH coords ≈ 0.7071 proves the slerp took
    //     the SHORT way to 45° (a sign error / long way would give e.g. (0.707, -0.707) or (-0.707, ...)).
    void test_dds_timesync_interpolates_orientation()
    {
        const std::string localization_topic{"rt/test_timesync_orient_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_timesync_orient_pointcloud2_topic"};
        const double h_a = 1.0;
        const double t_pc = 1.5;  // midpoint of [h_a, h_b]
        const double h_b = 2.0;

        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::atomic<std::size_t> calls{0};
        std::atomic<std::size_t> callback_count{0};

        // time_source sequencing:
        //   call 0 : fix A (yaw 0)        → receive time = h_a
        //   call 1 : on_pc2 (buffer)      → receive time = t_pc (well inside the 100s timeout)
        //   call 2 : covering fix B (yaw 90°) → receive time = t_pc + 0.1
        const std::vector<double> times{h_a, t_pc, t_pc + 0.1};

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // ego frame == radar frame
        options.accumulation = no_filters_options();
        options.kalman_localization = true;
        options.timesync_max_delay_seconds = 100.0;
        options.time_source = [&calls, &times] {
            const std::size_t i = calls.fetch_add(1);
            return times[std::min(i, times.size() - 1)];
        };
        options.on_point_cloud = [&callback_count](acc::dds_point_clouds_accumulator &) { ++callback_count; };

        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_ts_orient_radar"};
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(localization_publisher, pc2_publisher);

        // Fix A: origin, yaw 0.
        publish_odometry(*localization_publisher, acc::rigid_transform{{0, 0, 0}, {0, 0, 0}},
                         "provizio_radar_front_center", "test_world", h_a);
        // Wait until fix A is processed before the cloud.
        for (int waited = 0; calls.load() < 1 && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= 1, "dds_timesync_orient: fix A processed before the cloud");

        // Cloud header 1.5 is ahead of the latest localization header 1.0 — buffered. Point at local (1,0,0).
        publish_pc2(*pc2_publisher, radar_points{pc2::radar_point{1, 0, 0, 0, 100, 0}}, radar_id, t_pc);

        // Wait until on_pc2 has executed (calls bumped to 2).
        for (int waited = 0; calls.load() < 2 && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= 2, "dds_timesync_orient: on_pc2 callback executed");

        // Cloud must be buffered: no accumulated points yet.
        check(accumulator.get_points_local_frame_relative().empty(),
              "dds_timesync_orient: cloud must be buffered, not accumulated immediately");
        check(callback_count.load() == 0, "dds_timesync_orient: on_point_cloud must not fire while cloud is buffered");

        // Covering fix B: origin, yaw +90° (pi/2). Releases the cloud with brackets (h_a, yaw 0) and (h_b, yaw 90°),
        // frac 0.5 → yaw 45°.
        publish_odometry(*localization_publisher, acc::rigid_transform{{0, 0, 0}, {0, 0, pi / 2}},
                         "provizio_radar_front_center", "test_world", h_b);

        wait_for_accumulated_points(accumulator, 1);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 1, "dds_timesync_orient: one accumulated point after covering fix");

        // Interpolated yaw 45° rotates local (1,0,0) to world (cos45°, sin45°, 0) = (0.7071, 0.7071, 0).
        const double expected = std::sqrt(2.0) / 2.0;  // 0.70710678...
        std::cout << "dds_timesync_orient: world point = (" << local[0].position[0] << ", " << local[0].position[1]
                  << ", " << local[0].position[2] << "), expected (" << expected << ", " << expected << ", 0)"
                  << std::endl;
        check_near(local[0].position[0], expected, 1e-2,
                   "dds_timesync_orient: interpolated yaw 45° places x ≈ 0.7071 (short-way slerp)");
        check_near(local[0].position[1], expected, 1e-2,
                   "dds_timesync_orient: interpolated yaw 45° places y ≈ 0.7071 (short-way slerp)");

        check(callback_count.load() == 1, "dds_timesync_orient: on_point_cloud fired exactly once at release");
    }

    // Verifies timesync timeout release with forward extrapolation.
    //
    // Scenario:
    //   - N=20 odometry fixes, header stamps h_k = 0.1*k, ego x = k.
    //   - Cloud with header T_pc = 2.2, ahead of the latest fix (1.9) — buffered.
    //   - No covering fix ever arrives; a non-covering trigger fix (header 1.95, ego x = 19.5) is
    //     published with an advanced receive time such that (now - receive) = 0.4 >= max_delay (0.3).
    //   - The timeout release places the cloud at predict(T_pc = 2.2), which EXTRAPOLATES forward from
    //     the last known state (h=1.95, x=19.5 at v≈10 m/s), so position[0] > 19.5.
    //   - on_point_cloud fires exactly once, at timeout release.
    //
    // timesync_max_delay_seconds = 0.3.
    void test_dds_timesync_timeout_extrapolation()
    {
        const std::string localization_topic{"rt/test_timesync_timeout_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_timesync_timeout_pointcloud2_topic"};
        const std::size_t num_fixes = 20;
        // Cloud header — ahead of the last ramp fix (1.9) so it stays buffered.
        const double t_pc = 2.2;

        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::atomic<std::size_t> calls{0};
        std::atomic<std::size_t> callback_count{0};

        // time_source sequencing:
        //   calls 0..N-1  : ramp fixes     → receive times = header times (0.1*k)
        //   call  N       : on_pc2 (buffer)→ receive time for the cloud (= t_pc = 2.2)
        //   call  N+1     : trigger fix    → receive time = t_pc + 0.4 = 2.6
        //                                    → (now - receive) = 2.6 - 2.2 = 0.4 >= 0.3 → timeout
        std::vector<double> times(num_fixes + 2);
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            times[k] = 0.1 * static_cast<double>(k);
        }
        times[num_fixes] = t_pc;            // cloud receive time
        times[num_fixes + 1] = t_pc + 0.4;  // trigger fix receive time; exceeds max_delay (0.3) since cloud arrived

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // ego frame == radar frame
        options.accumulation = no_filters_options();
        options.kalman_localization = true;
        options.timesync_max_delay_seconds = 0.3;
        options.time_source = [&calls, &times] {
            const std::size_t i = calls.fetch_add(1);
            return times[std::min(i, times.size() - 1)];
        };
        options.on_point_cloud = [&callback_count](acc::dds_point_clouds_accumulator &) { ++callback_count; };

        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_ts_timeout_radar"};
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(localization_publisher, pc2_publisher);

        // Constant-velocity ramp: ego x = k; header stamp h_k = 0.1*k.
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            const double h_k = 0.1 * static_cast<double>(k);
            publish_odometry(*localization_publisher, acc::rigid_transform{{static_cast<double>(k), 0, 0}, {0, 0, 0}},
                             "provizio_radar_front_center", "test_world", h_k);
        }
        // Wait until all ramp fixes are processed before the cloud arrives.
        for (int waited = 0; calls.load() < num_fixes && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes, "dds_timesync_timeout: all ramp fixes processed before the cloud");

        // Cloud header T_pc = 2.2 is ahead of latest localization header 1.9 → buffered.
        publish_pc2(*pc2_publisher, radar_points{pc2::radar_point{0, 0, 0, 0, 100, 0}}, radar_id, t_pc);

        // Wait until on_pc2 has executed (calls bumped to N+1).
        for (int waited = 0; calls.load() < num_fixes + 1 && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes + 1, "dds_timesync_timeout: on_pc2 callback executed");

        // Cloud is buffered; its own flush sees (now - receive) = 0, not yet timed out.
        check(accumulator.get_points_local_frame_relative().empty(),
              "dds_timesync_timeout: cloud must be buffered immediately after on_pc2");
        check(callback_count.load() == 0, "dds_timesync_timeout: no callback while cloud is buffered");

        // Non-covering trigger fix: header 1.95 < T_pc = 2.2, does NOT cover the cloud.
        // Its on_odometry call gets receive time times[N+1] = 2.6; flush sees (2.6 - 2.2) = 0.4 >= 0.3 → timeout.
        // The cloud is placed at predict(T_pc = 2.2): forward extrapolation from the Kalman state
        // (last known fix at header 1.95 with x=19.5, velocity ≈ 10 m/s) → position[0] > 19.5.
        const double trigger_header = 1.95;
        const double trigger_x = 19.5;  // x = 10 * h = 19.5 (continuing the CV ramp)
        publish_odometry(*localization_publisher, acc::rigid_transform{{trigger_x, 0, 0}, {0, 0, 0}},
                         "provizio_radar_front_center", "test_world", trigger_header);

        // Wait for the timeout-released cloud to be accumulated.
        wait_for_accumulated_points(accumulator, 1);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 1, "dds_timesync_timeout: one accumulated point after timeout release");

        // predict(2.2) from the last state (h=1.95, x=19.5, v≈10 m/s) extrapolates dt=0.25s forward → ≈22.
        // The key invariant: position[0] must be GREATER than the trigger fix's x (19.5), proving forward
        // extrapolation, not clamping to the stale last-fix value.
        check(local[0].position[0] > trigger_x,
              "dds_timesync_timeout: cloud placed at forward-extrapolated pose, not clamped to last fix");

        // on_point_cloud fires exactly once, at timeout release (triggered by the non-covering odometry fix).
        check(callback_count.load() == 1, "dds_timesync_timeout: on_point_cloud fired exactly once at timeout release");
    }

    // Verifies that an out-of-order (stale) localization fix is silently dropped under timesync:
    // it must not overwrite latest_ego_localization, update the Kalman filter, become previous_fix /
    // latest_fix, or trigger a buffer flush.
    //
    // Scenario:
    //   - N=20 monotonic odometry fixes, header stamps h_k = 0.1*k, ego x = 10*h_k = k.
    //     After warmup: latest_fix = {header=1.9, x=19}.
    //   - Cloud with header 1.95 is published — buffered (not yet covered).
    //   - A STALE fix arrives with header 1.5 and ego x = 999 (obviously wrong).
    //     Because header_s (1.5) < latest_fix->header_seconds (1.9), it is dropped before any state
    //     mutation and before time_source() is called (no calls increment for the dropped fix).
    //   - A covering fix arrives at header 2.0, ego x = 20.
    //     previous_fix must still be {1.9, x=19} (not the stale 1.5/x=999 entry), so the
    //     interpolated position is 19.5, not some value corrupted by the dropped fix.
    //
    // timesync_max_delay_seconds = 100.0 (timeout never fires during the test).
    void test_dds_timesync_drops_out_of_order_fix()
    {
        const std::string localization_topic{"rt/test_timesync_drop_oor_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_timesync_drop_oor_pointcloud2_topic"};
        const std::size_t num_fixes = 20;
        // Cloud header strictly between h_{N-1}=1.9 and the covering fix at 2.0.
        const double t_pc = 1.95;

        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::atomic<std::size_t> calls{0};
        std::atomic<std::size_t> callback_count{0};

        // time_source sequencing (the stale fix is dropped before time_source() is called):
        //   calls 0..N-1 : ramp fixes     → receive times = header times (0.1*k)
        //   call  N      : on_pc2 (buffer) → receive time = t_pc (well inside the 100s timeout)
        //   [stale fix at header 1.5 is dropped with no time_source() call]
        //   call  N+1    : covering fix    → receive time = t_pc + 0.1
        std::vector<double> times(num_fixes + 2);
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            times[k] = 0.1 * static_cast<double>(k);
        }
        times[num_fixes] = t_pc;
        times[num_fixes + 1] = t_pc + 0.1;

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // ego frame == radar frame
        options.accumulation = no_filters_options();
        options.kalman_localization = true;
        options.timesync_max_delay_seconds = 100.0;
        options.time_source = [&calls, &times] {
            const std::size_t i = calls.fetch_add(1);
            return times[std::min(i, times.size() - 1)];
        };
        options.on_point_cloud = [&callback_count](acc::dds_point_clouds_accumulator &) { ++callback_count; };

        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_ts_drop_oor_radar"};
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(localization_publisher, pc2_publisher);

        // Constant-velocity ramp: ego x = 10 * h_k = k; header stamp h_k = 0.1*k.
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            const double h_k = 0.1 * static_cast<double>(k);
            publish_odometry(*localization_publisher, acc::rigid_transform{{10.0 * h_k, 0, 0}, {0, 0, 0}},
                             "provizio_radar_front_center", "test_world", h_k);
        }
        // Wait until all ramp fixes are processed before the cloud.
        for (int waited = 0; calls.load() < num_fixes && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes, "dds_timesync_drop_oor: all ramp fixes processed before the cloud");

        // Cloud header 1.95 is ahead of the latest localization header 1.9 — buffered.
        publish_pc2(*pc2_publisher, radar_points{pc2::radar_point{0, 0, 0, 0, 100, 0}}, radar_id, t_pc);

        // Wait until on_pc2 has executed (calls bumped to N+1).
        for (int waited = 0; calls.load() < num_fixes + 1 && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes + 1, "dds_timesync_drop_oor: on_pc2 callback executed");

        // Cloud must be buffered: no accumulated points yet.
        check(accumulator.get_points_local_frame_relative().empty(),
              "dds_timesync_drop_oor: cloud must be buffered, not accumulated immediately");
        check(callback_count.load() == 0,
              "dds_timesync_drop_oor: on_point_cloud must not fire while cloud is buffered");

        // Publish a STALE fix with header 1.5 (< latest_fix header 1.9) and an obviously-wrong x=999.
        // This must be silently dropped (no state mutation, no time_source() call).
        const double stale_header = 1.5;
        const double stale_x = 999.0;
        publish_odometry(*localization_publisher, acc::rigid_transform{{stale_x, 0, 0}, {0, 0, 0}},
                         "provizio_radar_front_center", "test_world", stale_header);

        // Allow the stale fix to be processed (it should be a no-op).
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // The stale fix must not have accumulated the cloud or fired the callback.
        check(accumulator.get_points_local_frame_relative().empty(),
              "dds_timesync_drop_oor: stale fix must not release the buffered cloud");
        check(callback_count.load() == 0, "dds_timesync_drop_oor: on_point_cloud must not fire after stale fix");
        // The stale fix is dropped before time_source(), so calls must still be exactly N+1.
        check(calls.load() == num_fixes + 1, "dds_timesync_drop_oor: stale fix must not call time_source");

        // Covering fix at header 2.0, ego x = 20. previous_fix must still be {1.9, x=19}.
        // frac = (1.95 - 1.9) / (2.0 - 1.9) = 0.5 → interpolated x = 19.5.
        const double covering_header = 2.0;
        const double covering_x = 20.0;
        publish_odometry(*localization_publisher, acc::rigid_transform{{covering_x, 0, 0}, {0, 0, 0}},
                         "provizio_radar_front_center", "test_world", covering_header);

        wait_for_accumulated_points(accumulator, 1);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 1, "dds_timesync_drop_oor: one accumulated point after covering fix");

        // If the stale fix had corrupted previous_fix the bracket would be wrong and interpolation would
        // not yield 19.5. The correct result proves the stale fix was fully ignored.
        check_near(local[0].position[0], 19.5, 1e-2,
                   "dds_timesync_drop_oor: point placed at interpolated pose x ≈ 19.5 (stale fix ignored)");

        check(callback_count.load() == 1, "dds_timesync_drop_oor: on_point_cloud fired exactly once at release");
    }

    // Verifies that a cloud whose header stamp is OLDER than the last-but-one retained fix is placed at
    // previous_fix (the closer, older kept fix), not at latest_fix.
    //
    // Scenario:
    //   - N=20 monotonic odometry fixes, header stamps h_k = 0.1*k, ego x = 10*h_k = k.
    //     After warmup: latest_fix  = {header=2.0, x=20},
    //                   previous_fix = {header=1.9, x=19}.
    //   - Cloud published with header T_pc = 1.5 — covered on arrival (latest_fix.header 2.0 >= 1.5)
    //     and released immediately, but T_pc < previous_fix.header (1.9), so the cloud is out-of-bracket.
    //   - Expected: position[0] ≈ 19 (previous_fix pose), not 20 (latest_fix) and not 15 (true-time pose).
    //
    // timesync_max_delay_seconds = 100.0 (timeout never fires during the test).
    void test_dds_timesync_stale_cloud_uses_previous_fix()
    {
        const std::string localization_topic{"rt/test_timesync_stale_cloud_localization_topic"};
        const std::string pointcloud2_topic{"rt/test_timesync_stale_cloud_pointcloud2_topic"};
        const std::size_t num_fixes = 20;
        // Cloud header is older than previous_fix (1.9); covered immediately by the last fix (2.0).
        const double t_pc = 1.5;

        const auto publishers_participant = provizio::dds::make_domain_participant(test_domain);
        const auto subscribers_participant = provizio::dds::make_domain_participant(test_domain);

        std::atomic<std::size_t> calls{0};
        std::atomic<std::size_t> callback_count{0};

        // time_source sequencing:
        //   calls 0..N-1 : ramp fixes     → receive times = header times (0.1*k)
        //   call  N      : on_pc2 (buffer then immediate release) → receive time = t_pc
        // The cloud is covered on arrival (latest_fix = 2.0 >= 1.5) so it is released inside on_pc2
        // without waiting for an additional fix.
        std::vector<double> times(num_fixes + 1);
        for (std::size_t k = 0; k < num_fixes; ++k)
        {
            times[k] = 0.1 * static_cast<double>(k);
        }
        times[num_fixes] = t_pc;  // cloud receive time (well inside the 100s timeout)

        acc::dds_accumulation_options options;
        options.localization = acc::localization_source::odometry;
        options.localization_topic = localization_topic;
        options.pointcloud2_topic = pointcloud2_topic;
        options.extrinsics_topics.clear();  // ego frame == radar frame
        options.accumulation = no_filters_options();
        options.kalman_localization = true;
        options.timesync_max_delay_seconds = 100.0;
        options.time_source = [&calls, &times] {
            const std::size_t i = calls.fetch_add(1);
            return times[std::min(i, times.size() - 1)];
        };
        options.on_point_cloud = [&callback_count](acc::dds_point_clouds_accumulator &) { ++callback_count; };

        const acc::dds_point_clouds_accumulator accumulator{2, options, subscribers_participant};

        const std::string radar_id{"test_ts_stale_cloud_radar"};
        const auto localization_publisher = provizio::dds::make_publisher<nav_msgs::msg::OdometryPubSubType>(
            publishers_participant, localization_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        const auto pc2_publisher = provizio::dds::make_publisher<sensor_msgs::msg::PointCloud2PubSubType>(
            publishers_participant, pointcloud2_topic, provizio::dds::RELIABLE_RELIABILITY_QOS);
        wait_until_matched(localization_publisher, pc2_publisher);

        // Constant-velocity ramp: ego x = 10 * h_k = k; header stamp h_k = 0.1*k.
        // After k=19: latest_fix = {header=1.9, x=19}.
        // After k=20 (final fix at h=2.0, x=20): latest_fix = {2.0, x=20}, previous_fix = {1.9, x=19}.
        for (std::size_t k = 1; k <= num_fixes; ++k)
        {
            const double h_k = 0.1 * static_cast<double>(k);
            publish_odometry(*localization_publisher, acc::rigid_transform{{10.0 * h_k, 0, 0}, {0, 0, 0}},
                             "provizio_radar_front_center", "test_world", h_k);
        }
        // Wait until all ramp fixes are processed before the cloud.
        for (int waited = 0; calls.load() < num_fixes && waited < 1000; ++waited)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check(calls.load() >= num_fixes, "dds_timesync_stale_cloud: all ramp fixes processed before the cloud");

        // Cloud header 1.5 is OLDER than previous_fix (1.9) but COVERED by latest_fix (2.0):
        // it should be buffered and released immediately inside on_pc2, using previous_fix.pose (x=19).
        publish_pc2(*pc2_publisher, radar_points{pc2::radar_point{0, 0, 0, 0, 100, 0}}, radar_id, t_pc);

        // Wait for the cloud to be accumulated (released inside on_pc2 callback).
        wait_for_accumulated_points(accumulator, 1);
        const auto local = accumulator.get_points_local_frame_relative();
        check(local.size() == 1, "dds_timesync_stale_cloud: one accumulated point");

        // previous_fix.pose has x=19; latest_fix.pose has x=20; true-time pose would have x=15.
        // The correct result is x ≈ 19 (closest retained fix, not the covering fix).
        check_near(local[0].position[0], 19.0, 1e-2,
                   "dds_timesync_stale_cloud: stale cloud placed at previous_fix pose, x ≈ 19");

        check(callback_count.load() == 1, "dds_timesync_stale_cloud: on_point_cloud fired exactly once");
    }
}  // namespace

int main(int argc, const char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: accumulation_test <subcommand>" << std::endl;
        return 2;
    }
    const std::string subcommand{argv[1]};  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    try
    {
        if (subcommand == "rigid_transform_golden")
        {
            test_rigid_transform_golden();
        }
        else if (subcommand == "zero_accumulated")
        {
            test_zero_accumulated();
        }
        else if (subcommand == "extrinsics")
        {
            test_extrinsics();
        }
        else if (subcommand == "move_no_extrinsics")
        {
            test_move_no_extrinsics();
        }
        else if (subcommand == "move_simple_extrinsics")
        {
            test_move_simple_extrinsics();
        }
        else if (subcommand == "overflow")
        {
            test_overflow();
        }
        else if (subcommand == "localization_from_sensor_to_ego")
        {
            test_localization_from_sensor_to_ego();
        }
        else if (subcommand == "localization_correction")
        {
            test_localization_correction();
        }
        else if (subcommand == "move_simple_extrinsics_snr_filter")
        {
            test_move_simple_extrinsics_snr_filter();
        }
        else if (subcommand == "snr_and_velocity_filters")
        {
            test_snr_and_velocity_filters();
        }
        else if (subcommand == "radar_filter")
        {
            test_radar_filter();
        }
        else if (subcommand == "rotation_yaw")
        {
            test_rotation_yaw();
        }
        else if (subcommand == "rotation_pitch")
        {
            test_rotation_pitch();
        }
        else if (subcommand == "rotation_roll")
        {
            test_rotation_roll();
        }
        else if (subcommand == "rotation_yaw_with_extrinsics")
        {
            test_rotation_yaw_with_extrinsics();
        }
        else if (subcommand == "rotation_and_move_simple_no_extrinsics")
        {
            test_rotation_and_move_simple_no_extrinsics();
        }
        else if (subcommand == "rotation_and_move_simple_with_extrinsics")
        {
            test_rotation_and_move_simple_with_extrinsics();
        }
        else if (subcommand == "rotation_and_move_no_extrinsics")
        {
            test_rotation_and_move_no_extrinsics();
        }
        else if (subcommand == "rotation_and_move_with_extrinsics")
        {
            test_rotation_and_move_with_extrinsics();
        }
        else if (subcommand == "gps_utils_golden")
        {
            test_gps_utils_golden();
        }
        else if (subcommand == "dds_simple")
        {
            test_dds_simple();
        }
        else if (subcommand == "dds_simple_extrinsics")
        {
            test_dds_simple_extrinsics();
        }
        else if (subcommand == "dds_simple_extrinsics_including_localization")
        {
            test_dds_simple_extrinsics_including_localization();
        }
        else if (subcommand == "dds_localization_extrinsics_late")
        {
            test_dds_localization_extrinsics_late();
        }
        else if (subcommand == "dds_no_localization_no_extrinsics")
        {
            test_dds_no_localization_no_extrinsics();
        }
        else if (subcommand == "dds_no_localization_with_extrinsics")
        {
            test_dds_no_localization_with_extrinsics();
        }
        else if (subcommand == "dds_on_pc2_callback")
        {
            test_dds_on_pc2_callback();
        }
        else if (subcommand == "dds_nav_sat_fix_localization")
        {
            test_dds_nav_sat_fix_localization();
        }
        else if (subcommand == "dds_kalman_localization")
        {
            test_dds_kalman_localization();
        }
        else if (subcommand == "dds_timesync_buffered_then_covered")
        {
            test_dds_timesync_buffered_then_covered();
        }
        else if (subcommand == "dds_timesync_timeout_extrapolation")
        {
            test_dds_timesync_timeout_extrapolation();
        }
        else if (subcommand == "dds_timesync_interpolates_between_fixes")
        {
            test_dds_timesync_interpolates_between_fixes();
        }
        else if (subcommand == "dds_timesync_interpolates_orientation")
        {
            test_dds_timesync_interpolates_orientation();
        }
        else if (subcommand == "dds_timesync_drops_out_of_order_fix")
        {
            test_dds_timesync_drops_out_of_order_fix();
        }
        else if (subcommand == "dds_timesync_stale_cloud_uses_previous_fix")
        {
            test_dds_timesync_stale_cloud_uses_previous_fix();
        }
        else
        {
            std::cerr << "Unknown subcommand: " << subcommand << std::endl;
            return 2;
        }
    }
    catch (const std::exception &exception)
    {
        std::cerr << "accumulation_test " << subcommand << " FAILED: " << exception.what() << std::endl;
        return 1;
    }
    std::cout << "accumulation_test " << subcommand << " PASSED" << std::endl;
    return 0;
}
