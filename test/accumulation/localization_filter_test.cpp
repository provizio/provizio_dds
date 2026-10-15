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

// Subcommand-driven unit tests for provizio::dds::accumulation::detail::localization_filter.
// Each ctest entry runs one subcommand so per-case failure stays isolated,
// mirroring the accumulation_test style.

#include "provizio/dds/detail/localization_filter.h"
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace acc = provizio::dds::accumulation;
using acc::rigid_transform;
using acc::detail::localization_filter;

namespace
{
    // Records a failure (and says why) when actual is further than the tolerance from expected.
    // Returns the number of failures recorded: 1 for a mismatch, 0 otherwise.
    int check_near(const double actual, const double expected, const double tolerance, const std::string &msg)
    {
        if (std::abs(actual - expected) > tolerance)
        {
            std::cerr << "FAIL: " << msg << " (actual " << actual << ", expected " << expected << ")\n";
            return 1;
        }
        return 0;
    }

    rigid_transform pose(const double position_x, const double position_y, const double yaw)
    {
        return rigid_transform{{position_x, position_y, 0.0}, std::array<double, 3>{0.0, 0.0, yaw}};
    }

    // Returns the number of failures recorded by the test case.
    int test_cold_start()
    {
        constexpr double first_update_time_s = 100.0;
        constexpr double held_position_x_m = 5.0;
        constexpr double predict_time_s = 100.5;
        constexpr double hold_tolerance_m = 1e-9;

        int failures = 0;
        localization_filter filter;
        if (filter.has_estimate())
        {
            std::cerr << "FAIL: empty filter has_estimate\n";
            ++failures;
        }
        filter.update(first_update_time_s, pose(held_position_x_m, 0.0, 0.0));
        if (!filter.has_estimate())
        {
            std::cerr << "FAIL: has_estimate after update\n";
            ++failures;
        }
        failures += check_near(filter.predict(predict_time_s).translation()[0], held_position_x_m, hold_tolerance_m,
                               "cold-start predict holds last x");
        return failures;
    }

    // Returns the number of failures recorded by the test case.
    int test_constant_velocity()
    {
        constexpr double velocity_mps = 2.0;
        constexpr double sample_period_s = 0.1;
        constexpr int sample_count = 25;
        constexpr double predict_time_s = 2.45;
        constexpr double position_tolerance_m = 0.02;

        localization_filter filter;
        double time_s = 0.0;
        for (int sample_index = 0; sample_index < sample_count; ++sample_index)
        {
            filter.update(time_s, pose(velocity_mps * time_s, 0.0, 0.0));
            time_s += sample_period_s;
        }
        return check_near(filter.predict(predict_time_s).translation()[0], velocity_mps * predict_time_s,
                          position_tolerance_m, "CV predict extrapolates x");
    }

    // Returns the number of failures recorded by the test case.
    int test_constant_yaw_rate()
    {
        constexpr double yaw_rate_rps = 0.5;
        constexpr double sample_period_s = 0.1;
        constexpr int sample_count = 25;
        constexpr double predict_time_s = 2.45;
        constexpr double yaw_tolerance_rad = 0.02;

        localization_filter filter;
        double time_s = 0.0;
        for (int sample_index = 0; sample_index < sample_count; ++sample_index)
        {
            filter.update(time_s,
                          pose(0.0, 0.0, std::atan2(std::sin(yaw_rate_rps * time_s), std::cos(yaw_rate_rps * time_s))));
            time_s += sample_period_s;
        }
        const auto roll_pitch_yaw = acc::math::euler_from_matrix(filter.predict(predict_time_s).matrix());
        return check_near(std::atan2(std::sin(roll_pitch_yaw[2] - yaw_rate_rps * predict_time_s),
                                     std::cos(roll_pitch_yaw[2] - yaw_rate_rps * predict_time_s)),
                          0.0, yaw_tolerance_rad, "CV predict extrapolates yaw");
    }

    // Returns the number of failures recorded by the test case.
    int test_yaw_wrap()
    {
        // Two yaws just either side of the +/-pi wrap, 0.1 s apart: the short way round is a small rotation
        constexpr double yaw_before_wrap_rad = 3.0;
        constexpr double yaw_after_wrap_rad = -3.0;
        constexpr double sample_period_s = 0.1;
        // A prediction taking the long way round (through zero) would land well inside this magnitude
        constexpr double min_wrapped_yaw_magnitude_rad = 2.5;

        localization_filter filter;
        filter.update(0.0, pose(0.0, 0.0, yaw_before_wrap_rad));
        filter.update(sample_period_s, pose(0.0, 0.0, yaw_after_wrap_rad));
        const auto roll_pitch_yaw = acc::math::euler_from_matrix(filter.predict(sample_period_s).matrix());
        if (std::abs(roll_pitch_yaw[2]) < min_wrapped_yaw_magnitude_rad)
        {
            std::cerr << "FAIL: yaw wrap produced a mid-range angle\n";
            return 1;
        }
        return 0;
    }
}  // namespace

int main(int argc, char **argv)
{
    const std::vector<std::string> args(
        argv, std::next(argv, argc));  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const std::string which = args.size() > 1 ? args.at(1) : "";
    int failures = 0;
    if (which == "cold_start")
    {
        failures = test_cold_start();
    }
    else if (which == "constant_velocity")
    {
        failures = test_constant_velocity();
    }
    else if (which == "constant_yaw_rate")
    {
        failures = test_constant_yaw_rate();
    }
    else if (which == "yaw_wrap")
    {
        failures = test_yaw_wrap();
    }
    else
    {
        std::cerr << "unknown subcommand: " << which << "\n";
        return 2;
    }
    return failures == 0 ? 0 : 1;
}
