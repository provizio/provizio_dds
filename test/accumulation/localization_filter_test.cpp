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
#include <cmath>
#include <iostream>
#include <string>

namespace acc = provizio::dds::accumulation;
using acc::rigid_transform;
using acc::detail::localization_filter;

namespace
{
    int failures = 0;
    void check_near(const double actual, const double expected, const double tol, const std::string &msg)
    {
        if (std::abs(actual - expected) > tol)
        {
            std::cerr << "FAIL: " << msg << " (actual " << actual << ", expected " << expected << ")\n";
            ++failures;
        }
    }
    rigid_transform pose(const double x, const double y, const double yaw)
    {
        return rigid_transform{{x, y, 0.0}, std::array<double, 3>{0.0, 0.0, yaw}};
    }
    void test_cold_start()
    {
        localization_filter f;
        if (f.has_estimate())
        {
            std::cerr << "FAIL: empty filter has_estimate\n";
            ++failures;
        }
        f.update(100.0, pose(5.0, 0.0, 0.0));
        if (!f.has_estimate())
        {
            std::cerr << "FAIL: has_estimate after update\n";
            ++failures;
        }
        check_near(f.predict(100.5).translation()[0], 5.0, 1e-9, "cold-start predict holds last x");
    }
    void test_constant_velocity()
    {
        localization_filter f;
        const double v = 2.0;
        double t = 0.0;
        for (int k = 0; k < 25; ++k)
        {
            f.update(t, pose(v * t, 0.0, 0.0));
            t += 0.1;
        }
        check_near(f.predict(2.45).translation()[0], v * 2.45, 0.02, "CV predict extrapolates x");
    }
    void test_constant_yaw_rate()
    {
        localization_filter f;
        const double w = 0.5;
        double t = 0.0;
        for (int k = 0; k < 25; ++k)
        {
            f.update(t, pose(0.0, 0.0, std::atan2(std::sin(w * t), std::cos(w * t))));
            t += 0.1;
        }
        const auto rpy = acc::math::euler_from_matrix(f.predict(2.45).matrix());
        check_near(std::atan2(std::sin(rpy[2] - w * 2.45), std::cos(rpy[2] - w * 2.45)), 0.0, 0.02,
                   "CV predict extrapolates yaw");
    }
    void test_yaw_wrap()
    {
        localization_filter f;
        f.update(0.0, pose(0.0, 0.0, 3.0));
        f.update(0.1, pose(0.0, 0.0, -3.0));
        const auto rpy = acc::math::euler_from_matrix(f.predict(0.1).matrix());
        if (std::abs(rpy[2]) < 2.5)
        {
            std::cerr << "FAIL: yaw wrap produced a mid-range angle\n";
            ++failures;
        }
    }
}  // namespace

int main(int argc, char **argv)
{
    const std::string which = argc > 1 ? argv[1] : "";
    if (which == "cold_start")
        test_cold_start();
    else if (which == "constant_velocity")
        test_constant_velocity();
    else if (which == "constant_yaw_rate")
        test_constant_yaw_rate();
    else if (which == "yaw_wrap")
        test_yaw_wrap();
    else
    {
        std::cerr << "unknown subcommand: " << which << "\n";
        return 2;
    }
    return failures == 0 ? 0 : 1;
}
