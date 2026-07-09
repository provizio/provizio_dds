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

#include "provizio/dds/detail/localization_filter.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace provizio::dds::accumulation::detail
{
    namespace
    {
        constexpr double pi_value = 3.14159265358979323846;

        constexpr bool is_angle_channel(const std::size_t channel) noexcept
        {
            return channel >= 3;
        }

        double wrap_to_pi(const double angle) noexcept
        {
            const double wrapped = std::fmod(angle + pi_value, 2 * pi_value);
            return (wrapped < 0 ? wrapped + 2 * pi_value : wrapped) - pi_value;
        }

        // Measurement noise variances and process (white-acceleration) spectral densities, per channel group.
        // Small measurement variance => trust the fixes; moderate process density => allow real acceleration.
        constexpr double r_position = 0.1 * 0.1;  // (m)^2
        constexpr double r_angle = 0.02 * 0.02;   // (rad)^2
        constexpr double q_position = 1.0;        // (m/s^2)^2
        constexpr double q_angle = 0.5;           // (rad/s^2)^2
        // initial covariance
        constexpr double initial_covariance = 1.0;

        double measurement_variance(const std::size_t channel) noexcept
        {
            return is_angle_channel(channel) ? r_angle : r_position;
        }
        double process_density(const std::size_t channel) noexcept
        {
            return is_angle_channel(channel) ? q_angle : q_position;
        }

        // Number of pose channels: [x, y, z, roll, pitch, yaw].
        constexpr std::size_t pose_channels = 6;

        std::array<double, pose_channels> pose_to_vector(const rigid_transform &pose) noexcept
        {
            const auto translation = pose.translation();
            const auto rpy = math::euler_from_matrix(pose.matrix());
            return {translation[0], translation[1], translation[2], rpy[0], rpy[1], rpy[2]};
        }
    }  // namespace

    localization_filter::localization_filter() noexcept = default;

    bool localization_filter::has_estimate() const noexcept
    {
        return initialized;
    }

    void localization_filter::reset() noexcept
    {
        initialized = false;
    }

    void localization_filter::update(const double time_seconds, const rigid_transform &pose) noexcept
    {
        const auto measurement = pose_to_vector(pose);
        for (const double channel_measurement : measurement)
        {
            if (std::isnan(channel_measurement))
            {
                // A NaN pose (e.g. from a malformed odometry message) would poison value/covariance for the rest
                // of the session: initialized never clears, so has_estimate() would keep returning a NaN estimate.
                // Drop the update and keep the prior state intact.
                return;
            }
        }
        if (!initialized)
        {
            for (std::size_t channel = 0; channel < channels; ++channel)
            {
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                value[channel] = measurement[channel];
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                rate[channel] = 0;
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                covariance[channel] = {{{initial_covariance, 0}, {0, initial_covariance}}};
            }
            last_time = time_seconds;
            initialized = true;
            return;
        }

        double delta_t = time_seconds - last_time;
        if (delta_t < 0)
        {
            delta_t = 0;  // a monotonic clock is expected; clamp defensively
        }
        const double dt2 = delta_t * delta_t;
        const double dt3 = dt2 * delta_t;

        for (std::size_t channel = 0; channel < channels; ++channel)
        {
            // Predict: value advances by its rate; covariance P = F P F^T + Q, F = [[1, dt],[0, 1]]
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            value[channel] += rate[channel] * delta_t;
            if (is_angle_channel(channel))
            {
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                value[channel] = wrap_to_pi(value[channel]);
            }
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            auto &channel_covariance = covariance[channel];
            const double p00 = channel_covariance[0][0] +
                               delta_t * (channel_covariance[1][0] + channel_covariance[0][1]) +
                               dt2 * channel_covariance[1][1];
            const double p01 = channel_covariance[0][1] + delta_t * channel_covariance[1][1];
            const double p10 = channel_covariance[1][0] + delta_t * channel_covariance[1][1];
            const double p11 = channel_covariance[1][1];
            const double process_noise = process_density(channel);
            // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            channel_covariance[0][0] = p00 + process_noise * dt3 / 3.0;
            // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            channel_covariance[0][1] = p01 + process_noise * dt2 / 2.0;
            // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            channel_covariance[1][0] = p10 + process_noise * dt2 / 2.0;
            channel_covariance[1][1] = p11 + process_noise * delta_t;

            // Measurement update with measurement[channel], H = [1, 0]
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            double innovation = measurement[channel] - value[channel];
            if (is_angle_channel(channel))
            {
                innovation = wrap_to_pi(innovation);
            }
            const double innovation_cov = channel_covariance[0][0] + measurement_variance(channel);
            const double gain_value = channel_covariance[0][0] / innovation_cov;
            const double gain_rate = channel_covariance[1][0] / innovation_cov;
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            value[channel] += gain_value * innovation;
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            rate[channel] += gain_rate * innovation;
            if (is_angle_channel(channel))
            {
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                value[channel] = wrap_to_pi(value[channel]);
            }
            const double n00 = (1 - gain_value) * channel_covariance[0][0];
            const double n01 = (1 - gain_value) * channel_covariance[0][1];
            const double n10 = channel_covariance[1][0] - gain_rate * channel_covariance[0][0];
            const double n11 = channel_covariance[1][1] - gain_rate * channel_covariance[0][1];
            channel_covariance[0][0] = n00;
            channel_covariance[0][1] = n01;
            channel_covariance[1][0] = n10;
            channel_covariance[1][1] = n11;
        }
        // Only ever advance the clock; an out-of-order (already dt-clamped) fix must not regress last_time,
        // or the next update would compute an inflated dt from the older timestamp.
        if (time_seconds > last_time)
        {
            last_time = time_seconds;
        }
    }

    rigid_transform localization_filter::predict(const double time_seconds) const noexcept
    {
        if (!initialized)
        {
            return rigid_transform{};
        }
        double delta_t = time_seconds - last_time;
        if (delta_t < 0)
        {
            delta_t = 0;
        }
        std::array<double, channels> predicted{};
        for (std::size_t channel = 0; channel < channels; ++channel)
        {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            double predicted_value = value[channel] + rate[channel] * delta_t;
            if (is_angle_channel(channel))
            {
                predicted_value = wrap_to_pi(predicted_value);
            }
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            predicted[channel] = predicted_value;
        }
        return rigid_transform{{predicted[0], predicted[1], predicted[2]},
                               // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
                               std::array<double, 3>{predicted[3], predicted[4], predicted[5]}};
    }
}  // namespace provizio::dds::accumulation::detail
