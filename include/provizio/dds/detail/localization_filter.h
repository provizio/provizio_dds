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

#ifndef DDS_DETAIL_LOCALIZATION_FILTER
#define DDS_DETAIL_LOCALIZATION_FILTER

#include <array>
#include <cstddef>

#include "provizio/dds/accumulation.h"  // rigid_transform + math::euler_from_matrix
#include "provizio/dds/common.h"

namespace provizio::dds::accumulation::detail
{
    /**
     * @brief Decoupled constant-velocity Kalman filter over an ego pose (x, y, z, roll, pitch, yaw). Used to
     * estimate the ego localization at a point cloud's receive time from the lower-rate, jittery localization
     * stream. Six independent [value, rate] channels; each channel's rate (velocity) is estimated internally
     * from successive fixes. roll/pitch/yaw are angle channels (wrapped to [-pi, pi)).
     *
     * Times are an opaque monotonic clock in seconds; only differences are used. With fewer than two updates the
     * rates are still zero, so predict() returns ~the last fix (matching the pre-filter behaviour). NOT internally
     * synchronized: the caller holds the accumulator's mutex.
     */
    class localization_filter
    {
      public:
        /**
         * @brief Constructs an uninitialized filter: has_estimate() is false and predict() returns an identity
         * pose until the first update().
         */
        PROVIZIO_DDS_API localization_filter() noexcept;

        /**
         * @brief Folds a localization fix into the filter, refining the internally estimated per-channel rates.
         *
         * @param time_seconds Fix time on an opaque monotonic clock (seconds); only differences between calls are
         * used, so any consistent epoch works. A non-monotonic (backwards) time is clamped to a zero time step.
         * @param pose The measured ego pose (x, y, z, roll, pitch, yaw) at @p time_seconds.
         */
        PROVIZIO_DDS_API void update(double time_seconds, const rigid_transform &pose) noexcept;

        /**
         * @brief Estimates the ego pose at a query time by extrapolating the last fix with the estimated rates.
         *
         * @param time_seconds Query time on the same monotonic clock passed to update().
         * @return The predicted pose. With fewer than two updates the rates are still zero, so the result is
         * approximately the most recent fix (an identity pose before any update()); angle channels are wrapped to
         * [-pi, pi).
         */
        PROVIZIO_DDS_API rigid_transform predict(double time_seconds) const noexcept;

        /**
         * @brief Whether at least one fix has been folded in.
         * @return true once update() has been called at least once; false on a freshly constructed filter.
         */
        PROVIZIO_DDS_API bool has_estimate() const noexcept;

        /**
         * @brief Discards the estimate, returning the filter to its freshly-constructed state (has_estimate() is
         * false; the next update() re-initializes from that fix). Used when the localization extrinsics changes:
         * the estimated rates were built from poses in the old extrinsics, so they are dropped rather than mapped
         * (a rigid right-multiply does not transform the decomposed per-channel pose and rate cleanly), and the
         * filter re-converges from subsequent fixes.
         */
        PROVIZIO_DDS_API void reset() noexcept;

      private:
        static constexpr std::size_t channels = 6;
        bool initialized{false};
        double last_time{0};
        std::array<double, channels> value{};
        std::array<double, channels> rate{};
        std::array<std::array<std::array<double, 2>, 2>, channels> covariance{};
    };
}  // namespace provizio::dds::accumulation::detail

#endif  // DDS_DETAIL_LOCALIZATION_FILTER
