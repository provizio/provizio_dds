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
//
// C++ port of python/gps_utils.py, which is MIT-licensed:
//
// MIT License
// Copyright (c) 2019 Michail Kalaitzakis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef DDS_DETAIL_GPS_UTILS
#define DDS_DETAIL_GPS_UTILS

#include <array>
#include <cmath>
#include <cstddef>

namespace provizio::dds::accumulation::detail
{
    /**
     * @brief Converts GPS readings (latitude, longitude, height) to a local Cartesian ENU system
     * (https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates). Internal: used by
     * dds_point_clouds_accumulator's NavSatFix localization path.
     *
     * Use set_enu_origin(lat, lon, height) to set the local ENU origin, then geo_to_enu(lat, lon, height) to get
     * positions in that local ENU frame. WGS-84 geodetic model.
     */
    class gps_utils
    {
      public:
        /**
         * @brief Sets the local ENU coordinate system origin.
         * @param lat_deg Latitude in degrees.
         * @param lon_deg Longitude in degrees.
         * @param height_m Height above the WGS-84 ellipsoid in metres.
         */
        void set_enu_origin(const double lat_deg, const double lon_deg, const double height_m)
        {
            origin_ecef = geo_to_ecef(lat_deg, lon_deg, height_m);

            const double phi = radians(lat_deg);
            const double lambda_rad = radians(lon_deg);
            const double cos_phi = std::cos(phi);
            const double cos_lambda = std::cos(lambda_rad);
            const double sin_phi = std::sin(phi);
            const double sin_lambda = std::sin(lambda_rad);

            // ECEF -> ENU rotation at the origin
            rotation[0] = {-sin_lambda, cos_lambda, 0.0};
            rotation[1] = {-sin_phi * cos_lambda, -sin_phi * sin_lambda, cos_phi};
            rotation[2] = {cos_phi * cos_lambda, cos_phi * sin_lambda, sin_phi};
        }

        /**
         * @brief Converts a geodetic position to the local ENU frame (set_enu_origin must have been called).
         * @param lat_deg Latitude in degrees.
         * @param lon_deg Longitude in degrees.
         * @param height_m Height above the WGS-84 ellipsoid in metres.
         * @return std::array<double,3> of (east, north, up) in metres.
         */
        std::array<double, 3> geo_to_enu(const double lat_deg, const double lon_deg, const double height_m) const
        {
            const auto ecef = geo_to_ecef(lat_deg, lon_deg, height_m);
            const std::array<double, 3> delta{ecef[0] - origin_ecef[0], ecef[1] - origin_ecef[1],
                                              ecef[2] - origin_ecef[2]};
            std::array<double, 3> enu{};
            for (std::size_t row = 0; row < 3; ++row)
            {
                enu[row] = rotation[row][0] * delta[0] + rotation[row][1] * delta[1] + rotation[row][2] * delta[2];
            }
            return enu;
        }

      private:
        static double radians(const double degrees)
        {
            constexpr double pi = 3.14159265358979323846;
            return degrees * pi / 180.0;
        }

        static std::array<double, 3> geo_to_ecef(const double lat_deg, const double lon_deg, const double height_m)
        {
            const double phi = radians(lat_deg);
            const double lambda_rad = radians(lon_deg);
            const double cos_phi = std::cos(phi);
            const double cos_lambda = std::cos(lambda_rad);
            const double sin_phi = std::sin(phi);
            const double sin_lambda = std::sin(lambda_rad);

            const double n = semi_major_axis / std::sqrt(1.0 - first_eccentricity_squared * sin_phi * sin_phi);

            return {(n + height_m) * cos_phi * cos_lambda, (n + height_m) * cos_phi * sin_lambda,
                    ((semi_minor_axis_squared / semi_major_axis_squared) * n + height_m) * sin_phi};
        }

        // Geodetic System WGS-84 axes
        static constexpr double semi_major_axis = 6378137.0;
        static constexpr double semi_minor_axis = 6356752.314245;
        static constexpr double semi_major_axis_squared = semi_major_axis * semi_major_axis;
        static constexpr double semi_minor_axis_squared = semi_minor_axis * semi_minor_axis;
        static constexpr double first_eccentricity_squared = 1.0 - (semi_minor_axis_squared / semi_major_axis_squared);

        std::array<double, 3> origin_ecef{};
        std::array<std::array<double, 3>, 3> rotation{};
    };
}  // namespace provizio::dds::accumulation::detail

#endif  // DDS_DETAIL_GPS_UTILS
