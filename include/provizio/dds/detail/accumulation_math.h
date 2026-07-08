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

// Linear algebra detail for provizio::dds::accumulation (header-only). Hybrid precision: all matrix composing,
// folding and inversion stays in double (large local-ENU coordinates must cancel before any precision is dropped),
// while the per-point batch transform runs in float32 — radar positions are float32 on the wire anyway, Eigen
// vectorizes 2x wider, and the worst-case error (~1.5e-4 m at 300 m range) is below both the radar resolution and
// the float32 output quantization for large local-frame coordinates. The only O(N) operation — the per-frame batch
// point transform — is implemented twice: with Eigen (vectorized) and as a plain-CPU loop.
// Which one compiles is decided RIGHT HERE, in the CONSUMER's translation unit: if the consumer's include paths can
// see Eigen, it is used; define PROVIZIO_DDS_DISABLE_EIGEN to force the plain-CPU fallback explicitly.
// libprovizio_dds never instantiates transform_points — the get_points* call path into it is header-inline by
// design — so prebuilt binaries / the bin cache are identical either way.
// The O(1) 4x4 helpers below are shared scalar code: they run once per frame/call, so there is nothing for Eigen to
// win there.

#ifndef DDS_ACCUMULATION
#error "Include provizio/dds/accumulation.h instead of this detail header directly."
#endif  // DDS_ACCUMULATION

#ifndef DDS_DETAIL_ACCUMULATION_MATH
#define DDS_DETAIL_ACCUMULATION_MATH

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#ifdef PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN
#error                                                                                                                 \
    "PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN is an output of this header's detection, not an input: define PROVIZIO_DDS_DISABLE_EIGEN to control the maths path"
#endif  // PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN

#if defined(PROVIZIO_DDS_DISABLE_EIGEN)
// Plain-CPU fallback explicitly requested
#elif defined(__has_include)
#if __has_include(<Eigen/Dense>)
#define PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN
#include <Eigen/Dense>
#endif  // __has_include(<Eigen/Dense>)
#endif  // PROVIZIO_DDS_DISABLE_EIGEN / __has_include

namespace provizio::dds::accumulation::math
{
    /**
     * @brief Returns the 4x4 identity matrix.
     */
    inline matrix4x4 identity() noexcept
    {
        matrix4x4 result{};
        result[0][0] = result[1][1] = result[2][2] = result[3][3] = 1.0;
        return result;
    }

    /**
     * @brief Multiplies two 4x4 matrices: result = a * b.
     */
    inline matrix4x4 multiply(const matrix4x4 &a, const matrix4x4 &b) noexcept
    {
        matrix4x4 result{};
        for (std::size_t row = 0; row < 4; ++row)
        {
            for (std::size_t column = 0; column < 4; ++column)
            {
                double sum = 0;
                for (std::size_t k = 0; k < 4; ++k)
                {
                    sum += a[row][k] * b[k][column];
                }
                result[row][column] = sum;
            }
        }
        return result;
    }

    /**
     * @brief Inverts a RIGID transform (orthonormal rotation + translation): inv = [R^T | -R^T*t].
     * Exact for every matrix this library composes; documented as a precondition of rigid_transform::from_matrix.
     */
    inline matrix4x4 rigid_inverse(const matrix4x4 &m) noexcept
    {
        matrix4x4 result = identity();
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                result[row][column] = m[column][row];
            }
        }
        for (std::size_t row = 0; row < 3; ++row)
        {
            result[row][3] = -(result[row][0] * m[0][3] + result[row][1] * m[1][3] + result[row][2] * m[2][3]);
        }
        return result;
    }

    /**
     * @brief Composes a 4x4 from translation + Euler angles (roll, pitch, yaw), static-frame 'sxyz' convention:
     * R = Rz(yaw) * Ry(pitch) * Rx(roll).
     */
    inline matrix4x4 compose(const std::array<double, 3> &translation,
                             const std::array<double, 3> &rotation_euler_roll_pitch_yaw) noexcept
    {
        const double cos_roll = std::cos(rotation_euler_roll_pitch_yaw[0]);
        const double sin_roll = std::sin(rotation_euler_roll_pitch_yaw[0]);
        const double cos_pitch = std::cos(rotation_euler_roll_pitch_yaw[1]);
        const double sin_pitch = std::sin(rotation_euler_roll_pitch_yaw[1]);
        const double cos_yaw = std::cos(rotation_euler_roll_pitch_yaw[2]);
        const double sin_yaw = std::sin(rotation_euler_roll_pitch_yaw[2]);

        matrix4x4 result = identity();
        result[0][0] = cos_yaw * cos_pitch;
        result[0][1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
        result[0][2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
        result[0][3] = translation[0];
        result[1][0] = sin_yaw * cos_pitch;
        result[1][1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
        result[1][2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
        result[1][3] = translation[1];
        result[2][0] = -sin_pitch;
        result[2][1] = cos_pitch * sin_roll;
        result[2][2] = cos_pitch * cos_roll;
        result[2][3] = translation[2];
        return result;
    }

    /**
     * @brief Composes a 4x4 from translation + quaternion (w, x, y, z); the quaternion is normalized first.
     * @note A zero-norm quaternion yields the identity rotation (translation preserved).
     */
    inline matrix4x4 compose_quaternion(const std::array<double, 3> &translation,
                                        const quaternion_wxyz &quaternion) noexcept
    {
        const double norm = std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x +
                                      quaternion.y * quaternion.y + quaternion.z * quaternion.z);
        const double scale = (norm > 0) ? (1.0 / norm) : 0.0;
        const double w = quaternion.w * scale;
        const double x = quaternion.x * scale;
        const double y = quaternion.y * scale;
        const double z = quaternion.z * scale;

        matrix4x4 result = identity();
        result[0][0] = 1 - 2 * (y * y + z * z);
        result[0][1] = 2 * (x * y - z * w);
        result[0][2] = 2 * (x * z + y * w);
        result[0][3] = translation[0];
        result[1][0] = 2 * (x * y + z * w);
        result[1][1] = 1 - 2 * (x * x + z * z);
        result[1][2] = 2 * (y * z - x * w);
        result[1][3] = translation[1];
        result[2][0] = 2 * (x * z - y * w);
        result[2][1] = 2 * (y * z + x * w);
        result[2][2] = 1 - 2 * (x * x + y * y);
        result[2][3] = translation[2];
        return result;
    }

    /**
     * @brief Extracts the rotation quaternion (w, x, y, z) from a rigid transform matrix (Shepperd's method:
     * branch on the largest of trace/diagonal for numeric stability).
     */
    inline quaternion_wxyz quaternion_from_matrix(const matrix4x4 &m) noexcept
    {
        const double trace = m[0][0] + m[1][1] + m[2][2];
        if (trace > 0)
        {
            const double s = std::sqrt(trace + 1.0) * 2;
            return {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
        }
        if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
        {
            const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
            return {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
        }
        if (m[1][1] > m[2][2])
        {
            const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
            return {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
        }
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
        return {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
    }

    /**
     * @brief Extracts Euler angles (roll, pitch, yaw) from a rigid transform matrix — the inverse of compose()
     * for the static-frame 'sxyz' convention (R = Rz(yaw) * Ry(pitch) * Rx(roll)). Handles the gimbal-lock
     * degeneracy (pitch ~ +/-90 deg) by fixing roll = 0 and folding the rotation into yaw.
     */
    inline std::array<double, 3> euler_from_matrix(const matrix4x4 &m) noexcept
    {
        const double cos_pitch = std::sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]);
        const double pitch = std::atan2(-m[2][0], cos_pitch);
        if (cos_pitch > 1e-9)
        {
            return {std::atan2(m[2][1], m[2][2]), pitch, std::atan2(m[1][0], m[0][0])};
        }
        // Gimbal lock: roll and yaw are coupled; pin roll = 0
        return {0.0, pitch, std::atan2(-m[0][1], m[1][1])};
    }

    /**
     * @brief Transforms the positions of all points by m, writing into out[i].position (metadata fields of out are
     * NOT touched). The single O(N) hot operation of accumulation: Eigen-vectorized when the including translation unit
     * can see Eigen3 (PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN), a plain-CPU loop otherwise. Both paths convert the
     * double matrix (composed and folded in double upstream, where large-ENU cancellation needs the precision) to
     * float32 ONCE per call and run the per-point batch entirely in float32: radar positions are float32 on the
     * wire anyway, Eigen vectorizes 2x wider, and the worst-case error (~1.5e-4 m at 300 m range) is below both the
     * radar resolution and the float32 output quantization for large local-frame coordinates.
     *
     * @param m The transformation matrix.
     * @param points The points to transform (positions read with a stride of sizeof(radar_point)).
     * @param out The output array of at least points.size() elements.
     */
#ifdef PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN
    // In an INLINE NAMESPACE so a consumer project that mixes translation units compiled with and without Eigen
    // visibility gets two DISTINCT symbols (each TU calls its own implementation) instead of silently violating
    // the one-definition rule.
    inline namespace eigen_impl
    {
#else
    inline namespace plain_impl
    {
#endif  // PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN
        inline void transform_points(const matrix4x4 &m, const std::vector<point_cloud2::radar_point> &points,
                                     accumulated_point *out)
        {
            if (points.empty())
            {
                return;
            }
#ifdef PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN
            static_assert(offsetof(point_cloud2::radar_point, y) == sizeof(float) &&
                              offsetof(point_cloud2::radar_point, z) == 2 * sizeof(float),
                          "the strided position Map relies on x, y, z being the first three packed floats");
            static_assert(sizeof(point_cloud2::radar_point) % sizeof(float) == 0,
                          "radar_point must be float-strideable");
            static_assert(sizeof(accumulated_point) % sizeof(float) == 0, "accumulated_point must be float-strideable");
            // The double matrix is converted to float32 ONCE per call; the per-point batch is pure float32
            Eigen::Matrix3f rotation_f;
            Eigen::Vector3f translation_f;
            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 3; ++column)
                {
                    rotation_f(row, column) =
                        static_cast<float>(m[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)]);
                }
                translation_f(row) = static_cast<float>(m[static_cast<std::size_t>(row)][3]);
            }
            constexpr auto in_stride = static_cast<Eigen::Index>(sizeof(point_cloud2::radar_point) / sizeof(float));
            constexpr auto out_stride = static_cast<Eigen::Index>(sizeof(accumulated_point) / sizeof(float));
            const auto num_points = static_cast<Eigen::Index>(points.size());
            const Eigen::Map<const Eigen::Matrix<float, 3, Eigen::Dynamic>, Eigen::Unaligned, Eigen::OuterStride<>> in{
                &points[0].x, 3, num_points, Eigen::OuterStride<>{in_stride}};
            Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>, Eigen::Unaligned, Eigen::OuterStride<>> result{
                out->position.data(), 3, num_points, Eigen::OuterStride<>{out_stride}};
            result = (rotation_f * in).colwise() + translation_f;
#else
            // The double matrix is converted to float32 ONCE per call; the per-point loop is pure float32
            const float m00 = static_cast<float>(m[0][0]);
            const float m01 = static_cast<float>(m[0][1]);
            const float m02 = static_cast<float>(m[0][2]);
            const float m03 = static_cast<float>(m[0][3]);
            const float m10 = static_cast<float>(m[1][0]);
            const float m11 = static_cast<float>(m[1][1]);
            const float m12 = static_cast<float>(m[1][2]);
            const float m13 = static_cast<float>(m[1][3]);
            const float m20 = static_cast<float>(m[2][0]);
            const float m21 = static_cast<float>(m[2][1]);
            const float m22 = static_cast<float>(m[2][2]);
            const float m23 = static_cast<float>(m[2][3]);
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const float x = points[i].x;
                const float y = points[i].y;
                const float z = points[i].z;
                auto &position = out[i].position;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                position[0] = m00 * x + m01 * y + m02 * z + m03;
                position[1] = m10 * x + m11 * y + m12 * z + m13;
                position[2] = m20 * x + m21 * y + m22 * z + m23;
            }
#endif  // PROVIZIO_DDS_ACCUMULATION_WITH_EIGEN
        }
    }  // namespace eigen_impl / plain_impl
}  // namespace provizio::dds::accumulation::math

#endif  // DDS_DETAIL_ACCUMULATION_MATH
