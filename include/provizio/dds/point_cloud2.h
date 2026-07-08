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

#ifndef DDS_POINT_CLOUD2
#define DDS_POINT_CLOUD2

/**
 * @file point_cloud2.h
 * @brief Utilities for creating and reading sensor_msgs PointCloud2 messages, generically and in the standard
 * Provizio radar point cloud layout.
 *
 * Non-template, Eigen-independent functions are declared here and compiled into libprovizio_dds
 * (src/point_cloud2.cpp); the templated creation functions and the per-point/per-scalar hot helpers they call stay
 * header-only (instantiated in the consumer's translation unit).
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <sensor_msgs/msg/PointCloud2.hpp>
#include <std_msgs/msg/Header.hpp>

#include "provizio/dds/common.h"

namespace provizio::dds::point_cloud2
{
    /**
     * @brief Returns the size in bytes of a single scalar of the given sensor_msgs PointField datatype.
     *
     * @param datatype One of sensor_msgs::msg::PointField_Constants::INT8, UINT8, INT16, UINT16, INT32, UINT32,
     * FLOAT32, FLOAT64.
     * @return Size in bytes, or 0 for an unknown datatype.
     */
    inline std::size_t datatype_size(const std::uint8_t datatype) noexcept
    {
        switch (datatype)
        {
        case sensor_msgs::msg::PointField_Constants::INT8:
        case sensor_msgs::msg::PointField_Constants::UINT8:
            return 1;
        case sensor_msgs::msg::PointField_Constants::INT16:
        case sensor_msgs::msg::PointField_Constants::UINT16:
            return 2;
        case sensor_msgs::msg::PointField_Constants::INT32:
        case sensor_msgs::msg::PointField_Constants::UINT32:
        case sensor_msgs::msg::PointField_Constants::FLOAT32:
            return 4;
        case sensor_msgs::msg::PointField_Constants::FLOAT64:
            return 8;
        default:
            return 0;
        }
    }

    /**
     * @brief A single Provizio radar point, with the field order exactly matching the standard Provizio radar
     * PointCloud2 wire layout (6 dense FLOAT32 fields, 24 bytes per point). See provizio_dds_idls TOPICS.md.
     */
    struct radar_point
    {
        float x{0};                               /**< @brief X position, meters (forward in radar frame) */
        float y{0};                               /**< @brief Y position, meters (left in radar frame) */
        float z{0};                               /**< @brief Z position, meters (up in radar frame) */
        float radar_relative_radial_velocity{0};  /**< @brief Radial velocity relative to the radar, m/s */
        float signal_to_noise_ratio{0};           /**< @brief Signal-to-noise ratio of the reflection */
        float ground_relative_radial_velocity{0}; /**< @brief Radial velocity relative to the ground, m/s
                                                       (NaN when not provided by the source cloud) */
    };
    static_assert(sizeof(radar_point) == 6 * sizeof(float),
                  "radar_point must be exactly 6 packed floats: its in-memory layout is relied upon to match the "
                  "standard radar PointCloud2 wire layout");

    /**
     * @brief Returns the PointField list describing the standard Provizio radar point cloud layout
     * (x, y, z, radar_relative_radial_velocity, signal_to_noise_ratio, ground_relative_radial_velocity; all FLOAT32,
     * densely packed, point_step = 24).
     *
     * @return The 6 standard radar PointField entries.
     */
    PROVIZIO_DDS_API std::vector<sensor_msgs::msg::PointField> radar_point_cloud_fields();

    /**
     * @brief Creates a std_msgs Header with the given timestamp and frame id.
     *
     * @param timestamp_sec Seconds part of the timestamp.
     * @param timestamp_nanosec Nanoseconds part of the timestamp.
     * @param frame_id Frame id, f.e. a radar position id such as "provizio_radar_front_center".
     * @return The Header.
     * @note timestamp_nanosec must be in [0, 999999999]; values >= 1e9 produce a malformed timestamp.
     */
    PROVIZIO_DDS_API std_msgs::msg::Header make_header(std::int32_t timestamp_sec, std::uint32_t timestamp_nanosec,
                                                       const std::string &frame_id);

    /**
     * @brief Describes one field for make_fields: name + datatype + scalar count.
     */
    struct field_description
    {
        std::string name;         /**< @brief Field name, f.e. "x" */
        std::uint8_t datatype{0}; /**< @brief One of the sensor_msgs::msg::PointField_Constants datatype constants */
        std::uint32_t count{1};   /**< @brief Number of scalars in the field. Defaults to 1 */
    };

    /**
     * @brief Builds a PointField list from field descriptions, auto-computing dense offsets (each field immediately
     * follows the previous one, no padding).
     *
     * @param fields The field descriptions, in layout order.
     * @return The PointField list, ready for create_cloud.
     * @throws std::invalid_argument When a description uses an unknown datatype or zero count, or the total layout
     * exceeds the uint32 point_step capacity.
     */
    PROVIZIO_DDS_API std::vector<sensor_msgs::msg::PointField> make_fields(
        const std::vector<field_description> &fields);

    namespace detail
    {
        /**
         * @brief Detects whether this host is big-endian (compile-time on all supported compilers).
         */
        constexpr bool is_host_big_endian() noexcept
        {
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__)
            return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
#else
            // MSVC (and every other supported target) is little-endian only
            return false;
#endif
        }

        /**
         * @brief Builds a PointCloud2 with everything set except the point data content: header, fields, sizes;
         * data() is resized to num_points * point_step (zero-filled) for the caller to fill in.
         * @note Internal helper shared by all create_cloud tiers; not part of the stable public API.
         * @throws std::invalid_argument When num_points * point_step overflows the message's 32-bit size fields.
         */
        // init_cloud / packed_point_step / scalar_layout_matches are non-template and Eigen-independent, so they are
        // compiled into libprovizio_dds (src/point_cloud2.cpp) — but they are called from the public templates in
        // CONSUMER translation units, hence the PROVIZIO_DDS_API export.
        PROVIZIO_DDS_API sensor_msgs::msg::PointCloud2 init_cloud(
            const std_msgs::msg::Header &header, const std::vector<sensor_msgs::msg::PointField> &fields,
            std::uint32_t point_step, std::size_t num_points, bool is_dense);

        // Contiguous-range detection: anything std::data/std::size accept (std::vector, std::array, C arrays,
        // span-likes). Used to pick the single-memcpy tier in the templated creation functions.
        template <typename range_type, typename = void> struct has_data_and_size : std::false_type
        {
        };
        template <typename range_type>
        struct has_data_and_size<range_type, std::void_t<decltype(std::data(std::declval<const range_type &>())),
                                                         decltype(std::size(std::declval<const range_type &>()))>>
            : std::true_type
        {
        };

        template <typename range_type>
        using range_element_t =
            std::remove_cv_t<std::remove_reference_t<decltype(*std::begin(std::declval<const range_type &>()))>>;

        /**
         * @brief Returns the tightly-packed point size for the given fields: max(offset + scalar_size * count).
         * @throws std::invalid_argument On an unknown field datatype.
         */
        PROVIZIO_DDS_API std::uint32_t packed_point_step(const std::vector<sensor_msgs::msg::PointField> &fields);

        /**
         * @brief Writes value into dest as scalar_type — the little-endian wire layout — narrowing/truncating per the
         * cast. For fields whose scalar type is known at compile time (e.g. the entities writer), so the wire type is
         * pinned at the call site rather than re-derived from a runtime datatype. @see pack_value for the
         * runtime-datatype counterpart.
         */
        template <typename scalar_type, typename value_type>
        inline void pack_scalar(std::uint8_t *dest, const value_type value) noexcept
        {
            const auto packed = static_cast<scalar_type>(value);
            std::memcpy(dest, &packed, sizeof(packed));
        }

        /**
         * @brief Packs one value into dest as the given runtime datatype (narrowing/truncating cast). For layouts
         * whose field datatypes are only known at run time (create_cloud's per-value tier) — a field of a
         * compile-time-known type should use pack_scalar directly. double carries the value because it represents
         * every PointField integer type (all <= 32-bit) exactly and is the widest field type (FLOAT64).
         */
        inline void pack_value(std::uint8_t *dest, const std::uint8_t datatype, const double value) noexcept
        {
            switch (datatype)
            {
            case sensor_msgs::msg::PointField_Constants::INT8:
                pack_scalar<std::int8_t>(dest, value);
                break;
            case sensor_msgs::msg::PointField_Constants::UINT8:
                pack_scalar<std::uint8_t>(dest, value);
                break;
            case sensor_msgs::msg::PointField_Constants::INT16:
                pack_scalar<std::int16_t>(dest, value);
                break;
            case sensor_msgs::msg::PointField_Constants::UINT16:
                pack_scalar<std::uint16_t>(dest, value);
                break;
            case sensor_msgs::msg::PointField_Constants::INT32:
                pack_scalar<std::int32_t>(dest, value);
                break;
            case sensor_msgs::msg::PointField_Constants::UINT32:
                pack_scalar<std::uint32_t>(dest, value);
                break;
            case sensor_msgs::msg::PointField_Constants::FLOAT32:
                pack_scalar<float>(dest, value);
                break;
            case sensor_msgs::msg::PointField_Constants::FLOAT64:
                pack_scalar<double>(dest, value);
                break;
            default:
                // Unreachable for clouds built via make_fields/packed_point_step (both validate datatypes)
                break;
            }
        }

        // Maps an arithmetic C++ type to its PointField datatype constant; 0 (invalid) when unmapped.
        template <typename scalar_type> constexpr std::uint8_t datatype_of() noexcept
        {
            if constexpr (std::is_same_v<scalar_type, std::int8_t>)
            {
                return sensor_msgs::msg::PointField_Constants::INT8;
            }
            else if constexpr (std::is_same_v<scalar_type, std::uint8_t>)
            {
                return sensor_msgs::msg::PointField_Constants::UINT8;
            }
            else if constexpr (std::is_same_v<scalar_type, std::int16_t>)
            {
                return sensor_msgs::msg::PointField_Constants::INT16;
            }
            else if constexpr (std::is_same_v<scalar_type, std::uint16_t>)
            {
                return sensor_msgs::msg::PointField_Constants::UINT16;
            }
            else if constexpr (std::is_same_v<scalar_type, std::int32_t>)
            {
                return sensor_msgs::msg::PointField_Constants::INT32;
            }
            else if constexpr (std::is_same_v<scalar_type, std::uint32_t>)
            {
                return sensor_msgs::msg::PointField_Constants::UINT32;
            }
            else if constexpr (std::is_same_v<scalar_type, float>)
            {
                return sensor_msgs::msg::PointField_Constants::FLOAT32;
            }
            else if constexpr (std::is_same_v<scalar_type, double>)
            {
                return sensor_msgs::msg::PointField_Constants::FLOAT64;
            }
            else
            {
                return 0;
            }
        }

        // Fixed-size contiguous-of-scalar element detection (std::array<T, N> / T[N] with arithmetic T) — the
        // element shapes that can qualify for the memcpy tiers of the templated create_cloud.
        template <typename element_type> struct contiguous_scalar_element : std::false_type
        {
        };
        template <typename scalar_type, std::size_t element_count>
        struct contiguous_scalar_element<std::array<scalar_type, element_count>>
            : std::bool_constant<std::is_arithmetic_v<scalar_type>>
        {
            using scalar = scalar_type;
        };
        template <typename scalar_type, std::size_t element_count>
        struct contiguous_scalar_element<scalar_type[element_count]>  // NOLINT(*-avoid-c-arrays)
            : std::bool_constant<std::is_arithmetic_v<scalar_type>>
        {
            using scalar = scalar_type;
        };

        /**
         * @brief Runtime check that the fields describe exactly a dense homogeneous array of `scalar_datatype`
         * scalars of total size element_size — i.e. that an element of that scalar type byte-matches the wire layout.
         */
        PROVIZIO_DDS_API bool scalar_layout_matches(const std::vector<sensor_msgs::msg::PointField> &fields,
                                                    std::uint8_t scalar_datatype, std::size_t element_size);
    }  // namespace detail

    /**
     * @brief A PointCloud2 field resolved by cloud_view::field(): its metadata plus byte offset within a point.
     */
    struct field_ref
    {
        std::string name;         /**< @brief Field name, f.e. "x" */
        std::uint8_t datatype{0}; /**< @brief One of the sensor_msgs::msg::PointField_Constants datatype constants */
        std::uint32_t count{1};   /**< @brief Number of scalars in the field (f.e. 4 for a quaternion field) */
        std::size_t offset{0};    /**< @brief Byte offset of the field within a point */
    };

    class cloud_view;

    /**
     * @brief A lightweight non-owning proxy for a single point inside a cloud_view. Valid only while the viewed
     * PointCloud2 is alive and unmodified.
     */
    class point_ref
    {
      public:
        /**
         * @brief Reads one scalar of a field of this point, converting from the stored datatype to T and
         * byte-swapping when the cloud's endianness differs from the host's.
         *
         * @tparam T Arithmetic type to convert the stored value to.
         * @param field A field of this cloud, as resolved by cloud_view::field().
         * @param subfield_index Index of the scalar within the field, < field.count. Defaults to 0.
         * @return The value, converted with an ordinary (truncating) C++ cast.
         * @throws std::out_of_range When subfield_index is out of the field's count or the read would exceed the
         * point's bounds.
         */
        template <typename T> T get(const field_ref &field, std::size_t subfield_index = 0) const;

      private:
        friend class cloud_view;
        point_ref(const std::uint8_t *point, std::uint32_t point_step, bool big_endian) noexcept
            : point(point), point_step(point_step), big_endian(big_endian)
        {
        }

        const std::uint8_t *point;
        std::uint32_t point_step;
        bool big_endian;
    };

    /**
     * @brief A lightweight non-owning view over a PointCloud2 for generic field-driven reading: resolves fields by
     * name and iterates points. Validates size consistency at construction. Valid only while the viewed PointCloud2
     * is alive and unmodified.
     */
    class cloud_view
    {
      public:
        /**
         * @brief A random-access iterator over the view's points, dereferencing to point_ref value-proxies.
         * @note operator-> is deliberately not provided (point_ref is a value proxy); use (*it).get(...) or range-for.
         */
        class iterator
        {
          public:
            using iterator_category = std::random_access_iterator_tag;
            using value_type = point_ref;
            using difference_type = std::ptrdiff_t;
            using pointer = const point_ref *;
            using reference = point_ref;  // proxy by value, like std::vector<bool>

            reference operator*() const noexcept
            {
                return view->operator[](index);
            }
            iterator &operator++() noexcept
            {
                ++index;
                return *this;
            }
            iterator operator++(int) noexcept
            {
                iterator copy = *this;
                ++index;
                return copy;
            }
            iterator &operator--() noexcept
            {
                --index;
                return *this;
            }
            iterator operator--(int) noexcept
            {
                iterator copy = *this;
                --index;
                return copy;
            }
            iterator &operator+=(difference_type delta) noexcept
            {
                index += static_cast<std::size_t>(delta);
                return *this;
            }
            iterator &operator-=(difference_type delta) noexcept
            {
                index -= static_cast<std::size_t>(delta);
                return *this;
            }
            friend iterator operator+(iterator it, difference_type delta) noexcept
            {
                it += delta;
                return it;
            }
            friend iterator operator+(difference_type delta, iterator it) noexcept
            {
                it += delta;
                return it;
            }
            friend iterator operator-(iterator it, difference_type delta) noexcept
            {
                it -= delta;
                return it;
            }
            friend difference_type operator-(const iterator &a, const iterator &b) noexcept
            {
                return static_cast<difference_type>(a.index) - static_cast<difference_type>(b.index);
            }
            reference operator[](difference_type delta) const noexcept
            {
                return *(*this + delta);
            }
            friend bool operator==(const iterator &a, const iterator &b) noexcept
            {
                return a.index == b.index;
            }
            friend bool operator!=(const iterator &a, const iterator &b) noexcept
            {
                return a.index != b.index;
            }
            friend bool operator<(const iterator &a, const iterator &b) noexcept
            {
                return a.index < b.index;
            }
            friend bool operator>(const iterator &a, const iterator &b) noexcept
            {
                return b < a;
            }
            friend bool operator<=(const iterator &a, const iterator &b) noexcept
            {
                return !(b < a);
            }
            friend bool operator>=(const iterator &a, const iterator &b) noexcept
            {
                return !(a < b);
            }

          private:
            friend class cloud_view;
            iterator(const cloud_view *view, std::size_t index) noexcept : view(view), index(index)
            {
            }
            const cloud_view *view;
            std::size_t index;
        };

        /**
         * @brief Constructs the view, validating the cloud's structural consistency.
         *
         * @param source_cloud The cloud to view; must outlive the view (and all point_refs/iterators obtained from it).
         * @throws std::invalid_argument When point_step is 0 while there are points, or the data buffer is smaller
         * than width * height * point_step.
         */
        // The constructor and the field() overloads are non-template and Eigen-independent: compiled into
        // libprovizio_dds (src/point_cloud2.cpp). Per-method PROVIZIO_DDS_API (the logging.h log_stream pattern)
        // rather than exporting the whole class, so the hot header-inline members below stay non-exported.
        PROVIZIO_DDS_API explicit cloud_view(const sensor_msgs::msg::PointCloud2 &source_cloud);

        /**
         * @brief Returns the number of points in the cloud (width * height).
         */
        std::size_t size() const noexcept
        {
            return num_points;
        }

        /**
         * @brief Returns a point proxy by index. No bounds check (mirrors operator[] convention); valid indices are
         * [0, size()).
         */
        point_ref operator[](const std::size_t index) const noexcept
        {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            return point_ref{data_begin + index * step, step, big_endian};
        }

        /** @brief Returns an iterator to the first point of the view. */
        iterator begin() const noexcept
        {
            return iterator{this, 0};
        }
        /** @brief Returns the past-the-end iterator of the view. */
        iterator end() const noexcept
        {
            return iterator{this, num_points};
        }

        /**
         * @brief Resolves a field by name.
         *
         * @param name The field name, f.e. "x".
         * @return The resolved field_ref, or std::nullopt when no such field exists or the field doesn't fit within
         * point_step (malformed cloud).
         */
        PROVIZIO_DDS_API std::optional<field_ref> field(std::string_view name) const;

        /**
         * @brief Resolves a field by name, validating expectations — the strict variant.
         *
         * @param name The field name, f.e. "x".
         * @param expected_datatype The required datatype, f.e. sensor_msgs::msg::PointField_Constants::FLOAT32.
         * @param min_count The minimum required scalar count in the field. Defaults to 1.
         * @return The resolved field_ref.
         * @throws std::invalid_argument When the field is missing or doesn't match the expectations.
         */
        PROVIZIO_DDS_API field_ref field(std::string_view name, std::uint8_t expected_datatype,
                                         std::uint32_t min_count = 1) const;

      private:
        const sensor_msgs::msg::PointCloud2 *cloud;
        const std::uint8_t *data_begin{nullptr};
        std::uint32_t step{0};
        std::size_t num_points{0};
        bool big_endian{false};
    };

    template <typename T> T point_ref::get(const field_ref &field, const std::size_t subfield_index) const
    {
        static_assert(std::is_arithmetic_v<T>, "point_ref::get requires an arithmetic type");
        const std::size_t scalar_size = datatype_size(field.datatype);
        if (subfield_index >= field.count ||
            field.offset + scalar_size * (subfield_index + 1) > static_cast<std::size_t>(point_step))
        {
            throw std::out_of_range{"point_cloud2::point_ref::get: field read out of the point's bounds"};
        }
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const std::uint8_t *position = point + field.offset + scalar_size * subfield_index;

        const bool swap = (big_endian != detail::is_host_big_endian());
        const auto read_scalar = [position, swap](auto scalar_prototype) {
            using scalar_type = decltype(scalar_prototype);
            scalar_type scalar;  // NOLINT(cppcoreguidelines-init-variables): set in the very next line
            std::memcpy(&scalar, position, sizeof(scalar_type));
            if (swap)
            {
                auto *bytes = reinterpret_cast<std::uint8_t *>(&scalar);  // NOLINT(*-reinterpret-cast)
                std::reverse(bytes, bytes + sizeof(scalar_type));         // NOLINT(*-pointer-arithmetic)
            }
            return scalar;
        };

        switch (field.datatype)
        {
        case sensor_msgs::msg::PointField_Constants::INT8:
            return static_cast<T>(read_scalar(std::int8_t{}));
        case sensor_msgs::msg::PointField_Constants::UINT8:
            return static_cast<T>(read_scalar(std::uint8_t{}));
        case sensor_msgs::msg::PointField_Constants::INT16:
            return static_cast<T>(read_scalar(std::int16_t{}));
        case sensor_msgs::msg::PointField_Constants::UINT16:
            return static_cast<T>(read_scalar(std::uint16_t{}));
        case sensor_msgs::msg::PointField_Constants::INT32:
            return static_cast<T>(read_scalar(std::int32_t{}));
        case sensor_msgs::msg::PointField_Constants::UINT32:
            return static_cast<T>(read_scalar(std::uint32_t{}));
        case sensor_msgs::msg::PointField_Constants::FLOAT32:
            return static_cast<T>(read_scalar(float{}));
        case sensor_msgs::msg::PointField_Constants::FLOAT64:
            return static_cast<T>(read_scalar(double{}));
        default:
            throw std::out_of_range{"point_cloud2::point_ref::get: unknown field datatype"};
        }
    }

    /**
     * @brief Creates a PointCloud2 from a raw pre-packed buffer. The caller guarantees points_data already matches
     * the given fields / point_step layout exactly, in HOST endianness (is_bigendian is set accordingly).
     * This is the zero-conversion power-user overload; the templated create_cloud overload dispatches here when the
     * input layout provably matches.
     *
     * @param header The point cloud header (see make_header).
     * @param fields The point cloud fields describing the layout.
     * @param point_step Size of a single point in bytes.
     * @param points_data Pre-packed point data, at least num_points * point_step bytes.
     * @param num_points Number of points in points_data.
     * @param is_dense True (default) if there are no invalid points.
     * @return The PointCloud2.
     * @throws std::invalid_argument When points_data is null while num_points > 0, or the total size overflows the
     * message's 32-bit size fields.
     */
    PROVIZIO_DDS_API sensor_msgs::msg::PointCloud2 create_cloud(const std_msgs::msg::Header &header,
                                                                const std::vector<sensor_msgs::msg::PointField> &fields,
                                                                std::uint32_t point_step, const void *points_data,
                                                                std::size_t num_points, bool is_dense = true);

    /**
     * @brief Creates a PointCloud2 from ANY iterable range of per-point rows. Each row is itself an iterable of
     * arithmetic values, providing exactly one value per field-count slot, flattened in field order (a count=4 field
     * consumes 4 consecutive values).
     *
     * Copying is tiered automatically, most efficient applicable tier first:
     * 1. Whole-range memcpy — contiguous outer range of fixed-size scalar rows (f.e.
     *    std::vector<std::array<float, 6>>) whose layout byte-matches the fields.
     * 2. Per-point memcpy — same row layout match, non-contiguous outer range (f.e. std::deque of std::array).
     * 3. Per-value conversion — everything else; each value is cast to its field's datatype while packing
     *    (still allocation-free: written straight into the message's preallocated buffer).
     *
     * @tparam points_range_type Any iterable range of per-point rows of arithmetic values.
     * @param header The point cloud header (see make_header).
     * @param fields The point cloud fields (see make_fields); offsets are expected dense (as make_fields builds).
     * @param points The points.
     * @param is_dense True (default) if there are no invalid points.
     * @return The PointCloud2.
     * @throws std::invalid_argument When a row provides a wrong number of values, a field has an unknown datatype, or
     * the fields describe a point exceeding the uint32 point_step capacity.
     * @note Values must be representable in the target field datatype; out-of-range conversions are undefined, as for
     * any C++ floating-point-to-integer cast.
     * @note points must be a multi-pass (forward) range: it is iterated twice (size, then fill).
     */
    template <typename points_range_type>
    sensor_msgs::msg::PointCloud2 create_cloud(const std_msgs::msg::Header &header,
                                               const std::vector<sensor_msgs::msg::PointField> &fields,
                                               const points_range_type &points, const bool is_dense = true)
    {
        using std::begin;
        using std::end;
        using element_type = detail::range_element_t<points_range_type>;
        const std::uint32_t point_step = detail::packed_point_step(fields);

        if constexpr (detail::contiguous_scalar_element<element_type>::value)
        {
            using scalar = typename detail::contiguous_scalar_element<element_type>::scalar;
            if (detail::scalar_layout_matches(fields, detail::datatype_of<scalar>(), sizeof(element_type)))
            {
                if constexpr (detail::has_data_and_size<points_range_type>::value)
                {
                    // Tier 1: single whole-range memcpy
                    return create_cloud(header, fields, point_step, std::data(points), std::size(points), is_dense);
                }
                else
                {
                    // Tier 2: per-point memcpy
                    const auto num_points = static_cast<std::size_t>(std::distance(begin(points), end(points)));
                    auto cloud = detail::init_cloud(header, fields, point_step, num_points, is_dense);
                    std::uint8_t *dest = cloud.data().data();
                    for (const auto &point : points)
                    {
                        std::memcpy(dest, &point, sizeof(element_type));
                        dest += point_step;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                    }
                    return cloud;
                }
            }
        }

        // Tier 3: per-value conversion (any iterable rows, any field datatypes)
        const auto num_points = static_cast<std::size_t>(std::distance(begin(points), end(points)));
        auto cloud = detail::init_cloud(header, fields, point_step, num_points, is_dense);
        std::uint8_t *dest = cloud.data().data();
        for (const auto &row : points)
        {
            auto value_it = begin(row);
            const auto row_end = end(row);
            for (const auto &field : fields)
            {
                const std::size_t scalar_size = datatype_size(field.datatype());
                for (std::uint32_t subfield = 0; subfield < field.count(); ++subfield)
                {
                    if (value_it == row_end)
                    {
                        throw std::invalid_argument{
                            "point_cloud2::create_cloud: a point provides fewer values than the fields require"};
                    }
                    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                    detail::pack_value(dest + field.offset() + scalar_size * subfield, field.datatype(),
                                       static_cast<double>(*value_it));
                    ++value_it;
                }
            }
            if (value_it != row_end)
            {
                throw std::invalid_argument{
                    "point_cloud2::create_cloud: a point provides more values than the fields require"};
            }
            dest += point_step;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return cloud;
    }

    /**
     * @brief Creates a standard Provizio radar PointCloud2 from any iterable range of radar_point. radar_point's
     * in-memory layout equals the wire layout by construction, so contiguous ranges (std::vector, std::array,
     * C arrays, span-likes) are written with a single memcpy and any other range with one memcpy per point — no
     * per-value conversion either way.
     *
     * @tparam radar_points_range_type Any iterable range of radar_point.
     * @param header The point cloud header (see make_header).
     * @param points The radar points.
     * @param is_dense True (default) if there are no invalid points.
     * @return The radar PointCloud2.
     */
    template <typename radar_points_range_type>
    sensor_msgs::msg::PointCloud2 make_radar_point_cloud(const std_msgs::msg::Header &header,
                                                         const radar_points_range_type &points,
                                                         const bool is_dense = true)
    {
        using std::begin;
        using std::end;
        static_assert(std::is_same_v<detail::range_element_t<radar_points_range_type>, radar_point>,
                      "make_radar_point_cloud expects a range of provizio::dds::point_cloud2::radar_point");
        const auto fields = radar_point_cloud_fields();
        constexpr auto point_step = static_cast<std::uint32_t>(sizeof(radar_point));
        if constexpr (detail::has_data_and_size<radar_points_range_type>::value)
        {
            // Contiguous: single whole-range memcpy
            return create_cloud(header, fields, point_step, std::data(points), std::size(points), is_dense);
        }
        else
        {
            // Non-contiguous: one memcpy per point
            const auto num_points = static_cast<std::size_t>(std::distance(begin(points), end(points)));
            auto cloud = detail::init_cloud(header, fields, point_step, num_points, is_dense);
            std::uint8_t *dest = cloud.data().data();
            for (const auto &point : points)
            {
                std::memcpy(dest, &point, sizeof(radar_point));
                dest += sizeof(radar_point);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            }
            return cloud;
        }
    }

    /**
     * @brief Reads all points of a (Provizio radar format) PointCloud2 into radar_point structs. When the cloud
     * uses the standard layout (the exact radar_point_cloud_fields() layout, host endianness) the read is a single
     * bulk copy; any other layout (reordered fields, different datatypes, foreign endianness) is read point-by-point
     * (each field by name) via cloud_view. Fields missing from the cloud (f.e. the optional
     * ground_relative_radial_velocity, see provizio_dds_idls TOPICS.md) are filled with NaN.
     *
     * @param cloud The cloud to read.
     * @return All points as radar_point structs.
     * @throws std::invalid_argument When the cloud is structurally malformed (see cloud_view).
     */
    PROVIZIO_DDS_API std::vector<radar_point> read_radar_points(const sensor_msgs::msg::PointCloud2 &cloud);

    /**
     * @brief Returns the PointField list for Provizio entity PointCloud2 messages (the standard entity layouts;
     * see provizio_dds_idls TOPICS.md).
     *
     * @param has_radar_data Whether radar-sourced fields are present (entity_id, radar/ground velocities,
     * orientation, size).
     * @param has_camera_data Whether camera-sourced fields are present (camera_entity_id, camera_bbox).
     * @return The PointField list in the standard entity layout order.
     */
    PROVIZIO_DDS_API std::vector<sensor_msgs::msg::PointField> entities_fields(bool has_radar_data,
                                                                               bool has_camera_data);

    /**
     * @brief Creates a PointCloud2 containing entities (radar, camera or fused). Entity rows are flattened scalars
     * in field order: a count=4 field (f.e. orientation) consumes 4 consecutive values.
     *
     * @tparam entities_range_type Any iterable range of per-entity rows of arithmetic values.
     * @param header The point cloud header (see make_header).
     * @param has_radar_data Whether radar-sourced fields are present.
     * @param has_camera_data Whether camera-sourced fields are present.
     * @param entities The entity rows, flattened in the order produced by entities_fields(has_radar_data,
     * has_camera_data); radar rows carry 16 values, camera 11, fused 21.
     * @return The entities PointCloud2.
     */
    template <typename entities_range_type>
    sensor_msgs::msg::PointCloud2 make_entities(const std_msgs::msg::Header &header, const bool has_radar_data,
                                                const bool has_camera_data, const entities_range_type &entities)
    {
        return create_cloud(header, entities_fields(has_radar_data, has_camera_data), entities);
    }

    /**
     * @brief Creates a PointCloud2 of radar entities (entity_id, entity_class, x, y, z,
     * radar_relative_radial_velocity, ground_relative_radial_velocity, orientation(x,y,z,w), size(x,y,z),
     * entity_confidence, entity_class_confidence) — 16 values per row.
     *
     * @tparam entities_range_type Any iterable range of per-entity rows of arithmetic values.
     * @param header The point cloud header (see make_header).
     * @param entities The entity rows.
     * @return The entities PointCloud2.
     */
    template <typename entities_range_type>
    sensor_msgs::msg::PointCloud2 make_radar_entities(const std_msgs::msg::Header &header,
                                                      const entities_range_type &entities)
    {
        return make_entities(header, true, false, entities);
    }

    /**
     * @brief Creates a PointCloud2 of camera entities (camera_entity_id, entity_class, x, y, z,
     * camera_bbox(left,top,right,bottom), entity_confidence, entity_class_confidence) — 11 values per row.
     *
     * @tparam entities_range_type Any iterable range of per-entity rows of arithmetic values.
     * @param header The point cloud header (see make_header).
     * @param entities The entity rows.
     * @return The entities PointCloud2.
     */
    template <typename entities_range_type>
    sensor_msgs::msg::PointCloud2 make_camera_entities(const std_msgs::msg::Header &header,
                                                       const entities_range_type &entities)
    {
        return make_entities(header, false, true, entities);
    }

    /**
     * @brief Creates a PointCloud2 of fused (radar + camera) entities — 21 values per row, field order: entity_id,
     * camera_entity_id, entity_class, x, y, z, radar_relative_radial_velocity, ground_relative_radial_velocity,
     * orientation(x,y,z,w), size(x,y,z), camera_bbox(left,top,right,bottom), entity_confidence,
     * entity_class_confidence.
     *
     * @tparam entities_range_type Any iterable range of per-entity rows of arithmetic values.
     * @param header The point cloud header (see make_header).
     * @param entities The entity rows.
     * @return The entities PointCloud2.
     */
    template <typename entities_range_type>
    sensor_msgs::msg::PointCloud2 make_fused_entities(const std_msgs::msg::Header &header,
                                                      const entities_range_type &entities)
    {
        return make_entities(header, true, true, entities);
    }

    /**
     * @brief The kind of a Provizio entities PointCloud2, detected by which entity id fields it carries.
     */
    enum class entities_kind
    {
        radar,  /**< @brief Radar-detected entities (entity_id, velocities, orientation, size) */
        camera, /**< @brief Camera-detected entities (camera_entity_id, camera_bbox) */
        fused   /**< @brief Fused radar + camera entities (both id kinds and all optional fields) */
    };

    /**
     * @brief The sentinel value of entity.entity_id / entity.camera_entity_id meaning the id is not present in the
     * source cloud (f.e. camera_entity_id of a radar-only entities cloud) — see get_entities_kind.
     */
    inline constexpr std::uint32_t no_entity_id = std::numeric_limits<std::uint32_t>::max();

    /**
     * @brief Detects the kind of a Provizio entities cloud: fused when both entity_id and camera_entity_id fields
     * are present, radar/camera when only the respective id field is, std::nullopt when neither is (not a Provizio
     * entities cloud).
     *
     * @param cloud The cloud to probe.
     * @return The detected kind, or std::nullopt for non-entities clouds.
     */
    PROVIZIO_DDS_API std::optional<entities_kind> get_entities_kind(const sensor_msgs::msg::PointCloud2 &cloud);

    /**
     * @brief A single Provizio entity of any kind (radar / camera / fused), as read by read_entities or constructed
     * for make_entities. Fields absent from the source cloud read as NaN (all float fields, including whole
     * multi-value groups) or no_entity_id (the integral ids — see get_entities_kind to learn which groups are
     * meaningful).
     */
    struct entity
    {
        std::uint32_t entity_id{no_entity_id}; /**< @brief Radar entity id; no_entity_id when the cloud carries no
                                                    radar data */
        std::uint32_t camera_entity_id{no_entity_id}; /**< @brief Camera entity id; no_entity_id when the cloud
                                                           carries no camera data */
        std::uint8_t entity_class{0};                 /**< @brief Entity classification */
        float x{0}; /**< @brief X position, meters (may be NaN, f.e. camera-only entities) */
        float y{0}; /**< @brief Y position, meters (may be NaN) */
        float z{0}; /**< @brief Z position, meters (may be NaN) */
        float radar_relative_radial_velocity{0};  /**< @brief m/s; NaN when the cloud carries no radar data */
        float ground_relative_radial_velocity{0}; /**< @brief m/s; NaN when the cloud carries no radar data */
        std::array<float, 4> orientation{};       /**< @brief Quaternion (x, y, z, w); NaN-filled without radar data */
        std::array<float, 3> size{};              /**< @brief Size (x, y, z), meters; NaN-filled without radar data */
        std::array<float, 4> camera_bbox{};      /**< @brief Bounding box (left, top, right, bottom), pixels; NaN-filled
                                                      without camera data */
        std::uint8_t entity_confidence{0};       /**< @brief Detection confidence */
        std::uint8_t entity_class_confidence{0}; /**< @brief Classification confidence */

        /**
         * @brief Returns true when this entity carries radar-sourced data (its entity_id is present, i.e. not
         * no_entity_id).
         */
        bool has_radar_data() const noexcept
        {
            return entity_id != no_entity_id;
        }

        /**
         * @brief Returns true when this entity carries camera-sourced data (its camera_entity_id is present, i.e.
         * not no_entity_id).
         */
        bool has_camera_data() const noexcept
        {
            return camera_entity_id != no_entity_id;
        }

        /**
         * @brief Returns the kind of this single entity: fused when it carries both radar and camera data, radar /
         * camera when only one, std::nullopt when neither (f.e. a default-constructed entity). The whole-cloud
         * counterpart is get_entities_kind.
         */
        std::optional<entities_kind> kind() const noexcept
        {
            if (has_radar_data() && has_camera_data())
            {
                return entities_kind::fused;
            }
            if (has_radar_data())
            {
                return entities_kind::radar;
            }
            if (has_camera_data())
            {
                return entities_kind::camera;
            }
            return std::nullopt;
        }
    };

    /**
     * @brief Reads all entities of a Provizio entities PointCloud2 (radar, camera or fused — see get_entities_kind)
     * into unified entity structs. Fields absent from the cloud are NaN-filled (floats) or no_entity_id (ids).
     *
     * @param cloud The entities cloud to read.
     * @return All entities.
     * @throws std::invalid_argument When the cloud is structurally malformed (see cloud_view).
     */
    PROVIZIO_DDS_API std::vector<entity> read_entities(const sensor_msgs::msg::PointCloud2 &cloud);

    /**
     * @brief Creates a Provizio entities PointCloud2 of the given kind from ANY iterable range of entity structs
     * (f.e. std::vector<entity>, std::array<entity, N>, C arrays, std::deque<entity>). Only the field groups of the
     * requested kind are written (f.e. camera_bbox of an entity is ignored when kind is entities_kind::radar);
     * read_entities reads such a cloud back losslessly for the written groups.
     *
     * @tparam entities_range_type Any iterable range of entity.
     * @param header The point cloud header (see make_header).
     * @param kind The kind of the entities cloud to create — defines which field groups are written.
     * @param entities The entities.
     * @return The entities PointCloud2.
     */
    template <typename entities_range_type>
    sensor_msgs::msg::PointCloud2 make_entities(const std_msgs::msg::Header &header, const entities_kind kind,
                                                const entities_range_type &entities)
    {
        static_assert(std::is_same_v<detail::range_element_t<entities_range_type>, entity>,
                      "make_entities(header, kind, range) expects a range of provizio::dds::point_cloud2::entity");

        const bool has_radar = (kind != entities_kind::camera);
        const bool has_camera = (kind != entities_kind::radar);
        const auto fields = entities_fields(has_radar, has_camera);
        const std::uint32_t point_step = detail::packed_point_step(fields);

        // Resolve each field's byte offset once, before the per-entity loop.
        struct offsets_t
        {
            std::uint32_t entity_id{0};
            std::uint32_t camera_entity_id{0};
            std::uint32_t entity_class{0};
            std::uint32_t x{0};
            std::uint32_t y{0};
            std::uint32_t z{0};
            std::uint32_t radar_relative_radial_velocity{0};
            std::uint32_t ground_relative_radial_velocity{0};
            std::uint32_t orientation{0};
            std::uint32_t size{0};
            std::uint32_t camera_bbox{0};
            std::uint32_t entity_confidence{0};
            std::uint32_t entity_class_confidence{0};
        } off;

        for (const auto &f : fields)
        {
            const auto nm = std::string_view{f.name()};
            if (nm == "entity_id")
            {
                off.entity_id = f.offset();
            }
            else if (nm == "camera_entity_id")
            {
                off.camera_entity_id = f.offset();
            }
            else if (nm == "entity_class")
            {
                off.entity_class = f.offset();
            }
            else if (nm == "x")
            {
                off.x = f.offset();
            }
            else if (nm == "y")
            {
                off.y = f.offset();
            }
            else if (nm == "z")
            {
                off.z = f.offset();
            }
            else if (nm == "radar_relative_radial_velocity")
            {
                off.radar_relative_radial_velocity = f.offset();
            }
            else if (nm == "ground_relative_radial_velocity")
            {
                off.ground_relative_radial_velocity = f.offset();
            }
            else if (nm == "orientation")
            {
                off.orientation = f.offset();
            }
            else if (nm == "size")
            {
                off.size = f.offset();
            }
            else if (nm == "camera_bbox")
            {
                off.camera_bbox = f.offset();
            }
            else if (nm == "entity_confidence")
            {
                off.entity_confidence = f.offset();
            }
            else if (nm == "entity_class_confidence")
            {
                off.entity_class_confidence = f.offset();
            }
        }

        using std::begin;
        using std::end;
        const auto num_entities = static_cast<std::size_t>(std::distance(begin(entities), end(entities)));
        auto cloud = detail::init_cloud(header, fields, point_step, num_entities, true);
        std::uint8_t *dest = cloud.data().data();

        // Field scalar types are fixed by entities_fields, so each value is written with its wire type pinned
        // at the call site (pack_scalar<T>) rather than funnelled through a runtime datatype. f32_sz steps across
        // the multi-element float fields (orientation / size / bbox).
        const std::size_t f32_sz = datatype_size(sensor_msgs::msg::PointField_Constants::FLOAT32);

        for (const auto &ent : entities)
        {
            // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            if (has_radar)
            {
                detail::pack_scalar<std::uint32_t>(dest + off.entity_id, ent.entity_id);
                detail::pack_scalar<float>(dest + off.radar_relative_radial_velocity,
                                           ent.radar_relative_radial_velocity);
                detail::pack_scalar<float>(dest + off.ground_relative_radial_velocity,
                                           ent.ground_relative_radial_velocity);
                for (std::size_t sub = 0; sub < ent.orientation.size(); ++sub)
                {
                    detail::pack_scalar<float>(dest + off.orientation + f32_sz * sub,
                                               ent.orientation[sub]);  // NOLINT(*-constant-array-index)
                }
                for (std::size_t sub = 0; sub < ent.size.size(); ++sub)
                {
                    detail::pack_scalar<float>(dest + off.size + f32_sz * sub,
                                               ent.size[sub]);  // NOLINT(*-constant-array-index)
                }
            }
            if (has_camera)
            {
                detail::pack_scalar<std::uint32_t>(dest + off.camera_entity_id, ent.camera_entity_id);
                for (std::size_t sub = 0; sub < ent.camera_bbox.size(); ++sub)
                {
                    detail::pack_scalar<float>(dest + off.camera_bbox + f32_sz * sub,
                                               ent.camera_bbox[sub]);  // NOLINT(*-constant-array-index)
                }
            }
            detail::pack_scalar<std::uint8_t>(dest + off.entity_class, ent.entity_class);
            detail::pack_scalar<float>(dest + off.x, ent.x);
            detail::pack_scalar<float>(dest + off.y, ent.y);
            detail::pack_scalar<float>(dest + off.z, ent.z);
            detail::pack_scalar<std::uint8_t>(dest + off.entity_confidence, ent.entity_confidence);
            detail::pack_scalar<std::uint8_t>(dest + off.entity_class_confidence, ent.entity_class_confidence);
            // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            dest += point_step;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return cloud;
    }
}  // namespace provizio::dds::point_cloud2

#endif  // DDS_POINT_CLOUD2
