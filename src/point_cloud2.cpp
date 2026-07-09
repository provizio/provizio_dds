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

#include "provizio/dds/point_cloud2.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace provizio::dds::point_cloud2
{
    std::vector<sensor_msgs::msg::PointField> radar_point_cloud_fields()
    {
        const std::array<const char *, 6> names{"x",
                                                "y",
                                                "z",
                                                "radar_relative_radial_velocity",
                                                "signal_to_noise_ratio",
                                                "ground_relative_radial_velocity"};
        std::vector<sensor_msgs::msg::PointField> fields{names.size()};
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            fields[i].name(names[i]);  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            fields[i].offset(static_cast<std::uint32_t>(i * sizeof(float)));
            fields[i].datatype(sensor_msgs::msg::PointField_Constants::FLOAT32);
            fields[i].count(1);
        }
        return fields;
    }

    std_msgs::msg::Header make_header(const std::int32_t timestamp_sec, const std::uint32_t timestamp_nanosec,
                                      const std::string &frame_id)
    {
        std_msgs::msg::Header header;
        header.stamp().sec(timestamp_sec);
        header.stamp().nanosec(timestamp_nanosec);
        header.frame_id(frame_id);
        return header;
    }

    std::vector<sensor_msgs::msg::PointField> make_fields(const std::vector<field_description> &fields)
    {
        std::vector<sensor_msgs::msg::PointField> result{fields.size()};
        std::uint32_t offset = 0;
        for (std::size_t i = 0; i < fields.size(); ++i)
        {
            const auto &description = fields[i];
            const std::size_t scalar_size = datatype_size(description.datatype);
            if (scalar_size == 0 || description.count == 0)
            {
                throw std::invalid_argument{"point_cloud2::make_fields: unknown datatype or zero count in field \"" +
                                            description.name + "\""};
            }
            auto &field = result[i];  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            field.name(description.name);
            field.offset(offset);
            field.datatype(description.datatype);
            field.count(description.count);
            // uint64_t arithmetic so the capacity check is correct even on 32-bit platforms, where size_t is
            // 32-bit and scalar_size * count could otherwise wrap before the comparison below.
            const std::uint64_t next_offset =
                static_cast<std::uint64_t>(offset) + static_cast<std::uint64_t>(scalar_size) * description.count;
            if (next_offset > std::numeric_limits<std::uint32_t>::max())
            {
                throw std::invalid_argument{
                    "point_cloud2::make_fields: the fields describe a point larger than the PointCloud2 "
                    "point_step capacity (uint32) at field \"" +
                    description.name + "\""};
            }
            offset = static_cast<std::uint32_t>(next_offset);
        }
        return result;
    }

    namespace detail
    {
        sensor_msgs::msg::PointCloud2 init_cloud(const std_msgs::msg::Header &header,
                                                 const std::vector<sensor_msgs::msg::PointField> &fields,
                                                 const std::uint32_t point_step, const std::size_t num_points,
                                                 const bool is_dense)
        {
            if (num_points > 0 && point_step == 0)
            {
                throw std::invalid_argument{"point_cloud2: point_step is 0 while the cloud has points"};
            }
            if (point_step > 0 && num_points > std::numeric_limits<std::uint32_t>::max() / point_step)
            {
                throw std::invalid_argument{
                    "point_cloud2: num_points * point_step exceeds the PointCloud2 row_step capacity (uint32)"};
            }
            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header(header);
            cloud.height(1);
            cloud.width(static_cast<std::uint32_t>(num_points));
            cloud.fields(fields);
            cloud.is_bigendian(detail::is_host_big_endian());
            cloud.point_step(point_step);
            cloud.row_step(static_cast<std::uint32_t>(point_step * num_points));
            cloud.is_dense(is_dense);
            cloud.data().resize(num_points * point_step);
            return cloud;
        }

        std::uint32_t packed_point_step(const std::vector<sensor_msgs::msg::PointField> &fields)
        {
            // uint64_t so the capacity check below is correct even on 32-bit platforms (size_t is 32-bit there,
            // where field.offset() + scalar_size * count could wrap and underestimate point_step).
            std::uint64_t step = 0;
            for (const auto &field : fields)
            {
                const std::size_t scalar_size = datatype_size(field.datatype());
                if (scalar_size == 0)
                {
                    throw std::invalid_argument{"point_cloud2: unknown datatype in field \"" + field.name() + "\""};
                }
                if (field.count() == 0)
                {
                    throw std::invalid_argument{"point_cloud2: field \"" + field.name() + "\" has count 0"};
                }
                step = std::max(step, static_cast<std::uint64_t>(field.offset()) +
                                          static_cast<std::uint64_t>(scalar_size) * field.count());
            }
            if (step > std::numeric_limits<std::uint32_t>::max())
            {
                throw std::invalid_argument{
                    "point_cloud2: the fields describe a point larger than the PointCloud2 point_step capacity "
                    "(uint32)"};
            }
            return static_cast<std::uint32_t>(step);
        }

        bool scalar_layout_matches(const std::vector<sensor_msgs::msg::PointField> &fields,
                                   const std::uint8_t scalar_datatype, const std::size_t element_size)
        {
            const std::size_t scalar_size = datatype_size(scalar_datatype);
            if (scalar_size == 0)
            {
                return false;
            }
            std::size_t expected_offset = 0;
            for (const auto &field : fields)
            {
                if (field.datatype() != scalar_datatype || field.offset() != expected_offset)
                {
                    return false;
                }
                expected_offset += scalar_size * field.count();
            }
            return expected_offset == element_size;
        }
    }  // namespace detail

    cloud_view::cloud_view(const sensor_msgs::msg::PointCloud2 &source_cloud)
        : cloud(&source_cloud), data_begin(source_cloud.data().data()), step(source_cloud.point_step()),
          num_points(static_cast<std::size_t>(source_cloud.width()) * source_cloud.height()),
          big_endian(source_cloud.is_bigendian())
    {
        // width and height are uint32 off the wire; on a 32-bit std::size_t their product (num_points, in the
        // initializer list above) could wrap. Reject that explicitly so every bounds check below relies on a
        // correct count. On the 64-bit targets this builds for (2^32-1)^2 < SIZE_MAX, so this never triggers there.
        const std::size_t cloud_width = source_cloud.width();
        const std::size_t cloud_height = source_cloud.height();
        if (cloud_height != 0 && cloud_width > std::numeric_limits<std::size_t>::max() / cloud_height)
        {
            throw std::invalid_argument{"point_cloud2::cloud_view: width * height overflows size_t"};
        }
        if (num_points > 0 && step == 0)
        {
            throw std::invalid_argument{"point_cloud2::cloud_view: point_step is 0 while the cloud has points"};
        }
        // Reject buffers smaller than width * height * point_step. The comparison is done via division to avoid
        // overflowing size_t: width, height and point_step all come straight off the wire, so the product
        // num_points * step could wrap to a small value and let an undersized buffer pass a naive "size < product"
        // check, opening out-of-bounds reads during iteration. step is non-zero here whenever num_points > 0 (the
        // check above guarantees it), so the division is safe.
        if (step != 0 && num_points > source_cloud.data().size() / static_cast<std::size_t>(step))
        {
            throw std::invalid_argument{
                "point_cloud2::cloud_view: the data buffer is smaller than width * height * point_step"};
        }
        // The reader walks points as a flat data_begin + index * point_step array. For an organized cloud
        // (height > 1) that is only correct when the rows are tightly packed (row_step == width * point_step);
        // reject a row-padded organized cloud rather than silently misreading the padding as point data. An
        // unorganized cloud (height <= 1, the Provizio radar / entities case) has a single row, so row_step does
        // not affect flat indexing.
        if (source_cloud.height() > 1 &&
            static_cast<std::size_t>(source_cloud.row_step()) != static_cast<std::size_t>(source_cloud.width()) * step)
        {
            throw std::invalid_argument{
                "point_cloud2::cloud_view: row_step != width * point_step for an organized (height > 1) cloud "
                "(row padding is not supported)"};
        }
    }

    std::optional<field_ref> cloud_view::field(const std::string_view name) const
    {
        for (const auto &cloud_field : cloud->fields())
        {
            if (std::string_view{cloud_field.name()} == name)
            {
                const std::size_t scalar_size = datatype_size(cloud_field.datatype());
                const auto offset = static_cast<std::size_t>(cloud_field.offset());
                const auto step_size = static_cast<std::size_t>(step);
                // Reject malformed fields. The natural test is offset + scalar_size * count <= step, but
                // that product can overflow size_t on 32-bit platforms and wrap a bogus huge-count field
                // back into range; rearrange via division (scalar_size > 0 once past the first clause) so
                // nothing can overflow.
                if (scalar_size == 0 || cloud_field.count() == 0 || offset > step_size ||
                    cloud_field.count() > (step_size - offset) / scalar_size)
                {
                    // Malformed field (unknown datatype, zero count, or lies beyond the point): treat as unresolvable
                    return std::nullopt;
                }
                return field_ref{cloud_field.name(), cloud_field.datatype(), cloud_field.count(), cloud_field.offset()};
            }
        }
        return std::nullopt;
    }

    field_ref cloud_view::field(const std::string_view name, const std::uint8_t expected_datatype,
                                const std::uint32_t min_count) const
    {
        const auto resolved = field(name);
        if (!resolved)
        {
            throw std::invalid_argument{"point_cloud2::cloud_view::field: no field \"" + std::string{name} +
                                        "\" in this cloud"};
        }
        if (resolved->datatype != expected_datatype)
        {
            throw std::invalid_argument{"point_cloud2::cloud_view::field: field \"" + std::string{name} +
                                        "\" has datatype " + std::to_string(resolved->datatype) + " but " +
                                        std::to_string(expected_datatype) + " was expected"};
        }
        if (resolved->count < min_count)
        {
            throw std::invalid_argument{"point_cloud2::cloud_view::field: field \"" + std::string{name} +
                                        "\" has count " + std::to_string(resolved->count) + " but at least " +
                                        std::to_string(min_count) + " was expected"};
        }
        return *resolved;
    }

    sensor_msgs::msg::PointCloud2 create_cloud(const std_msgs::msg::Header &header,
                                               const std::vector<sensor_msgs::msg::PointField> &fields,
                                               const std::uint32_t point_step, const void *points_data,
                                               const std::size_t num_points, const bool is_dense)
    {
        auto cloud = detail::init_cloud(header, fields, point_step, num_points, is_dense);
        if (num_points > 0)
        {
            if (points_data == nullptr)
            {
                throw std::invalid_argument{"point_cloud2::create_cloud: points_data is null while num_points > 0"};
            }
            std::memcpy(cloud.data().data(), points_data, num_points * static_cast<std::size_t>(point_step));
        }
        return cloud;
    }

    namespace
    {
        /**
         * @brief True when the cloud's fields are exactly the standard radar layout (same names, FLOAT32, dense
         * offsets, point_step 24) and the endianness is the host's — the precondition for the single-memcpy read.
         */
        bool is_standard_radar_layout(const sensor_msgs::msg::PointCloud2 &cloud)
        {
            if (cloud.point_step() != sizeof(radar_point) || cloud.is_bigendian() != detail::is_host_big_endian())
            {
                return false;
            }
            // read_radar_points calls this per message at sensor rate; radar_point_cloud_fields() is a pure,
            // argument-free constant, so compute it once and reuse it rather than reallocating (and re-copying
            // its field-name strings) on every call.
            static const auto standard_fields = radar_point_cloud_fields();
            const auto &fields = cloud.fields();
            if (fields.size() != standard_fields.size())
            {
                return false;
            }
            for (std::size_t i = 0; i < fields.size(); ++i)
            {
                if (fields[i].name() != standard_fields[i].name() ||
                    fields[i].offset() != standard_fields[i].offset() ||
                    fields[i].datatype() != standard_fields[i].datatype() ||
                    fields[i].count() != standard_fields[i].count())
                {
                    return false;
                }
            }
            return true;
        }
    }  // namespace

    std::vector<radar_point> read_radar_points(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        const cloud_view view{cloud};
        std::vector<radar_point> points{view.size()};
        if (points.empty())
        {
            return points;
        }

        if (is_standard_radar_layout(cloud))
        {
            // Fast bulk path: wire layout == radar_point layout
            std::memcpy(points.data(), cloud.data().data(), points.size() * sizeof(radar_point));
            return points;
        }

        // Generic fallback: resolve each standard field by name; missing ones are NaN-filled
        // static so the read lambda below can use it without capturing (MSVC C3493 rejects implicit
        // capture of a constexpr local, unlike gcc/clang; a static has no capture requirement anywhere).
        static constexpr float not_available = std::numeric_limits<float>::quiet_NaN();
        enum class radar_field_index : std::uint8_t
        {
            x,
            y,
            z,
            radar_relative_radial_velocity,
            signal_to_noise_ratio,
            ground_relative_radial_velocity
        };
        const std::array<std::optional<field_ref>, 6> fields{view.field("x"),
                                                             view.field("y"),
                                                             view.field("z"),
                                                             view.field("radar_relative_radial_velocity"),
                                                             view.field("signal_to_noise_ratio"),
                                                             view.field("ground_relative_radial_velocity")};
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const auto point = view[i];
            const auto read = [&point](const std::optional<field_ref> &field) {
                return field ? point.get<float>(*field) : not_available;
            };
            const auto idx_of = [](const radar_field_index idx) { return static_cast<std::size_t>(idx); };
            points[i] = radar_point{
                read(fields[idx_of(radar_field_index::x)]),  // NOLINT(*-constant-array-index)
                read(fields[idx_of(radar_field_index::y)]),  // NOLINT(*-constant-array-index)
                read(fields[idx_of(radar_field_index::z)]),  // NOLINT(*-constant-array-index)
                read(fields[idx_of(
                    radar_field_index::radar_relative_radial_velocity)]),        // NOLINT(*-constant-array-index)
                read(fields[idx_of(radar_field_index::signal_to_noise_ratio)]),  // NOLINT(*-constant-array-index)
                read(fields[idx_of(
                    radar_field_index::ground_relative_radial_velocity)])  // NOLINT(*-constant-array-index)
            };
        }
        return points;
    }

    std::vector<sensor_msgs::msg::PointField> entities_fields(const bool has_radar_data, const bool has_camera_data)
    {
        // Field order is part of the wire-format contract — see TOPICS.md of provizio_dds_idls
        // Maximum number of fields for a fused-kind entity cloud (both radar and camera data present).
        constexpr std::size_t max_entities_field_count = 13;
        std::vector<field_description> descriptions;
        descriptions.reserve(max_entities_field_count);
        if (has_radar_data)
        {
            descriptions.push_back({"entity_id", sensor_msgs::msg::PointField_Constants::UINT32, 1});
        }
        if (has_camera_data)
        {
            descriptions.push_back({"camera_entity_id", sensor_msgs::msg::PointField_Constants::UINT32, 1});
        }
        descriptions.push_back({"entity_class", sensor_msgs::msg::PointField_Constants::UINT8, 1});
        descriptions.push_back({"x", sensor_msgs::msg::PointField_Constants::FLOAT32, 1});
        descriptions.push_back({"y", sensor_msgs::msg::PointField_Constants::FLOAT32, 1});
        descriptions.push_back({"z", sensor_msgs::msg::PointField_Constants::FLOAT32, 1});
        if (has_radar_data)
        {
            descriptions.push_back(
                {"radar_relative_radial_velocity", sensor_msgs::msg::PointField_Constants::FLOAT32, 1});
            descriptions.push_back(
                {"ground_relative_radial_velocity", sensor_msgs::msg::PointField_Constants::FLOAT32, 1});
            descriptions.push_back({"orientation", sensor_msgs::msg::PointField_Constants::FLOAT32, 4});
            descriptions.push_back({"size", sensor_msgs::msg::PointField_Constants::FLOAT32, 3});
        }
        if (has_camera_data)
        {
            descriptions.push_back({"camera_bbox", sensor_msgs::msg::PointField_Constants::FLOAT32, 4});
        }
        descriptions.push_back({"entity_confidence", sensor_msgs::msg::PointField_Constants::UINT8, 1});
        descriptions.push_back({"entity_class_confidence", sensor_msgs::msg::PointField_Constants::UINT8, 1});
        return make_fields(descriptions);
    }

    std::optional<entities_kind> get_entities_kind(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        const cloud_view view{cloud};
        const bool has_entity_id = view.field("entity_id").has_value();
        const bool has_camera_entity_id = view.field("camera_entity_id").has_value();
        if (has_entity_id && has_camera_entity_id)
        {
            return entities_kind::fused;
        }
        if (has_entity_id)
        {
            return entities_kind::radar;
        }
        if (has_camera_entity_id)
        {
            return entities_kind::camera;
        }
        return std::nullopt;
    }

    std::vector<entity> read_entities(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        const cloud_view view{cloud};
        std::vector<entity> result{view.size()};
        if (result.empty())
        {
            return result;
        }

        // Resolve all 13 logical fields once; absent float fields read as NaN; absent id fields read as
        // no_entity_id; absent entity_class / confidence fields read as 0.
        const std::optional<field_ref> f_entity_id = view.field("entity_id");
        const std::optional<field_ref> f_camera_entity_id = view.field("camera_entity_id");
        const std::optional<field_ref> f_entity_class = view.field("entity_class");
        const std::optional<field_ref> f_x = view.field("x");
        const std::optional<field_ref> f_y = view.field("y");
        const std::optional<field_ref> f_z = view.field("z");
        const std::optional<field_ref> f_radar_relative_radial_velocity = view.field("radar_relative_radial_velocity");
        const std::optional<field_ref> f_ground_relative_radial_velocity =
            view.field("ground_relative_radial_velocity");
        std::optional<field_ref> f_orientation = view.field("orientation");
        std::optional<field_ref> f_size = view.field("size");
        std::optional<field_ref> f_camera_bbox = view.field("camera_bbox");
        // Treat a multi-component field as absent (read as NaN) unless it declares at least the full expected
        // component count, so a malformed short field can't make point_ref::get throw mid-read. Expected sizes
        // match the entity arrays: orientation and camera_bbox are 4, size is 3.
        if (f_orientation && f_orientation->count < 4)
        {
            f_orientation.reset();
        }
        if (f_size && f_size->count < 3)
        {
            f_size.reset();
        }
        if (f_camera_bbox && f_camera_bbox->count < 4)
        {
            f_camera_bbox.reset();
        }
        const std::optional<field_ref> f_entity_confidence = view.field("entity_confidence");
        const std::optional<field_ref> f_entity_class_confidence = view.field("entity_class_confidence");

        // static so the read lambdas below can use it without capturing (MSVC C3493 rejects implicit
        // capture of a constexpr local, unlike gcc/clang; a static has no capture requirement anywhere).
        static constexpr float nan_f = std::numeric_limits<float>::quiet_NaN();

        for (std::size_t i = 0; i < result.size(); ++i)
        {
            const auto point = view[i];
            entity &ent = result[i];

            const auto read_f32 = [&point](const std::optional<field_ref> &field) -> float {
                return field ? point.get<float>(*field) : nan_f;
            };
            const auto read_f32_sub = [&point](const std::optional<field_ref> &field, const std::size_t sub) -> float {
                return field ? point.get<float>(*field, sub) : nan_f;
            };
            const auto read_u32 = [&point](const std::optional<field_ref> &field) -> std::uint32_t {
                return field ? point.get<std::uint32_t>(*field) : no_entity_id;
            };
            const auto read_u8 = [&point](const std::optional<field_ref> &field) -> std::uint8_t {
                return field ? point.get<std::uint8_t>(*field) : std::uint8_t{0};
            };

            ent.entity_id = read_u32(f_entity_id);
            ent.camera_entity_id = read_u32(f_camera_entity_id);
            ent.entity_class = read_u8(f_entity_class);
            ent.x = read_f32(f_x);
            ent.y = read_f32(f_y);
            ent.z = read_f32(f_z);
            ent.radar_relative_radial_velocity = read_f32(f_radar_relative_radial_velocity);
            ent.ground_relative_radial_velocity = read_f32(f_ground_relative_radial_velocity);
            for (std::size_t sub = 0; sub < ent.orientation.size(); ++sub)
            {
                ent.orientation[sub] = read_f32_sub(f_orientation, sub);  // NOLINT(*-constant-array-index)
            }
            for (std::size_t sub = 0; sub < ent.size.size(); ++sub)
            {
                ent.size[sub] = read_f32_sub(f_size, sub);  // NOLINT(*-constant-array-index)
            }
            for (std::size_t sub = 0; sub < ent.camera_bbox.size(); ++sub)
            {
                ent.camera_bbox[sub] = read_f32_sub(f_camera_bbox, sub);  // NOLINT(*-constant-array-index)
            }
            ent.entity_confidence = read_u8(f_entity_confidence);
            ent.entity_class_confidence = read_u8(f_entity_class_confidence);
        }
        return result;
    }
}  // namespace provizio::dds::point_cloud2
