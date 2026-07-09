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
// Subcommand-driven tests for the provizio::dds::point_cloud2 utility
// (generic + Provizio-radar-specific PointCloud2 reading/writing). Each ctest
// entry runs this binary with one subcommand so per-case failure is isolated,
// mirroring the match_publisher_default test layout. No DDS participants are
// involved: these are pure message-manipulation tests.

#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "provizio/dds/point_cloud2.h"

namespace
{
    namespace pc2 = provizio::dds::point_cloud2;

    /// Throws std::runtime_error on failure; caught (and reported) in main.
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

    void test_radar_roundtrip()
    {
        const std::vector<pc2::radar_point> points{{1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F},
                                                   {10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F}};
        const auto cloud = pc2::make_radar_point_cloud(pc2::make_header(1, 2U, "test_radar"), points);

        check(cloud.header().frame_id() == "test_radar", "radar_roundtrip: frame_id mismatch");
        check(cloud.height() == 1U, "radar_roundtrip: height != 1");
        check(cloud.width() == 2U, "radar_roundtrip: width != 2");
        check(cloud.point_step() == sizeof(pc2::radar_point), "radar_roundtrip: point_step != 24");
        check(cloud.row_step() == 2U * sizeof(pc2::radar_point), "radar_roundtrip: row_step mismatch");
        check(cloud.fields().size() == 6, "radar_roundtrip: 6 fields expected");
        check(cloud.is_dense(), "radar_roundtrip: is_dense expected to default to true");
        check(cloud.data().size() == 2 * sizeof(pc2::radar_point), "radar_roundtrip: data size mismatch");
        // The wire layout must byte-match the radar_point array (host-endian)
        check(std::memcmp(cloud.data().data(), points.data(), cloud.data().size()) == 0,
              "radar_roundtrip: data bytes mismatch");
        check(cloud.is_bigendian() == pc2::detail::is_host_big_endian(),
              "radar_roundtrip: is_bigendian must reflect the host endianness");

        // Raw overload directly
        const auto raw_cloud = pc2::create_cloud(pc2::make_header(1, 2U, "test_radar"), pc2::radar_point_cloud_fields(),
                                                 sizeof(pc2::radar_point), points.data(), points.size());
        check(raw_cloud.data() == cloud.data() && raw_cloud.point_step() == cloud.point_step(),
              "radar_roundtrip: raw create_cloud differs from make_radar_point_cloud");

        // Empty cloud
        const auto empty_cloud =
            pc2::make_radar_point_cloud(pc2::make_header(0, 0U, "r"), std::vector<pc2::radar_point>{});
        check(empty_cloud.width() == 0U && empty_cloud.data().empty(), "radar_roundtrip: empty cloud mismatch");

        // Non-contiguous range of radar_point (per-point memcpy tier)
        const std::deque<pc2::radar_point> deque_points{points.begin(), points.end()};
        const auto deque_cloud = pc2::make_radar_point_cloud(pc2::make_header(1, 2U, "test_radar"), deque_points);
        check(deque_cloud.data() == cloud.data(), "radar_roundtrip: deque-built cloud differs from vector-built");

        bool threw = false;
        try
        {
            (void)pc2::create_cloud(pc2::make_header(0, 0U, "r"), pc2::radar_point_cloud_fields(),
                                    sizeof(pc2::radar_point), nullptr, 1);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "radar_roundtrip: null points_data with num_points > 0 must throw");
    }

    void test_basics()
    {
        const auto header = pc2::make_header(123, 456U, "test_frame");
        check(header.stamp().sec() == 123, "make_header: sec mismatch");
        check(header.stamp().nanosec() == 456U, "make_header: nanosec mismatch");
        check(header.frame_id() == "test_frame", "make_header: frame_id mismatch");

        const auto fields = pc2::radar_point_cloud_fields();
        check(fields.size() == 6, "radar_point_cloud_fields: 6 fields expected");
        check(fields[5].name() == "ground_relative_radial_velocity", "radar_point_cloud_fields: unexpected last field");
        check(fields[5].offset() == 20, "radar_point_cloud_fields: unexpected last offset");

        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::INT8) == 1, "datatype_size(INT8) != 1");
        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::UINT8) == 1, "datatype_size(UINT8) != 1");
        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::INT16) == 2, "datatype_size(INT16) != 2");
        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::UINT16) == 2, "datatype_size(UINT16) != 2");
        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::INT32) == 4, "datatype_size(INT32) != 4");
        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::UINT32) == 4, "datatype_size(UINT32) != 4");
        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::FLOAT32) == 4, "datatype_size(FLOAT32) != 4");
        check(pc2::datatype_size(sensor_msgs::msg::PointField_Constants::FLOAT64) == 8, "datatype_size(FLOAT64) != 8");
        check(pc2::datatype_size(0) == 0, "datatype_size(invalid) != 0");
    }
    void test_cloud_view_basic()
    {
        const std::vector<pc2::radar_point> points{{1.5F, -2.5F, 3.5F, -4.5F, 5.5F, -6.5F},
                                                   {7.5F, 8.5F, 9.5F, 10.5F, 11.5F, 12.5F}};
        const auto cloud = pc2::make_radar_point_cloud(pc2::make_header(0, 0U, "r"), points);
        const pc2::cloud_view view{cloud};

        check(view.size() == 2, "cloud_view: size != 2");
        check(!view.field("no_such_field").has_value(), "cloud_view: unexpected field resolved");

        const auto x_field = view.field("x");
        const auto snr_field = view.field("signal_to_noise_ratio");
        check(x_field.has_value() && snr_field.has_value(), "cloud_view: standard fields not resolved");
        check(x_field->offset == 0 && x_field->datatype == sensor_msgs::msg::PointField_Constants::FLOAT32 &&
                  x_field->count == 1,
              "cloud_view: x field metadata mismatch");
        check(snr_field->offset == 16, "cloud_view: snr field offset mismatch");

        // Indexed access + conversion-on-read (float storage read as double and as int)
        check_near(view[0].get<double>(*x_field), 1.5, 1e-9, "cloud_view: get<double> of x[0]");
        check(view[1].get<int>(*snr_field) == 11, "cloud_view: get<int> of snr[1] (truncating conversion)");

        // Iteration
        std::size_t index = 0;
        float sum_x = 0;
        for (const auto point : view)
        {
            sum_x += point.get<float>(*x_field);
            ++index;
        }
        check(index == 2, "cloud_view: iterated wrong number of points");
        check_near(sum_x, 9.0, 1e-6, "cloud_view: x sum mismatch");

        // The iterator satisfies the random-access contract (std algorithms compile and work)
        const auto second = view.begin() + 1;
        check((1 + view.begin()) == second && (view.end() - 1) == second, "cloud_view: iterator +/- forms");
        check_near(second[0].get<float>(*x_field), 7.5, 1e-6, "cloud_view: iterator operator[]");
        check(view.begin() <= second && second >= view.begin() && view.end() > view.begin(),
              "cloud_view: iterator comparisons");
        check(std::distance(view.begin(), view.end()) == 2, "cloud_view: std::distance");

        // Strict field(): correct expectations pass, wrong datatype throws
        (void)view.field("x", sensor_msgs::msg::PointField_Constants::FLOAT32, 1);
        bool threw = false;
        try
        {
            (void)view.field("x", sensor_msgs::msg::PointField_Constants::FLOAT64, 1);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "cloud_view: strict field() with wrong datatype did not throw");

        threw = false;
        try
        {
            (void)view.field("x", sensor_msgs::msg::PointField_Constants::FLOAT32, 2);  // x has count 1
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "cloud_view: strict field() with too-small count did not throw");
    }

    void test_cloud_view_endianness()
    {
        // Hand-build a foreign-endian cloud with one FLOAT32 field "v" of value 42.5f
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header(pc2::make_header(0, 0U, "endian"));
        cloud.height(1);
        cloud.width(1);
        std::vector<sensor_msgs::msg::PointField> fields{1};
        fields[0].name("v");
        fields[0].offset(0);
        fields[0].datatype(sensor_msgs::msg::PointField_Constants::FLOAT32);
        fields[0].count(1);
        cloud.fields(fields);
        cloud.is_bigendian(!pc2::detail::is_host_big_endian());  // declare FOREIGN endianness
        cloud.point_step(4);
        cloud.row_step(4);
        cloud.is_dense(true);
        const float value = 42.5F;
        std::array<std::uint8_t, 4> bytes{};
        std::memcpy(bytes.data(), &value, sizeof(value));
        std::reverse(bytes.begin(), bytes.end());  // store byte-swapped
        cloud.data().assign(bytes.begin(), bytes.end());

        const pc2::cloud_view view{cloud};
        const auto v_field = view.field("v");
        check(v_field.has_value(), "endianness: field not resolved");
        check_near(view[0].get<float>(*v_field), 42.5, 1e-6, "endianness: byte-swapped read mismatch");
    }

    void test_read_radar_points()
    {
        // Standard layout cloud → must round-trip exactly (fast bulk path)
        const std::vector<pc2::radar_point> points{{1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F},
                                                   {-1.5F, -2.5F, -3.5F, -4.5F, -5.5F, -6.5F}};
        const auto standard_cloud = pc2::make_radar_point_cloud(pc2::make_header(0, 0U, "r"), points);
        const auto read_back = pc2::read_radar_points(standard_cloud);
        check(read_back.size() == 2, "read_radar_points: standard size mismatch");
        check(std::memcmp(read_back.data(), points.data(), points.size() * sizeof(pc2::radar_point)) == 0,
              "read_radar_points: standard roundtrip mismatch");

        // Shuffled field order (still all 6 present) → generic path, same values.
        // Build a cloud where the fields vector is re-ordered (x↔grrv by name, but each field retains its correct
        // byte offset so name-based resolution still returns the right value).  Only the field-list ORDER differs
        // from standard, so is_standard_radar_layout() returns false and the generic fallback is exercised.
        auto shuffled_fields = pc2::radar_point_cloud_fields();
        // Keep both fields' offsets correct; only swap their position in the list so the order is non-standard.
        std::swap(shuffled_fields[0], shuffled_fields[5]);  // x <-> ground_relative_radial_velocity positions
        // Restore the offsets so each field still describes the right byte position in the packed data:
        //   the entry now at index 0 was originally grrv (offset 20); it must keep offset 20 to find grrv data.
        //   the entry now at index 5 was originally x   (offset  0); it must keep offset  0 to find x data.
        // Nothing to do — std::swap already moved the offset along with the name, so they are consistent.
        // Data is the ORIGINAL points (no byte-level re-ordering): offset 0 holds x_original, offset 20 grrv_original.
        const auto shuffled_cloud = pc2::create_cloud(pc2::make_header(0, 0U, "r"), shuffled_fields,
                                                      sizeof(pc2::radar_point), points.data(), points.size());
        const auto shuffled_read = pc2::read_radar_points(shuffled_cloud);
        check(shuffled_read.size() == 2, "read_radar_points: shuffled size mismatch");
        check(std::memcmp(shuffled_read.data(), points.data(), points.size() * sizeof(pc2::radar_point)) == 0,
              "read_radar_points: shuffled values mismatch (generic path broken)");

        // Offset-swapped cloud: x stored at byte 20, ground_relative_radial_velocity at byte 0 (field names point
        // at the right offsets). The generic path must reconstruct the ORIGINAL values; a broken
        // is_standard_radar_layout taking the bulk path here would visibly produce swapped values instead.
        auto offset_swapped_fields = pc2::radar_point_cloud_fields();
        const auto x_offset = offset_swapped_fields[0].offset();
        offset_swapped_fields[0].offset(offset_swapped_fields[5].offset());
        offset_swapped_fields[5].offset(x_offset);
        auto repacked_points = points;
        for (auto &point : repacked_points)
        {
            std::swap(point.x, point.ground_relative_radial_velocity);
        }
        const auto offset_swapped_cloud =
            pc2::create_cloud(pc2::make_header(0, 0U, "r"), offset_swapped_fields, sizeof(pc2::radar_point),
                              repacked_points.data(), repacked_points.size());
        const auto offset_swapped_read = pc2::read_radar_points(offset_swapped_cloud);
        check(offset_swapped_read.size() == 2, "read_radar_points: offset-swapped size mismatch");
        check(std::memcmp(offset_swapped_read.data(), points.data(), points.size() * sizeof(pc2::radar_point)) == 0,
              "read_radar_points: offset-swapped values mismatch (bulk path taken for a non-standard layout?)");

        // Missing optional ground_relative_radial_velocity → NaN-filled, other fields read fine
        auto five_fields = pc2::radar_point_cloud_fields();
        five_fields.pop_back();
        const std::vector<std::array<float, 5>> five_field_points{{1.0F, 2.0F, 3.0F, 4.0F, 5.0F}};
        const auto five_field_cloud = pc2::create_cloud(pc2::make_header(0, 0U, "r"), five_fields, 5 * sizeof(float),
                                                        five_field_points.data(), five_field_points.size());
        const auto five_field_read = pc2::read_radar_points(five_field_cloud);
        check(five_field_read.size() == 1, "read_radar_points: 5-field size mismatch");
        check(five_field_read[0].x == 1.0F && five_field_read[0].signal_to_noise_ratio == 5.0F,
              "read_radar_points: 5-field values mismatch");
        check(std::isnan(five_field_read[0].ground_relative_radial_velocity),
              "read_radar_points: missing optional field must be NaN-filled");

        // Empty cloud
        check(pc2::read_radar_points(
                  pc2::make_radar_point_cloud(pc2::make_header(0, 0U, "r"), std::vector<pc2::radar_point>{}))
                  .empty(),
              "read_radar_points: empty cloud must yield no points");
    }

    void test_create_cloud_tiers()
    {
        // The same data through all three tiers must produce byte-identical messages.
        const auto header = pc2::make_header(7, 8U, "tiers");
        const auto fields =
            pc2::make_fields({{"x", sensor_msgs::msg::PointField_Constants::FLOAT32},
                              {"y", sensor_msgs::msg::PointField_Constants::FLOAT32},
                              {"z", sensor_msgs::msg::PointField_Constants::FLOAT32},
                              {"radar_relative_radial_velocity", sensor_msgs::msg::PointField_Constants::FLOAT32},
                              {"signal_to_noise_ratio", sensor_msgs::msg::PointField_Constants::FLOAT32},
                              {"ground_relative_radial_velocity", sensor_msgs::msg::PointField_Constants::FLOAT32}});
        check(fields.size() == 6 && fields[5].offset() == 20, "make_fields: offsets not auto-computed densely");

        const std::vector<std::array<float, 6>> tier1_points{{1, 2, 3, 4, 5, 6}, {7, 8, 9, 10, 11, 12}};
        const std::deque<std::array<float, 6>> tier2_points{tier1_points.begin(), tier1_points.end()};
        const std::vector<std::vector<double>> tier3_points{{1, 2, 3, 4, 5, 6}, {7, 8, 9, 10, 11, 12}};

        const auto tier1_cloud = pc2::create_cloud(header, fields, tier1_points);  // contiguous: whole-range memcpy
        const auto tier2_cloud = pc2::create_cloud(header, fields, tier2_points);  // per-point memcpy
        const auto tier3_cloud = pc2::create_cloud(header, fields, tier3_points);  // per-value conversion

        check(tier1_cloud.point_step() == 24 && tier1_cloud.width() == 2, "tiers: tier1 structure mismatch");
        check(tier1_cloud.data() == tier2_cloud.data(), "tiers: tier2 (deque) data differs from tier1");
        check(tier1_cloud.data() == tier3_cloud.data(), "tiers: tier3 (vector<vector<double>>) data differs");

        const float c_points[2][6]{{1, 2, 3, 4, 5, 6}, {7, 8, 9, 10, 11, 12}};  // NOLINT(*-avoid-c-arrays)
        const auto c_array_cloud = pc2::create_cloud(header, fields, c_points);
        check(c_array_cloud.data() == tier1_cloud.data(), "tiers: C-array data differs from tier1");

        // Heterogeneous fields (UINT32 + FLOAT32 + FLOAT64) — must take the conversion tier and pack correctly
        const auto mixed_fields = pc2::make_fields({{"id", sensor_msgs::msg::PointField_Constants::UINT32},
                                                    {"value", sensor_msgs::msg::PointField_Constants::FLOAT32},
                                                    {"precise", sensor_msgs::msg::PointField_Constants::FLOAT64}});
        const std::vector<std::array<double, 3>> mixed_points{{42.0, 1.5, 1e-12}, {43.0, -2.5, -2e9}};
        const auto mixed_cloud = pc2::create_cloud(header, mixed_fields, mixed_points);
        const pc2::cloud_view mixed_view{mixed_cloud};
        check(mixed_view[0].get<std::uint32_t>(*mixed_view.field("id")) == 42U, "tiers: mixed id mismatch");
        check_near(mixed_view[1].get<float>(*mixed_view.field("value")), -2.5, 1e-6, "tiers: mixed value mismatch");
        check_near(mixed_view[0].get<double>(*mixed_view.field("precise")), 1e-12, 1e-20, "tiers: FLOAT64 packing");
        check_near(mixed_view[1].get<double>(*mixed_view.field("precise")), -2e9, 1e-3, "tiers: FLOAT64 packing 2");

        // A count>1 field consumes count values per row (flattened rows)
        const auto multi_count_fields =
            pc2::make_fields({{"id", sensor_msgs::msg::PointField_Constants::UINT32},
                              {"orientation", sensor_msgs::msg::PointField_Constants::FLOAT32, 4}});
        const std::vector<std::vector<double>> multi_count_points{{1.0, 0.5, 0.6, 0.7, 0.8}};
        const auto multi_count_cloud = pc2::create_cloud(header, multi_count_fields, multi_count_points);
        const pc2::cloud_view multi_count_view{multi_count_cloud};
        check_near(multi_count_view[0].get<float>(*multi_count_view.field("orientation"), 3), 0.8, 1e-6,
                   "tiers: count>1 subfield mismatch");

        // A row with the wrong number of values must throw
        bool threw = false;
        try
        {
            (void)pc2::create_cloud(header, fields, std::vector<std::vector<double>>{{1, 2, 3}});
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "tiers: short row did not throw");

        threw = false;
        try
        {
            (void)pc2::create_cloud(header, fields, std::vector<std::vector<double>>{{1, 2, 3, 4, 5, 6, 7}});
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "tiers: over-long row did not throw");

        threw = false;
        try
        {
            (void)pc2::make_fields({{"bad", 0}});
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "tiers: make_fields with unknown datatype did not throw");
    }

    void test_entities()
    {
        const auto header = pc2::make_header(0, 0U, "entities");

        // Radar entities: entity_id, entity_class, x, y, z, radar_relative_radial_velocity,
        // ground_relative_radial_velocity, orientation(4), size(3), entity_confidence, entity_class_confidence
        const std::vector<std::vector<double>> radar_entities{
            {42, 3, 1.5, -2.5, 0.5, 10.5, 9.5, 0.1, 0.2, 0.3, 0.4, 4.5, 1.8, 1.5, 90, 80}};
        const auto radar_cloud = pc2::make_radar_entities(header, radar_entities);
        check(radar_cloud.point_step() == 55, "entities: radar point_step != 55");
        const pc2::cloud_view radar_view{radar_cloud};
        check(radar_view.size() == 1, "entities: radar size mismatch");
        check(radar_view[0].get<std::uint32_t>(*radar_view.field("entity_id")) == 42U, "entities: entity_id");
        check(radar_view[0].get<std::uint32_t>(*radar_view.field("entity_class")) == 3U, "entities: entity_class");
        check(radar_view.field("x")->offset == 5, "entities: x must start at the unaligned offset 5");
        check_near(radar_view[0].get<float>(*radar_view.field("x")), 1.5, 1e-6, "entities: x (unaligned)");
        check_near(radar_view[0].get<float>(*radar_view.field("orientation"), 3), 0.4, 1e-6,
                   "entities: orientation[3]");
        check_near(radar_view[0].get<float>(*radar_view.field("size"), 2), 1.5, 1e-6, "entities: size[2]");
        check(radar_view[0].get<int>(*radar_view.field("entity_class_confidence")) == 80,
              "entities: entity_class_confidence");
        check(!radar_view.field("camera_bbox").has_value(), "entities: radar cloud must have no camera_bbox");
        check(!radar_view.field("camera_entity_id").has_value(), "entities: radar cloud must have no camera_entity_id");

        // Camera entities: camera_entity_id, entity_class, x, y, z, camera_bbox(4), entity_confidence,
        // entity_class_confidence
        const std::vector<std::vector<double>> camera_entities{{7, 1, 1.0, 2.0, 3.0, 10, 20, 30, 40, 95, 85}};
        const auto camera_cloud = pc2::make_camera_entities(header, camera_entities);
        check(camera_cloud.point_step() == 35, "entities: camera point_step != 35");
        const pc2::cloud_view camera_view{camera_cloud};
        check(camera_view[0].get<std::uint32_t>(*camera_view.field("camera_entity_id")) == 7U,
              "entities: camera_entity_id");
        check_near(camera_view[0].get<float>(*camera_view.field("camera_bbox"), 3), 40, 1e-6,
                   "entities: camera_bbox[3]");
        check(camera_view[0].get<int>(*camera_view.field("entity_confidence")) == 95, "entities: camera confidence");
        check(!camera_view.field("entity_id").has_value(), "entities: camera cloud must have no radar entity_id");

        // Fused entities: both id kinds + all optional fields; row of 21 values
        const std::vector<std::vector<double>> fused_entities{
            {42, 7, 2, 1, 2, 3, 4, 5, 0.1, 0.2, 0.3, 0.4, 5, 6, 7, 11, 22, 33, 44, 70, 60}};
        const auto fused_cloud = pc2::make_fused_entities(header, fused_entities);
        check(fused_cloud.point_step() == 75, "entities: fused point_step != 75");
        const pc2::cloud_view fused_view{fused_cloud};
        check(fused_view[0].get<std::uint32_t>(*fused_view.field("entity_id")) == 42U, "entities: fused entity_id");
        check(fused_view[0].get<std::uint32_t>(*fused_view.field("camera_entity_id")) == 7U,
              "entities: fused camera_entity_id");
        check_near(fused_view[0].get<float>(*fused_view.field("camera_bbox"), 0), 11, 1e-6,
                   "entities: fused camera_bbox[0]");
        check(fused_view[0].get<int>(*fused_view.field("entity_confidence")) == 70, "entities: fused confidence");

        // get_entities_kind: radar/camera/fused detection
        check(pc2::get_entities_kind(radar_cloud) == pc2::entities_kind::radar, "entities_kind: radar cloud");
        check(pc2::get_entities_kind(camera_cloud) == pc2::entities_kind::camera, "entities_kind: camera cloud");
        check(pc2::get_entities_kind(fused_cloud) == pc2::entities_kind::fused, "entities_kind: fused cloud");

        // get_entities_kind: a plain radar POINT cloud carries neither entity id field
        const auto plain_radar_cloud =
            pc2::make_radar_point_cloud(header, std::vector<pc2::radar_point>{{1, 2, 3, 4, 5, 6}});
        check(!pc2::get_entities_kind(plain_radar_cloud).has_value(),
              "entities_kind: radar point cloud must be nullopt");

        // read_entities: radar cloud round-trip
        const auto radar_ents = pc2::read_entities(radar_cloud);
        check(radar_ents.size() == 1, "read_entities: radar size mismatch");
        check(radar_ents[0].entity_id == 42U, "read_entities: radar entity_id");
        check(radar_ents[0].entity_class == 3U, "read_entities: radar entity_class");
        check_near(radar_ents[0].x, 1.5, 1e-6, "read_entities: radar x");
        check_near(radar_ents[0].orientation[0], 0.1F, 1e-6, "read_entities: radar orientation[0]");
        check_near(radar_ents[0].orientation[1], 0.2F, 1e-6, "read_entities: radar orientation[1]");
        check_near(radar_ents[0].orientation[2], 0.3F, 1e-6, "read_entities: radar orientation[2]");
        check_near(radar_ents[0].orientation[3], 0.4F, 1e-6, "read_entities: radar orientation[3]");
        check_near(radar_ents[0].size[2], 1.5F, 1e-6, "read_entities: radar size[2]");
        check(radar_ents[0].entity_confidence == 90U, "read_entities: radar entity_confidence");
        check(radar_ents[0].entity_class_confidence == 80U, "read_entities: radar entity_class_confidence");
        check(radar_ents[0].camera_entity_id == pc2::no_entity_id,
              "read_entities: radar camera_entity_id must be no_entity_id");
        check(std::isnan(radar_ents[0].camera_bbox[0]), "read_entities: radar camera_bbox[0] must be NaN");
        check(std::isnan(radar_ents[0].camera_bbox[1]), "read_entities: radar camera_bbox[1] must be NaN");
        check(std::isnan(radar_ents[0].camera_bbox[2]), "read_entities: radar camera_bbox[2] must be NaN");
        check(std::isnan(radar_ents[0].camera_bbox[3]), "read_entities: radar camera_bbox[3] must be NaN");

        // read_entities: camera cloud round-trip
        const auto camera_ents = pc2::read_entities(camera_cloud);
        check(camera_ents.size() == 1, "read_entities: camera size mismatch");
        check(camera_ents[0].camera_entity_id == 7U, "read_entities: camera camera_entity_id");
        check_near(camera_ents[0].camera_bbox[0], 10.0F, 1e-6, "read_entities: camera camera_bbox[0]");
        check_near(camera_ents[0].camera_bbox[1], 20.0F, 1e-6, "read_entities: camera camera_bbox[1]");
        check_near(camera_ents[0].camera_bbox[2], 30.0F, 1e-6, "read_entities: camera camera_bbox[2]");
        check_near(camera_ents[0].camera_bbox[3], 40.0F, 1e-6, "read_entities: camera camera_bbox[3]");
        check(camera_ents[0].entity_id == pc2::no_entity_id, "read_entities: camera entity_id must be no_entity_id");
        check(std::isnan(camera_ents[0].radar_relative_radial_velocity),
              "read_entities: camera radar_relative_radial_velocity must be NaN");
        check(std::isnan(camera_ents[0].orientation[0]), "read_entities: camera orientation[0] must be NaN");

        // read_entities: fused cloud round-trip
        const auto fused_ents = pc2::read_entities(fused_cloud);
        check(fused_ents.size() == 1, "read_entities: fused size mismatch");
        check(fused_ents[0].entity_id == 42U, "read_entities: fused entity_id");
        check(fused_ents[0].camera_entity_id == 7U, "read_entities: fused camera_entity_id");
        check_near(fused_ents[0].camera_bbox[0], 11.0F, 1e-6, "read_entities: fused camera_bbox[0]");
        check(fused_ents[0].entity_confidence == 70U, "read_entities: fused entity_confidence");

        // no_entity_id sentinel value
        check(pc2::no_entity_id == std::numeric_limits<std::uint32_t>::max(), "no_entity_id must be uint32 max");

        // make_entities(header, kind, range): struct-based overload — radar kind
        pc2::entity e1{};
        e1.entity_id = 11U;
        e1.entity_class = 2U;
        e1.x = 3.0F;
        e1.y = -1.5F;
        e1.z = 0.25F;
        e1.radar_relative_radial_velocity = 5.5F;
        e1.ground_relative_radial_velocity = 4.5F;
        e1.orientation = {0.1F, 0.2F, 0.3F, 0.4F};
        e1.size = {3.0F, 1.5F, 1.0F};
        e1.entity_confidence = 80U;
        e1.entity_class_confidence = 70U;

        pc2::entity e2{};
        e2.entity_id = 22U;
        e2.entity_class = 5U;
        e2.x = -2.0F;
        e2.y = 1.0F;
        e2.z = 0.5F;
        e2.radar_relative_radial_velocity = -3.5F;
        e2.ground_relative_radial_velocity = -2.5F;
        e2.orientation = {0.5F, 0.6F, 0.7F, 0.8F};
        e2.size = {2.0F, 1.0F, 0.5F};
        e2.entity_confidence = 90U;
        e2.entity_class_confidence = 85U;

        const auto struct_radar_cloud =
            pc2::make_entities(header, pc2::entities_kind::radar, std::vector<pc2::entity>{e1, e2});
        check(pc2::get_entities_kind(struct_radar_cloud) == pc2::entities_kind::radar,
              "make_entities(kind): radar cloud kind mismatch");
        const auto struct_radar_ents = pc2::read_entities(struct_radar_cloud);
        check(struct_radar_ents.size() == 2, "make_entities(kind): radar size mismatch");
        check(struct_radar_ents[0].entity_id == 11U, "make_entities(kind): radar entity_id[0]");
        check(struct_radar_ents[0].entity_class == 2U, "make_entities(kind): radar entity_class[0]");
        check_near(struct_radar_ents[0].x, 3.0F, 1e-6F, "make_entities(kind): radar x[0]");
        check_near(struct_radar_ents[0].y, -1.5F, 1e-6F, "make_entities(kind): radar y[0]");
        check_near(struct_radar_ents[0].radar_relative_radial_velocity, 5.5F, 1e-6F,
                   "make_entities(kind): radar rrv[0]");
        check_near(struct_radar_ents[0].orientation[3], 0.4F, 1e-6F, "make_entities(kind): radar orientation[3][0]");
        check_near(struct_radar_ents[0].size[2], 1.0F, 1e-6F, "make_entities(kind): radar size[2][0]");
        check(struct_radar_ents[0].entity_confidence == 80U, "make_entities(kind): radar confidence[0]");
        check(struct_radar_ents[0].camera_entity_id == pc2::no_entity_id,
              "make_entities(kind): radar absent camera_entity_id must be no_entity_id");
        check(std::isnan(struct_radar_ents[0].camera_bbox[0]),
              "make_entities(kind): radar absent camera_bbox must be NaN");
        check(struct_radar_ents[1].entity_id == 22U, "make_entities(kind): radar entity_id[1]");
        check_near(struct_radar_ents[1].x, -2.0F, 1e-6F, "make_entities(kind): radar x[1]");

        // make_entities(header, kind, range): camera kind
        pc2::entity ec1{};
        ec1.camera_entity_id = 7U;
        ec1.entity_class = 3U;
        ec1.x = 1.0F;
        ec1.y = 2.0F;
        ec1.z = 3.0F;
        ec1.camera_bbox = {10.0F, 20.0F, 30.0F, 40.0F};
        ec1.entity_confidence = 95U;
        ec1.entity_class_confidence = 88U;

        pc2::entity ec2{};
        ec2.camera_entity_id = 8U;
        ec2.entity_class = 1U;
        ec2.x = 4.0F;
        ec2.y = 5.0F;
        ec2.z = 6.0F;
        ec2.camera_bbox = {50.0F, 60.0F, 70.0F, 80.0F};
        ec2.entity_confidence = 75U;
        ec2.entity_class_confidence = 65U;

        const auto struct_camera_cloud =
            pc2::make_entities(header, pc2::entities_kind::camera, std::vector<pc2::entity>{ec1, ec2});
        check(pc2::get_entities_kind(struct_camera_cloud) == pc2::entities_kind::camera,
              "make_entities(kind): camera cloud kind mismatch");
        const auto struct_camera_ents = pc2::read_entities(struct_camera_cloud);
        check(struct_camera_ents.size() == 2, "make_entities(kind): camera size mismatch");
        check(struct_camera_ents[0].camera_entity_id == 7U, "make_entities(kind): camera camera_entity_id[0]");
        check_near(struct_camera_ents[0].camera_bbox[0], 10.0F, 1e-6F, "make_entities(kind): camera camera_bbox[0][0]");
        check_near(struct_camera_ents[0].camera_bbox[3], 40.0F, 1e-6F, "make_entities(kind): camera camera_bbox[3][0]");
        check(struct_camera_ents[0].entity_id == pc2::no_entity_id,
              "make_entities(kind): camera absent entity_id must be no_entity_id");
        check(std::isnan(struct_camera_ents[0].radar_relative_radial_velocity),
              "make_entities(kind): camera absent radar_rrv must be NaN");
        check(struct_camera_ents[1].camera_entity_id == 8U, "make_entities(kind): camera camera_entity_id[1]");

        // make_entities(header, kind, range): fused kind
        pc2::entity ef1{};
        ef1.entity_id = 42U;
        ef1.camera_entity_id = 7U;
        ef1.entity_class = 2U;
        ef1.x = 1.0F;
        ef1.y = 2.0F;
        ef1.z = 3.0F;
        ef1.radar_relative_radial_velocity = 4.0F;
        ef1.ground_relative_radial_velocity = 5.0F;
        ef1.orientation = {0.1F, 0.2F, 0.3F, 0.4F};
        ef1.size = {5.0F, 6.0F, 7.0F};
        ef1.camera_bbox = {11.0F, 22.0F, 33.0F, 44.0F};
        ef1.entity_confidence = 70U;
        ef1.entity_class_confidence = 60U;

        pc2::entity ef2{};
        ef2.entity_id = 99U;
        ef2.camera_entity_id = 12U;
        ef2.entity_class = 4U;
        ef2.x = -1.0F;
        ef2.y = -2.0F;
        ef2.z = -3.0F;
        ef2.radar_relative_radial_velocity = -4.0F;
        ef2.ground_relative_radial_velocity = -5.0F;
        ef2.orientation = {0.5F, 0.6F, 0.7F, 0.8F};
        ef2.size = {2.0F, 3.0F, 4.0F};
        ef2.camera_bbox = {55.0F, 66.0F, 77.0F, 88.0F};
        ef2.entity_confidence = 50U;
        ef2.entity_class_confidence = 40U;

        const auto struct_fused_cloud =
            pc2::make_entities(header, pc2::entities_kind::fused, std::vector<pc2::entity>{ef1, ef2});
        check(pc2::get_entities_kind(struct_fused_cloud) == pc2::entities_kind::fused,
              "make_entities(kind): fused cloud kind mismatch");
        const auto struct_fused_ents = pc2::read_entities(struct_fused_cloud);
        check(struct_fused_ents.size() == 2, "make_entities(kind): fused size mismatch");
        check(struct_fused_ents[0].entity_id == 42U, "make_entities(kind): fused entity_id[0]");
        check(struct_fused_ents[0].camera_entity_id == 7U, "make_entities(kind): fused camera_entity_id[0]");
        check_near(struct_fused_ents[0].camera_bbox[0], 11.0F, 1e-6F, "make_entities(kind): fused camera_bbox[0][0]");
        check_near(struct_fused_ents[0].orientation[3], 0.4F, 1e-6F, "make_entities(kind): fused orientation[3][0]");
        check(struct_fused_ents[1].entity_id == 99U, "make_entities(kind): fused entity_id[1]");
        check(struct_fused_ents[1].camera_entity_id == 12U, "make_entities(kind): fused camera_entity_id[1]");
        check_near(struct_fused_ents[1].camera_bbox[3], 88.0F, 1e-6F, "make_entities(kind): fused camera_bbox[3][1]");

        // Non-vector iterable: std::array<entity, 2>
        const std::array<pc2::entity, 2> arr_entities{e1, e2};
        const auto arr_cloud = pc2::make_entities(header, pc2::entities_kind::radar, arr_entities);
        const auto arr_ents = pc2::read_entities(arr_cloud);
        check(arr_ents.size() == 2, "make_entities(kind): array range size mismatch");
        check(arr_ents[0].entity_id == e1.entity_id, "make_entities(kind): array range entity_id[0]");
        check(arr_ents[1].entity_id == e2.entity_id, "make_entities(kind): array range entity_id[1]");

        // Per-entity kind() / has_radar_data() / has_camera_data() methods
        // radar entity: has_radar_data true, has_camera_data false, kind == radar
        check(radar_ents[0].has_radar_data(), "entity::has_radar_data: radar entity must be true");
        check(!radar_ents[0].has_camera_data(), "entity::has_camera_data: radar entity must be false");
        check(radar_ents[0].kind() == pc2::entities_kind::radar,
              "entity::kind: radar entity must return entities_kind::radar");

        // camera entity: has_radar_data false, has_camera_data true, kind == camera
        check(!camera_ents[0].has_radar_data(), "entity::has_radar_data: camera entity must be false");
        check(camera_ents[0].has_camera_data(), "entity::has_camera_data: camera entity must be true");
        check(camera_ents[0].kind() == pc2::entities_kind::camera,
              "entity::kind: camera entity must return entities_kind::camera");

        // fused entity: both true, kind == fused
        check(fused_ents[0].has_radar_data(), "entity::has_radar_data: fused entity must be true");
        check(fused_ents[0].has_camera_data(), "entity::has_camera_data: fused entity must be true");
        check(fused_ents[0].kind() == pc2::entities_kind::fused,
              "entity::kind: fused entity must return entities_kind::fused");

        // default-constructed entity: both ids are no_entity_id, kind == nullopt
        const pc2::entity default_entity{};
        check(!default_entity.has_radar_data(), "entity::has_radar_data: default entity must be false");
        check(!default_entity.has_camera_data(), "entity::has_camera_data: default entity must be false");
        check(!default_entity.kind().has_value(), "entity::kind: default entity must return nullopt");
    }

    void test_cloud_view_bounds()
    {
        const std::vector<pc2::radar_point> points{{1, 2, 3, 4, 5, 6}};
        auto cloud = pc2::make_radar_point_cloud(pc2::make_header(0, 0U, "r"), points);

        const pc2::cloud_view view{cloud};
        const auto x_field = view.field("x");
        bool threw = false;
        try
        {
            (void)view[0].get<float>(*x_field, 1);  // count is 1 => subfield 1 is out of range
        }
        catch (const std::out_of_range &)
        {
            threw = true;
        }
        check(threw, "bounds: subfield out of range did not throw");

        // A field that lies beyond point_step must be rejected at resolution time
        sensor_msgs::msg::PointCloud2 bad_cloud = cloud;
        auto bad_fields = bad_cloud.fields();
        bad_fields[0].offset(100);  // beyond point_step 24
        bad_cloud.fields(bad_fields);
        const pc2::cloud_view bad_view{bad_cloud};
        check(!bad_view.field("x").has_value(), "bounds: out-of-step field resolved as valid");

        // Truncated data buffer must be rejected at view construction
        bad_cloud = cloud;
        bad_cloud.data().resize(10);  // < width * point_step
        threw = false;
        try
        {
            const pc2::cloud_view truncated_view{bad_cloud};
            (void)truncated_view;
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        check(threw, "bounds: truncated cloud did not throw on view construction");
    }
}  // namespace

int main(int argc, const char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: point_cloud2_test <subcommand>" << std::endl;
        return 2;
    }
    const std::string subcommand{argv[1]};  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    try
    {
        if (subcommand == "basics")
        {
            test_basics();
        }
        else if (subcommand == "radar_roundtrip")
        {
            test_radar_roundtrip();
        }
        else if (subcommand == "cloud_view_basic")
        {
            test_cloud_view_basic();
        }
        else if (subcommand == "cloud_view_endianness")
        {
            test_cloud_view_endianness();
        }
        else if (subcommand == "cloud_view_bounds")
        {
            test_cloud_view_bounds();
        }
        else if (subcommand == "read_radar_points")
        {
            test_read_radar_points();
        }
        else if (subcommand == "create_cloud_tiers")
        {
            test_create_cloud_tiers();
        }
        else if (subcommand == "entities")
        {
            test_entities();
        }
        else
        {
            std::cerr << "Unknown subcommand: " << subcommand << std::endl;
            return 2;
        }
    }
    catch (const std::exception &exception)
    {
        std::cerr << "point_cloud2_test " << subcommand << " FAILED: " << exception.what() << std::endl;
        return 1;
    }
    std::cout << "point_cloud2_test " << subcommand << " PASSED" << std::endl;
    return 0;
}
