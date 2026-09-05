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

// Compile-time verification that the QoS specializations in qos_defaults.h resolve for the
// real generated types, and that each one still carries the values it was written to carry.
// The specializations key on forward-declared class names, so a rename or namespace change
// in the generated code would silently select the primary template (synchronous publish,
// the primary depths) with no build error. Including the real generated headers here turns
// any such mismatch into a build failure. Restating the values catches the other direction:
// a depth or a reliability edited in one tier without the intent being reconsidered for the
// types that inherit it. This translation unit intentionally produces no code.

#include <geometry_msgs/msg/PolygonInstanceStampedPubSubTypes.hpp>
#include <geometry_msgs/msg/PolygonStampedPubSubTypes.hpp>
#include <geometry_msgs/msg/TransformStampedPubSubTypes.hpp>
#include <nav_msgs/msg/OccupancyGridPubSubTypes.hpp>
#include <nav_msgs/msg/OdometryPubSubTypes.hpp>
#include <provizio/msg/camera_intrinsicsPubSubTypes.hpp>
#include <provizio/msg/metadataPubSubTypes.hpp>
#include <provizio/msg/radar_infoPubSubTypes.hpp>
#include <sensor_msgs/msg/CompressedImagePubSubTypes.hpp>
#include <sensor_msgs/msg/ImagePubSubTypes.hpp>
#include <sensor_msgs/msg/MultiEchoLaserScanPubSubTypes.hpp>
#include <sensor_msgs/msg/NavSatFixPubSubTypes.hpp>
#include <sensor_msgs/msg/PointCloud2PubSubTypes.hpp>
#include <std_msgs/msg/StringPubSubTypes.hpp>

#include "provizio/dds/qos_defaults.h"

namespace provizio::dds
{
    namespace
    {
        // One predicate per QoS default that a specialization can change, each taking the value the
        // type is expected to have. Spelling the expectation out at every call site (rather than
        // comparing against the tier struct the specialization inherits from) is the point: a
        // comparison against the tier would agree with itself no matter what the tier was edited to.
        template <typename pub_sub_type> constexpr bool publishes(const PublishModeQosPolicyKind expected_publish_mode)
        {
            return qos_defaults<pub_sub_type>::datawriter_publish_mode == expected_publish_mode;
        }

        template <typename pub_sub_type>
        constexpr bool writes_with_reliability(const ReliabilityQosPolicyKind expected_reliability_kind)
        {
            return qos_defaults<pub_sub_type>::datawriter_reliability_kind == expected_reliability_kind;
        }

        template <typename pub_sub_type>
        constexpr bool keeps_history(const std::int32_t reader, const std::int32_t writer)
        {
            return qos_defaults<pub_sub_type>::datareader_keep_last_history_depth == reader &&
                   qos_defaults<pub_sub_type>::datawriter_keep_last_history_depth == writer;
        }

        // Every default subscriber adopts its publisher's reliability (deferred DataReader creation).
        // No specialization is meant to opt out of that, including the best-effort Image writer: a
        // provizio_dds subscriber follows whatever the writer offers, so only the writer side of a
        // reliability decision needs stating.
        template <typename pub_sub_type> constexpr bool reads_with_matched_reliability()
        {
            return qos_defaults<pub_sub_type>::datareader_reliability_kind == match_publisher_reliability_qos;
        }

        // True when none of the listed types' qos_defaults declares the removed keep_last_history_depth
        // member. A C++17 fold rather than a chain of && so the check reads as one expression per
        // call site however many types it is given.
        template <typename... pub_sub_types> constexpr bool none_declares_removed_history_depth()
        {
            return (... && !detail::declares_removed_keep_last_history_depth<qos_defaults<pub_sub_types>>::value);
        }

        // The depths each tier is expected to carry, spelled out here instead of being read back
        // from the detail:: constants the tiers are built from. Reading them back would make every
        // assertion below agree with itself no matter what those constants were edited to, and a
        // history depth is exactly the kind of value that deserves a second, deliberate statement:
        // getting it wrong costs samples silently, with no counter and no log to notice it by.
        constexpr std::int32_t expected_fleet_shared_reader_depth = 32;
        constexpr std::int32_t expected_writer_depth = 8;
        constexpr std::int32_t expected_large_sample_depth = 4;
        constexpr std::int32_t expected_compressed_image_reader_depth = 6;
        // 0 is "no per-type override": whatever Fast-DDS's own reader default happens to be.
        constexpr std::int32_t expected_no_reader_override = 0;
    }  // namespace

    // The fleet-shared KB-scale types: every sensor publishes to one keyless topic, so the reader
    // history has to cover the whole fleet rather than one emitter (detail::
    // fleet_shared_datareader_keep_last_history_depth explains the arithmetic). Point clouds and
    // polygons additionally publish asynchronously because they fragment over UDP.
    static_assert(publishes<sensor_msgs::msg::PointCloud2PubSubType>(ASYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<sensor_msgs::msg::PointCloud2PubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<sensor_msgs::msg::PointCloud2PubSubType>() &&
                      keeps_history<sensor_msgs::msg::PointCloud2PubSubType>(expected_fleet_shared_reader_depth,
                                                                             expected_writer_depth),
                  "PointCloud2 must resolve to the asynchronous fleet-shared QoS defaults");
    static_assert(
        publishes<geometry_msgs::msg::PolygonInstanceStampedPubSubType>(ASYNCHRONOUS_PUBLISH_MODE) &&
            writes_with_reliability<geometry_msgs::msg::PolygonInstanceStampedPubSubType>(RELIABLE_RELIABILITY_QOS) &&
            reads_with_matched_reliability<geometry_msgs::msg::PolygonInstanceStampedPubSubType>() &&
            keeps_history<geometry_msgs::msg::PolygonInstanceStampedPubSubType>(expected_fleet_shared_reader_depth,
                                                                                expected_writer_depth),
        "PolygonInstanceStamped must resolve to the asynchronous fleet-shared QoS defaults");
    static_assert(publishes<geometry_msgs::msg::PolygonStampedPubSubType>(ASYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<geometry_msgs::msg::PolygonStampedPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<geometry_msgs::msg::PolygonStampedPubSubType>() &&
                      keeps_history<geometry_msgs::msg::PolygonStampedPubSubType>(expected_fleet_shared_reader_depth,
                                                                                  expected_writer_depth),
                  "PolygonStamped must resolve to the asynchronous fleet-shared QoS defaults");

    // The small fleet-shared types. Same reader depth, synchronous publish: a few hundred bytes
    // never fragments, so handing the write to the async sender thread would only add latency.
    static_assert(publishes<nav_msgs::msg::OdometryPubSubType>(SYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<nav_msgs::msg::OdometryPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<nav_msgs::msg::OdometryPubSubType>() &&
                      keeps_history<nav_msgs::msg::OdometryPubSubType>(expected_fleet_shared_reader_depth,
                                                                       expected_writer_depth),
                  "Odometry must resolve to the synchronous fleet-shared QoS defaults");
    static_assert(
        publishes<geometry_msgs::msg::TransformStampedPubSubType>(SYNCHRONOUS_PUBLISH_MODE) &&
            writes_with_reliability<geometry_msgs::msg::TransformStampedPubSubType>(RELIABLE_RELIABILITY_QOS) &&
            reads_with_matched_reliability<geometry_msgs::msg::TransformStampedPubSubType>() &&
            keeps_history<geometry_msgs::msg::TransformStampedPubSubType>(expected_fleet_shared_reader_depth,
                                                                          expected_writer_depth),
        "TransformStamped must resolve to the synchronous fleet-shared QoS defaults");
    static_assert(publishes<sensor_msgs::msg::NavSatFixPubSubType>(SYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<sensor_msgs::msg::NavSatFixPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<sensor_msgs::msg::NavSatFixPubSubType>() &&
                      keeps_history<sensor_msgs::msg::NavSatFixPubSubType>(expected_fleet_shared_reader_depth,
                                                                           expected_writer_depth),
                  "NavSatFix must resolve to the synchronous fleet-shared QoS defaults");
    static_assert(publishes<provizio::msg::radar_infoPubSubType>(SYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<provizio::msg::radar_infoPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<provizio::msg::radar_infoPubSubType>() &&
                      keeps_history<provizio::msg::radar_infoPubSubType>(expected_fleet_shared_reader_depth,
                                                                         expected_writer_depth),
                  "radar_info must resolve to the synchronous fleet-shared QoS defaults");
    static_assert(publishes<provizio::msg::camera_intrinsicsPubSubType>(SYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<provizio::msg::camera_intrinsicsPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<provizio::msg::camera_intrinsicsPubSubType>() &&
                      keeps_history<provizio::msg::camera_intrinsicsPubSubType>(expected_fleet_shared_reader_depth,
                                                                                expected_writer_depth),
                  "camera_intrinsics must resolve to the synchronous fleet-shared QoS defaults");
    static_assert(publishes<provizio::msg::metadataPubSubType>(SYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<provizio::msg::metadataPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<provizio::msg::metadataPubSubType>() &&
                      keeps_history<provizio::msg::metadataPubSubType>(expected_fleet_shared_reader_depth,
                                                                       expected_writer_depth),
                  "metadata must resolve to the synchronous fleet-shared QoS defaults");

    // Large samples that no Provizio fleet topic shares: asynchronous publish, and a history kept
    // shallow because at these sizes a slot costs a megabyte-scale sample.
    static_assert(
        publishes<sensor_msgs::msg::MultiEchoLaserScanPubSubType>(ASYNCHRONOUS_PUBLISH_MODE) &&
            writes_with_reliability<sensor_msgs::msg::MultiEchoLaserScanPubSubType>(RELIABLE_RELIABILITY_QOS) &&
            reads_with_matched_reliability<sensor_msgs::msg::MultiEchoLaserScanPubSubType>() &&
            keeps_history<sensor_msgs::msg::MultiEchoLaserScanPubSubType>(expected_large_sample_depth,
                                                                          expected_large_sample_depth),
        "MultiEchoLaserScan must resolve to the large-sample QoS defaults");
    static_assert(publishes<nav_msgs::msg::OccupancyGridPubSubType>(ASYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<nav_msgs::msg::OccupancyGridPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<nav_msgs::msg::OccupancyGridPubSubType>() &&
                      keeps_history<nav_msgs::msg::OccupancyGridPubSubType>(expected_large_sample_depth,
                                                                            expected_large_sample_depth),
                  "OccupancyGrid must resolve to the large-sample QoS defaults");

    // Raw images are the one type whose writer is best-effort by default: the product is migrating
    // to compressed frames and to polygonal freespace, so a raw frame is both the heaviest sample
    // on the wire and the one it is right to drop first. Deliberately a breaking interop change --
    // a ROS 2 subscriber on default (RELIABLE) QoS will not match this writer without SensorDataQoS.
    static_assert(publishes<sensor_msgs::msg::ImagePubSubType>(ASYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<sensor_msgs::msg::ImagePubSubType>(BEST_EFFORT_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<sensor_msgs::msg::ImagePubSubType>() &&
                      keeps_history<sensor_msgs::msg::ImagePubSubType>(expected_large_sample_depth,
                                                                       expected_large_sample_depth),
                  "Image must resolve to best-effort large-sample QoS defaults with a shallow history");
    // Compressed frames stay reliable and keep a reader history sized for up to three cameras
    // sharing the keyless topic (two frames each), which the specialization's comment prices out.
    static_assert(publishes<sensor_msgs::msg::CompressedImagePubSubType>(ASYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<sensor_msgs::msg::CompressedImagePubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<sensor_msgs::msg::CompressedImagePubSubType>() &&
                      keeps_history<sensor_msgs::msg::CompressedImagePubSubType>(expected_compressed_image_reader_depth,
                                                                                 expected_large_sample_depth),
                  "CompressedImage must resolve to reliable large-sample QoS defaults with a 3-camera reader history");

    // And the primary template must still apply where it should. std_msgs::String is deliberately
    // chosen: nothing in this library or the IDLs treats it specially, so it is the type that shows
    // what an unspecialized one gets -- Fast-DDS's own reader history (the 0 "no override") and the
    // writer depth that keeps a reliable writer off stop-and-wait.
    static_assert(publishes<std_msgs::msg::StringPubSubType>(SYNCHRONOUS_PUBLISH_MODE) &&
                      writes_with_reliability<std_msgs::msg::StringPubSubType>(RELIABLE_RELIABILITY_QOS) &&
                      reads_with_matched_reliability<std_msgs::msg::StringPubSubType>() &&
                      keeps_history<std_msgs::msg::StringPubSubType>(expected_no_reader_override,
                                                                     expected_writer_depth),
                  "an unspecialized type must keep the primary-template QoS defaults");

    namespace
    {
        // Positive control for the detection trait below. Without it a trait that had silently
        // stopped detecting anything (a typo in the member name, a change to the detection idiom)
        // would leave every assertion here passing while catching nothing at all.
        struct stale_specialization_shape
        {
            static constexpr std::int32_t keep_last_history_depth = expected_large_sample_depth;
        };
    }  // namespace

    static_assert(detail::declares_removed_keep_last_history_depth<stale_specialization_shape>::value,
                  "the pre-split keep_last_history_depth detection no longer detects the member it exists to "
                  "reject -- a consumer's stale qos_defaults specialization would now be ignored silently");

    // And none of this library's own specializations may declare the removed member. They are the
    // examples a consumer copies from, and one of them carrying it would fail every build that used
    // the type -- caught here, on the library's own build, rather than in a customer's.
    static_assert(
        none_declares_removed_history_depth<
            std_msgs::msg::StringPubSubType, sensor_msgs::msg::ImagePubSubType,
            sensor_msgs::msg::CompressedImagePubSubType, sensor_msgs::msg::MultiEchoLaserScanPubSubType,
            sensor_msgs::msg::PointCloud2PubSubType, sensor_msgs::msg::NavSatFixPubSubType,
            nav_msgs::msg::OccupancyGridPubSubType, nav_msgs::msg::OdometryPubSubType,
            geometry_msgs::msg::PolygonInstanceStampedPubSubType, geometry_msgs::msg::PolygonStampedPubSubType,
            geometry_msgs::msg::TransformStampedPubSubType, provizio::msg::radar_infoPubSubType,
            provizio::msg::camera_intrinsicsPubSubType, provizio::msg::metadataPubSubType>(),
        "a provizio_dds qos_defaults specialization still declares the removed keep_last_history_depth");
}  // namespace provizio::dds
