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

// Compile-time verification that the large-sample QoS specializations in qos_defaults.h
// resolve for the real generated types. The specializations key on forward-declared
// class names, so a rename or namespace change in the generated code would silently
// select the primary template (synchronous publish, default history) with no build
// error. Including the real generated headers here turns any such mismatch into a
// build failure. This translation unit intentionally produces no code.

#include <geometry_msgs/msg/PolygonInstanceStampedPubSubTypes.hpp>
#include <geometry_msgs/msg/PolygonStampedPubSubTypes.hpp>
#include <nav_msgs/msg/OccupancyGridPubSubTypes.hpp>
#include <nav_msgs/msg/OdometryPubSubTypes.hpp>
#include <sensor_msgs/msg/CompressedImagePubSubTypes.hpp>
#include <sensor_msgs/msg/ImagePubSubTypes.hpp>
#include <sensor_msgs/msg/MultiEchoLaserScanPubSubTypes.hpp>
#include <sensor_msgs/msg/PointCloud2PubSubTypes.hpp>

#include "provizio/dds/qos_defaults.h"

namespace provizio::dds
{
    namespace
    {
        template <typename pub_sub_type> constexpr bool resolves_to_large_sample_defaults()
        {
            return qos_defaults<pub_sub_type>::datawriter_publish_mode == ASYNCHRONOUS_PUBLISH_MODE &&
                   qos_defaults<pub_sub_type>::keep_last_history_depth ==
                       detail::large_sample_qos_defaults::keep_last_history_depth &&
                   qos_defaults<pub_sub_type>::datawriter_reliability_kind == RELIABLE_RELIABILITY_QOS;
        }
    }  // namespace

    static_assert(resolves_to_large_sample_defaults<sensor_msgs::msg::ImagePubSubType>(),
                  "Image must resolve to the large-sample QoS defaults");
    static_assert(resolves_to_large_sample_defaults<sensor_msgs::msg::CompressedImagePubSubType>(),
                  "CompressedImage must resolve to the large-sample QoS defaults");
    static_assert(resolves_to_large_sample_defaults<sensor_msgs::msg::MultiEchoLaserScanPubSubType>(),
                  "MultiEchoLaserScan must resolve to the large-sample QoS defaults");
    static_assert(resolves_to_large_sample_defaults<sensor_msgs::msg::PointCloud2PubSubType>(),
                  "PointCloud2 must resolve to the large-sample QoS defaults");
    static_assert(resolves_to_large_sample_defaults<nav_msgs::msg::OccupancyGridPubSubType>(),
                  "OccupancyGrid must resolve to the large-sample QoS defaults");
    static_assert(resolves_to_large_sample_defaults<geometry_msgs::msg::PolygonStampedPubSubType>(),
                  "PolygonStamped must resolve to the large-sample QoS defaults");
    static_assert(resolves_to_large_sample_defaults<geometry_msgs::msg::PolygonInstanceStampedPubSubType>(),
                  "PolygonInstanceStamped must resolve to the large-sample QoS defaults");

    // And the primary template must still apply where it should: odometry is a small
    // high-rate sample and keeps synchronous publish with the Fast-DDS default history.
    static_assert(qos_defaults<nav_msgs::msg::OdometryPubSubType>::datawriter_publish_mode ==
                          SYNCHRONOUS_PUBLISH_MODE &&
                      qos_defaults<nav_msgs::msg::OdometryPubSubType>::keep_last_history_depth == 0,
                  "Odometry must keep the primary-template QoS defaults");
}  // namespace provizio::dds
