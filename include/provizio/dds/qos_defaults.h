// Copyright 2023 Provizio Ltd.
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

#ifndef DDS_QOS_DEFAULTS
#define DDS_QOS_DEFAULTS

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/rtps/attributes/ResourceManagement.hpp>

#include "provizio/dds/common.h"

namespace provizio::dds
{
    /**
     * @brief Sentinel @c ReliabilityQosPolicyKind value meaning "match the discovered publisher's reliability".
     *
     * The default @c datareader_reliability_kind for every subscriber. A subscriber created with this value
     * does NOT create its internal DataReader eagerly; instead it defers creation until the first matching
     * remote DataWriter is discovered on its topic, then builds the reader with that writer's offered
     * reliability (reliability only — durability is configured independently). Customers who pass an explicit
     * @c BEST_EFFORT_RELIABILITY_QOS / @c RELIABLE_RELIABILITY_QOS override this and get an eagerly-created
     * reader with the requested reliability (the historical behaviour). For heterogeneous publishers the first
     * discovered writer fixes the reader's reliability (match-first).
     *
     * The numeric value @c 0 is distinct from the only reliability kinds this code ever compares the sentinel
     * against — @c BEST_EFFORT_RELIABILITY_QOS is @c 0x01 and @c RELIABLE_RELIABILITY_QOS is @c 0x02 — so it is
     * safe to overload @c 0 as a sentinel. Mirrors the @c use_default_history_depth (-1) sentinel idiom.
     */
    constexpr eprosima::fastdds::dds::ReliabilityQosPolicyKind match_publisher_reliability_qos =
        static_cast<eprosima::fastdds::dds::ReliabilityQosPolicyKind>(0);

    // The sentinel (@c 0) only needs to stay distinct from the reliability kinds actually compared against it —
    // @c BEST_EFFORT_RELIABILITY_QOS and @c RELIABLE_RELIABILITY_QOS. The enum cannot be reflected over to prove
    // @c 0 is unused by every (possibly future) kind, but the realistic risk is a Fast-DDS upgrade renumbering
    // one of those two standard kinds to @c 0 — which this assert catches, failing the build loudly rather than
    // letting every default (match-publisher) subscriber silently misbehave.
    static_assert(match_publisher_reliability_qos != BEST_EFFORT_RELIABILITY_QOS &&
                      match_publisher_reliability_qos != RELIABLE_RELIABILITY_QOS,
                  "match_publisher_reliability_qos sentinel (0) now collides with BEST_EFFORT/RELIABLE — "
                  "pick an unused value for the sentinel");

    /**
     * @file qos_defaults.h
     * @brief Defines default QOS policies for a DDS data type. They can be overriden in template specializations
     * for specific types.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/policy.html
     */
    template <typename data_pub_sub_type> struct qos_defaults final
    {
        /**
         * @brief Defines whether to use reliable data writer DDS QOS policies. RELIABLE_RELIABILITY_QOS by default
         * in Fast-DDS.
         *
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        static constexpr ReliabilityQosPolicyKind datawriter_reliability_kind = RELIABLE_RELIABILITY_QOS;

        /**
         * @brief Default data reader reliability. @c match_publisher_reliability_qos: the subscriber adopts the
         * discovered publisher's reliability by deferring DataReader creation until the first matching writer is
         * seen (see @c match_publisher_reliability_qos). Override with an explicit @c BEST_EFFORT_RELIABILITY_QOS
         * / @c RELIABLE_RELIABILITY_QOS for an eagerly-created reader of fixed reliability (Fast-DDS's own
         * DataReader default is @c BEST_EFFORT_RELIABILITY_QOS).
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        static constexpr ReliabilityQosPolicyKind datareader_reliability_kind = match_publisher_reliability_qos;

        /**
         * @brief Defines the default memory policy for both data reader and data writer.
         * PREALLOCATED_WITH_REALLOC_MEMORY_MODE in Fast-DDS 2.9+.
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/rtps/resources/MemoryManagementPolicy.html
         */
        static constexpr auto memory_policy = eprosima::fastdds::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

        /**
         * @brief DataWriter publish mode. SYNCHRONOUS_PUBLISH_MODE by default. Large sample
         * types (images, point clouds) specialize this to ASYNCHRONOUS so a multi-MB write
         * hands off to the participant's async sender thread instead of blocking the
         * publishing thread while it fragments and sends. Writer-local — NOT an RxO QoS, so
         * it never affects reader/writer matching or ROS2 interop.
         */
        static constexpr PublishModeQosPolicyKind datawriter_publish_mode = SYNCHRONOUS_PUBLISH_MODE;

        /**
         * @brief Per-type default KEEP_LAST history depth, applied to writer and reader
         * whenever the caller leaves @c history_depth / @c max_history_depth non-positive
         * (the @c use_default_history_depth sentinel, or any other value <= 0). This
         * default value being @c 0 means "no per-type override" (keep the Fast-DDS default).
         * Large sample types specialize this to a small depth (e.g. 4) so a momentarily slow
         * consumer doesn't drop frames. Sets history depth only — durability is controlled
         * separately, so it is NOT an RxO QoS and doesn't affect matching or ROS2 interop.
         */
        static constexpr std::int32_t keep_last_history_depth = 0;
    };

    /**
     * @brief Sentinel for the @c history_depth / @c max_history_depth parameters: use the default KEEP_LAST history
     * depth — the per-type @c qos_defaults::keep_last_history_depth if specialized, otherwise Fast-DDS's default.
     * History depth only; durability is configured separately via the @c durability_kind parameter.
     */
    constexpr std::int32_t use_default_history_depth = -1;

    /**
     * @brief Sentinel value @c 0 for request/response @c max_history_depth: select the minimal bounded request
     * queue. (For make_publisher / make_subscriber any non-positive @c history_depth simply keeps the default depth.)
     */
    constexpr std::int32_t minimal_request_queue = 0;
}  // namespace provizio::dds

// Forward-declare the large message PubSubTypes so qos_defaults can be specialized for
// them without pulling the heavy generated headers into this core header. The names match
// the eProsima-generated classes; an IDL rename would make the specialization silently
// fall back to the primary template (no error), so keep this list in sync.
namespace sensor_msgs::msg
{
    class ImagePubSubType;
    class CompressedImagePubSubType;
    class MultiEchoLaserScanPubSubType;
    class PointCloud2PubSubType;
}  // namespace sensor_msgs::msg
namespace nav_msgs::msg
{
    class OccupancyGridPubSubType;
}  // namespace nav_msgs::msg

namespace provizio::dds
{
    namespace detail
    {
        /**
         * @brief Shared QoS defaults for large sample types (images, point clouds):
         * ASYNCHRONOUS publish on writers + a modest KEEP_LAST(4) history so a momentarily
         * slow consumer doesn't drop big frames. Reliability and memory policy match the
         * primary template; history is set without touching durability, so none of this is
         * an RxO QoS — reader/writer matching and ROS2 interop are unaffected.
         */
        struct large_sample_qos_defaults
        {
            static constexpr ReliabilityQosPolicyKind datawriter_reliability_kind = RELIABLE_RELIABILITY_QOS;
            // Match the discovered publisher's reliability by default (deferred reader creation), same as the
            // primary template. Large sample types are typically published RELIABLE, so this makes the default
            // subscriber lossless without the caller having to opt in.
            static constexpr ReliabilityQosPolicyKind datareader_reliability_kind = match_publisher_reliability_qos;
            static constexpr auto memory_policy = eprosima::fastdds::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
            static constexpr PublishModeQosPolicyKind datawriter_publish_mode = ASYNCHRONOUS_PUBLISH_MODE;
            static constexpr std::int32_t keep_last_history_depth = 4;
        };
    }  // namespace detail

    template <> struct qos_defaults<::sensor_msgs::msg::ImagePubSubType> final : detail::large_sample_qos_defaults
    {
    };
    template <>
    struct qos_defaults<::sensor_msgs::msg::CompressedImagePubSubType> final : detail::large_sample_qos_defaults
    {
    };
    template <>
    struct qos_defaults<::sensor_msgs::msg::MultiEchoLaserScanPubSubType> final : detail::large_sample_qos_defaults
    {
    };
    template <> struct qos_defaults<::sensor_msgs::msg::PointCloud2PubSubType> final : detail::large_sample_qos_defaults
    {
    };
    template <> struct qos_defaults<::nav_msgs::msg::OccupancyGridPubSubType> final : detail::large_sample_qos_defaults
    {
    };
}  // namespace provizio::dds

#endif  // DDS_QOS_DEFAULTS
