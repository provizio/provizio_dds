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

#include <cstdint>
#include <type_traits>

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
                  "match_publisher_reliability_qos sentinel (0) now collides with BEST_EFFORT/RELIABLE -- "
                  "pick an unused value for the sentinel");

    namespace detail
    {
        /**
         * @brief The KEEP_LAST depth every DataWriter gets unless its type asks for something else.
         *
         * Fast-DDS's own writer default is KEEP_LAST(1), which turns a RELIABLE writer into stop-and-wait:
         * the single history slot is overwritten by the next sample, so a sample a reader has not yet
         * acknowledged can no longer be retransmitted, and rather than lose it the writer blocks the
         * publishing thread until the acknowledgement arrives or @c max_blocking_time (Fast-DDS's inherited
         * 100 ms — deliberately left untouched here) runs out. Eight slots let the emitter keep publishing
         * across a NACK/repair round trip instead of serialising on it. A writer's history holds only that
         * one emitter's own recent samples, so the cost is eight of this type's samples on the emitting
         * device and does not scale with the number of peers.
         */
        constexpr std::int32_t default_datawriter_keep_last_history_depth = 8;

        /**
         * @brief The KEEP_LAST depth given to the DataReader of a topic several emitters share.
         *
         * KEEP_LAST depth is a PER INSTANCE budget, and a keyless topic has exactly one instance. Every
         * Provizio fleet topic is keyless and shared: all radars publish their point clouds to
         * @c rt/provizio_radar_point_cloud, all of them publish freespace polygons to
         * @c rt/provizio_freespace_poly, and a sample is attributed to its emitter only by the @c frame_id
         * in its header. A reader's depth is therefore divided across the whole fleet rather than granted
         * per emitter: at 6 radars publishing 15 Hz, a depth of 4 is four samples for all of them together,
         * about 44 ms of traffic, and anything the consuming thread does not collect within that window is
         * overwritten.
         *
         * 32 is sized as roughly two frames from each of a 16-emitter fleet, which leaves head-room above
         * the largest configurations in the field and absorbs a consumer that stalls for a frame or two.
         * The types that carry it are all KB-scale (point clouds, polygons, odometry, calibration), and one
         * history slot costs one serialized sample, so the whole reader history is single-digit MB.
         *
         * Sizing this correctly matters more than it looks, because overrunning a reader's history is
         * completely silent. The sample arrived, RTPS acknowledged it, and the reader's own history then
         * discarded it to make room: no RTPS loss is recorded, no reliability guarantee is violated, no
         * counter moves and nothing is logged. RELIABLE does not help — the loss happens after the
         * reliability contract has already been satisfied. The only symptom is a callback that never fires.
         */
        constexpr std::int32_t fleet_shared_datareader_keep_last_history_depth = 32;
    }  // namespace detail

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
         * @brief Per-type default KEEP_LAST history depth for a DataReader, applied whenever the caller leaves
         * @c max_history_depth non-positive (the @c use_default_history_depth sentinel, or any other value <= 0).
         * @c 0 means "no per-type override": Fast-DDS's own reader default, KEEP_LAST(1), stands.
         *
         * Reader and writer depth are separate members because the two endpoints do unrelated jobs, usually on
         * different machines. A writer's history is a retransmission buffer holding one emitter's own recent
         * samples, paid for on a frequently memory-constrained sensor. A reader's history is a jitter buffer
         * shared by every writer on the topic, paid for on the consuming host, and it has to absorb the
         * combined rate of the whole fleet. One constant for both forced a single value to be simultaneously
         * too deep for the sensor and far too shallow for the consumer.
         *
         * Every type published to a topic that several emitters share specializes this to
         * @c detail::fleet_shared_datareader_keep_last_history_depth, which explains the arithmetic and why
         * running out of reader history is a silent loss. Unspecialized types keep KEEP_LAST(1), which is the
         * right answer only for a topic with a single emitter whose consumer never wants a stale sample.
         *
         * Sets history depth only — durability is controlled separately, so it is NOT an RxO QoS and doesn't
         * affect matching or ROS2 interop.
         */
        static constexpr std::int32_t datareader_keep_last_history_depth = 0;

        /**
         * @brief Per-type default KEEP_LAST history depth for a DataWriter, applied whenever the caller leaves
         * @c history_depth non-positive (the @c use_default_history_depth sentinel, or any other value <= 0).
         *
         * @c detail::default_datawriter_keep_last_history_depth rather than @c 0 ("keep the Fast-DDS default"),
         * because Fast-DDS's writer default of KEEP_LAST(1) makes a RELIABLE writer stop-and-wait — see that
         * constant for why eight slots and why @c max_blocking_time is left alone. Types whose samples are
         * megabyte-scale specialize it back down, trading retransmission head-room for memory on the emitter.
         *
         * Sets history depth only — durability is controlled separately, so it is NOT an RxO QoS and doesn't
         * affect matching or ROS2 interop.
         */
        static constexpr std::int32_t datawriter_keep_last_history_depth =
            detail::default_datawriter_keep_last_history_depth;
    };

    namespace detail
    {
        /**
         * @brief Detects a @c qos_defaults specialization that still declares the pre-split
         * @c keep_last_history_depth member, which no longer has any meaning.
         *
         * @c qos_defaults is public API: a consumer may specialize it for its own type, and before the split a
         * specialization configured reader and writer history through one @c keep_last_history_depth member.
         * Nothing in the language stops that member from continuing to compile after the split — it would
         * simply never be read again, and the endpoints would quietly fall back to the primary template's
         * depths. That is precisely the class of silent misconfiguration this header exists to prevent, so
         * @c datareader_history_depth / @c datawriter_history_depth turn it into a build failure instead.
         *
         * @tparam qos_defaults_type The @c qos_defaults specialization to inspect.
         */
        template <typename qos_defaults_type, typename = void>
        struct declares_removed_keep_last_history_depth : std::false_type
        {
        };

        /**
         * @brief Partial specialization selected (by SFINAE on the member's type) when the inspected
         * @c qos_defaults specialization does declare @c keep_last_history_depth, whether directly or through
         * a base class.
         */
        template <typename qos_defaults_type>
        struct declares_removed_keep_last_history_depth<
            qos_defaults_type, std::void_t<decltype(qos_defaults_type::keep_last_history_depth)>> : std::true_type
        {
        };

        /**
         * @brief Empty type whose instantiation fails the build if @c qos_defaults_type still declares the
         * removed @c keep_last_history_depth member.
         *
         * A @c static_assert message has to be a string literal, so the reader and writer helpers below cannot
         * share a named constant; they share this type instead, which keeps the wording in one place without
         * defining a macro in a public header. Both helpers instantiate it (and discard the temporary) before
         * reading the per-type depth, so any specialization the library actually uses is checked.
         *
         * @tparam qos_defaults_type The @c qos_defaults specialization to check.
         */
        template <typename qos_defaults_type> struct assert_keep_last_history_depth_split_honoured
        {
            static_assert(!declares_removed_keep_last_history_depth<qos_defaults_type>::value,
                          "provizio::dds::qos_defaults<T> declares keep_last_history_depth, which no longer exists. It "
                          "was split into datareader_keep_last_history_depth (the reader's jitter buffer, shared by "
                          "every writer on the topic) and datawriter_keep_last_history_depth (the writer's "
                          "retransmission buffer for its own samples), which are read independently -- so a "
                          "specialization that still defines the old member has no effect at all. Rename the member in "
                          "your qos_defaults specialization to whichever of the two you meant, or define both.");
        };

        /**
         * @brief Resolves the KEEP_LAST history depth to configure on a DataReader of @c data_pub_sub_type.
         *
         * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
         * @param requested_depth The depth the caller asked for. A positive value wins; any non-positive value
         * (including the @c use_default_history_depth sentinel) selects the per-type default.
         * @return The depth to apply, or a non-positive value meaning "leave the Fast-DDS default alone".
         */
        template <typename data_pub_sub_type>
        constexpr std::int32_t datareader_history_depth(const std::int32_t requested_depth) noexcept
        {
            static_cast<void>(assert_keep_last_history_depth_split_honoured<qos_defaults<data_pub_sub_type>>{});
            return (requested_depth > 0) ? requested_depth
                                         : qos_defaults<data_pub_sub_type>::datareader_keep_last_history_depth;
        }

        /**
         * @brief Resolves the KEEP_LAST history depth to configure on a DataWriter of @c data_pub_sub_type.
         *
         * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
         * @param requested_depth The depth the caller asked for. A positive value wins; any non-positive value
         * (including the @c use_default_history_depth sentinel) selects the per-type default.
         * @return The depth to apply, or a non-positive value meaning "leave the Fast-DDS default alone".
         */
        template <typename data_pub_sub_type>
        constexpr std::int32_t datawriter_history_depth(const std::int32_t requested_depth) noexcept
        {
            static_cast<void>(assert_keep_last_history_depth_split_honoured<qos_defaults<data_pub_sub_type>>{});
            return (requested_depth > 0) ? requested_depth
                                         : qos_defaults<data_pub_sub_type>::datawriter_keep_last_history_depth;
        }
    }  // namespace detail

    /**
     * @brief Sentinel for the @c history_depth / @c max_history_depth parameters: use the default KEEP_LAST history
     * depth — the per-type @c qos_defaults::datawriter_keep_last_history_depth (publishers) or
     * @c qos_defaults::datareader_keep_last_history_depth (subscribers) if specialized, otherwise Fast-DDS's
     * default. History depth only; durability is configured separately via the @c durability_kind parameter.
     */
    constexpr std::int32_t use_default_history_depth = -1;

    /**
     * @brief Sentinel value @c 0 for request/response @c max_history_depth: select the minimal bounded request
     * queue. (For make_publisher / make_subscriber any non-positive @c history_depth simply keeps the default depth.)
     */
    constexpr std::int32_t minimal_request_queue = 0;
}  // namespace provizio::dds

// Forward-declare the PubSubTypes whose QoS defaults differ from the primary template's, so
// qos_defaults can be specialized for them without pulling the heavy generated headers into
// this core header. The names match the eProsima-generated classes; an IDL rename would make
// the specialization silently fall back to the primary template (no error), so keep this list
// in sync — src/qos_defaults_checks.cpp turns exactly that mismatch into a build failure by
// including the real generated headers and re-asserting every specialization below.
namespace sensor_msgs::msg
{
    class ImagePubSubType;
    class CompressedImagePubSubType;
    class MultiEchoLaserScanPubSubType;
    class PointCloud2PubSubType;
    class NavSatFixPubSubType;
}  // namespace sensor_msgs::msg
namespace nav_msgs::msg
{
    class OccupancyGridPubSubType;
    class OdometryPubSubType;
}  // namespace nav_msgs::msg
namespace geometry_msgs::msg
{
    class PolygonInstanceStampedPubSubType;
    class PolygonStampedPubSubType;
    class TransformStampedPubSubType;
}  // namespace geometry_msgs::msg
namespace provizio::msg
{
    class radar_infoPubSubType;
    class camera_intrinsicsPubSubType;
    class metadataPubSubType;
}  // namespace provizio::msg

namespace provizio::dds
{
    namespace detail
    {
        /**
         * @brief QoS defaults for a large sample type published to a topic that is NOT shared by a fleet of
         * emitters: ASYNCHRONOUS publish so a multi-megabyte write hands off to the participant's async sender
         * thread instead of blocking the publishing thread while it fragments and sends, plus a deliberately
         * shallow KEEP_LAST(4) history on both endpoints.
         *
         * The history stays shallow here for one reason: one history slot costs one serialized sample (measured
         * on a KEEP_LAST writer, RSS grew by 200 KB per slot for a 200 KB sample and by 500 KB per slot for a
         * 500 KB sample — linear, with no per-slot overhead worth naming), and these are the types whose samples
         * run to megabytes. Depth buys jitter tolerance at a price per slot two to three orders of magnitude
         * above what the KB-scale fleet-shared types pay, so it is bought sparingly and the caller who needs
         * more asks for it explicitly through @c max_history_depth.
         *
         * Reliability and memory policy match the primary template; history is set without touching durability,
         * so none of this is an RxO QoS — reader/writer matching and ROS2 interop are unaffected.
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
            static constexpr std::int32_t datareader_keep_last_history_depth = 4;
            // Below the primary template's eight slots: a megabyte-scale sample makes a deep retransmission
            // buffer the single most expensive thing on a sensor's heap, and four samples already span several
            // NACK/repair round trips at any realistic frame rate.
            static constexpr std::int32_t datawriter_keep_last_history_depth = 4;
        };

        /**
         * @brief QoS defaults for a KB-scale type published to a keyless topic that every sensor in the fleet
         * shares: the primary template's reliability, memory policy and writer depth, with the reader history
         * raised to @c fleet_shared_datareader_keep_last_history_depth so the depth is not divided down to a few
         * milliseconds of fleet traffic. See that constant for the arithmetic and for why an overrun is silent.
         *
         * History depth is not an RxO QoS and durability is untouched, so raising it changes nothing about
         * reader/writer matching or ROS2 interop.
         */
        struct fleet_shared_qos_defaults
        {
            static constexpr ReliabilityQosPolicyKind datawriter_reliability_kind = RELIABLE_RELIABILITY_QOS;
            static constexpr ReliabilityQosPolicyKind datareader_reliability_kind = match_publisher_reliability_qos;
            static constexpr auto memory_policy = eprosima::fastdds::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
            static constexpr PublishModeQosPolicyKind datawriter_publish_mode = SYNCHRONOUS_PUBLISH_MODE;
            static constexpr std::int32_t datareader_keep_last_history_depth =
                fleet_shared_datareader_keep_last_history_depth;
            static constexpr std::int32_t datawriter_keep_last_history_depth =
                default_datawriter_keep_last_history_depth;
        };

        /**
         * @brief QoS defaults for a fleet-shared type whose samples, while still KB-scale, are large enough to
         * fragment over UDP — point clouds and freespace polygons. Identical to @c fleet_shared_qos_defaults
         * except that the writer publishes ASYNCHRONOUSly, so fragmenting and sending a sample happens on the
         * participant's async sender thread rather than on the caller's publishing thread.
         *
         * Publish mode is writer-local and therefore, like the history depth, not an RxO QoS.
         */
        struct fleet_shared_large_sample_qos_defaults : fleet_shared_qos_defaults
        {
            static constexpr PublishModeQosPolicyKind datawriter_publish_mode = ASYNCHRONOUS_PUBLISH_MODE;
        };
    }  // namespace detail

    /**
     * @brief Raw camera frames (@c rt/provizio_camera) and raw-image freespace (@c rt/provizio_freespace).
     *
     * The only type whose writer defaults to BEST_EFFORT, and it does so because the product is migrating off
     * it. Both raw-image topics have a named successor, and both successors keep a reliable writer and a
     * history worth having: raw camera frames are superseded by @c CompressedImage, and raw-image freespace by
     * the polygonal freespace on @c rt/provizio_freespace_poly, which carries
     * @c geometry_msgs::msg::PolygonInstanceStamped — one of the fleet-shared types that gets the deep reader
     * history. History is therefore spent where the data is moving TO, while the form it is moving away from —
     * simultaneously the heaviest sample on the wire and the lowest-priority one — is the first thing dropped
     * when a link or a consumer cannot keep up. That is a deliberate product direction rather than a
     * regression: an uncompressed frame that has to be dropped should be dropped instead of blocking a
     * reliable writer's publishing thread behind a retransmission of a frame nobody is waiting for any more.
     *
     * The history stays as shallow as @c large_sample_qos_defaults makes it, for the memory reason given
     * there: at megabytes per slot, raw images are the most expensive place in the library to spend history.
     * Durability is unaffected — it is an opt-in @c durability_kind parameter that nothing here sets, so raw
     * images were, and remain, VOLATILE. The pairing is therefore volatile + best-effort by default.
     *
     * Two consequences worth knowing before relying on this:
     *  - A consumer that genuinely needs lossless raw images must ask for it: pass an explicit
     *    @c RELIABLE_RELIABILITY_QOS (and, on a busy topic, an explicit @c history_depth) to
     *    @c make_publisher. provizio_dds subscribers need no change — they default to
     *    @c match_publisher_reliability_qos and adopt whatever the writer offers.
     *  - Reliability IS an RxO policy, so a ROS 2 subscriber on the default (RELIABLE) QoS will not match a
     *    best-effort writer at all. Such a subscriber must request @c SensorDataQoS, which is best-effort.
     *    See DETAILS.md for the full interop note.
     */
    template <> struct qos_defaults<::sensor_msgs::msg::ImagePubSubType> final : detail::large_sample_qos_defaults
    {
        static constexpr ReliabilityQosPolicyKind datawriter_reliability_kind = BEST_EFFORT_RELIABILITY_QOS;
    };

    /**
     * @brief Compressed camera frames — the successor to raw @c Image, and so the one image form that keeps a
     * RELIABLE writer and a history sized to be useful rather than merely non-zero.
     *
     * The reader depth of 6 is two frames from each of at most three cameras sharing the (keyless) topic; the
     * fleet-shared depth of 32 is not affordable here because a compressed frame is one to two orders of
     * magnitude larger than a point cloud. The figures behind that: one history slot costs one serialized
     * sample, measured as 0.20 MB per slot for a 200 KB frame and 0.50 MB per slot for a 500 KB frame (RSS,
     * linear in depth across 1..32). So depth 6 costs 1.2 MB per endpoint at 200 KB frames and 3.0 MB at
     * 500 KB, against 0.8/2.0 MB for a two-camera depth of 4 and 6.4/16.0 MB if these frames took the
     * fleet-shared 32. Six buys a third camera and a frame of consumer stall for about a megabyte more than
     * four does; 32 would buy nothing this topic needs, for more than five times the memory.
     *
     * The writer keeps @c large_sample_qos_defaults' four slots: a writer's history covers only its own
     * camera's stream, so it needs no fleet multiplier, and four frames is already several NACK/repair round
     * trips at any realistic frame rate — 2.0 MB at 500 KB frames, against 4.0 MB for the primary template's
     * eight, on a device that is also holding the encoder's buffers.
     */
    template <>
    struct qos_defaults<::sensor_msgs::msg::CompressedImagePubSubType> final : detail::large_sample_qos_defaults
    {
        static constexpr std::int32_t datareader_keep_last_history_depth = 6;
    };

    template <>
    struct qos_defaults<::sensor_msgs::msg::MultiEchoLaserScanPubSubType> final : detail::large_sample_qos_defaults
    {
    };
    template <> struct qos_defaults<::nav_msgs::msg::OccupancyGridPubSubType> final : detail::large_sample_qos_defaults
    {
    };

    // Radar point clouds (rt/provizio_radar_point_cloud, its super-resolution variant) and the entity clouds
    // (rt/provizio_entities and friends): every radar in the fleet publishes to the same keyless topic, and a
    // dense cloud is large enough to fragment over UDP.
    //
    // The type here is the GENERIC sensor_msgs::msg::PointCloud2, so this default reaches clouds that are
    // nothing to do with a Provizio radar, and the reader depth it grants -- 32, from the fleet-shared tier --
    // is sized for the KB-scale samples a radar produces. A dense lidar cloud running to megabytes a sample
    // pays that depth at the large-sample tier's price rather than this one's, which is hundreds of MB per
    // subscriber instead of single-digit. The fleet's arithmetic is what this library defaults to because the
    // fleet is what it exists to carry; a consumer subscribing to something heavier caps it with an explicit
    // max_history_depth, and this note is here so the memory reads as a decision rather than a surprise.
    template <>
    struct qos_defaults<::sensor_msgs::msg::PointCloud2PubSubType> final
        : detail::fleet_shared_large_sample_qos_defaults
    {
    };
    // Freespace polygons (rt/provizio_freespace_poly, rt/provizio_freespace_camera_poly), the successor to
    // raw-image freespace. A polygon is a sequence of vertices, so a dense one runs to several KB and
    // fragments over UDP; multiple polygons per frame make one emitter's share of the topic several samples
    // wide, on top of the fleet sharing it.
    template <>
    struct qos_defaults<::geometry_msgs::msg::PolygonInstanceStampedPubSubType> final
        : detail::fleet_shared_large_sample_qos_defaults
    {
    };
    template <>
    struct qos_defaults<::geometry_msgs::msg::PolygonStampedPubSubType> final
        : detail::fleet_shared_large_sample_qos_defaults
    {
    };

    // The small fleet-shared types — odometry, transforms, radar info, camera intrinsics, GNSS fixes and the
    // open-schema per-frame metadata on rt/provizio_metadata. Each is a few hundred bytes, published to a
    // keyless topic that every sensor shares, and each one used to land on the primary template's "no override"
    // — which meant Fast-DDS's KEEP_LAST(1): a single reader slot for the entire fleet, so any two sensors
    // publishing within one callback of each other cost one of them its sample, silently. They are small enough
    // that a depth of 32 is tens of KB, so there is no reason to buy less.
    template <> struct qos_defaults<::nav_msgs::msg::OdometryPubSubType> final : detail::fleet_shared_qos_defaults
    {
    };
    template <>
    struct qos_defaults<::geometry_msgs::msg::TransformStampedPubSubType> final : detail::fleet_shared_qos_defaults
    {
    };
    template <> struct qos_defaults<::sensor_msgs::msg::NavSatFixPubSubType> final : detail::fleet_shared_qos_defaults
    {
    };
    template <> struct qos_defaults<::provizio::msg::radar_infoPubSubType> final : detail::fleet_shared_qos_defaults
    {
    };
    template <>
    struct qos_defaults<::provizio::msg::camera_intrinsicsPubSubType> final : detail::fleet_shared_qos_defaults
    {
    };
    template <> struct qos_defaults<::provizio::msg::metadataPubSubType> final : detail::fleet_shared_qos_defaults
    {
    };
}  // namespace provizio::dds

#endif  // DDS_QOS_DEFAULTS
