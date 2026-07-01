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

#ifndef DDS_PUBLISHER
#define DDS_PUBLISHER

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <fastdds/dds/core/status/PublicationMatchedStatus.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/detail/listener_drain.h"
#include "provizio/dds/detail/resettable_endpoint.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/function_traits.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/qos_defaults.h"

namespace provizio::dds
{
    using WriteParams = ::eprosima::fastdds::rtps::WriteParams;

    namespace detail
    {
        template <typename data_pub_sub_type, typename on_matched_function_type = void *> class data_writer_listener;
    }  // namespace detail

    /**
     * @file publisher.h
     * @brief RAII publisher wrappers and helpers for sending DDS data.
     */

    /**
     * @brief Abstract interface that provides publishing functionality for a DDS data type. Normally created using
     * provizio::dds::make_publisher.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     *
     * @see provizio::dds::make_publisher
     * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/publisher.html
     *
     * In addition to publish APIs, the interface provides a helper to wait until the writer is matched and stable
     * for a short "settle" period (useful before the first write to avoid races right after discovery):
     * get_num_matched_subscribers().
     */
    template <typename data_pub_sub_type> class data_publisher
    {
      public:
        using data_type = typename data_pub_sub_type::type;

        /**
         * @brief Destroys the data publisher object
         */
        virtual ~data_publisher() = default;

        /**
         * @brief Publishes the DDS data
         *
         * @param data Actual DDS data to be published, f.e. std_msgs::msg::String
         * @return true if published successfully, false otherwise
         */
        virtual bool publish(data_type &data) = 0;

        /**
         * @brief Publishes the DDS data with specific WriteParams.
         * @param data Actual DDS data to be published, f.e. std_msgs::msg::String
         * @param params Write parameters, used for advanced features like request-response correlation.
         * @return true if published successfully, false otherwise
         */
        virtual bool publish(data_type &data, WriteParams &params) = 0;

        /**
         * @brief Returns GUID of the underlying DataWriter.
         */
        virtual guid get_guid() const = 0;

        /**
         * @brief Blocks until this publisher has at least one stable match for a short settle window.
         * @note This is a blocking call; invoke in a background thread if you must not block the caller thread.
         * @param timeout Total timeout duration. The implementation always waits at least ~50ms per attempt, so
         * timeouts smaller than that (including 0) still incur a short minimum wait.
         * @param settle_time The minimum time the match must remain stable (no further status changes) to be
         * considered "ready".
         * @return non-negative number of matched subscribers if stable within timeout, -1 otherwise.
         */
        virtual int get_num_matched_subscribers(std::chrono::milliseconds timeout,
                                                std::chrono::milliseconds settle_time) const = 0;
    };

    /**
     * @brief Encapsulates DDS Publisher and DataWriter functionality in a single entity with automatic life cycle
     * management. Optionally can be provided with a function or function object to be invoked on matching first /
     * unmatching last subscriber. Normally created using provizio::dds::make_publisher.
     *
     * @note When the parent participant has network auto-recovery enabled, the
     * underlying Fast-DDS Publisher / DataWriter objects may be swapped under this
     * handle as part of a participant reset (see network_recovery.h). The caller-held
     * shared_ptr to the handle and the user-supplied on_matched callback both survive.
     *
     * @note The on_matched callback is invoked from a Fast-DDS internal thread.
     * Calling other provizio APIs that don't create new endpoints
     * (@c publish on a sibling publisher, @c get_guid, etc.) is safe. However,
     * the callback MUST NOT block indefinitely (resets wait for in-flight
     * callbacks to drain) and MUST NOT create new endpoints
     * (@c make_publisher / @c make_subscriber / @c register_topic /
     * @c register_type) — those acquire @c domain_participant::registration_mutex
     * which a concurrent reset is holding while waiting for this very callback
     * to return, producing an AB-BA deadlock.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_matched_function_type Optionally a function / function object type to be invoked on
     * matching first / unmatching last subscriber. Takes two arguments: a reference to this publisher_handle and a
     * bool: true when the first subscriber is matched, false when the last subscriber is unmatched. Alternatively,
     * it can accept a third argument of `provizio::dds::guid` type, which will be the GUID of the (un)matched
     * subscriber, then it gets invoked on every match/unmatch.
     * @see provizio::dds::make_publisher
     * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/publisher.html
     * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/dataWriter/dataWriter.html
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/dataWriterListener/dataWriterListener.html#dds-layer-publisher-datawriterlistener
     */
    template <typename data_pub_sub_type, typename on_matched_function_type = void *>
    class publisher_handle final : public data_publisher<data_pub_sub_type>, public detail::resettable_endpoint
    {
      public:
        using data_type = typename data_publisher<data_pub_sub_type>::data_type;

        ~publisher_handle() override;

        /**
         * @copydoc provizio::dds::data_publisher::publish(data_type &)
         */
        bool publish(data_type &data) override;

        /**
         * @copydoc provizio::dds::data_publisher::publish(data_type &, WriteParams &)
         */
        bool publish(data_type &data, WriteParams &params) override;

        /**
         * @copydoc provizio::dds::data_publisher::get_guid()
         */
        guid get_guid() const override;

        /**
         * @copydoc provizio::dds::data_publisher::get_num_matched_subscribers(std::chrono::milliseconds,
         *                                                                     std::chrono::milliseconds) const
         */
        int get_num_matched_subscribers(std::chrono::milliseconds timeout,
                                        std::chrono::milliseconds settle_time) const override;

      private:
        // Construction is intentionally private: a publisher_handle must be
        // owned by a shared_ptr (the network-recovery registry holds one) and
        // must be registered with its parent participant so network-recovery
        // resets can rebuild it. Both invariants are upheld by make_publisher,
        // which is the only friend allowed to instantiate the class.

        /**
         * @brief Constructs a new publisher_handle object.
         *
         * The constructor only captures the parameters needed to build (and later
         * rebuild) the underlying Fast-DDS objects; the initial Fast-DDS
         * Publisher / DataWriter are created by @c domain_participant::register_endpoint
         * under the lifecycle lock, atomically with adding the handle to the
         * recovery registry. This eliminates the race window where a concurrent
         * reset could otherwise destroy the participant a freshly-constructed
         * handle is about to publish on.
         *
         * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
         * @param topic_name A DDS Topic Name
         * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS
         * DataWriter, which makes publishing slower but more reliable
         * @param history_depth KEEP_LAST history depth. use_default_history_depth (-1) uses the per-type qos_defaults
         * depth (else Fast-DDS's default); a positive value sets KEEP_LAST of that depth (any non-positive value,
         * including 0, uses the default). Durability is configured separately via durability_kind.
         * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS for late-joiner
         * support); std::nullopt keeps the Fast-DDS/XML default.
         * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
         * @see provizio::dds::make_publisher
         * @see provizio::dds::make_domain_participant
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        publisher_handle(
            std::shared_ptr<domain_participant> participant, const std::string &topic_name,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind,
            std::int32_t history_depth = use_default_history_depth,
            std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt);

        /**
         * @brief Constructs a new publisher_handle with an on_matched_function invoked on subscriber match changes.
         *
         * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
         * @param topic_name A DDS Topic Name
         * @param on_matched_function Function / function object invoked on subscriber match changes. With two
         * arguments @c (publisher_handle&, bool) it fires only on first match (true) and last unmatch (false). With
         * three arguments @c (publisher_handle&, bool, const guid&) it fires on every match/unmatch with the
         * (un)matched subscriber's GUID; the bool indicates whether the change is a match (true) or unmatch (false).
         * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS
         * DataWriter, which makes publishing slower but more reliable
         * @param history_depth KEEP_LAST history depth. use_default_history_depth (-1) uses the per-type qos_defaults
         * depth (else Fast-DDS's default); a positive value sets KEEP_LAST of that depth (any non-positive value,
         * including 0, uses the default). Durability is configured separately via durability_kind.
         * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS for late-joiner
         * support); std::nullopt keeps the Fast-DDS/XML default.
         * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
         * @see provizio::dds::make_publisher
         * @see provizio::dds::make_domain_participant
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        publisher_handle(
            std::shared_ptr<domain_participant> participant, const std::string &topic_name,
            on_matched_function_type on_matched_function,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind,
            std::int32_t history_depth = use_default_history_depth,
            std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt);

        publisher_handle(std::shared_ptr<domain_participant> participant, const std::string &topic_name,
                         on_matched_function_type on_matched_function, std::unique_ptr<DataWriterListener> &&listener,
                         ReliabilityQosPolicyKind reliability_kind, std::int32_t history_depth,
                         std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt);

        /// @internal Called by domain_participant during network-recovery reset.
        void detach_for_reset() noexcept override;
        void on_participant_reset(eprosima::fastdds::dds::DomainParticipant &old_participant) noexcept override;
        void on_new_participant_started(eprosima::fastdds::dds::DomainParticipant &new_participant) override;

        /// @brief (Re)build the Fast-DDS Publisher + DataWriter against @c on_participant.
        /// Caller is either domain_participant::register_endpoint (holds shared
        /// lifecycle lock) or the reset path (holds it exclusively). Records the
        /// participant's generation so a later teardown can detect a stale
        /// reference if the participant has since been replaced without us.
        void build_state(eprosima::fastdds::dds::DomainParticipant &on_participant);
        /// @brief Tear down the current Publisher + DataWriter on @c on_participant
        /// without touching the preserved listener / on-matched function. If our
        /// generation marker doesn't match the participant's current generation,
        /// the Fast-DDS objects we used to point at have already been freed by a
        /// concurrent reset that excluded us — we just clear the local pointers.
        void teardown_state(eprosima::fastdds::dds::DomainParticipant &on_participant) noexcept;

        int num_matched_subscribers{0};
        mutable std::mutex num_matched_subscribers_mutex;
        mutable std::condition_variable num_matched_subscribers_cv;
        std::shared_ptr<domain_participant> participant;
        dds::TypeSupport type_support;
        on_matched_function_type on_matched_function;
        std::unique_ptr<DataWriterListener> listener;
        detail::listener_drain match_drain;

        // State that gets swapped on reset. Guarded by the participant's lifecycle
        // mutex (shared for publish, unique for reset).
        std::shared_ptr<topic> the_topic;
        Publisher *publisher = nullptr;
        DataWriter *data_writer = nullptr;
        // 0 = never built (e.g. construction failed before register_endpoint did its
        // initial build). Otherwise: domain_participant::participant_generation()
        // observed at the moment build_state ran.
        std::uint64_t built_against_generation{0};

        friend class detail::data_writer_listener<data_pub_sub_type, on_matched_function_type>;

        // Captured construction parameters for replay after a reset.
        std::string captured_topic_name;
        ReliabilityQosPolicyKind reliability_kind;
        std::int32_t history_depth{use_default_history_depth};
        std::optional<DurabilityQosPolicyKind> durability_kind;

        // make_publisher needs to register the freshly-constructed shared_ptr
        // with the participant; that requires reaching `participant`, which is
        // private. Friending the factory overloads keeps the registration call
        // site at make_publisher (where the shared_ptr is born) without
        // exposing internals to user code.
        template <typename pub_sub_type>
        friend std::shared_ptr<publisher_handle<pub_sub_type>> make_publisher(std::shared_ptr<domain_participant>,
                                                                              const std::string &,
                                                                              ReliabilityQosPolicyKind, std::int32_t,
                                                                              std::optional<DurabilityQosPolicyKind>);

        template <typename pub_sub_type, typename matched_function_type>
        friend std::shared_ptr<publisher_handle<pub_sub_type, matched_function_type>> make_publisher(
            std::shared_ptr<domain_participant>, const std::string &, matched_function_type, ReliabilityQosPolicyKind,
            std::int32_t, std::optional<DurabilityQosPolicyKind>);
    };

    /**
     * @brief Creates a new publisher_handle object as a shared_ptr. The publisher_handle is automatically deleted
     * correctly on destroying its last shared_ptr.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
     * @param topic_name A DDS Topic Name
     * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataWriter,
     * which makes publishing slower but more reliable
     * @param history_depth KEEP_LAST history depth. use_default_history_depth (-1) uses the per-type qos_defaults
     * depth (else Fast-DDS's default); a positive value sets KEEP_LAST of that depth (any non-positive value, including
     * 0, uses the default). Durability is configured separately via durability_kind.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS for late-joiner
     * support); std::nullopt keeps the Fast-DDS/XML default.
     * @return std::shared_ptr<publisher_handle<data_pub_sub_type>>
     * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
     * @see provizio::dds::publisher_handle
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
     */
    template <typename data_pub_sub_type>
    std::shared_ptr<publisher_handle<data_pub_sub_type>> make_publisher(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind,
        std::int32_t history_depth = use_default_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt)
    {
        // shared_ptr(new T(...)) rather than make_shared so the constructor's
        // accessibility check happens in this function (a friend of
        // publisher_handle) instead of inside std::make_shared's internals.
        std::shared_ptr<publisher_handle<data_pub_sub_type>> handle{new publisher_handle<data_pub_sub_type>(
            std::move(participant), topic_name, reliability_kind, history_depth, durability_kind)};
        // register_endpoint does the initial build_state under the lifecycle
        // lock, atomically with adding the handle to the recovery registry.
        handle->participant->register_endpoint(handle);
        return handle;
    }

    /**
     * @brief Creates a new publisher_handle object as a shared_ptr, with an on_matched_function invoked on
     * subscriber match changes. The publisher_handle is automatically deleted correctly on destroying its last
     * shared_ptr.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_matched_function_type Function / function object type to be invoked on subscriber match changes.
     * Takes two arguments — a reference to this publisher_handle and a bool that is true when the first subscriber
     * is matched and false when the last subscriber is unmatched. Alternatively it can accept a third argument of
     * @c provizio::dds::guid type (the (un)matched subscriber's GUID), in which case it is invoked on every
     * match/unmatch.
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
     * @param topic_name A DDS Topic Name
     * @param on_matched_function Function / function object invoked on subscriber match changes; see the
     * @c on_matched_function_type description.
     * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataWriter,
     * which makes publishing slower but more reliable
     * @param history_depth KEEP_LAST history depth. use_default_history_depth (-1) uses the per-type qos_defaults
     * depth (else Fast-DDS's default); a positive value sets KEEP_LAST of that depth (any non-positive value, including
     * 0, uses the default). Durability is configured separately via durability_kind.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS for late-joiner
     * support); std::nullopt keeps the Fast-DDS/XML default.
     * @return std::shared_ptr<publisher_handle<data_pub_sub_type, on_matched_function_type>>
     * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
     * @see provizio::dds::publisher_handle
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
     */
    template <typename data_pub_sub_type, typename on_matched_function_type>
    std::shared_ptr<publisher_handle<data_pub_sub_type, on_matched_function_type>> make_publisher(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_matched_function_type on_matched_function,
        ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind,
        std::int32_t history_depth = use_default_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt)
    {
        // shared_ptr(new T(...)) rather than make_shared — see the comment in
        // the no-callback overload above.
        std::shared_ptr<publisher_handle<data_pub_sub_type, on_matched_function_type>> handle{
            new publisher_handle<data_pub_sub_type, on_matched_function_type>(
                std::move(participant), topic_name, std::move(on_matched_function), reliability_kind, history_depth,
                durability_kind)};
        handle->participant->register_endpoint(handle);
        return handle;
    }

    namespace detail
    {
        template <typename data_pub_sub_type, typename on_matched_function_type>
        class data_writer_listener : public DataWriterListener
        {
          public:
            data_writer_listener(publisher_handle<data_pub_sub_type, on_matched_function_type> &publisher)
                : publisher(publisher)
            {
            }

            void on_publication_matched(DataWriter *writer, const PublicationMatchedStatus &info) override
            {
                (void)writer;

                // Drain-aware entry: if a reset is in progress, the body must not
                // call into user code (which may re-enter provizio APIs that take
                // the lifecycle lock shared). The matched-count bookkeeping is
                // also skipped — it would otherwise re-converge once the new
                // listener is bound and the next match status arrives.
                listener_drain::scoped_call call{publisher.match_drain};
                if (!call.should_run())
                {
                    return;
                }

                {
                    std::lock_guard<std::mutex> lock{publisher.num_matched_subscribers_mutex};
                    publisher.num_matched_subscribers = info.current_count;
                }
                publisher.num_matched_subscribers_cv.notify_all();

                if constexpr (!std::is_same_v<on_matched_function_type, void *>)
                {
                    // A throwing user callback must not escape into the Fast-DDS listener
                    // thread — it would std::terminate the process. Report it through the
                    // configurable logger and continue.
                    try
                    {
                        constexpr size_t arity = function_traits<on_matched_function_type>::arity;
                        if constexpr (arity == 2)
                        {
                            if (info.current_count > 0 && info.current_count_change == info.current_count)
                            {
                                // Just matched the first subscriber
                                publisher.on_matched_function(publisher, true);
                            }
                            else if (info.current_count == 0 && info.current_count_change < 0)
                            {
                                // Just unmatched the last subscriber
                                publisher.on_matched_function(publisher, false);
                            }
                        }
                        else
                        {
                            publisher.on_matched_function(publisher, info.current_count_change > 0,
                                                          static_cast<const guid &>(info.last_subscription_handle));
                        }
                    }
                    catch (const std::exception &exception)
                    {
                        // Guard the logging too — it can throw std::bad_alloc on stream growth,
                        // which must not escape into the Fast-DDS listener thread.
                        try
                        {
                            log_error() << "publisher on_matched callback threw: " << exception.what();
                        }
                        catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                        {
                        }
                    }
                    catch (...)
                    {
                        try
                        {
                            log_error() << "publisher on_matched callback threw a non-std::exception";
                        }
                        catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                        {
                        }
                    }
                }
            }

          private:
            publisher_handle<data_pub_sub_type, on_matched_function_type> &publisher;
        };
    }  // namespace detail

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::publisher_handle(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        const ReliabilityQosPolicyKind reliability_kind, const std::int32_t history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind)
        : publisher_handle(std::move(participant), topic_name, nullptr,
                           std::make_unique<detail::data_writer_listener<data_pub_sub_type, void *>>(*this),
                           reliability_kind, history_depth, durability_kind)
    {
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::publisher_handle(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_matched_function_type on_matched_function, const ReliabilityQosPolicyKind reliability_kind,
        const std::int32_t history_depth, std::optional<DurabilityQosPolicyKind> durability_kind)
        : publisher_handle(
              std::move(participant), topic_name, std::move(on_matched_function),
              std::make_unique<detail::data_writer_listener<data_pub_sub_type, on_matched_function_type>>(*this),
              reliability_kind, history_depth, durability_kind)
    {
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::publisher_handle(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_matched_function_type on_matched_function, std::unique_ptr<DataWriterListener> &&listener,
        const ReliabilityQosPolicyKind reliability_kind, const std::int32_t history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind)
        // The handle holds a strong shared_ptr to its participant, so the participant
        // outlives every endpoint created against it — even after the caller releases
        // its own participant handle. register_type() must run on that stored
        // participant, so the member is initialised first (move) and reused below via
        // this->participant.
        : participant(std::move(participant)),
          type_support(this->participant->template register_type<data_pub_sub_type>()),
          on_matched_function(std::move(on_matched_function)), listener(std::move(listener)),
          captured_topic_name(topic_name), reliability_kind(reliability_kind), history_depth(history_depth),
          durability_kind(durability_kind)
    {
        // Intentionally NO build_state here. make_publisher calls
        // participant->register_endpoint(handle), which performs the initial
        // build under the lifecycle lock atomically with adding us to the
        // recovery registry. This eliminates the otherwise-possible window where
        // a concurrent reset destroys the participant we just built against.
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    void publisher_handle<data_pub_sub_type, on_matched_function_type>::build_state(
        eprosima::fastdds::dds::DomainParticipant &on_participant)
    {
        const auto &topic_qos = TOPIC_QOS_DEFAULT;
        const auto &publisher_qos = PUBLISHER_QOS_DEFAULT;

        // _locked variant: build_state is always called with reset_mutex held
        // (shared inside register_endpoint, exclusive inside
        // trigger_network_recovery_reset). Re-acquiring it here would be a
        // recursive same-thread shared lock, which is undefined for
        // std::shared_mutex and aborts with EDEADLK on glibc.
        the_topic = participant->register_topic_locked(captured_topic_name, type_support->get_name(), topic_qos);
        publisher = on_participant.create_publisher(publisher_qos);
        if (publisher == nullptr)
        {
            throw std::runtime_error{"publisher_handle: create_publisher returned nullptr for topic " +
                                     captured_topic_name};
        }

        DataWriterQos datawriter_qos;
        publisher->get_default_datawriter_qos(datawriter_qos);
        // History (untied from durability): an explicit positive depth wins, else fall back to
        // the per-type default (0 = leave the Fast-DDS default). KEEP_LAST only — durability is
        // configured independently below, so this is not an RxO QoS (ROS2 interop unaffected).
        const std::int32_t effective_history_depth =
            (history_depth > 0) ? history_depth : qos_defaults<data_pub_sub_type>::keep_last_history_depth;
        if (effective_history_depth > 0)
        {
            datawriter_qos.history().kind = KEEP_LAST_HISTORY_QOS;
            datawriter_qos.history().depth = effective_history_depth;
        }
        // Durability is now independent of history: applied only if the caller requested it,
        // otherwise the Fast-DDS/XML default is preserved. Mirrors the reliability_kind parameter.
        if (durability_kind.has_value())
        {
            datawriter_qos.durability().kind = *durability_kind;
        }
        datawriter_qos.reliability().kind = reliability_kind;
        datawriter_qos.publish_mode().kind = qos_defaults<data_pub_sub_type>::datawriter_publish_mode;
        datawriter_qos.endpoint().history_memory_policy = qos_defaults<data_pub_sub_type>::memory_policy;

#if defined(_MSC_VER) || defined(__APPLE__)
        // Disable data sharing on Windows and macOS: it uses shared memory segments
        // that may be unavailable or leak resources. On Windows the interprocess
        // directory may not exist; on macOS the system-wide SHM limits are low.
        datawriter_qos.data_sharing().off();
#endif

        // Prime the listener BEFORE create_datawriter attaches it to the new
        // DataWriter: Fast-DDS can fire on_publication_matched on an internal
        // thread before create_datawriter returns to us, and we must observe
        // those callbacks rather than clobber them. Zeroing the match count
        // and clearing the drain afterwards (the previous order) raced the
        // first match callback to zero, which Fast-DDS would never re-fire
        // because the underlying matched state never changed again — leaving
        // get_num_matched_subscribers stuck at 0 forever and request/response
        // clients timing out waiting for the service.
        {
            const std::lock_guard<std::mutex> lock{num_matched_subscribers_mutex};
            num_matched_subscribers = 0;
        }
        num_matched_subscribers_cv.notify_all();
        match_drain.reattach();

        data_writer = publisher->create_datawriter(the_topic->get(), datawriter_qos, listener.get());
        if (data_writer == nullptr)
        {
            // Roll back the Publisher and topic handle so partial state isn't
            // left dangling. teardown_state is purpose-built for this kind of
            // partial-rollback because the next attempt (e.g. retry from a
            // future reset) will start from a clean slate.
            on_participant.delete_publisher(publisher);
            publisher = nullptr;
            the_topic.reset();
            throw std::runtime_error{"publisher_handle: create_datawriter returned nullptr for topic " +
                                     captured_topic_name};
        }

        built_against_generation = participant->participant_generation();
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    void publisher_handle<data_pub_sub_type, on_matched_function_type>::teardown_state(
        eprosima::fastdds::dds::DomainParticipant &on_participant) noexcept
    {
        // Caller holds the lifecycle lock shared (locked_participant) or unique
        // (reset path) — no concurrent reset can run.

        if (built_against_generation == 0)
        {
            // Never built. Nothing to free.
            return;
        }

        if (built_against_generation != participant->participant_generation())
        {
            // A reset replaced the participant we built against without
            // rebuilding us (we were being destroyed concurrently and didn't
            // make it into the reset's endpoint snapshot). The old participant's
            // delete_contained_entities() has already freed the Publisher and
            // DataWriter we held raw pointers to — calling delete_publisher /
            // delete_datawriter again would use-after-free.
            data_writer = nullptr;
            publisher = nullptr;
            the_topic.reset();
            built_against_generation = 0;
            return;
        }

        if (data_writer != nullptr && publisher != nullptr)
        {
            publisher->delete_datawriter(data_writer);
        }
        data_writer = nullptr;
        if (publisher != nullptr)
        {
            on_participant.delete_publisher(publisher);
        }
        publisher = nullptr;
        the_topic.reset();
        built_against_generation = 0;
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::~publisher_handle()
    {
        // Detach the listener drain BEFORE teardown. Fast-DDS's
        // delete_datawriter synchronously waits for in-flight
        // on_publication_matched callbacks to return; if the user-supplied
        // on_matched callback (run inside our listener_drain::scoped_call
        // body) is blocked acquiring a mutex / condition variable held by
        // the thread that is now destroying us, the wait never completes
        // and ~publisher_handle deadlocks. Detaching flips the drain's
        // detached flag (so subsequent scoped_call bodies early-return)
        // and waits for currently-in-flight bodies to leave the scope;
        // Fast-DDS's internal callback-drain inside delete_datawriter is
        // then a no-op. Without this, any test that synchronises with
        // on_matched via a Condition and tears the handle down while
        // holding that Condition is a deadlock.
        match_drain.detach_and_drain();

        // Acquire the lifecycle lock so a concurrent reset waits to take
        // exclusive ownership until our teardown finishes (or, if it already
        // took it, our teardown happens against the post-reset participant —
        // which teardown_state detects via generation mismatch and handles
        // safely).
        auto fdds = participant->fastdds_participant();
        participant->deregister_endpoint(this);
        if (fdds.get() != nullptr)
        {
            teardown_state(*fdds);
        }
        else
        {
            // Fast-DDS participant is currently null — typically because a
            // network-recovery reset's create_participant call failed and the
            // participant has been left in the dead state documented in
            // trigger_network_recovery_reset. delete_contained_entities has
            // already freed every contained Publisher / DataWriter, so our
            // raw pointers are stale and must NOT be passed to any Fast-DDS
            // API. Just clear them.
            data_writer = nullptr;
            publisher = nullptr;
            the_topic.reset();
            built_against_generation = 0;
        }
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    void publisher_handle<data_pub_sub_type, on_matched_function_type>::detach_for_reset() noexcept
    {
        // Block new listener callback bodies and wait for any in-flight ones
        // to return. Called by domain_participant::trigger_network_recovery_reset
        // BEFORE it takes the exclusive lifecycle lock, so a callback that
        // re-enters provizio APIs (which take the lifecycle lock shared) can
        // still complete and the drain does not deadlock.
        match_drain.detach_and_drain();
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    void publisher_handle<data_pub_sub_type, on_matched_function_type>::on_participant_reset(
        eprosima::fastdds::dds::DomainParticipant &old_participant) noexcept
    {
        teardown_state(old_participant);
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    void publisher_handle<data_pub_sub_type, on_matched_function_type>::on_new_participant_started(
        eprosima::fastdds::dds::DomainParticipant &new_participant)
    {
        build_state(new_participant);
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    bool publisher_handle<data_pub_sub_type, on_matched_function_type>::publish(data_type &data)
    {
        [[maybe_unused]] const auto fdds_lock = participant->fastdds_participant();
        if (data_writer == nullptr || built_against_generation != participant->participant_generation())
        {
            // Either never built, or a network-recovery reset rebuilt the
            // participant without rebuilding us (e.g. we were created from
            // inside a listener callback during the reset's drain phase, so
            // we missed the snapshot). The DataWriter we hold a raw pointer
            // to has been freed; the next reset will rebuild us. Surface a
            // clean failure to the caller rather than use-after-free.
            return false;
        }
        // DataWriter::write returns ReturnCode_t (an integer status), not bool —
        // compare explicitly so only the success code (RETCODE_OK) maps to true.
        return data_writer->write(&data) == RETCODE_OK;
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    bool publisher_handle<data_pub_sub_type, on_matched_function_type>::publish(data_type &data, WriteParams &params)
    {
        [[maybe_unused]] const auto fdds_lock = participant->fastdds_participant();
        if (data_writer == nullptr || built_against_generation != participant->participant_generation())
        {
            return false;
        }
        return data_writer->write(&data, params) == RETCODE_OK;
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    guid publisher_handle<data_pub_sub_type, on_matched_function_type>::get_guid() const
    {
        [[maybe_unused]] const auto fdds_lock = participant->fastdds_participant();
        if (data_writer == nullptr || built_against_generation != participant->participant_generation())
        {
            return guid{};
        }
        return data_writer->guid();
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    int publisher_handle<data_pub_sub_type, on_matched_function_type>::get_num_matched_subscribers(
        const std::chrono::milliseconds timeout, const std::chrono::milliseconds settle_time) const
    {
        std::unique_lock<std::mutex> lock{num_matched_subscribers_mutex};
        const auto timeout_point = std::chrono::steady_clock::now() + timeout;
        const std::chrono::milliseconds min_attempt_time{50};
        if (!num_matched_subscribers_cv.wait_for(lock, std::max(timeout - settle_time, min_attempt_time),
                                                 [this] { return num_matched_subscribers > 0; }))
        {
            // No matches
            return 0;
        }

        if (settle_time.count() > 0)
        {
            do
            {
                if (!num_matched_subscribers_cv.wait_for(
                        lock, settle_time, [this, num_matched_subscribers_was = num_matched_subscribers] {
                            return num_matched_subscribers_was != num_matched_subscribers;
                        }))
                {
                    // No change during the settle_time period
                    return num_matched_subscribers;
                }
            } while (std::chrono::steady_clock::now() < timeout_point);

            // Wasn't ever stable for settle_time until the timeout
            return -1;
        }

        return num_matched_subscribers;
    }
}  // namespace provizio::dds

#endif  // DDS_PUBLISHER
