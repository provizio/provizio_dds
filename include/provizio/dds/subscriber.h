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

#ifndef DDS_SUBSCRIBER
#define DDS_SUBSCRIBER

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/detail/bounded_wait.h"
#include "provizio/dds/detail/listener_drain.h"
#include "provizio/dds/detail/resettable_endpoint.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/function_traits.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/qos_defaults.h"

namespace provizio::dds
{
    template <typename data_pub_sub_type> class subscriber_handle;

    namespace detail
    {
        /**
         * @brief Internal DataReaderListener that tracks publisher match counts
         * for readiness helpers. Wraps every callback in a @c listener_drain
         * scope so the network-recovery reset path can quiesce the listener
         * before tearing the DataReader down.
         *
         * In @c provizio::dds::detail rather than the public namespace because
         * the type is not part of the customer-facing API surface — users
         * supply their callback via @c make_subscriber and never see this
         * class directly.
         */
        class data_reader_listener : public DataReaderListener
        {
          public:
            void on_subscription_matched(DataReader *reader, const SubscriptionMatchedStatus &info) override
            {
                (void)reader;
                listener_drain::scoped_call call{drain};
                if (!call.should_run())
                {
                    return;
                }
                {
                    const std::lock_guard<std::mutex> lock{num_matched_publishers_mutex};
                    num_matched_publishers = info.current_count;
                }
                num_matched_publishers_cv.notify_all();
            }

          protected:
            // Subclasses that override on_data_available must wrap their body in
            // a listener_drain::scoped_call too — see on_data_function_data_listener.
            listener_drain drain;

          private:
            mutable std::mutex num_matched_publishers_mutex;
            mutable std::condition_variable num_matched_publishers_cv;
            int num_matched_publishers{0};

            template <typename data_pub_sub_type> friend class provizio::dds::subscriber_handle;
        };
    }  // namespace detail

    /**
     * @brief Encapsulates DDS Subscriber and DataReader functionality in a single entity with automatic life cycle
     * management. Normally created with provizio::dds::make_subscriber.
     *
     * @note When the parent participant has network auto-recovery enabled, the
     * underlying Fast-DDS Subscriber / DataReader objects may be swapped under this
     * handle as part of a participant reset (see network_recovery.h). The caller-held
     * shared_ptr to the handle and the user-supplied data_listener (with its callback)
     * both survive — the listener is re-attached to the new DataReader.
     *
     * @note The on-data callback is invoked from a Fast-DDS internal thread.
     * Calling other provizio APIs that don't create new endpoints (@c publish on
     * a sibling publisher, @c get_guid, etc.) is safe — the listener drain
     * happens BEFORE the lifecycle lock is taken exclusively, specifically to
     * avoid that deadlock. However, callbacks MUST NOT block indefinitely
     * (resets wait for in-flight callbacks to drain) and MUST NOT create new
     * endpoints (@c make_publisher / @c make_subscriber / @c register_topic /
     * @c register_type) — those acquire @c domain_participant::registration_mutex
     * which a concurrent reset is holding while waiting for this very callback
     * to return, producing an AB-BA deadlock. A handle created via
     * @c make_publisher from inside a callback during a reset would also miss
     * the reset's snapshot and end up with a stale @c DataWriter; subsequent
     * @c publish calls would safely return @c false until the next reset
     * rebuilds it.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @see provizio::dds::make_subscriber
     */
    template <typename data_pub_sub_type> class subscriber_handle final : public detail::resettable_endpoint
    {
      public:
        using data_type = typename data_pub_sub_type::type;

        ~subscriber_handle() override;

        /**
         * @brief Returns GUID of the underlying DataReader.
         */
        guid get_guid() const;

        /**
         * @brief Blocks until this subscriber has at least one stable match for a short settle window.
         * @note This is a blocking call; invoke in a background thread if you must not block the caller thread.
         * @note For a match-publisher (default) subscriber the DataReader is not created until a matching writer
         * is discovered (see @c match_publisher_reliability_qos). This call honours @p timeout in that state too:
         * it blocks waiting for a writer to be discovered, the deferred reader to be built, and the first match —
         * returning the matched count if that happens within @p timeout, rather than returning 0 at t=0 while
         * reliability discovery is still in progress.
         * @param timeout Total timeout duration. The implementation always waits at least ~50ms per attempt, so
         * timeouts smaller than that (including 0) still incur a short minimum wait.
         * @param settle_time The minimum time the match must remain stable (no further status changes) to be
         * considered "ready".
         * @return non-negative number of matched publishers if stable within timeout, -1 otherwise.
         */
        int get_num_matched_publishers(std::chrono::milliseconds timeout, std::chrono::milliseconds settle_time) const;

      private:
        // Construction is intentionally private: a subscriber_handle must be
        // owned by a shared_ptr (the network-recovery registry holds one) and
        // must be registered with its parent participant so network-recovery
        // resets can rebuild it. Both invariants are upheld by make_subscriber,
        // which is the only friend allowed to instantiate the class.

        /**
         * @brief Constructs a new subscriber_handle object.
         *
         * The constructor only captures the parameters needed to build (and later
         * rebuild) the underlying Fast-DDS objects. The initial Fast-DDS
         * Subscriber / DataReader are created by @c domain_participant::register_endpoint
         * under the lifecycle lock, atomically with adding the handle to the
         * recovery registry.
         *
         * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
         * @param topic_name A DDS Topic Name
         * @param data_listener A DataReaderListener that receives incoming samples and tracks publisher match
         * counts. Normally constructed by @c provizio::dds::make_subscriber from a user-supplied on-data callback.
         * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS
         * DataReader; reliable subscribers are slower but lossless on matched publishers
         * @param max_history_depth KEEP_LAST history depth. use_default_history_depth (-1) uses the per-type
         * qos_defaults depth (else Fast-DDS's default); a positive value sets KEEP_LAST of that depth (any non-positive
         * value, including 0, uses the default). Durability is configured separately via durability_kind.
         * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS for late-joiner
         * support); std::nullopt keeps the Fast-DDS/XML default.
         * @note A BEST_EFFORT_RELIABILITY_QOS subscriber will not match reliable publishers.
         * @see provizio::dds::make_subscriber
         * @see provizio::dds::make_domain_participant
         * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/subscriber/dataReaderListener/dataReaderListener.html
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        subscriber_handle(
            std::shared_ptr<domain_participant> participant, const std::string &topic_name,
            std::shared_ptr<detail::data_reader_listener> data_listener,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datareader_reliability_kind,
            std::int32_t max_history_depth = use_default_history_depth,
            std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt);

        /// @internal Called by domain_participant during network-recovery reset.
        void detach_for_reset() noexcept override;
        void on_participant_reset(eprosima::fastdds::dds::DomainParticipant &old_participant) noexcept override;
        void on_new_participant_started(eprosima::fastdds::dds::DomainParticipant &new_participant) override;

        /// @internal Match-mode hooks (see detail::resettable_endpoint). start_deferred_build spawns
        /// the off-thread reader build (storing build_future) when a matching writer is discovered;
        /// build_deferred_locked runs that build under the participant's registration + reset locks.
        void start_deferred_build(ReliabilityQosPolicyKind resolved) override;
        void build_deferred_locked(eprosima::fastdds::dds::DomainParticipant &on_participant,
                                   ReliabilityQosPolicyKind resolved) override;

        /// @brief The reliability to build the DataReader with: the resolved discovered-publisher
        /// reliability once a matching writer has been seen, otherwise the configured
        /// @c reliability_kind (which may itself be the match sentinel → still pending).
        ReliabilityQosPolicyKind effective_reliability() const
        {
            return resolved_match_reliability.value_or(reliability_kind);
        }

        void build_state(eprosima::fastdds::dds::DomainParticipant &on_participant);
        void teardown_state(eprosima::fastdds::dds::DomainParticipant &on_participant) noexcept;

        std::shared_ptr<domain_participant> participant;
        dds::TypeSupport type_support;
        std::shared_ptr<detail::data_reader_listener> data_listener;

        // State that gets swapped on reset.
        std::shared_ptr<topic> the_topic;
        Subscriber *subscriber = nullptr;
        DataReader *data_reader = nullptr;
        std::uint64_t built_against_generation{0};

        // Captured construction parameters for replay after a reset.
        std::string captured_topic_name;
        ReliabilityQosPolicyKind reliability_kind;
        std::int32_t max_history_depth{use_default_history_depth};
        std::optional<DurabilityQosPolicyKind> durability_kind;

        // Match-publisher mode: when reliability_kind == match_publisher_reliability_qos the
        // reader is not built until a matching writer is discovered; this holds the adopted
        // reliability once resolved. Survives a network-recovery reset, so a resolved match-mode
        // subscriber rebuilds with the same adopted reliability instead of re-deferring. Written
        // and read under the participant lifecycle lock the build / reset paths already hold
        // (the initial build_state runs under reset_mutex shared; the deferred build via
        // build_deferred_locked and the reset path both run under reset_mutex exclusive).
        std::optional<ReliabilityQosPolicyKind> resolved_match_reliability;

        // Match-publisher mode: the off-thread first build, launched by start_deferred_build via
        // std::async when a matching writer is discovered. The task captures `this` by raw pointer
        // (it does NOT own the handle) and runs build_deferred_locked. ~subscriber_handle waits on
        // this future BEFORE tearing down anything the task uses, so the task can never touch a
        // freed handle and teardown always runs on the owner thread. Launched at most once (the
        // future stays valid after the first launch), so repeated writer events or a reset
        // re-registering an unresolved subscriber never spawn a second builder.
        std::future<void> build_future;

        // make_subscriber needs to register the freshly-constructed shared_ptr
        // with the participant; that requires reaching `participant`, which is
        // private. Friending the factory overloads keeps the registration call
        // site at make_subscriber (where the shared_ptr is born) without
        // exposing internals to user code.
        template <typename pub_sub_type, typename data_function_type>
        friend std::shared_ptr<subscriber_handle<pub_sub_type>> make_subscriber(std::shared_ptr<domain_participant>,
                                                                                const std::string &, data_function_type,
                                                                                ReliabilityQosPolicyKind, std::int32_t,
                                                                                std::optional<DurabilityQosPolicyKind>);

        template <typename pub_sub_type, typename data_function_type, typename publisher_changed_function_type>
        friend std::shared_ptr<subscriber_handle<pub_sub_type>> make_subscriber(std::shared_ptr<domain_participant>,
                                                                                const std::string &, data_function_type,
                                                                                publisher_changed_function_type,
                                                                                ReliabilityQosPolicyKind, std::int32_t,
                                                                                std::optional<DurabilityQosPolicyKind>);
    };

    /**
     * @brief Creates a new subscriber_handle object as a shared_ptr with a function / function object to be invoked
     * on receiving data. The subscriber_handle is automatically deleted correctly on destroying its last
     * shared_ptr.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_data_function_type Function / function object type invoked for each received sample. Accepts
     * either one argument @c (data_type&) or two arguments @c (data_type&, const SampleInfo&).
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
     * @param topic_name A DDS Topic Name
     * @param on_data_function Function / function object invoked for each received sample.
     * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataReader;
     * reliable subscribers are slower but lossless on matched publishers
     * @param max_history_depth KEEP_LAST history depth. use_default_history_depth (-1) uses the per-type qos_defaults
     * depth (else Fast-DDS's default); a positive value sets KEEP_LAST of that depth (any non-positive value, including
     * 0, uses the default). Durability is configured separately via durability_kind.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS for late-joiner
     * support); std::nullopt keeps the Fast-DDS/XML default.
     * @return std::shared_ptr<subscriber_handle<data_pub_sub_type>>
     * @note A BEST_EFFORT_RELIABILITY_QOS subscriber will not match reliable publishers.
     * @note @c on_data_function is invoked on a Fast-DDS internal thread; treat it as a hot callback.
     * @see provizio::dds::subscriber_handle
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
     */
    template <typename data_pub_sub_type, typename on_data_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function,
        ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datareader_reliability_kind,
        std::int32_t max_history_depth = use_default_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt);

    /**
     * @brief Creates a new subscriber_handle object as a shared_ptr with a data callback and a publisher-match
     * callback. The subscriber_handle is automatically deleted correctly on destroying its last shared_ptr.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_data_function_type Function / function object type invoked for each received sample. Accepts
     * either one argument @c (data_type&) or two arguments @c (data_type&, const SampleInfo&).
     * @tparam on_has_publisher_changed_function_type Function / function object type invoked when the matched
     * publisher count crosses zero. Takes one @c bool argument: true on matching the first publisher, false on
     * unmatching the last publisher.
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
     * @param topic_name A DDS Topic Name
     * @param on_data_function Function / function object invoked for each received sample.
     * @param on_has_publisher_changed_function Function / function object invoked on first match / last unmatch.
     * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataReader;
     * reliable subscribers are slower but lossless on matched publishers
     * @param max_history_depth KEEP_LAST history depth. use_default_history_depth (-1) uses the per-type qos_defaults
     * depth (else Fast-DDS's default); a positive value sets KEEP_LAST of that depth (any non-positive value, including
     * 0, uses the default). Durability is configured separately via durability_kind.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS for late-joiner
     * support); std::nullopt keeps the Fast-DDS/XML default.
     * @return std::shared_ptr<subscriber_handle<data_pub_sub_type>>
     * @note A BEST_EFFORT_RELIABILITY_QOS subscriber will not match reliable publishers.
     * @note Both callbacks are invoked on Fast-DDS internal threads; treat them as hot callbacks.
     * @see provizio::dds::subscriber_handle
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
     */
    template <typename data_pub_sub_type, typename on_data_function_type,
              typename on_has_publisher_changed_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function,
        on_has_publisher_changed_function_type on_has_publisher_changed_function,
        const ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datareader_reliability_kind,
        std::int32_t max_history_depth = use_default_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt);

    template <typename data_pub_sub_type>
    subscriber_handle<data_pub_sub_type>::subscriber_handle(std::shared_ptr<domain_participant> participant,
                                                            const std::string &topic_name,
                                                            std::shared_ptr<detail::data_reader_listener> data_listener,
                                                            const ReliabilityQosPolicyKind reliability_kind,
                                                            const std::int32_t max_history_depth,
                                                            std::optional<DurabilityQosPolicyKind> durability_kind)
        // The handle holds a strong shared_ptr to its participant, so the participant
        // (and the discovery listener that drives the match-publisher deferred build)
        // outlives every endpoint created against it — even after the caller releases
        // its own participant handle. register_type() must run on that stored
        // participant, so the member is initialised first (move) and reused below via
        // this->participant.
        : participant(std::move(participant)),
          type_support(this->participant->template register_type<data_pub_sub_type>()),
          data_listener(std::move(data_listener)), captured_topic_name(topic_name), reliability_kind(reliability_kind),
          max_history_depth(max_history_depth), durability_kind(durability_kind)
    {
        // Intentionally NO build_state here — see publisher_handle ctor for the
        // rationale. The participant's register_endpoint() does the initial
        // build atomically with adding us to the recovery registry.
    }

    template <typename data_pub_sub_type>
    void subscriber_handle<data_pub_sub_type>::build_state(eprosima::fastdds::dds::DomainParticipant &on_participant)
    {
        // Match-publisher mode: when the effective reliability is still the match sentinel (no
        // matching writer has been discovered yet), DO NOT create the Subscriber / DataReader.
        // Reader QoS (reliability) is immutable after enable, so we can't create now and fix it
        // later — we must wait for the writer's offered reliability. Hand the participant a
        // NON-owning back-reference (raw this) so its discovery listener can route a matching-writer
        // event back to us; on the first such writer it calls start_deferred_build, which hops the
        // build onto its own thread (off the Fast-DDS discovery thread) and runs build_deferred_locked
        // → build_state again with resolved_match_reliability set. Re-entrant on a network-recovery
        // reset: an unresolved match-mode subscriber simply re-defers here; a resolved one falls
        // through and rebuilds with the adopted reliability.
        const ReliabilityQosPolicyKind effective_reliability_kind = effective_reliability();
        if (effective_reliability_kind == match_publisher_reliability_qos)
        {
            participant->register_deferred_subscriber(captured_topic_name, this);
            return;
        }

        const auto &topic_qos = TOPIC_QOS_DEFAULT;
        const auto &subscriber_qos = SUBSCRIBER_QOS_DEFAULT;

        // _locked variant: see publisher_handle::build_state for the rationale —
        // build_state is always called with reset_mutex already held, so we
        // must not try to re-acquire it.
        the_topic = participant->register_topic_locked(captured_topic_name, type_support->get_name(), topic_qos);
        subscriber = on_participant.create_subscriber(subscriber_qos);
        if (subscriber == nullptr)
        {
            throw std::runtime_error{"subscriber_handle: create_subscriber returned nullptr for topic " +
                                     captured_topic_name};
        }

        DataReaderQos datareader_qos;
        subscriber->get_default_datareader_qos(datareader_qos);
        datareader_qos.reliability().kind = effective_reliability_kind;
        datareader_qos.endpoint().history_memory_policy = qos_defaults<data_pub_sub_type>::memory_policy;
        // History (untied from durability): an explicit positive depth wins, else fall back to
        // the per-type default (0 = leave the Fast-DDS default). KEEP_LAST only — durability is
        // configured independently below, so this is not an RxO QoS (ROS2 interop unaffected).
        const std::int32_t effective_history_depth =
            (max_history_depth > 0) ? max_history_depth : qos_defaults<data_pub_sub_type>::keep_last_history_depth;
        if (effective_history_depth > 0)
        {
            datareader_qos.history().kind = HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
            datareader_qos.history().depth = effective_history_depth;
        }
        // Durability is now independent of history: applied only if the caller requested it,
        // otherwise the Fast-DDS/XML default is preserved. Mirrors the reliability_kind parameter.
        if (durability_kind.has_value())
        {
            datareader_qos.durability().kind = *durability_kind;
        }

#if defined(_MSC_VER) || defined(__APPLE__)
        // Disable data sharing on Windows and macOS: it uses shared memory segments
        // that may be unavailable or leak resources. On Windows the interprocess
        // directory may not exist; on macOS the system-wide SHM limits are low.
        datareader_qos.data_sharing().off();
#endif

        // Prime the listener BEFORE create_datareader attaches it to the new
        // DataReader: Fast-DDS can fire on_subscription_matched on an internal
        // thread before create_datareader returns to us, and we must observe
        // those callbacks rather than clobber them. Zeroing the match count
        // and clearing the drain afterwards (the previous order) raced the
        // first match callback to zero, which Fast-DDS would never re-fire
        // because the underlying matched state never changed again — leaving
        // get_num_matched_publishers stuck at 0 forever and request/response
        // clients timing out waiting for the service.
        {
            const std::lock_guard<std::mutex> lock{data_listener->num_matched_publishers_mutex};
            data_listener->num_matched_publishers = 0;
        }
        data_listener->num_matched_publishers_cv.notify_all();
        data_listener->drain.reattach();

        data_reader = subscriber->create_datareader(the_topic->get(), datareader_qos, data_listener.get());
        if (data_reader == nullptr)
        {
            on_participant.delete_subscriber(subscriber);
            subscriber = nullptr;
            the_topic.reset();
            throw std::runtime_error{"subscriber_handle: create_datareader returned nullptr for topic " +
                                     captured_topic_name};
        }

        built_against_generation = participant->participant_generation();
    }

    template <typename data_pub_sub_type>
    void subscriber_handle<data_pub_sub_type>::teardown_state(
        eprosima::fastdds::dds::DomainParticipant &on_participant) noexcept
    {
        if (built_against_generation == 0)
        {
            return;
        }

        if (built_against_generation != participant->participant_generation())
        {
            // See publisher_handle::teardown_state for the rationale: our state
            // points into a participant that's already been freed by a reset we
            // missed. Just clear the local pointers.
            data_reader = nullptr;
            subscriber = nullptr;
            the_topic.reset();
            built_against_generation = 0;
            return;
        }

        if (data_reader != nullptr && subscriber != nullptr)
        {
            subscriber->delete_datareader(data_reader);
        }
        data_reader = nullptr;
        if (subscriber != nullptr)
        {
            on_participant.delete_subscriber(subscriber);
        }
        subscriber = nullptr;
        the_topic.reset();
        built_against_generation = 0;
    }

    template <typename data_pub_sub_type> subscriber_handle<data_pub_sub_type>::~subscriber_handle()
    {
        // Match-publisher mode only: close the door, then drain. Only subscribers created with the
        // match-publisher sentinel ever register a deferred back-reference (build_state registers iff
        // effective_reliability() == match_publisher_reliability_qos, and resolved_match_reliability is
        // never the sentinel — so that is exactly reliability_kind == match_publisher_reliability_qos).
        // Guarding on it avoids taking deferred_mutex on every explicit-reliability teardown.
        // deregister_deferred_subscriber removes our non-owning back-reference from the participant's
        // registry under deferred_mutex — the same mutex the discovery thread holds to dispatch us — so
        // once it returns no new deferred build can be launched against us. Then wait on any build
        // already in flight: the task captured `this` raw, so it must finish using `this` before we tear
        // anything down. Both run BEFORE we touch the listener / Fast-DDS state the build creates.
        if (reliability_kind == match_publisher_reliability_qos)
        {
            participant->deregister_deferred_subscriber(captured_topic_name, this);
        }
        if (build_future.valid())
        {
            build_future.wait();
        }

        // Detach the listener drain BEFORE teardown — see
        // publisher_handle::~publisher_handle for the full rationale. In
        // short: Fast-DDS's delete_datareader synchronously waits for any
        // in-flight on_data_available / on_subscription_matched callback
        // to return, and a user callback blocked on a mutex held by the
        // destroying thread would otherwise deadlock the destructor (e.g.
        // python tests' `with cv: ...; del subscriber` pattern).
        data_listener->drain.detach_and_drain();

        // Hold the shared lifecycle lock for the duration of destruction; the
        // generation check inside teardown_state handles the case where a
        // concurrent reset already destroyed our underlying participant without
        // including us in its endpoint snapshot.
        auto fdds = participant->fastdds_participant();
        participant->deregister_endpoint(this);
        if (fdds.get() != nullptr)
        {
            teardown_state(*fdds);
        }
        else
        {
            // See publisher_handle::~publisher_handle for the rationale.
            data_reader = nullptr;
            subscriber = nullptr;
            the_topic.reset();
            built_against_generation = 0;
        }
    }

    template <typename data_pub_sub_type> void subscriber_handle<data_pub_sub_type>::detach_for_reset() noexcept
    {
        data_listener->drain.detach_and_drain();
    }

    template <typename data_pub_sub_type>
    void subscriber_handle<data_pub_sub_type>::on_participant_reset(
        eprosima::fastdds::dds::DomainParticipant &old_participant) noexcept
    {
        teardown_state(old_participant);
    }

    template <typename data_pub_sub_type>
    void subscriber_handle<data_pub_sub_type>::on_new_participant_started(
        eprosima::fastdds::dds::DomainParticipant &new_participant)
    {
        build_state(new_participant);
    }

    template <typename data_pub_sub_type>
    void subscriber_handle<data_pub_sub_type>::start_deferred_build(const ReliabilityQosPolicyKind resolved)
    {
        // Called by the participant's deferred-subscriber registry under deferred_mutex when a
        // matching writer is discovered. Building the DataReader must NOT run on the Fast-DDS
        // discovery thread (Fast-DDS reentrancy + an AB-BA deadlock against a concurrent reset),
        // so hop it onto its own thread and return immediately. The task captures `this` raw — it
        // does not own the handle; ~subscriber_handle waits on build_future before teardown, so
        // `this` (and the participant it reaches through) stays valid for the task.
        //
        // This thread is one-shot and short-lived: run_deferred_build creates the DataReader once
        // and returns, then the thread exits. So the number of concurrent build threads is bounded
        // by how many subscribers resolve at the same instant (a writer just appeared for each),
        // not by the total subscriber count. A shared bounded executor was considered for the
        // many-topics-discovering-at-once case and deliberately not used: pooling would have to
        // reproduce the registration_mutex + EXCLUSIVE reset_mutex fencing run_deferred_build
        // relies on, and the build_future join in ~subscriber_handle that keeps `this` alive — i.e.
        // it would reintroduce the very reset-vs-build lifetime ordering this per-build thread sidesteps.
        //
        // Don't launch a second build while one is IN FLIGHT — reassigning build_future then would
        // block inside std::future's destructor waiting for the running task. But DO allow a relaunch
        // once a previous build has COMPLETED: if that build FAILED (e.g. create_datareader returned
        // null under resource exhaustion) the subscriber is left parked, so the next discovered writer
        // retries here instead of the subscriber staying inactive until the next network-recovery reset;
        // if it SUCCEEDED, build_deferred_locked already deregistered us, so resolve_deferred_for_writer
        // won't dispatch us again. Reassigning a ready (or never-launched) future does not block — the
        // prior task, if any, has finished. start_deferred_build is only ever called under the
        // participant's deferred_mutex (from resolve_deferred_for_writer / register_deferred_subscriber),
        // so these calls are serialized per handle and this check-then-reassign cannot race itself.
        if (build_future.valid() && build_future.wait_for(std::chrono::seconds{0}) != std::future_status::ready)
        {
            return;
        }
        // std::async can throw (e.g. std::system_error when a new thread can't be created under
        // resource exhaustion). This is invoked on the Fast-DDS discovery thread via the deferred
        // registry, where an escaping exception would std::terminate the process, so catch it and
        // log rather than throw — never rely solely on the caller's guard. build_future is left
        // invalid, so the subscriber stays deferred and a later writer event retries the launch.
        try
        {
            build_future =
                std::async(std::launch::async, [this, resolved] { participant->run_deferred_build(this, resolved); });
        }
        catch (const std::exception &exception)
        {
            // The streaming logger can itself throw (allocation/stream growth); guard it with an inner
            // try/catch(...) so nothing escapes onto the Fast-DDS discovery thread. Mirrors the
            // request_handler logging guards elsewhere in this codebase.
            try
            {
                log_error() << "start_deferred_build: failed to launch deferred reader build for topic "
                            << captured_topic_name << ": " << exception.what();
            }
            catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
            {
            }
        }
        catch (...)
        {
            // A non-std::exception must not escape this Fast-DDS-thread entry point either.
            try
            {
                log_error() << "start_deferred_build: failed to launch deferred reader build for topic "
                            << captured_topic_name << " (non-std::exception)";
            }
            catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
            {
            }
        }
    }

    template <typename data_pub_sub_type>
    void subscriber_handle<data_pub_sub_type>::build_deferred_locked(
        eprosima::fastdds::dds::DomainParticipant &on_participant, const ReliabilityQosPolicyKind resolved)
    {
        // Called by domain_participant::run_deferred_build on the build thread, which already holds
        // registration_mutex + reset_mutex (the latter EXCLUSIVE, to fence this reader swap against
        // concurrent get_guid / get_num_matched_publishers readers that hold reset_mutex shared) —
        // so build_state (which uses the _locked register_topic variant) must NOT re-acquire them.
        // Idempotent: a network-recovery reset that rebuilt us first leaves a non-null data_reader —
        // do nothing rather than build a second reader and orphan the first. Either way, once a reader
        // exists the build has succeeded, so drop our parked back-reference to stop the retry loop
        // (deregister takes deferred_mutex — a leaf — which is a valid acquire under the held
        // registration_mutex + reset_mutex; see the lock-order note in domain_participant.h).
        if (data_reader != nullptr)
        {
            participant->deregister_deferred_subscriber(captured_topic_name, this);
            return;
        }
        resolved_match_reliability = resolved;
        // If build_state throws (e.g. create_datareader returns null under resource exhaustion),
        // it propagates to run_deferred_build, which logs and swallows it. We deliberately do NOT
        // deregister in that case: the subscriber stays parked so the next discovered writer on this
        // topic re-dispatches and retries the build (start_deferred_build relaunches once this build
        // future is ready). On success we fall through and deregister, since the reader now exists.
        build_state(on_participant);
        participant->deregister_deferred_subscriber(captured_topic_name, this);
    }

    template <typename data_pub_sub_type> guid subscriber_handle<data_pub_sub_type>::get_guid() const
    {
        [[maybe_unused]] const auto fdds_lock = participant->fastdds_participant();
        if (data_reader == nullptr || built_against_generation != participant->participant_generation())
        {
            // See publisher_handle::publish for the rationale on the
            // generation check.
            return guid{};
        }
        return data_reader->guid();
    }

    template <typename data_pub_sub_type>
    int subscriber_handle<data_pub_sub_type>::get_num_matched_publishers(
        const std::chrono::milliseconds timeout, const std::chrono::milliseconds settle_time) const
    {
        {
            // Read the reader pointer + generation under the lifecycle lock so this can't race a
            // concurrent build_state swapping the reader in. (The blocking wait below intentionally
            // does NOT hold this lock.)
            //
            // A generation mismatch is treated as "no usable reader": in the documented "missed
            // snapshot" reset scenario (see publisher_handle::publish) data_reader can be non-null
            // but point into an already-freed previous participant generation, so its listener's
            // match count is stale — return 0 rather than report it (mirrors get_guid).
            //
            // A NULL data_reader is the match-publisher deferred case: do NOT early-return here.
            // Until a matching writer is discovered the DataReader doesn't exist yet, but the
            // data_listener (and its cv) are members that exist before — and survive — the reader,
            // so we fall through to the timed wait below. The deferred build, triggered when a
            // matching writer is discovered, creates the reader and its first on_subscription_matched
            // notifies the cv; a caller that blocked here therefore observes a match that happens
            // within `timeout` instead of getting 0 at t=0 while reliability discovery is still in
            // progress. (If no writer ever appears the wait simply times out and returns 0.)
            [[maybe_unused]] const auto fdds_lock = participant->fastdds_participant();
            if (data_reader != nullptr && built_against_generation != participant->participant_generation())
            {
                return 0;
            }
        }

        std::unique_lock<std::mutex> lock{data_listener->num_matched_publishers_mutex};
        // Saturating: a near-max timeout would overflow this into the past, and the
        // settle loop below would then conclude at once that the count never settled
        // (see detail/bounded_wait.h).
        const auto timeout_point = detail::saturating_deadline<std::chrono::steady_clock>(timeout);
        const std::chrono::milliseconds min_attempt_time{50};
        // A DEADLINE, not a duration: wait_for computes now() + duration internally, which a
        // near-max timeout overflows -- measured, it then returns at once and this reports
        // "no matches" for a caller who asked to wait as long as it takes. The deadline is
        // saturated and the wait sliced (see detail/bounded_wait.h). Never less than
        // min_attempt_time, as before.
        //
        // Reserving the settle time through the helper rather than subtracting it here:
        // `timeout_point - settle_time` converts the settle time into the clock's own ticks
        // first, so a near-max one overflows and lands in the past -- undoing the saturation
        // above on the very next line.
        const auto match_deadline = detail::deadline_reserving<std::chrono::steady_clock>(
            timeout_point, settle_time, std::chrono::steady_clock::now() + min_attempt_time);
        if (!detail::wait_until_bounded(data_listener->num_matched_publishers_cv, lock, match_deadline,
                                        [this] { return data_listener->num_matched_publishers > 0; }))
        {
            // No matches
            return 0;
        }

        if (settle_time.count() > 0)
        {
            do
            {
                if (!detail::wait_until_bounded(
                        data_listener->num_matched_publishers_cv, lock,
                        detail::saturating_deadline<std::chrono::steady_clock>(settle_time),
                        [this, num_matched_publishers_was = data_listener->num_matched_publishers] {
                            return num_matched_publishers_was != data_listener->num_matched_publishers;
                        }))
                {
                    // No change during the settle_time period
                    return data_listener->num_matched_publishers;
                }
            } while (std::chrono::steady_clock::now() < timeout_point);

            // Wasn't ever stable for settle_time until the timeout
            return -1;
        }

        return data_listener->num_matched_publishers;
    }

    namespace detail
    {
        template <typename data_type, typename on_data_function_type>
        class on_data_function_data_listener : public data_reader_listener
        {
          public:
            on_data_function_data_listener(on_data_function_type &&on_data_function)
                : on_data_function(std::move(on_data_function))
            {
            }

            void on_data_available(DataReader *reader) override
            {
                listener_drain::scoped_call call{drain};
                if (!call.should_run())
                {
                    return;
                }

                data_type data;
                SampleInfo info;
                while (reader->take_next_sample(&data, &info) == RETCODE_OK)
                {
                    if (info.valid_data)
                    {
                        constexpr size_t arity = function_traits<on_data_function_type>::arity;
                        static_assert(arity == 1 || arity == 2, "Incorrect number of arguments in on_data_function");
                        // A throwing user callback must not escape into the Fast-DDS reception
                        // thread — it would propagate into Fast-DDS and std::terminate the
                        // process. Report it through the configurable logger and carry on with
                        // the next sample.
                        try
                        {
                            if constexpr (arity == 1)
                            {
                                on_data_function(data);
                            }
                            else
                            {
                                on_data_function(data, info);
                            }
                        }
                        catch (const std::exception &exception)
                        {
                            // The logging path itself can allocate and throw (std::bad_alloc on
                            // ostringstream growth); guard it so nothing escapes into the Fast-DDS
                            // reception thread. Mirrors the discovery dispatch path.
                            try
                            {
                                log_error() << "subscriber on_data callback threw: " << exception.what();
                            }
                            catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                            {
                            }
                        }
                        catch (...)
                        {
                            try
                            {
                                log_error() << "subscriber on_data callback threw a non-std::exception";
                            }
                            catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                            {
                            }
                        }
                    }
                }
            }

          private:
            on_data_function_type on_data_function;
        };
    }  // namespace detail

    template <typename data_pub_sub_type, typename on_data_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function, const ReliabilityQosPolicyKind reliability_kind,
        const std::int32_t max_history_depth, std::optional<DurabilityQosPolicyKind> durability_kind)
    {
        // shared_ptr(new T(...)) rather than make_shared so the constructor's
        // accessibility check happens in this function (a friend of
        // subscriber_handle) instead of inside std::make_shared's internals.
        std::shared_ptr<subscriber_handle<data_pub_sub_type>> handle{new subscriber_handle<data_pub_sub_type>(
            std::move(participant), topic_name,
            std::make_shared<
                detail::on_data_function_data_listener<typename data_pub_sub_type::type, on_data_function_type>>(
                std::move(on_data_function)),
            reliability_kind, max_history_depth, durability_kind)};
        handle->participant->register_endpoint(handle);
        return handle;
    }

    namespace detail
    {
        template <typename data_type, typename on_data_function_type, typename on_has_publisher_changed_function_type>
        class functional_data_listener : public on_data_function_data_listener<data_type, on_data_function_type>
        {
          public:
            functional_data_listener(on_data_function_type &&on_data_function,
                                     on_has_publisher_changed_function_type &&on_has_publisher_changed_function)
                : on_data_function_data_listener<data_type, on_data_function_type>(std::move(on_data_function)),
                  on_has_publisher_changed_function(std::move(on_has_publisher_changed_function))
            {
            }

            void on_subscription_matched(DataReader *reader, const SubscriptionMatchedStatus &info) override
            {
                // Outer scoped_call protects the user callback below: it keeps
                // the in-flight counter > 0 across the whole method body so a
                // reset's detach_and_drain() blocks until we're done — even if
                // the user callback re-enters provizio APIs that need the
                // lifecycle lock shared. The base's own scoped_call nests
                // inside (counter briefly goes to 2) which is intentional and
                // harmless.
                listener_drain::scoped_call call{this->drain};
                if (!call.should_run())
                {
                    return;
                }

                on_data_function_data_listener<data_type, on_data_function_type>::on_subscription_matched(reader, info);

                // A throwing user callback must not escape into the Fast-DDS listener
                // thread — it would std::terminate the process. Report it through the
                // configurable logger and continue.
                try
                {
                    if (info.current_count > 0 && info.current_count_change == info.current_count)
                    {
                        // Just matched the first publisher
                        on_has_publisher_changed_function(true);
                    }
                    else if (info.current_count == 0 && info.current_count_change < 0)
                    {
                        // Just unmatched the last publisher
                        on_has_publisher_changed_function(false);
                    }
                }
                catch (const std::exception &exception)
                {
                    // Guard the logging too — it can throw std::bad_alloc on stream growth,
                    // which must not escape into the Fast-DDS listener thread.
                    try
                    {
                        log_error() << "subscriber on_has_publisher_changed callback threw: " << exception.what();
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                    {
                    }
                }
                catch (...)
                {
                    try
                    {
                        log_error() << "subscriber on_has_publisher_changed callback threw a non-std::exception";
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                    {
                    }
                }
            }

          private:
            on_has_publisher_changed_function_type on_has_publisher_changed_function;
        };
    }  // namespace detail

    template <typename data_pub_sub_type, typename on_data_function_type,
              typename on_has_publisher_changed_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function,
        on_has_publisher_changed_function_type on_has_publisher_changed_function,
        const ReliabilityQosPolicyKind reliability_kind, const std::int32_t max_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind)
    {
        // shared_ptr(new T(...)) rather than make_shared — see the comment in
        // the single-callback overload above.
        std::shared_ptr<subscriber_handle<data_pub_sub_type>> handle{new subscriber_handle<data_pub_sub_type>(
            std::move(participant), topic_name,
            std::make_shared<detail::functional_data_listener<typename data_pub_sub_type::type, on_data_function_type,
                                                              on_has_publisher_changed_function_type>>(
                std::move(on_data_function), std::move(on_has_publisher_changed_function)),
            reliability_kind, max_history_depth, durability_kind)};
        handle->participant->register_endpoint(handle);
        return handle;
    }
}  // namespace provizio::dds

#endif  // DDS_SUBSCRIBER
