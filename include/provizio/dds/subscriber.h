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
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/detail/listener_drain.h"
#include "provizio/dds/detail/resettable_endpoint.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/function_traits.h"
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
         * @param max_history_depth Controls durability QoS: -1 keeps defaults, 0 forces VOLATILE (no history),
         * positive values enable TRANSIENT_LOCAL with KEEP_LAST of the given depth.
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
            std::int32_t max_history_depth = use_default_qos_durability);

        /// @internal Called by domain_participant during network-recovery reset.
        void detach_for_reset() noexcept override;
        void on_participant_reset(eprosima::fastdds::dds::DomainParticipant &old_participant) noexcept override;
        void on_new_participant_started(eprosima::fastdds::dds::DomainParticipant &new_participant) override;

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
        std::int32_t max_history_depth{use_default_qos_durability};

        // make_subscriber needs to register the freshly-constructed shared_ptr
        // with the participant; that requires reaching `participant`, which is
        // private. Friending the factory overloads keeps the registration call
        // site at make_subscriber (where the shared_ptr is born) without
        // exposing internals to user code.
        template <typename pub_sub_type, typename data_function_type>
        friend std::shared_ptr<subscriber_handle<pub_sub_type>> make_subscriber(std::shared_ptr<domain_participant>,
                                                                                const std::string &, data_function_type,
                                                                                ReliabilityQosPolicyKind, std::int32_t);

        template <typename pub_sub_type, typename data_function_type, typename publisher_changed_function_type>
        friend std::shared_ptr<subscriber_handle<pub_sub_type>> make_subscriber(std::shared_ptr<domain_participant>,
                                                                                const std::string &, data_function_type,
                                                                                publisher_changed_function_type,
                                                                                ReliabilityQosPolicyKind, std::int32_t);
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
     * @param max_history_depth Controls durability QoS: -1 keeps defaults, 0 forces VOLATILE (no history),
     * positive values enable TRANSIENT_LOCAL with KEEP_LAST of the given depth.
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
        std::int32_t max_history_depth = use_default_qos_durability);

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
     * @param max_history_depth Controls durability QoS: -1 keeps defaults, 0 forces VOLATILE (no history),
     * positive values enable TRANSIENT_LOCAL with KEEP_LAST of the given depth.
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
        std::int32_t max_history_depth = use_default_qos_durability);

    template <typename data_pub_sub_type>
    subscriber_handle<data_pub_sub_type>::subscriber_handle(std::shared_ptr<domain_participant> participant,
                                                            const std::string &topic_name,
                                                            std::shared_ptr<detail::data_reader_listener> data_listener,
                                                            const ReliabilityQosPolicyKind reliability_kind,
                                                            const std::int32_t max_history_depth)
        : participant(std::move(participant)),
          type_support(this->participant->template register_type<data_pub_sub_type>()),
          data_listener(std::move(data_listener)), captured_topic_name(topic_name), reliability_kind(reliability_kind),
          max_history_depth(max_history_depth)
    {
        // Intentionally NO build_state here — see publisher_handle ctor for the
        // rationale. The participant's register_endpoint() does the initial
        // build atomically with adding us to the recovery registry.
    }

    template <typename data_pub_sub_type>
    void subscriber_handle<data_pub_sub_type>::build_state(eprosima::fastdds::dds::DomainParticipant &on_participant)
    {
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
        datareader_qos.reliability().kind = reliability_kind;
        datareader_qos.endpoint().history_memory_policy = qos_defaults<data_pub_sub_type>::memory_policy;
        if (max_history_depth == use_default_qos_durability)
        {
            // keep defaults
        }
        else if (max_history_depth == no_history)
        {
            // Explicitly no history: VOLATILE
            datareader_qos.durability().kind = VOLATILE_DURABILITY_QOS;
        }
        else if (max_history_depth > 0)
        {
            datareader_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
            datareader_qos.history().kind = HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
            datareader_qos.history().depth = max_history_depth;
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
        std::unique_lock<std::mutex> lock{data_listener->num_matched_publishers_mutex};
        const auto timeout_point = std::chrono::steady_clock::now() + timeout;
        const std::chrono::milliseconds min_attempt_time{50};
        if (!data_listener->num_matched_publishers_cv.wait_for(
                lock, std::max(timeout - settle_time, min_attempt_time),
                [this] { return data_listener->num_matched_publishers > 0; }))
        {
            // No matches
            return 0;
        }

        if (settle_time.count() > 0)
        {
            do
            {
                if (!data_listener->num_matched_publishers_cv.wait_for(
                        lock, settle_time, [this, num_matched_publishers_was = data_listener->num_matched_publishers] {
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
                        if constexpr (arity == 1)
                        {
                            on_data_function(data);
                        }
                        else
                        {
                            on_data_function(data, info);
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
        const std::int32_t max_history_depth)
    {
        // shared_ptr(new T(...)) rather than make_shared so the constructor's
        // accessibility check happens in this function (a friend of
        // subscriber_handle) instead of inside std::make_shared's internals.
        std::shared_ptr<subscriber_handle<data_pub_sub_type>> handle{new subscriber_handle<data_pub_sub_type>(
            std::move(participant), topic_name,
            std::make_shared<
                detail::on_data_function_data_listener<typename data_pub_sub_type::type, on_data_function_type>>(
                std::move(on_data_function)),
            reliability_kind, max_history_depth)};
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
        const ReliabilityQosPolicyKind reliability_kind, const std::int32_t max_history_depth)
    {
        // shared_ptr(new T(...)) rather than make_shared — see the comment in
        // the single-callback overload above.
        std::shared_ptr<subscriber_handle<data_pub_sub_type>> handle{new subscriber_handle<data_pub_sub_type>(
            std::move(participant), topic_name,
            std::make_shared<detail::functional_data_listener<typename data_pub_sub_type::type, on_data_function_type,
                                                              on_has_publisher_changed_function_type>>(
                std::move(on_data_function), std::move(on_has_publisher_changed_function)),
            reliability_kind, max_history_depth)};
        handle->participant->register_endpoint(handle);
        return handle;
    }
}  // namespace provizio::dds

#endif  // DDS_SUBSCRIBER
