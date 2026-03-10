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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/function_traits.h"
#include "provizio/dds/qos_defaults.h"

namespace provizio::dds
{
    /**
     * @file subscriber.h
     * @brief RAII subscriber wrappers and helpers for receiving DDS data.
     */

    template <typename data_pub_sub_type> class subscriber_handle;
    /**
     * @brief Internal DataReaderListener that tracks publisher match counts for readiness helpers.
     *
     * Kept header-only (no PROVIZIO_DDS_API) to avoid exporting std::mutex /
     * std::condition_variable across the DLL boundary.
     */
    class data_reader_listener : public DataReaderListener
    {
      public:
        void on_subscription_matched(DataReader *reader, const SubscriptionMatchedStatus &info) override
        {
            (void)reader;
            {
                const std::lock_guard<std::mutex> lock{num_matched_publishers_mutex};
                num_matched_publishers = info.current_count;
            }
            num_matched_publishers_cv.notify_all();
        }

      private:
        mutable std::mutex num_matched_publishers_mutex;
        mutable std::condition_variable num_matched_publishers_cv;
        int num_matched_publishers{0};

        template <typename data_pub_sub_type> friend class subscriber_handle;
    };

    /**
     * @brief Encapsulates DDS Subscriber and DataReader functionality in a single entity with automatic life cycle
     * management. Normally created with provizio::dds::make_subscriber.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @see provizio::dds::make_subscriber
     */
    template <typename data_pub_sub_type> class subscriber_handle final
    {
      public:
        using data_type = typename data_pub_sub_type::type;

      public:
        /**
         * @brief Constructs a new subscriber_handle object.
         *
         * @param participant A DDS Domain Participant, as created by provizio::dds::make_participant
         * @param topic_name A DDS Topic Name
         * @param data_listener A DDS DataReaderListener as a shared_ptr
         * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS
         * DataReader, which makes receiving data slower but more reliable
         * @param max_history_depth The maximum number of samples to keep in the history. Use 0 for unlimited, -1
         * for default.
         * @see provizio::dds::make_subscriber
         * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/subscriber/dataReaderListener/dataReaderListener.html
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        subscriber_handle(
            std::shared_ptr<domain_participant> participant, const std::string &topic_name,
            std::shared_ptr<data_reader_listener> data_listener,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datareader_reliability_kind,
            std::int32_t max_history_depth = use_default_qos_durability);
        ~subscriber_handle();

        /**
         * @brief Returns GUID of the underlying DataReader.
         */
        guid get_guid() const;

        /**
         * @brief Blocks until this subscriber has at least one stable match for a short settle window.
         * @param timeout Total timeout duration, takes a single attempt when 0.
         * @param settle_time The minimum time the match must remain stable (no further status changes) to be
         * considered "ready".
         * @return non-negative number of matched publishers if stable within timeout, -1 otherwise.
         */
        int get_num_matched_publishers(std::chrono::milliseconds timeout, std::chrono::milliseconds settle_time) const;

      private:
        std::shared_ptr<domain_participant> participant;
        dds::TypeSupport type_support;
        std::shared_ptr<data_reader_listener> data_listener;
        std::shared_ptr<topic> the_topic;
        Subscriber *subscriber = nullptr;
        DataReader *data_reader = nullptr;
    };

    /**
     * @brief Creates a new subscriber_handle object as a shared_ptr with a function / function object to be invoked
     * on receiving data. The subscriber_handle is automatically deleted correctly on destroying its last
     * shared_ptr. Usually the function type is auto-detected from the provided argument value.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_data_function_type Type of a function / function object to be invoked on receiving data.
     * It can take one argument: a const reference to the data type (e.g., `const std_msgs::msg::String&`).
     * Alternatively, it can take a second argument: a const reference to `provizio::dds::SampleInfo`.
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_participant
     * @param topic_name A DDS Topic Name
     * @param on_data_function Function / function object to be invoked on receiving data
     * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataReader,
     * which makes receiving data slower but more reliable
     * @param max_history_depth The maximum number of samples to keep in the history. Use 0 for unlimited, -1
     * for default.
     * @return std::shared_ptr to the created subscriber_handle
     * @see provizio::dds::subscriber_handle
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     */
    template <typename data_pub_sub_type, typename on_data_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function,
        ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datareader_reliability_kind,
        std::int32_t max_history_depth = use_default_qos_durability);

    /**
     * @brief Creates a new subscriber_handle object as a shared_ptr with a function / function object to be invoked
     * on receiving data and another function / function object to be invoked on matching first /
     * umatching last publisher. The subscriber_handle is automatically deleted correctly on destroying its last
     * shared_ptr. Usually the function types are auto-detected from the provided argument values.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_data_function_type Type of a function / function object to be invoked on receiving data.
     * See the other `make_subscriber` overload for details.
     * @tparam on_has_publisher_changed_function_type Type of a function / function object to be invoked on matching
     * first / umatching last publisher, takes a single bool argument: true when the first publisher is matched,
     * false when the last publisher is unmatched
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_participant
     * @param topic_name A DDS Topic Name
     * @param on_data_function Function / function object to be invoked on receiving data
     * @param on_has_publisher_changed_function The on_has_publisher_changed function
     * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataReader,
     * which makes receiving data slower but more reliable
     * @param max_history_depth The maximum number of samples to keep in the history. Use 0 for unlimited, -1
     * for default.
     * @return std::shared_ptr to the created subscriber_handle
     * @see provizio::dds::subscriber_handle
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
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
                                                            std::shared_ptr<data_reader_listener> data_listener,
                                                            const ReliabilityQosPolicyKind reliability_kind,
                                                            const std::int32_t max_history_depth)
        : participant(std::move(participant)),
          type_support(this->participant->template register_type<data_pub_sub_type>()),
          data_listener(std::move(data_listener))
    {
        const auto &topic_qos = TOPIC_QOS_DEFAULT;
        const auto &subscriber_qos = SUBSCRIBER_QOS_DEFAULT;

        the_topic = this->participant->register_topic(topic_name, type_support->getName(), topic_qos);
        subscriber = this->participant->fastdds_participant().create_subscriber(subscriber_qos);

        DataReaderQos datareader_qos;
        subscriber->get_default_datareader_qos(datareader_qos);
        datareader_qos.reliability().kind = reliability_kind;
        datareader_qos.endpoint().history_memory_policy = qos_defaults<data_pub_sub_type>::memory_policy;
        if (max_history_depth == use_default_qos_durability)
        {
            // Keep defaults
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
        // Disable data sharing on Windows and macOS: mirrors the DataWriter-side
        // disable in publisher.h — see the comment there for the full rationale.
        datareader_qos.data_sharing().off();
#endif

        data_reader = subscriber->create_datareader(the_topic->get(), datareader_qos, this->data_listener.get());
    }

    template <typename data_pub_sub_type> subscriber_handle<data_pub_sub_type>::~subscriber_handle()
    {
        if (data_reader != nullptr)
        {
            subscriber->delete_datareader(data_reader);
        }

        if (subscriber != nullptr)
        {
            participant->fastdds_participant().delete_subscriber(subscriber);
        }

        the_topic.reset();
    }

    template <typename data_pub_sub_type> guid subscriber_handle<data_pub_sub_type>::get_guid() const
    {
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
                data_type data;
                SampleInfo info;
                while (reader->take_next_sample(&data, &info) == ReturnCode_t::RETCODE_OK)
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
    } // namespace detail

    template <typename data_pub_sub_type, typename on_data_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function, const ReliabilityQosPolicyKind reliability_kind,
        const std::int32_t max_history_depth)
    {
        return std::make_shared<subscriber_handle<data_pub_sub_type>>(
            std::move(participant), topic_name,
            std::make_shared<
                detail::on_data_function_data_listener<typename data_pub_sub_type::type, on_data_function_type>>(
                std::move(on_data_function)),
            reliability_kind, max_history_depth);
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
    } // namespace detail

    template <typename data_pub_sub_type, typename on_data_function_type,
              typename on_has_publisher_changed_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function,
        on_has_publisher_changed_function_type on_has_publisher_changed_function,
        const ReliabilityQosPolicyKind reliability_kind, const std::int32_t max_history_depth)
    {
        return std::make_shared<subscriber_handle<data_pub_sub_type>>(
            std::move(participant), topic_name,
            std::make_shared<detail::functional_data_listener<typename data_pub_sub_type::type, on_data_function_type,
                                                              on_has_publisher_changed_function_type>>(
                std::move(on_data_function), std::move(on_has_publisher_changed_function)),
            reliability_kind, max_history_depth);
    }
} // namespace provizio::dds

#endif // DDS_SUBSCRIBER
