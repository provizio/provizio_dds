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

#include <memory>
#include <string>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/function_traits.h"
#include "provizio/dds/qos_defaults.h"

namespace provizio::dds
{
    constexpr std::int32_t use_default_qos_durability = -1;
    constexpr std::int32_t unlimited_history_depth = 0;

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
            std::shared_ptr<DataReaderListener> data_listener,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datareader_reliability_kind,
            std::int32_t max_history_depth = use_default_qos_durability);
        ~subscriber_handle();

        /**
         * @brief Get the guid of the underlying DataReader.
         * @return The GUID of the reader.
         */
        guid get_guid() const;

      private:
        std::shared_ptr<domain_participant> participant;
        dds::TypeSupport type_support;
        std::shared_ptr<DataReaderListener> data_listener;
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
                                                            std::shared_ptr<DataReaderListener> data_listener,
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
        if (max_history_depth > 0)
        {
            datareader_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
            datareader_qos.history().kind = HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
            datareader_qos.history().depth = max_history_depth;
        }
        else if (max_history_depth == 0)
        {
            datareader_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
            datareader_qos.history().kind = HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
        }
        // else keep datareader_qos.history() default

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

    template <typename data_type, typename on_data_function_type>
    class on_data_function_data_listener : public DataReaderListener
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
            if (reader->take_next_sample(&data, &info) == ReturnCode_t::RETCODE_OK && info.valid_data)
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

      private:
        on_data_function_type on_data_function;
    };

    template <typename data_pub_sub_type, typename on_data_function_type>
    std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_data_function_type on_data_function, const ReliabilityQosPolicyKind reliability_kind,
        const std::int32_t max_history_depth)
    {
        return std::make_shared<subscriber_handle<data_pub_sub_type>>(
            std::move(participant), topic_name,
            std::make_shared<on_data_function_data_listener<typename data_pub_sub_type::type, on_data_function_type>>(
                std::move(on_data_function)),
            reliability_kind, max_history_depth);
    }

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
            (void)reader;
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
            std::make_shared<functional_data_listener<typename data_pub_sub_type::type, on_data_function_type,
                                                      on_has_publisher_changed_function_type>>(
                std::move(on_data_function), std::move(on_has_publisher_changed_function)),
            reliability_kind, max_history_depth);
    }
} // namespace provizio::dds

#endif // DDS_SUBSCRIBER
