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

namespace provizio
{
    namespace dds
    {
        template <typename data_pub_sub_type> struct subscriber_policies final
        {
            /**
             * @brief Defines whether to use reliable data reader DDS QOS policies.
             * @sa
             * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
             */
            static constexpr bool default_reliable_qos = false;
        };

        template <typename data_pub_sub_type> class subscriber_handle final
        {
          public:
            using data_type = typename data_pub_sub_type::type;

          public:
            subscriber_handle(std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
                              std::shared_ptr<DataReaderListener> data_listener,
                              bool reliable_qos = subscriber_policies<data_pub_sub_type>::default_reliable_qos);
            ~subscriber_handle();

          private:
            std::shared_ptr<DomainParticipant> domain_participant;
            dds::TypeSupport type_support;
            std::shared_ptr<DataReaderListener> data_listener;
            Topic *topic = nullptr;
            Subscriber *subscriber = nullptr;
            DataReader *data_reader = nullptr;
        };

        template <typename data_pub_sub_type, typename on_data_function_type>
        std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_data_function_type on_data_function,
            bool reliable_qos = subscriber_policies<data_pub_sub_type>::default_reliable_qos);

        template <typename data_pub_sub_type, typename on_data_function_type,
                  typename on_has_publisher_changed_function_type>
        std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_data_function_type on_data_function,
            on_has_publisher_changed_function_type on_has_publisher_changed_function,
            bool reliable_qos = subscriber_policies<data_pub_sub_type>::default_reliable_qos);

        template <typename data_pub_sub_type>
        subscriber_handle<data_pub_sub_type>::subscriber_handle(std::shared_ptr<DomainParticipant> domain_participant,
                                                                const std::string &topic_name,
                                                                std::shared_ptr<DataReaderListener> data_listener,
                                                                const bool reliable_qos)
            : domain_participant(std::move(domain_participant)), type_support(new data_pub_sub_type()),
              data_listener(std::move(data_listener))
        {
            const auto &topic_qos = TOPIC_QOS_DEFAULT;
            const auto &subscriber_qos = SUBSCRIBER_QOS_DEFAULT;
            auto datareader_qos = DATAREADER_QOS_DEFAULT;
            datareader_qos.reliability().kind = reliable_qos ? RELIABLE_RELIABILITY_QOS : BEST_EFFORT_RELIABILITY_QOS;

            type_support.register_type(this->domain_participant.get());
            topic = this->domain_participant->create_topic(topic_name, type_support->getName(), topic_qos);
            subscriber = this->domain_participant->create_subscriber(subscriber_qos);
            data_reader = subscriber->create_datareader(topic, datareader_qos, this->data_listener.get());
        }

        template <typename data_pub_sub_type> subscriber_handle<data_pub_sub_type>::~subscriber_handle()
        {
            if (data_reader != nullptr)
            {
                subscriber->delete_datareader(data_reader);
            }

            if (subscriber != nullptr)
            {
                domain_participant->delete_subscriber(subscriber);
            }

            if (topic != nullptr)
            {
                domain_participant->delete_topic(topic);
            }
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
                SampleInfo info;
                data_type data;
                if (reader->take_next_sample(&data, &info) == ReturnCode_t::RETCODE_OK && info.valid_data)
                {
                    on_data_function(data);
                }
            }

          private:
            on_data_function_type on_data_function;
        };

        template <typename data_pub_sub_type, typename on_data_function_type>
        std::shared_ptr<subscriber_handle<data_pub_sub_type>> make_subscriber(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_data_function_type on_data_function, const bool reliable_qos)
        {
            return std::make_shared<subscriber_handle<data_pub_sub_type>>(
                std::move(domain_participant), topic_name,
                std::make_shared<
                    on_data_function_data_listener<typename data_pub_sub_type::type, on_data_function_type>>(
                    std::move(on_data_function)),
                reliable_qos);
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
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_data_function_type on_data_function,
            on_has_publisher_changed_function_type on_has_publisher_changed_function, const bool reliable_qos)
        {
            return std::make_shared<subscriber_handle<data_pub_sub_type>>(
                std::move(domain_participant), topic_name,
                std::make_shared<functional_data_listener<typename data_pub_sub_type::type, on_data_function_type,
                                                          on_has_publisher_changed_function_type>>(
                    std::move(on_data_function), std::move(on_has_publisher_changed_function)),
                reliable_qos);
        }
    } // namespace dds
} // namespace provizio

#endif // DDS_SUBSCRIBER
