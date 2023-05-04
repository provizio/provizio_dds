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

#include <memory>
#include <string>

#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/qos_defaults.h"

namespace provizio
{
    namespace dds
    {
        namespace detail
        {
            template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type = void *>
            class data_writer_listener;
        } // namespace detail

        /**
         * @brief Abstract interface that provides publishing functionality for a DDS data type. Normally created using
         * provizio::dds::make_publisher.
         *
         * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
         *
         * @see provizio::dds::make_publisher
         * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/publisher.html
         */
        template <typename data_pub_sub_type> class data_publisher
        {
          public:
            using data_type = typename data_pub_sub_type::type;

          public:
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
        };

        /**
         * @brief Encapsulates DDS Publisher and DataWriter functionality in a single entity with automatic life cycle
         * management. Optionally can be provided with a function or function object to be invoked on matching first /
         * umatching last subscriber. Normally created using provizio::dds::make_publisher.
         *
         * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
         * @tparam on_has_subscriber_changed_function_type Optionally a function / function object type to be invoked on
         * matching first / umatching last subscriber. Takes two arguments: a reference to the publisher_handle and a
         * bool: true when the first subscriber is matched, false when the last subscriber is unmatched.
         * @see provizio::dds::make_publisher
         * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/publisher.html
         * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/dataWriter/dataWriter.html
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/dataWriterListener/dataWriterListener.html#dds-layer-publisher-datawriterlistener
         */
        template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type = void *>
        class publisher_handle final : public data_publisher<data_pub_sub_type>
        {
          public:
            using data_type = typename data_publisher<data_pub_sub_type>::data_type;

          public:
            /**
             * @brief Constructs a new publisher_handle object.
             *
             * @param domain_participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
             * @param topic_name A DDS Topic Name
             * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS
             * DataWriter, which makes publishing slower but more reliable
             * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
             * @see provizio::dds::make_publisher
             * @see provizio::dds::make_domain_participant
             * @see provizio::dds::publisher_policies
             * @see
             * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
             */
            publisher_handle(std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
                             ReliabilityQosPolicyKind reliability_kind =
                                 qos_defaults<data_pub_sub_type>::datawriter_reliability_kind);

            /**
             * @brief Constructs a new publisher_handle object with an on_has_subscriber_changed function to be invoked
             * on matching first / umatching last subscriber.
             *
             * @param domain_participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
             * @param topic_name A DDS Topic Name
             * @param on_has_subscriber_changed_function Function to be invoked on matching first / umatching last
             * subscriber, takes two arguments: a reference to the publisher_handle and a bool: true when the first
             * subscriber is matched, false when the last subscriber is unmatched.
             * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS
             * DataWriter, which makes publishing slower but more reliable
             * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
             * @see provizio::dds::make_publisher
             * @see provizio::dds::make_domain_participant
             * @see provizio::dds::publisher_policies
             * @see
             * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
             */
            publisher_handle(std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
                             on_has_subscriber_changed_function_type on_has_subscriber_changed_function,
                             ReliabilityQosPolicyKind reliability_kind =
                                 qos_defaults<data_pub_sub_type>::datawriter_reliability_kind);
            ~publisher_handle();

            /**
             * @brief Publishes the DDS data
             *
             * @param data Actual DDS data to be published, f.e. std_msgs::msg::String
             * @return true if published successfully, false otherwise
             */
            bool publish(data_type &data) override;

          private:
            publisher_handle(std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
                             on_has_subscriber_changed_function_type on_has_subscriber_changed_function,
                             std::unique_ptr<DataWriterListener> &&listener, ReliabilityQosPolicyKind reliability_kind);

            std::shared_ptr<DomainParticipant> domain_participant;
            dds::TypeSupport type_support;
            on_has_subscriber_changed_function_type on_has_subscriber_changed_function;
            std::unique_ptr<DataWriterListener> listener;
            Topic *topic = nullptr;
            Publisher *publisher = nullptr;
            DataWriter *data_writer = nullptr;

            friend class detail::data_writer_listener<data_pub_sub_type, on_has_subscriber_changed_function_type>;
        };

        /**
         * @brief Creates a new publisher_handle object as a shared_ptr. The publisher_handle is automatically
         * deleted correctly on destroying its last shared_ptr.
         *
         * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
         * @param domain_participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
         * @param topic_name A DDS Topic Name
         * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataWriter,
         * which makes publishing slower but more reliable
         * @return std::shared_ptr to the created publisher_handle
         * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
         * @see provizio::dds::publisher_handle
         * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        template <typename data_pub_sub_type>
        std::shared_ptr<publisher_handle<data_pub_sub_type>> make_publisher(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind)
        {
            return std::make_shared<publisher_handle<data_pub_sub_type>>(std::move(domain_participant), topic_name,
                                                                         reliability_kind);
        }

        /**
         * @brief Creates a new publisher_handle object as a shared_ptr with an on_has_subscriber_changed function to be
         * invoked on matching first / umatching last subscriber. The publisher_handle is automatically deleted
         * correctly on destroying its last shared_ptr.
         *
         * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
         * @tparam on_has_subscriber_changed_function_type Type of function to be invoked on matching first / umatching
         * last subscriber, takes two arguments: a reference to the publisher_handle and a bool: true when the first
         * subscriber is matched, false when the last subscriber is unmatched. Usually the function type is
         * auto-detected from the provided argument value.
         * @param domain_participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
         * @param topic_name A DDS Topic Name
         * @param on_has_subscriber_changed_function The on_has_subscriber_changed function
         * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataWriter,
         * which makes publishing slower but more reliable
         * @return std::shared_ptr to the created publisher_handle
         * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
         * @see provizio::dds::publisher_handle
         * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type>
        std::shared_ptr<publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type>> make_publisher(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_has_subscriber_changed_function_type on_has_subscriber_changed_function,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind)
        {
            return std::make_shared<publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type>>(
                std::move(domain_participant), topic_name, std::move(on_has_subscriber_changed_function),
                reliability_kind);
        }

        namespace detail
        {
            template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type>
            class data_writer_listener : public DataWriterListener
            {
              public:
                data_writer_listener(
                    publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type> &publisher)
                    : publisher(publisher)
                {
                }

                void on_publication_matched(DataWriter *writer, const PublicationMatchedStatus &info) override
                {
                    (void)writer;
                    if (info.current_count > 0 && info.current_count_change == info.current_count)
                    {
                        // Just matched the first publisher
                        publisher.on_has_subscriber_changed_function(publisher, true);
                    }
                    else if (info.current_count == 0 && info.current_count_change < 0)
                    {
                        // Just unmatched the last publisher
                        publisher.on_has_subscriber_changed_function(publisher, false);
                    }
                }

              private:
                publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type> &publisher;
            };
        } // namespace detail

        template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type>::publisher_handle(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            const ReliabilityQosPolicyKind reliability_kind)
            : publisher_handle(std::move(domain_participant), topic_name, nullptr,
                               std::unique_ptr<DataWriterListener>{}, reliability_kind)
        {
        }

        template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type>::publisher_handle(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_has_subscriber_changed_function_type on_has_subscriber_changed_function,
            const ReliabilityQosPolicyKind reliability_kind)
            : publisher_handle(
                  std::move(domain_participant), topic_name, std::move(on_has_subscriber_changed_function),
                  std::make_unique<
                      detail::data_writer_listener<data_pub_sub_type, on_has_subscriber_changed_function_type>>(*this),
                  reliability_kind)
        {
        }

        template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type>::publisher_handle(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_has_subscriber_changed_function_type on_has_subscriber_changed_function,
            std::unique_ptr<DataWriterListener> &&listener, const ReliabilityQosPolicyKind reliability_kind)
            : domain_participant(std::move(domain_participant)), type_support(new data_pub_sub_type()),
              on_has_subscriber_changed_function(std::move(on_has_subscriber_changed_function)),
              listener(std::move(listener))
        {
            const auto &topic_qos = TOPIC_QOS_DEFAULT;
            const auto &publisher_qos = PUBLISHER_QOS_DEFAULT;
            auto datawriter_qos = DATAWRITER_QOS_DEFAULT;
            datawriter_qos.reliability().kind = reliability_kind;
            datawriter_qos.endpoint().history_memory_policy = qos_defaults<data_pub_sub_type>::memory_policy;

            type_support.register_type(this->domain_participant.get());
            topic = this->domain_participant->create_topic(topic_name, type_support->getName(), topic_qos);
            publisher = this->domain_participant->create_publisher(publisher_qos);
            data_writer = publisher->create_datawriter(topic, datawriter_qos, this->listener.get());
        }

        template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type>::~publisher_handle()
        {
            if (data_writer != nullptr)
            {
                publisher->delete_datawriter(data_writer);
            }

            if (publisher != nullptr)
            {
                domain_participant->delete_publisher(publisher);
            }

            if (topic != nullptr)
            {
                domain_participant->delete_topic(topic);
            }
        }

        template <typename data_pub_sub_type, typename on_has_subscriber_changed_function_type>
        bool publisher_handle<data_pub_sub_type, on_has_subscriber_changed_function_type>::publish(data_type &data)
        {
            return data_writer->write(&data);
        }
    } // namespace dds
} // namespace provizio

#endif // DDS_PUBLISHER
