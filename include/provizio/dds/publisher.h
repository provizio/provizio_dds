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
#include "provizio/dds/function_traits.h"
#include "provizio/dds/qos_defaults.h"

namespace provizio::dds
{
    using WriteParams = ::eprosima::fastrtps::rtps::WriteParams;

    namespace detail
    {
        template <typename data_pub_sub_type, typename on_matched_function_type = void *> class data_writer_listener;
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

        /**
         * @brief Publishes the DDS data with specific WriteParams.
         * @param data Actual DDS data to be published, f.e. std_msgs::msg::String
         * @param params Write parameters, used for advanced features like request-response correlation.
         * @return true if published successfully, false otherwise
         */
        virtual bool publish(data_type &data, WriteParams &params) = 0;

        /**
         * @brief Get the guid of the underlying DataWriter.
         * @return The GUID of the writer.
         */
        virtual guid get_guid() const = 0;
    };

    /**
     * @brief Encapsulates DDS Publisher and DataWriter functionality in a single entity with automatic life cycle
     * management. Optionally can be provided with a function or function object to be invoked on matching first /
     * umatching last subscriber. Normally created using provizio::dds::make_publisher.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_matched_function_type Optionally a function / function object type to be invoked on
     * matching first / umatching last subscriber. Takes two arguments: a reference to this publisher_handle and a
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
    class publisher_handle final : public data_publisher<data_pub_sub_type>
    {
      public:
        using data_type = typename data_publisher<data_pub_sub_type>::data_type;

      public:
        /**
         * @brief Constructs a new publisher_handle object.
         *
         * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
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
        publisher_handle(
            std::shared_ptr<domain_participant> participant, const std::string &topic_name,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind);

        /**
         * @brief Constructs a new publisher_handle object with an on_has_subscriber_changed function to be invoked
         * on matching first / umatching last subscriber.
         *
         * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
         * @param topic_name A DDS Topic Name
         * @param on_matched_function Function to be invoked on matching first / umatching last
         * (or any) subscriber. See on_matched_function_type documentation for details on accepted arguments.
         * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS
         * DataWriter, which makes publishing slower but more reliable
         * @note Using BEST_EFFORT_RELIABILITY_QOS reliability_kind makes it incompatible with reliable subscribers
         * @see provizio::dds::make_publisher
         * @see provizio::dds::make_domain_participant
         * @see provizio::dds::publisher_policies
         * @see
         * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
         */
        publisher_handle(
            std::shared_ptr<domain_participant> participant, const std::string &topic_name,
            on_matched_function_type on_matched_function,
            ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind);
        ~publisher_handle();

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

      private:
        publisher_handle(std::shared_ptr<domain_participant> participant, const std::string &topic_name,
                         on_matched_function_type on_matched_function, std::unique_ptr<DataWriterListener> &&listener,
                         ReliabilityQosPolicyKind reliability_kind);

        std::shared_ptr<domain_participant> participant;
        dds::TypeSupport type_support;
        on_matched_function_type on_matched_function;
        std::unique_ptr<DataWriterListener> listener;
        std::shared_ptr<topic> the_topic;
        Publisher *publisher = nullptr;
        DataWriter *data_writer = nullptr;

        friend class detail::data_writer_listener<data_pub_sub_type, on_matched_function_type>;
    };

    /**
     * @brief Creates a new publisher_handle object as a shared_ptr. The publisher_handle is automatically
     * deleted correctly on destroying its last shared_ptr.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
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
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind)
    {
        return std::make_shared<publisher_handle<data_pub_sub_type>>(std::move(participant), topic_name,
                                                                     reliability_kind);
    }

    /**
     * @brief Creates a new publisher_handle object as a shared_ptr with an on_has_subscriber_changed function to be
     * invoked on matching first / umatching last subscriber. The publisher_handle is automatically deleted
     * correctly on destroying its last shared_ptr.
     *
     * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
     * @tparam on_matched_function_type Type of function to be invoked on matching first / umatching
     * last (or any) subscriber. See on_matched_function_type documentation for details on accepted arguments.
     * Usually the function type is auto-detected from the provided argument value.
     * @param participant A DDS Domain Participant, as created by provizio::dds::make_domain_participant
     * @param topic_name A DDS Topic Name
     * @param on_matched_function The on_has_subscriber_changed function
     * @param reliability_kind Defines whether RELIABLE_RELIABILITY_QOS should be enabled for the DDS DataWriter,
     * which makes publishing slower but more reliable
     * @return std::shared_ptr to the created publisher_handle
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
        ReliabilityQosPolicyKind reliability_kind = qos_defaults<data_pub_sub_type>::datawriter_reliability_kind)
    {
        return std::make_shared<publisher_handle<data_pub_sub_type, on_matched_function_type>>(
            std::move(participant), topic_name, std::move(on_matched_function), reliability_kind);
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
                constexpr size_t arity = function_traits<on_matched_function_type>::arity;
                if constexpr (arity == 2)
                {
                    if (info.current_count > 0 && info.current_count_change == info.current_count)
                    {
                        // Just matched the first publisher
                        publisher.on_matched_function(publisher, true);
                    }
                    else if (info.current_count == 0 && info.current_count_change < 0)
                    {
                        // Just unmatched the last publisher
                        publisher.on_matched_function(publisher, false);
                    }
                }
                else
                {
                    publisher.on_matched_function(publisher, info.current_count_change > 0,
                                                  static_cast<const guid &>(info.last_subscription_handle));
                }
            }

          private:
            publisher_handle<data_pub_sub_type, on_matched_function_type> &publisher;
        };
    } // namespace detail

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::publisher_handle(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        const ReliabilityQosPolicyKind reliability_kind)
        : publisher_handle(std::move(participant), topic_name, nullptr, std::unique_ptr<DataWriterListener>{},
                           reliability_kind)
    {
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::publisher_handle(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_matched_function_type on_matched_function, const ReliabilityQosPolicyKind reliability_kind)
        : publisher_handle(
              std::move(participant), topic_name, std::move(on_matched_function),
              std::make_unique<detail::data_writer_listener<data_pub_sub_type, on_matched_function_type>>(*this),
              reliability_kind)
    {
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::publisher_handle(
        std::shared_ptr<domain_participant> participant, const std::string &topic_name,
        on_matched_function_type on_matched_function, std::unique_ptr<DataWriterListener> &&listener,
        const ReliabilityQosPolicyKind reliability_kind)
        : participant(std::move(participant)),
          type_support(this->participant->template register_type<data_pub_sub_type>()),
          on_matched_function(std::move(on_matched_function)), listener(std::move(listener))
    {
        const auto &topic_qos = TOPIC_QOS_DEFAULT;
        const auto &publisher_qos = PUBLISHER_QOS_DEFAULT;

        the_topic = this->participant->register_topic(topic_name, type_support->getName(), topic_qos);
        publisher = this->participant->fastdds_participant().create_publisher(publisher_qos);

        DataWriterQos datawriter_qos;
        publisher->get_default_datawriter_qos(datawriter_qos);
        datawriter_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        datawriter_qos.history().kind = KEEP_LAST_HISTORY_QOS;
        datawriter_qos.history().depth = 1;
        datawriter_qos.reliability().kind = reliability_kind;
        datawriter_qos.endpoint().history_memory_policy = qos_defaults<data_pub_sub_type>::memory_policy;

        data_writer = publisher->create_datawriter(the_topic->get(), datawriter_qos, this->listener.get());
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    publisher_handle<data_pub_sub_type, on_matched_function_type>::~publisher_handle()
    {
        if (data_writer != nullptr)
        {
            publisher->delete_datawriter(data_writer);
        }

        if (publisher != nullptr)
        {
            participant->fastdds_participant().delete_publisher(publisher);
        }

        the_topic.reset();
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    bool publisher_handle<data_pub_sub_type, on_matched_function_type>::publish(data_type &data)
    {
        return data_writer->write(&data);
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    bool publisher_handle<data_pub_sub_type, on_matched_function_type>::publish(data_type &data, WriteParams &params)
    {
        return data_writer->write(&data, params);
    }

    template <typename data_pub_sub_type, typename on_matched_function_type>
    guid publisher_handle<data_pub_sub_type, on_matched_function_type>::get_guid() const
    {
        return data_writer->guid();
    }
} // namespace provizio::dds

#endif // DDS_PUBLISHER
