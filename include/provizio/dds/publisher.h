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

namespace provizio
{
    namespace dds
    {
        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type = void *>
        class data_writer_listener;

        template <typename data_pub_sub_type> struct publisher_policies final
        {
            /**
             * @brief Defines whether to use reliable data writer DDS QOS policies.
             * @sa
             * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
             */
            static constexpr bool default_reliable_qos = true;
        };

        template <typename data_pub_sub_type> class data_publisher
        {
          public:
            using data_type = typename data_pub_sub_type::type;

          public:
            virtual ~data_publisher() = default;
            virtual bool publish(data_type &data) = 0;
        };

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type = void *>
        class publisher_handle final : public data_publisher<data_pub_sub_type>
        {
          public:
            using data_type = typename data_publisher<data_pub_sub_type>::data_type;

          public:
            publisher_handle(std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
                             bool reliable_qos = publisher_policies<data_pub_sub_type>::default_reliable_qos);
            publisher_handle(std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
                             on_has_publisher_changed_function_type on_has_publisher_changed_function,
                             bool reliable_qos = publisher_policies<data_pub_sub_type>::default_reliable_qos);
            ~publisher_handle();

            bool publish(data_type &data) override;

          private:
            publisher_handle(std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
                             on_has_publisher_changed_function_type on_has_publisher_changed_function,
                             std::unique_ptr<DataWriterListener> &&listener, bool reliable_qos);

            std::shared_ptr<DomainParticipant> domain_participant;
            dds::TypeSupport type_support;
            on_has_publisher_changed_function_type on_has_publisher_changed_function;
            std::unique_ptr<DataWriterListener> listener;
            Topic *topic = nullptr;
            Publisher *publisher = nullptr;
            DataWriter *data_writer = nullptr;

            friend class data_writer_listener<data_pub_sub_type, on_has_publisher_changed_function_type>;
        };

        template <typename data_pub_sub_type>
        std::shared_ptr<publisher_handle<data_pub_sub_type>> make_publisher(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            bool reliable_qos = publisher_policies<data_pub_sub_type>::default_reliable_qos)
        {
            return std::make_shared<publisher_handle<data_pub_sub_type>>(std::move(domain_participant), topic_name,
                                                                         reliable_qos);
        }

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type>
        std::shared_ptr<publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type>> make_publisher(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_has_publisher_changed_function_type on_has_publisher_changed_function,
            bool reliable_qos = publisher_policies<data_pub_sub_type>::default_reliable_qos)
        {
            return std::make_shared<publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type>>(
                std::move(domain_participant), topic_name, std::move(on_has_publisher_changed_function), reliable_qos);
        }

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type>
        class data_writer_listener : public DataWriterListener
        {
          public:
            data_writer_listener(publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type> &publisher)
                : publisher(publisher)
            {
            }

            void on_publication_matched(DataWriter *writer, const PublicationMatchedStatus &info) override
            {
                (void)writer;
                if (info.current_count > 0 && info.current_count_change == info.current_count)
                {
                    // Just matched the first publisher
                    publisher.on_has_publisher_changed_function(publisher, true);
                }
                else if (info.current_count == 0 && info.current_count_change < 0)
                {
                    // Just unmatched the last publisher
                    publisher.on_has_publisher_changed_function(publisher, false);
                }
            }

          private:
            publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type> &publisher;
        };

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type>::publisher_handle(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name, bool reliable_qos)
            : publisher_handle(std::move(domain_participant), topic_name, nullptr,
                               std::unique_ptr<DataWriterListener>{}, reliable_qos)
        {
        }

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type>::publisher_handle(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_has_publisher_changed_function_type on_has_publisher_changed_function, bool reliable_qos)
            : publisher_handle(
                  std::move(domain_participant), topic_name, std::move(on_has_publisher_changed_function),
                  std::make_unique<data_writer_listener<data_pub_sub_type, on_has_publisher_changed_function_type>>(
                      *this),
                  reliable_qos)
        {
        }

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type>::publisher_handle(
            std::shared_ptr<DomainParticipant> domain_participant, const std::string &topic_name,
            on_has_publisher_changed_function_type on_has_publisher_changed_function,
            std::unique_ptr<DataWriterListener> &&listener, bool reliable_qos)
            : domain_participant(std::move(domain_participant)), type_support(new data_pub_sub_type()),
              on_has_publisher_changed_function(std::move(on_has_publisher_changed_function)),
              listener(std::move(listener))
        {
            const auto &topic_qos = TOPIC_QOS_DEFAULT;
            const auto &publisher_qos = PUBLISHER_QOS_DEFAULT;
            auto datawriter_qos = DATAWRITER_QOS_DEFAULT;
            datawriter_qos.reliability().kind = reliable_qos ? RELIABLE_RELIABILITY_QOS : BEST_EFFORT_RELIABILITY_QOS;

            type_support.register_type(this->domain_participant.get());
            topic = this->domain_participant->create_topic(topic_name, type_support->getName(), topic_qos);
            publisher = this->domain_participant->create_publisher(publisher_qos);
            data_writer = publisher->create_datawriter(topic, datawriter_qos, this->listener.get());
        }

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type>
        publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type>::~publisher_handle()
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

        template <typename data_pub_sub_type, typename on_has_publisher_changed_function_type>
        bool publisher_handle<data_pub_sub_type, on_has_publisher_changed_function_type>::publish(data_type &data)
        {
            return data_writer->write(&data);
        }
    } // namespace dds
} // namespace provizio

#endif // DDS_PUBLISHER
