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

#ifndef DDS_DOMAIN_PARTICIPANT
#define DDS_DOMAIN_PARTICIPANT

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/topic.h"

namespace provizio::dds
{
    /**
     * @file domain_participant.h
     * @brief Wrapper over eprosima::fastdds::dds::DomainParticipant taking care of TypeSupport and Topic
     * registration.
     *
     * @note register_type and register_topic do so just once per type/topic name, any consequent registration
     * simply reuses the registered TypeSupport/Topic.
     */
    class domain_participant
    {
      public:
        /**
         * @brief Construct a new domain participant object
         *
         * @param domain_id DDS domain_id, 0 by default
         */
        domain_participant(DomainId_t domain_id = 0);
        ~domain_participant();

        /**
         * @brief Registers the type in the domain participant, only once per domain participant, thread-safe.
         *
         * @tparam data_pub_sub_type PubSub type to register
         * @return TypeSupport that has been registered in the domain participant
         */
        template <typename data_pub_sub_type> TypeSupport register_type();

        /**
         * @brief Registers the topic in the domain participant, only once per domain participant, thread-safe.
         *
         * @param topic_name Name of the topic.
         * @param type_name Name of the type.
         * @param qos QoS profile for the topic.
         * @return std::shared_ptr<topic> A handle to the registered topic.
         * @throws std::runtime_error if the topic has been already registered with a different QoS.
         */
        std::shared_ptr<topic> register_topic(const std::string &topic_name, const std::string &type_name,
                                              const TopicQos &qos);

        /**
         * @brief Returns the underlying Fast-DDS DomainParticipant
         *
         * @return eprosima::fastdds::dds::DomainParticipant&
         */
        inline eprosima::fastdds::dds::DomainParticipant &fastdds_participant()
        {
            return *participant;
        }

      private:
        eprosima::fastdds::dds::DomainParticipant *participant;
        std::mutex registered_types_mutex;
        std::unordered_map<std::string, TypeSupport> registered_types;
        std::shared_ptr<std::mutex> registered_topics_mutex;
        std::unordered_map<std::string, std::weak_ptr<topic>> registered_topics;
    };
    using DomainParticipant = domain_participant; // To match DDS domain participant name, as previously used directly

    template <typename data_pub_sub_type> TypeSupport domain_participant::register_type()
    {
        const std::lock_guard<std::mutex> lock{registered_types_mutex};

        auto pub_sub = std::make_unique<data_pub_sub_type>();
        const auto type_it = registered_types.find(pub_sub->getName());
        if (type_it != registered_types.end())
        {
            return type_it->second;
        }
        else
        {
            TypeSupport type_support{pub_sub.release()};
            participant->register_type(type_support);
            registered_types.insert({type_support->getName(), type_support});
            return type_support;
        }
    }

    /**
     * @brief Creates a new DDS Domain Participant as a shared_ptr. The participant is automatically deleted
     * correctly on destroying its last shared_ptr.
     *
     * @param domain_id DDS domain_id, 0 by default
     * @return std::shared_ptr<domain_participant>
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/domain/domainparticipant.html
     */
    std::shared_ptr<domain_participant> make_domain_participant(DomainId_t domain_id = 0);
} // namespace provizio::dds

#endif // DDS_DOMAIN_PARTICIPANT
