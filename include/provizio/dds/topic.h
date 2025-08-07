// Copyright 2025 Provizio Ltd.
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

#ifndef DDS_TOPIC
#define DDS_TOPIC

#include <mutex>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include "provizio/dds/common.h"

namespace provizio::dds
{
    /**
     * @brief RAII wrapper for a Topic, automatically unregistering it from the DomainParticipant on destruction.
     * @see provizio::dds::domain_participant::register_topic
     */
    class topic
    {
      public:
        /**
         * @brief Construct a new topic object. For internal use only.
         * @note To register a topic, use provizio::dds::domain_participant::register_topic.
         *
         * @param participant Owning DomainParticipant.
         * @param registered_topics_mutex Mutex from the owning DomainParticipant to synchronise access to its
         * topics container.
         * @param the_topic Topic to own.
         * @param qos QoS of the owned topic.
         */
        topic(eprosima::fastdds::dds::DomainParticipant *participant, std::mutex &registered_topics_mutex,
              Topic *the_topic, TopicQos qos);
        /**
         * @brief Destroy the topic object, unregistering the topic from the DomainParticipant.
         */
        ~topic();

        /**
         * @brief Get the owned Topic.
         * @return Topic*.
         */
        inline Topic *get()
        {
            return the_topic;
        }

        /**
         * @brief Get the QoS of the owned topic.
         * @return const TopicQos&.
         */
        inline const TopicQos &qos()
        {
            return the_qos;
        }

        /**
         * @brief Releases the ownership of the topic, so it won't be unregistered on destruction.
         * For internal use only.
         */
        inline void release()
        {
            participant = nullptr;
        }

      private:
        eprosima::fastdds::dds::DomainParticipant *participant;
        std::mutex &registered_topics_mutex;
        Topic *const the_topic;
        const TopicQos the_qos;
    };
} // namespace provizio::dds

#endif // DDS_TOPIC
