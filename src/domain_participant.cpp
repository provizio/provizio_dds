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

#include "provizio/dds/domain_participant.h"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastrtps/types/TypesBase.h>
#include "provizio/dds/topic.h"

namespace provizio::dds
{
    namespace
    {
        // More reliable participants matching (only 5 multicast announcements are sent 0.1 seconds apart by default
        // and then only once in 3 seconds, which is often not enough when nearing 100% bandwidth load)
        const eprosima::fastrtps::Duration_t initial_announcements_period{0.05};       // NOLINT: Doesn't throw
        const eprosima::fastrtps::Duration_t lease_duration_announcement_period{1, 0}; // NOLINT: Doesn't throw
        constexpr std::uint32_t num_initial_discovery_announcements = 200;

        // In Fast-DDS 3 it's now DEFAULT_FASTDDS_ENV_VARIABLE and its value has changed from
        // FASTRTPS_DEFAULT_PROFILES_FILE to FASTDDS_DEFAULT_PROFILES_FILE. When upgrading, make sure to update it
        // in provizio_dds.py too.
        // Note: hardcoded instead of using eprosima::fastrtps::xmlparser::DEFAULT_FASTRTPS_ENV_VARIABLE
        // because that extern const lacks __declspec(dllimport) in Fast-DDS headers, causing LNK2019 on MSVC.
        constexpr char xml_profiles_env_variable[] = "FASTRTPS_DEFAULT_PROFILES_FILE";
    } // namespace

    domain_participant::domain_participant(const DomainId_t domain_id)
        : registered_topics_mutex(std::make_shared<std::mutex>())
    {
        DomainParticipantQos customized_qos;

        bool xml_profile = false;
        if (auto *const file_path = std::getenv(xml_profiles_env_variable)) // NOLINT: getenv required
        {
            xml_profile = std::filesystem::exists(file_path) && !std::filesystem::is_directory(file_path);
        }

        auto participant_factory = dds::DomainParticipantFactory::get_shared_instance();
        if (!xml_profile) // Unless configured via the XML profile
        {
            participant_factory->load_profiles();
            participant_factory->get_default_participant_qos(customized_qos);

            customized_qos.wire_protocol().builtin.discovery_config.initial_announcements.count =
                num_initial_discovery_announcements;
            customized_qos.wire_protocol().builtin.discovery_config.initial_announcements.period =
                initial_announcements_period;
            customized_qos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod =
                lease_duration_announcement_period;
        }

        participant = participant_factory->create_participant(
            domain_id, xml_profile ? PARTICIPANT_QOS_DEFAULT : customized_qos, nullptr);
    }

    domain_participant::~domain_participant()
    {
        {
            // Make sure neither of registered topic handles that are still alive won't try to unregister themselves
            // on destruction
            const std::lock_guard<std::mutex> lock{*registered_topics_mutex};
            for (auto &topic_pair : registered_topics)
            {
                auto handle = topic_pair.second.lock();
                if (handle)
                {
                    handle->release_mutex_prelocked();
                }
            }
        }

        DomainParticipantFactory::get_instance()->delete_participant(participant);
    }

    std::shared_ptr<topic> domain_participant::register_topic(const std::string &topic_name,
                                                              const std::string &type_name, const TopicQos &qos)
    {
        const std::lock_guard<std::mutex> lock{*registered_topics_mutex};
        const auto topic_iterator = registered_topics.find(topic_name);
        if (topic_iterator != registered_topics.end())
        {
            auto handle = topic_iterator->second.lock();
            if (handle)
            {
                // Ensure the type matches the already registered topic's type
                const std::string existing_type{handle->get()->get_type_name()};
                if (existing_type != type_name)
                {
                    throw std::runtime_error{"Topic " + topic_name +
                                             " has been already registered, but with a different type (existing: '" +
                                             existing_type + "', requested: '" + type_name + "')!"};
                }

                // Ensure QoS is the same
                if (!(handle->qos() == qos)) // Yep, TopicQos defines operator== but not operator!=
                {
                    throw std::runtime_error{"Topic " + topic_name +
                                             " has been already registered, but with a different QoS!"};
                }
                return handle;
            }
        }

        auto handle = std::make_shared<topic>(participant, registered_topics_mutex,
                                              participant->create_topic(topic_name, type_name, qos), qos);
        registered_topics[topic_name] = handle;
        return handle;
    }

    std::shared_ptr<domain_participant> make_domain_participant(const DomainId_t domain_id)
    {
        return std::make_shared<domain_participant>(domain_id);
    }
} // namespace provizio::dds
