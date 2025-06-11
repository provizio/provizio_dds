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
#include <memory>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastrtps/types/TypesBase.h>

namespace provizio // NOLINT: nesting namespace old-school way to support C++14
{
    namespace dds
    {
        namespace
        {
            const eprosima::fastrtps::Duration_t initial_announcements_period{0.05};      // NOLINT: Doesn't throw
            const eprosima::fastrtps::Duration_t lease_duration_announcement_period{0.5}; // NOLINT: Doesn't throw
            constexpr std::uint32_t num_initial_discovery_announcements = 200;
        } // namespace

        domain_participant::domain_participant(const DomainId_t domain_id)
        {
            auto qos = PARTICIPANT_QOS_DEFAULT;

            // More reliable matching (only 5 multicast announcements are sent 0.1 seconds apart by default and then
            // only once in 3 seconds, which is often not enough when nearing 100% bandwidth load)
            qos.wire_protocol().builtin.discovery_config.initial_announcements.count =
                num_initial_discovery_announcements;
            qos.wire_protocol().builtin.discovery_config.initial_announcements.period = initial_announcements_period;
            qos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod =
                lease_duration_announcement_period;

            participant = dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, qos, nullptr);
        }

        domain_participant::~domain_participant()
        {
            DomainParticipantFactory::get_instance()->delete_participant(participant);
        }

        std::shared_ptr<domain_participant> make_domain_participant(const DomainId_t domain_id)
        {
            return std::make_shared<domain_participant>(domain_id);
        }
    } // namespace dds
} // namespace provizio
