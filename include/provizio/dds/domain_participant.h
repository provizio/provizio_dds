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

#include <fastdds/dds/domain/DomainParticipant.hpp>

#include "provizio/dds/common.h"

namespace provizio
{
    namespace dds
    {
        /**
         * @brief Creates a new DDS Domain Participant as a shared_ptr. The participant is automatically deleted
         * correctly on destroying its last shared_ptr.
         *
         * @param domain_id domain_id, 0 by default
         * @return std::shared_ptr<DomainParticipant>
         * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
         * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/domain/domainparticipant.html
         */
        std::shared_ptr<DomainParticipant> make_domain_participant(DomainId_t domain_id = 0);
    } // namespace dds
} // namespace provizio

#endif // DDS_DOMAIN_PARTICIPANT
