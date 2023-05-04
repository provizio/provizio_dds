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

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

namespace provizio
{
    namespace dds
    {
        namespace
        {
            void delete_participant(dds::DomainParticipant *participant)
            {
                DomainParticipantFactory::get_instance()->delete_participant(participant);
            }
        } // namespace

        std::shared_ptr<DomainParticipant> make_domain_participant(DomainId_t domain_id)
        {
            return {dds::DomainParticipantFactory::get_instance()->create_participant(domain_id,
                                                                                      PARTICIPANT_QOS_DEFAULT, nullptr),
                    delete_participant};
        }
    } // namespace dds
} // namespace provizio
