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

#include "provizio/dds/topic.h"

#include <memory>
#include <mutex>
#include <utility>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

namespace provizio::dds
{
    topic::topic(eprosima::fastdds::dds::DomainParticipant *participant,
                 std::shared_ptr<std::mutex> registered_topics_mutex, Topic *the_topic, TopicQos qos)
        : participant(participant), registered_topics_mutex(std::move(registered_topics_mutex)), the_topic(the_topic),
          the_qos(std::move(qos))
    {
    }

    topic::~topic()
    {
        const std::lock_guard<std::mutex> lock{*registered_topics_mutex};
        if (participant != nullptr)
        {
            participant->delete_topic(the_topic);
        }
    }
} // namespace provizio::dds
