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

#ifndef DDS_COMMON
#define DDS_COMMON

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/rtps/common/Guid.h>

namespace provizio::dds
{
    /**
     * @file common.h
     * @brief Common aliases and imports for Fast-DDS types used across the API.
     *
     * Brings eProsima Fast-DDS symbols into the `provizio::dds` namespace and
     * defines frequently used aliases such as `guid`.
     */
    /**
     * @brief Make Fast-DDS entities available in `provizio::dds`.
     */
    using namespace eprosima::fastdds::dds;

    /**
     * @brief Alias for Fast RTPS GUID type.
     */
    using guid = eprosima::fastrtps::rtps::GUID_t;
} // namespace provizio::dds

#endif // DDS_COMMON
