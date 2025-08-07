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

#include "provizio/dds/request_response_details.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <limits>

#include "provizio/dds/common.h"
#include "provizio/dds/subscriber.h"

namespace provizio::dds::detail
{
    // NOLINTBEGIN: Can't throw
    const std::string request_prefix{"rq/"};
    const std::string response_prefix{"rr/"};
    const std::string request_suffix{"Request"};
    const std::string response_suffix{"Reply"};
    const std::string requests_queue_full_error_message{
        "provizio_dds: The service requests queue is full! A request will be dropped."};
    // NOLINTEND

    std::size_t to_max_queue_size(const std::int32_t max_history_depth)
    {
        constexpr std::size_t default_queue_size = 10;

        if (max_history_depth > 0)
        {
            return static_cast<std::size_t>(max_history_depth);
        }
        if (max_history_depth == unlimited_history_depth)
        {
            return std::numeric_limits<std::size_t>::max();
        }

        // Default queue size
        return default_queue_size;
    }

    bool is_subscriber_guid(const guid &guid_to_check)
    {
        constexpr std::uint8_t subscriber_guid_test_bitmask = 0x04;
        constexpr std::size_t guid_test_byte_index = 3;
        return (guid_to_check.entityId.value[guid_test_byte_index] & subscriber_guid_test_bitmask) != 0;
    }

    std::size_t guid_hash::operator()(const guid &the_guid) const
    {
        constexpr std::size_t guid_size = 16;
        constexpr std::size_t size_in_uints = guid_size / sizeof(std::uint32_t);
        static_assert(sizeof(the_guid) == guid_size, "Sizes mismatch");
        static_assert(size_in_uints * sizeof(std::uint32_t) == guid_size,
                      "guid_size must be dividable by sizeof(std::uint32_t)");
        static_assert(sizeof(std::array<std::uint32_t, size_in_uints>) == sizeof(the_guid), "Sizes mismatch");

        std::array<std::uint32_t, size_in_uints> plain_guid;  // NOLINT: Initialized next line
        std::memcpy(plain_guid.data(), &the_guid, guid_size); // NOLINT: memcpy is required here

        constexpr std::size_t prime_1 = 7;
        constexpr std::size_t prime_2 = 31;
        constexpr std::size_t prime_3 = 59;

        size_t hash = prime_1 * plain_guid[0];
        hash = prime_2 * (plain_guid[1] + hash);
        hash = prime_3 * (plain_guid[2] + hash);
        hash += plain_guid[3];

        return hash;
    }
} // namespace provizio::dds::detail
