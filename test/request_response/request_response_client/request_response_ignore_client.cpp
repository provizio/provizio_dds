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

#include <array>
#include <chrono>
#include <iostream>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.h>
#include <std_msgs/msg/Int64PubSubTypes.h>

constexpr const char *log_prefix = "request_response_ignore_client: ";

int main()
try
{
    const std::string service_name{"provizio_dds_test_request_response_ignore"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr std::chrono::seconds time_to_match{2};
    constexpr std::chrono::seconds timeout{5};
    const std::array<int, 5> requests{1, 2, 3, 4, 5};

    auto participant = provizio::dds::make_domain_participant(domain_id);

    int received = 0;

    std::this_thread::sleep_for(time_to_match);

    for (const auto &value : requests)
    {
        std_msgs::msg::Int32 request;
        request.data(value);
        auto future = provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
            participant, service_name, request);

        if (value % 2 == 0)
        {
            // Ignored; should timeout
            if (future.wait_for(timeout) != std::future_status::timeout)
            {
                std::cerr << log_prefix << "Expected timeout for ignored request " << value << std::endl;
                return 1;
            }
        }
        else
        {
            if (future.wait_for(timeout) != std::future_status::ready)
            {
                std::cerr << log_prefix << "Timeout! Expected response for request " << value << std::endl;
                return 1;
            }
            std::cout << log_prefix << "Received response for request " << value << std::endl;

            const auto &reply = future.get();
            const auto expected = static_cast<std::int64_t>(value) * value;
            if (reply.data() != expected)
            {
                std::cerr << log_prefix << "Unexpected response for " << value << ": got " << reply.data()
                          << ", expected " << expected << std::endl;
                return 1;
            }
            ++received;
        }
    }

    if (received != 3)
    {
        std::cerr << log_prefix << "Expected 3 responses, got " << received << std::endl;
        return 1;
    }

    std::cout << log_prefix << "Successfully validated ignore_request" << std::endl;
    return 0;
}
catch (const std::exception &e)
{
    std::cerr << log_prefix << "Exception: " << e.what() << std::endl;
    return 1;
}
