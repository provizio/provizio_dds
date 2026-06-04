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

#include <chrono>
#include <iostream>
#include <vector>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>
#include <std_msgs/msg/Int64PubSubTypes.hpp>

constexpr const char *log_prefix = "request_response_dual_service: ";

int main()
try
{
    using namespace std::chrono_literals;

    constexpr provizio::dds::DomainId_t domain_id = 14;
    const std::string service_name{"provizio_dds_test_request_response_dual"};

    auto participant = provizio::dds::make_domain_participant(domain_id);

    // Service #1: serves odd numbers only
    auto odd_service = provizio::dds::make_service<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
        participant, service_name, [](const std_msgs::msg::Int32 &request) {
            if ((request.data() % 2) == 0)
            {
                throw provizio::dds::ignore_request{};  // drop even requests here
            }
            std_msgs::msg::Int64 response;
            const std::int64_t value = static_cast<std::int64_t>(request.data()) * request.data();
            response.data(value);
            return response;
        });

    // Service #2: serves even numbers only
    auto even_service = provizio::dds::make_service<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
        participant, service_name, [](const std_msgs::msg::Int32 &request) {
            if ((request.data() % 2) != 0)
            {
                throw provizio::dds::ignore_request{};  // drop odd requests here
            }
            std_msgs::msg::Int64 response;
            const std::int64_t value = static_cast<std::int64_t>(request.data()) * request.data();
            response.data(value);
            return response;
        });

    std::cout << log_prefix << "Starting..." << '\n';
    const std::vector<int> values{1, 2, 3, 4, 5};
    for (const int value : values)
    {
        std_msgs::msg::Int32 req;
        req.data(value);
        auto fut = provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
            participant, service_name, req);

        if (fut.wait_for(10s) != std::future_status::ready)
        {
            std::cerr << log_prefix << "Timeout waiting for response to " << value << '\n';
            return 1;
        }
        const auto &resp = fut.get();
        const auto expected = static_cast<std::int64_t>(value) * value;
        if (resp.data() != expected)
        {
            std::cerr << log_prefix << "Unexpected response for " << value << ": got " << resp.data() << ", expected "
                      << expected << '\n';
            return 1;
        }
        std::cout << log_prefix << "Got response for " << value << ": " << resp.data() << '\n';
    }

    std::cout << log_prefix << "Success" << '\n';
    return 0;
}
catch (const std::exception &e)
{
    std::cerr << log_prefix << "Exception: " << e.what() << '\n';
    return 1;
}
