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
#include <utility>
#include <vector>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>
#include <std_msgs/msg/Int64PubSubTypes.hpp>

constexpr const char *log_prefix = "request_response_concurrent_requests_service_client: ";

// Validates service_client issuing many PARALLEL requests fired immediately after construction (without
// wait_for_service) -- exercising the auto-defer-until-matched path. Reuses request_response_service.
int main()
try
{
    const std::string service_name{"provizio_dds_test_request_response"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr std::chrono::seconds timeout{30};

    // x and x^2 pairs; expected value 0 means "fire the request but don't validate the response".
    const std::array<std::pair<std::int32_t, std::int64_t>, 5> expected_request_response_pairs{
        {{50, 0LL}, {10, 100LL}, {20, 400LL}, {-15, 225LL}, {75, 0LL}}};

    auto domain_participant = provizio::dds::make_domain_participant(domain_id);
    auto client = provizio::dds::make_service_client<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
        domain_participant, service_name);

    // No explicit wait_for_service(): requests issued before matching auto-defer and flush once matched.
    std::vector<std::pair<
        provizio::dds::future_response<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>, std::int64_t>>
        future_responses;
    future_responses.reserve(expected_request_response_pairs.size());
    for (const auto &request_response_pair : expected_request_response_pairs)
    {
        std_msgs::msg::Int32 request;
        request.data(request_response_pair.first);

        std::cout << log_prefix << "Requesting " << request_response_pair.first << "..." << '\n';
        future_responses.emplace_back(client->request(request), request_response_pair.second);
    }

    std::cout << log_prefix << "Requests sent, expecting responses now..." << '\n';
    for (auto &future_response : future_responses)
    {
        // Expected value 0 means "fire but don't validate" — don't wait on those responses (a
        // non-responding service would otherwise block the test for the full timeout).
        if (future_response.second != 0)
        {
            if (future_response.first.wait_for(timeout) != std::future_status::ready)
            {
                std::cerr << log_prefix << "Timeout waiting when " << future_response.second << " was expected!"
                          << '\n';
                return 1;
            }

            const auto received = future_response.first.get().data();
            if (received != future_response.second)
            {
                std::cerr << log_prefix << "Unexpected value! " << future_response.second << " expected, " << received
                          << " received." << '\n';
                return 1;
            }

            std::cout << log_prefix << "Correctly got expected response = " << received << '\n';
        }
    }

    std::cout << log_prefix << "Successfully completed" << '\n';
    return 0;
}
catch (const std::exception &exception)
{
    std::cout << log_prefix << "Exception during the test: " << exception.what() << '\n';
    return 1;
}
