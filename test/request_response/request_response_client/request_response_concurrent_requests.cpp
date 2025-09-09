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
#include <thread>
#include <utility>
#include <vector>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.h>
#include <std_msgs/msg/Int64PubSubTypes.h>

constexpr const char *log_prefix = "request_response_concurrent_requests: ";

int main()
try
{
    const std::string service_name{"provizio_dds_test_request_response"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr std::chrono::seconds time_to_match{3};
    constexpr std::chrono::seconds timeout{10};

    // These are x and x^2 pairs, except of those with expected values of 0 which means we want to interrupt the
    // requests and make sure all behaves well
    const std::array<std::pair<std::int32_t, std::int64_t>, 5> expected_request_response_pairs{
        {{50, 0LL}, {10, 100LL}, {20, 400LL}, {-15, 225LL}, {75, 0LL}}};

    auto domain_participant = provizio::dds::make_domain_participant(domain_id);

    std::vector<std::pair<
        provizio::dds::future_response<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>, std::int64_t>>
        future_responses;
    future_responses.reserve(expected_request_response_pairs.size());
    for (const auto &request_response_pair : expected_request_response_pairs)
    {
        std_msgs::msg::Int32 request;
        request.data(request_response_pair.first);

        std::cout << log_prefix << "Requesting " << request_response_pair.first << " with expected response of "
                  << request_response_pair.second << "..." << std::endl;
        future_responses.emplace_back(
            provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
                domain_participant, service_name, request),
            request_response_pair.second);
    }

    std::this_thread::sleep_for(time_to_match);
    std::cout << log_prefix << "Requests sent, expecting responses now..." << std::endl;

    // Now we're waiting for all responses, except those that are expected to timeout
    for (auto &future_response : future_responses)
    {
        auto status = future_response.first.wait_for(timeout);
        if (future_response.second != 0)
        {
            if (status != std::future_status::ready)
            {
                std::cerr << log_prefix << "Timeout waiting when " << future_response.second << " was expected!"
                          << std::endl;
                return 1;
            }

            const auto received = future_response.first.get().data();
            if (received != future_response.second)
            {
                std::cerr << log_prefix << "Unexpected value received from test_service! " << future_response.second
                          << " expected, " << received << " received." << std::endl;
                return 1;
            }

            std::cout << log_prefix << "Correctly got expected response = " << received << std::endl;
        }
    }

    std::cout << log_prefix << "Successfully complete" << std::endl;

    return 0;
}
catch (const std::exception &exception)
{
    std::cout << log_prefix << "Exception during the test: " << exception.what() << std::endl;

    return 1;
}
