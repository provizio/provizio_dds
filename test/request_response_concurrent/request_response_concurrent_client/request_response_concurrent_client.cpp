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

constexpr const char *log_prefix = "request_response_concurrent_client: ";

int main()
try
{
    const std::string service_name{"provizio_dds_test_request_response"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr std::chrono::seconds time_to_match{5};
    constexpr std::chrono::milliseconds timeout{10000}; // DDS entity matching can be slow on some platforms (e.g. Windows)

    // These are x and x^2 pairs, except of those with expected values of 0 which means we want to interrupt the
    // requests and make sure all behaves well
    const std::array<std::pair<std::int32_t, std::int64_t>, 7> expected_request_response_pairs{
        {{50, 0LL}, {10, 100LL}, {20, 400LL}, {30, 900LL}, {-1, 1LL}, {-15, 225LL}, {75, 0LL}}};

    auto domain_participant = provizio::dds::make_domain_participant(domain_id);
    std::vector<std::pair<
        std::unique_ptr<provizio::dds::future_response<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>>,
        std::int64_t>>
        requests_and_expected_responses;

    // All requests are sent at once
    for (const auto &request_response_pair : expected_request_response_pairs)
    {
        std::cout << log_prefix << "Requesting " << request_response_pair.first << " with expected response of "
                  << request_response_pair.second << "..." << std::endl;

        std_msgs::msg::Int32 request;
        request.data(request_response_pair.first);
        requests_and_expected_responses.emplace_back(
            std::make_unique<
                provizio::dds::future_response<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>>(
                provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
                    domain_participant, service_name, request)),
            request_response_pair.second);
    }

    // Some time to let participants match with each other, before measuring timeouts below
    std::this_thread::sleep_for(time_to_match);

    // Now we're waiting for all responses, except those that are expected to timeout
    for (auto &request_response_pair : requests_and_expected_responses)
    {
        auto status = request_response_pair.first->wait_for(timeout);
        if (request_response_pair.second != 0)
        {
            if (status != std::future_status::ready)
            {
                std::cerr << log_prefix << "Timeout waiting when " << request_response_pair.second << " was expected!"
                          << std::endl;
                return 1;
            }

            const auto received = request_response_pair.first->get().data();
            if (received != request_response_pair.second)
            {
                std::cerr << log_prefix << "Unexpected value received from test_service! "
                          << request_response_pair.second << " expected, " << received << " received." << std::endl;
                return 1;
            }

            std::cout << log_prefix << "Correctly got expected response = " << received << std::endl;
        }

        // Validates normal destroying or interrupting requests works fine and doesn't affect following requests
        request_response_pair.first.reset();
    }

    std::cout << log_prefix << "Successfully complete" << std::endl;

    return 0;
}
catch (const std::exception &exception)
{
    std::cout << log_prefix << "Exception during the test: " << exception.what() << std::endl;

    return 1;
}
