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

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>
#include <std_msgs/msg/Int64PubSubTypes.hpp>

constexpr const char *log_prefix = "request_response_service_client: ";

// Validates service_client issuing CONSECUTIVE requests from a single, created-in-advance client, after an
// explicit wait_for_service() readiness check. Reuses request_response_service (sync x^2 service).
int main()
try
{
    const std::string service_name{"provizio_dds_test_request_response"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr std::chrono::seconds timeout{30 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};

    // x and x^2 pairs; expected value 0 means "fire the request but don't validate the response".
    const std::array<std::pair<std::int32_t, std::int64_t>, 5> expected_request_response_pairs{
        {{50, 0LL}, {10, 100LL}, {20, 400LL}, {-15, 225LL}, {75, 0LL}}};

    auto domain_participant = provizio::dds::make_domain_participant(domain_id);
    auto client = provizio::dds::make_service_client<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
        domain_participant, service_name);

    // The client is created in advance and matches the service(s) over time; block until ready.
    if (!client->wait_for_service(timeout))
    {
        std::cerr << log_prefix << "Service did not become ready within the timeout!" << '\n';
        return 1;
    }
    std::cout << log_prefix << "Service ready; sending requests..." << '\n';

    for (const auto &request_response_pair : expected_request_response_pairs)
    {
        std_msgs::msg::Int32 request;
        request.data(request_response_pair.first);

        auto future_response = client->request(request);

        // Expected value 0 means "fire but don't validate" — don't wait on those responses (a
        // non-responding service would otherwise block the test for the full timeout).
        if (request_response_pair.second != 0)
        {
            if (future_response.wait_for(timeout) != std::future_status::ready)
            {
                std::cerr << log_prefix << "Timeout waiting when " << request_response_pair.second << " was expected!"
                          << '\n';
                return 1;
            }

            const auto received = future_response.get().data();
            if (received != request_response_pair.second)
            {
                std::cerr << log_prefix << "Unexpected value received! " << request_response_pair.second
                          << " expected, " << received << " received." << '\n';
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
