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
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>
#include <std_msgs/msg/Int64PubSubTypes.hpp>

int main(int argc, char *argv[])
{
    constexpr const char *log_prefix = "request_response_concurrent_service: ";
    const std::string service_name{"provizio_dds_test_request_response"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr int requests_expected_default = 7;
    constexpr std::chrono::seconds total_timeout{
        60};  // DDS entity matching can be slow on some platforms (e.g. Windows)
    constexpr std::chrono::seconds end_sleep{4};

    int requests_expected = requests_expected_default;
    if (argc == 2)
    {
        // Supports specifying the expected number of requests
        const auto argument = argv[1];  // NOLINT: Pointer coming from main() arguments
        requests_expected = std::stoi(argument);
        if (requests_expected <= 0)
        {
            std::cerr << log_prefix << "Incorrect argument: " << argument << ". Number of requests to wait expected!"
                      << '\n';
            return 1;
        }
    }

    const std::int32_t max_history_depth = requests_expected;

    std::mutex mutex;
    std::condition_variable condition_variable;
    int got_requests = 0;

    auto domain_participant = provizio::dds::make_domain_participant(domain_id);

    std::cout << log_prefix << "Waiting for " << requests_expected << " requests... " << '\n';
    auto service = provizio::dds::make_service<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
        domain_participant, service_name,
        [&](const std_msgs::msg::Int32 &request) {
    // returns std::future for a delayed response
    // log_prefix capture is required by MSVC but clang considers it unnecessary for constexpr
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-lambda-capture"
#endif
            return std::async(std::launch::async, [request, &mutex, &got_requests, &condition_variable,
                                                   requests_expected, log_prefix]() {
                std_msgs::msg::Int64 response;

                const std::int64_t value = static_cast<std::int64_t>(request.data()) * request.data();

                std::cout << log_prefix << "Processing request " << request.data() << " => " << value << '\n';
                std::this_thread::sleep_for(std::chrono::milliseconds{value});

                response.data(value);

                const std::lock_guard<std::mutex> lock{mutex};
                ++got_requests;
                if (got_requests >= requests_expected)
                {
                    condition_variable.notify_all();
                }

                std::cout << log_prefix << "Response sent (" << request.data() << " => " << value << ")" << '\n';
                return response;
            });
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
        },
        max_history_depth);

    std::unique_lock<std::mutex> lock{mutex};
    if (!condition_variable.wait_for(lock, total_timeout, [&]() { return got_requests >= requests_expected; }))
    {
        std::cerr << log_prefix << "Timeout! Got " << got_requests << " requests so far" << '\n';
        return 1;
    }

    // Make sure to deliver the last response
    std::this_thread::sleep_for(end_sleep);

    std::cout << log_prefix << "Successfully sent all the responses" << '\n';
    return 0;
}
