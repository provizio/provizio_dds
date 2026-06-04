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
#include <cstdint>
#include <iostream>
#include <mutex>
#include <random>
#include <string>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>

std::string timestamp()
{
    constexpr double ms_in_s = 1000.0;
    static auto initial = std::chrono::steady_clock::now();

    return "[" +
           std::to_string(static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                 std::chrono::steady_clock::now() - initial)
                                                 .count()) /
                          ms_in_s) +
           "] ";
}

// Arguments: test_name_postfix first_iteration_value num_iterations [domain_id]
int main(int argc, char *argv[])
{
    constexpr int max_wait_rnd = 1999;
    constexpr int half_wait_rnd = 1000;
    constexpr std::chrono::milliseconds time_to_deliver_last_response{250};
    constexpr int min_argc = 4;
    constexpr int max_argc = 5;

    if (argc < min_argc || argc > max_argc)
    {
        std::cerr << "Wrong number of arguments!" << '\n';
        return 1;
    }

    const std::string test_name_postfix = argv[1];  // NOLINT: OK in a unit test
    auto expected_value = std::atoi(argv[2]);       // NOLINT: OK in a unit test
    auto num_iterations = std::atoi(argv[3]);       // NOLINT: OK in a unit test
    // Use per-iteration domain IDs (100-127) to avoid DDS multicast discovery state accumulation across rapid
    // participant create/destroy cycles (root cause of flaky timeouts on Windows CI).
    // High range avoids conflicts with other tests; wraps at 127 (max Fast-DDS domain ID).
    constexpr provizio::dds::DomainId_t base_domain_id = 100;
    constexpr provizio::dds::DomainId_t domain_range = 28;  // 100..127
    const provizio::dds::DomainId_t domain_id =
        (argc > 4)
            ? static_cast<provizio::dds::DomainId_t>(base_domain_id + (std::atoi(argv[4]) % domain_range))  // NOLINT
            : 14;  // NOLINT: OK in a unit test
    const std::chrono::seconds wait_timeout{num_iterations * 3 + 30};

    const std::string log_prefix = "request_response_reliability_service" + test_name_postfix + ": ";
    const std::string service_name{"provizio_dds_test_request_response_reliability" + test_name_postfix};

    // Sleep some random time, to test different combinations of client/service startup times
    std::mt19937 engine(expected_value);
    const auto wait = std::uniform_int_distribution<int>{0, max_wait_rnd}(engine);
    if (wait >= half_wait_rnd)
    {
        std::cout << log_prefix << timestamp() << "Waiting " << wait << "ms..." << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds{wait});
    }

    std::mutex mutex;
    std::condition_variable condition_variable;
    bool requests_processed = false;
    bool received_expected_values = true;

    auto service = provizio::dds::make_service<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int32PubSubType>(
        provizio::dds::make_domain_participant(domain_id), service_name, [&](const std_msgs::msg::Int32 &request) {
            std::cout << log_prefix << timestamp() << "Got request: " << request.data() << '\n';

            const auto value = request.data();

            std_msgs::msg::Int32 response;
            response.data(expected_value);

            const std::lock_guard<std::mutex> lock{mutex};

            requests_processed = (num_iterations <= 1);
            if (value != expected_value)
            {
                std::cerr << log_prefix << timestamp() << "Incorrect request received: " << value << " when "
                          << expected_value << " was expected!" << '\n';
                received_expected_values = false;
            }

            ++expected_value;
            --num_iterations;

            if (requests_processed)
            {
                condition_variable.notify_all();
            }

            std::cout << log_prefix << timestamp() << "Responding value: " << response.data() << '\n';

            return response;
        });

    std::unique_lock<std::mutex> lock{mutex};
    if (!condition_variable.wait_for(lock, wait_timeout, [&]() { return requests_processed; }))
    {
        std::cerr << log_prefix << timestamp() << "Timeout waiting for request" << '\n';
        return 1;
    }

    if (!received_expected_values)
    {
        return 1;
    }

    std::this_thread::sleep_for(time_to_deliver_last_response);

    std::cout << log_prefix << timestamp() << "Successfully processed value " << expected_value << '\n';
    return 0;
}
