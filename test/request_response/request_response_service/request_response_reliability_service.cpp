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
#include <string>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.h>

std::string timestamp()
{
    static auto initial = std::chrono::steady_clock::now();

    return "[" +
           std::to_string(static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                 std::chrono::steady_clock::now() - initial)
                                                 .count()) /
                          1000.0) +
           "] ";
}

int main(int argc, char *argv[])
{
    constexpr const char *log_prefix = "request_response_reliability_service: ";
    const std::string service_name{"provizio_dds_test_request_response_reliability"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr std::chrono::seconds wait_timeout{10};

    if (argc != 2)
    {
        std::cerr << log_prefix << timestamp() << "Wrong number of arguments!" << std::endl;
        return 1;
    }

    const auto expected_value = std::atoi(argv[1]);

    // Sleep some random time, to test different combinations of client/service startup times
    srand(expected_value + 1000);
    const auto wait = rand() % 2000;
    std::cout << log_prefix << timestamp() << "Waiting " << wait << "ms..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds{wait});

    std::mutex mutex;
    std::condition_variable condition_variable;
    bool request_processed = false;
    bool received_expected_value = true;

    auto service = provizio::dds::make_service<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int32PubSubType>(
        provizio::dds::make_domain_participant(domain_id), service_name, [&](const std_msgs::msg::Int32 &request) {
            std::cout << log_prefix << timestamp() << "Got request: " << request.data() << std::endl;

            return std::async(std::launch::async, [&, value = request.data()]() {
                std_msgs::msg::Int32 response;
                response.data(expected_value);

                {
                    std::lock_guard<std::mutex> lock{mutex};
                    request_processed = true;
                    if (value != expected_value)
                    {
                        std::cerr << log_prefix << timestamp() << "Incorrect request received: " << value << " when "
                                  << expected_value << " was expected!" << std::endl;
                        received_expected_value = false;
                    }
                }
                condition_variable.notify_all();

                std::cout << log_prefix << timestamp() << "Responding value: " << response.data() << std::endl;
                return response;
            });
        });

    std::unique_lock<std::mutex> lock{mutex};
    if (!condition_variable.wait_for(lock, wait_timeout, [&]() { return request_processed; }))
    {
        std::cerr << log_prefix << timestamp() << "Timeout waiting for request" << std::endl;
        return 1;
    }

    if (!received_expected_value)
    {
        return 1;
    }

    // Some time to deliver the message before killing the service
    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    std::cout << log_prefix << timestamp() << "Successfully processed value " << expected_value << std::endl;
    return 0;
}
