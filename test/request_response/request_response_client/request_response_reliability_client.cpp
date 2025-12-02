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
#include <cstdint>
#include <future>
#include <iostream>
#include <string>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.h>

constexpr const char *log_prefix = "request_response_reliability_client: ";

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
try
{
    const std::string service_name{"provizio_dds_test_request_response_reliability"};
    constexpr provizio::dds::DomainId_t domain_id = 14;
    constexpr std::chrono::seconds timeout{10};

    if (argc != 2)
    {
        std::cerr << log_prefix << timestamp() << "Wrong number of arguments!" << std::endl;
        return 1;
    }

    const auto value = std::atoi(argv[1]);

    // Sleep some random time, to test different combinations of client/service startup times
    srand(value);
    const auto wait = rand() % 1000;
    std::cout << log_prefix << timestamp() << "Waiting " << wait << "ms..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds{wait});

    std_msgs::msg::Int32 request;
    request.data(value);

    std::cout << log_prefix << timestamp() << "Requesting " << value << "..." << std::endl;
    auto future_response = provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int32PubSubType>(
        provizio::dds::make_domain_participant(domain_id), service_name, request);

    const auto wait_status = future_response.wait_for(timeout);
    if (wait_status != std::future_status::ready)
    {
        std::cerr << log_prefix << timestamp() << "Timeout waiting for response" << std::endl;
        return 1;
    }

    const auto received_value = future_response.get().data();
    if (received_value != value)
    {
        std::cerr << log_prefix << timestamp() << "Unexpected response " << received_value << ", expected " << value
                  << std::endl;
        return 1;
    }

    std::cout << log_prefix << timestamp() << "Successfully received expected response = " << received_value
              << std::endl;
    return 0;
}
catch (const std::exception &exception)
{
    std::cerr << log_prefix << timestamp() << "Exception during the test: " << exception.what() << std::endl;
    return 1;
}
