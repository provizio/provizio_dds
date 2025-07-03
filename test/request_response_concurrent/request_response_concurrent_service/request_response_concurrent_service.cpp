// Copyright 2023 Provizio Ltd.
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
#include <mutex>
#include <string>
#include <thread>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.h>
#include <std_msgs/msg/Int64PubSubTypes.h>

int main()
{
    constexpr const char *log_prefix = "request_response_concurrent_service: ";
    const std::string service_name{"provizio_dds_test_request_response_concurrent"};
    constexpr provizio::dds::DomainId_t domain_id = 15;
    constexpr int requests_expected = 7;
    constexpr std::chrono::seconds total_timeout{10};
    constexpr std::chrono::seconds end_sleep{4};
    constexpr std::int32_t max_history_depth = requests_expected;

    std::mutex mutex;
    std::condition_variable condition_variable;
    int got_requests = 0;

    auto domain_participant = provizio::dds::make_domain_participant(domain_id);

    std::cout << log_prefix << "Waiting for requests..." << std::endl;
    auto service = provizio::dds::make_service<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
        domain_participant, service_name,
        [&](const std_msgs::msg::Int32 &request) {
            // returns std::future for a delayed response
            return std::async(std::launch::async, [request, &mutex, &got_requests, &condition_variable]() {
                std_msgs::msg::Int64 response;

                const std::int64_t value = static_cast<std::int64_t>(request.data()) * request.data();

                std::cout << log_prefix << "Processing request " << request.data() << " => " << value << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds{value});

                response.data(value);

                std::lock_guard<std::mutex> lock{mutex};
                ++got_requests;
                if (got_requests >= requests_expected)
                {
                    condition_variable.notify_all();
                }

                std::cout << log_prefix << "Response sent (" << request.data() << " => " << value << ")" << std::endl;
                return response;
            });
        },
        max_history_depth);

    std::unique_lock<std::mutex> lock{mutex};
    if (!condition_variable.wait_for(lock, total_timeout, [&]() { return got_requests >= requests_expected; }))
    {
        std::cerr << log_prefix << "Timeout! Got " << got_requests << " requests so far" << std::endl;
        return 1;
    }

    // Make sure to deliver the last response
    std::this_thread::sleep_for(end_sleep);

    std::cout << log_prefix << "Successfully sent all the responses" << std::endl;
    return 0;
}
