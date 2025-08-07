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
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <std_srvs/srv/SetBoolPubSubTypes.h>

#include "provizio/dds/request_response.h"

int main()
{
    constexpr const char *log_prefix = "ros_interop_service: ";
    const std::string service_name{"provizio_dds_test_ros_interop_service"};
    constexpr int requests_expected = 2;
    constexpr std::chrono::seconds total_timeout{5};
    constexpr std::chrono::seconds end_sleep{2};
    constexpr std::int32_t max_history_depth =
        provizio::dds::use_default_qos_durability; // Keep all default for ROS2 compatibility

    std::mutex mutex;
    std::condition_variable condition_variable;
    int got_requests = 0;

    auto domain_participant = provizio::dds::make_domain_participant();

    std::cout << log_prefix << "Waiting for requests..." << std::endl;
    auto service = provizio::dds::make_service<std_srvs::srv::SetBool_RequestPubSubType,
                                               std_srvs::srv::SetBool_ResponsePubSubType>(
        domain_participant, service_name,
        [&](const std_srvs::srv::SetBool_Request &request) {
            std_srvs::srv::SetBool_Response response;
            response.message(service_name);
            response.success(request.data());

            const std::lock_guard<std::mutex> lock{mutex};
            ++got_requests;
            if (got_requests >= requests_expected)
            {
                condition_variable.notify_all();
            }

            std::cout << log_prefix << "Response sent (" << request.data() << " => " << response.success() << ")"
                      << std::endl;
            return response;
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
