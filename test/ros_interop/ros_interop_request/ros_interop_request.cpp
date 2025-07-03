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
#include <iostream>
#include <string>

#include <std_srvs/srv/SetBoolPubSubTypes.h>

#include "provizio/dds/request_response.h"

int main()
{
    constexpr const char *log_prefix = "ros_interop_request: ";
    const std::string service_name{"provizio_dds_test_ros_interop_request"};
    constexpr std::chrono::seconds timeout{4};

    auto domain_participant = provizio::dds::make_domain_participant();

    for (bool data = false;; data = !data)
    {
        std_srvs::srv::SetBool_Request request;
        request.data(data);

        std::cout << log_prefix << "Requesting " << data << "..." << std::endl;

        auto future_response =
            provizio::dds::request<std_srvs::srv::SetBool_RequestPubSubType, std_srvs::srv::SetBool_ResponsePubSubType>(
                domain_participant, service_name, request);

        if (future_response.wait_for(timeout) != std::future_status::ready)
        {
            std::cerr << log_prefix << "Timed out waiting for the response (" << data << " was expected)" << std::endl;
            return 1;
        }

        auto response = future_response.get();
        if (data != response.success())
        {
            std::cerr << log_prefix << "Got " << future_response.get().success() << " when " << data << " was expected!"
                      << std::endl;
            return 1;
        }

        std::cout << log_prefix << "Successfully received " << response.success()
                  << " with message: " << response.message() << std::endl;

        if (data)
        {
            break;
        }
    }

    std::cout << log_prefix << "Successfully got all the responses" << std::endl;
    return 0;
}
