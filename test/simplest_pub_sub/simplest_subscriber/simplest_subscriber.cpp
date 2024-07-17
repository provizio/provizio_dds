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

#include <condition_variable>
#include <iostream>
#include <mutex>

#include "provizio/dds/subscriber.h"

#include <std_msgs/msg/StringPubSubTypes.h>

int main()
{
    const std::string topic_name{"provizio_dds_test_simplest_pub_sub_topic"};
    const std::string expected_value{"provizio_dds_test"};
    const std::chrono::seconds wait_time{3};

    std::mutex mutex;
    std::condition_variable condition_variable;
    std::string string;
    std::cout << "simplest_subscriber: Waiting for messages..." << std::endl;
    const auto subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
        provizio::dds::make_domain_participant(), topic_name, [&](const std_msgs::msg::String &message) {
            std::lock_guard<std::mutex> lock{mutex};
            string = message.data();
            condition_variable.notify_one();
        });

    {
        std::unique_lock<std::mutex> lock{mutex};
        condition_variable.wait_for(lock, wait_time, [&]() { return string == expected_value; });

        if (string != expected_value)
        {
            std::cerr << "simplest_subscriber: " << expected_value << " was expected but "
                      << (string.empty() ? "nothing" : string) << " was received!" << std::endl;
            return 1;
        }
    }

    std::cout << "simplest_subscriber: Success" << std::endl;

    return 0;
}
