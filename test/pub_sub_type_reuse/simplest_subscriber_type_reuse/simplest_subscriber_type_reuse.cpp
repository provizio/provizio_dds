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

#include <condition_variable>
#include <iostream>
#include <mutex>

#include "provizio/dds/subscriber.h"

#include <std_msgs/msg/StringPubSubTypes.h>

int main()
{
    const std::string topic_name_1{"provizio_dds_test_simplest_pub_sub_type_reuse_topic_1"};
    const std::string topic_name_2{"provizio_dds_test_simplest_pub_sub_type_reuse_topic_2"};
    const std::string expected_value{"provizio_dds_test"};
    const std::chrono::seconds wait_time{3};

    std::mutex mutex;
    std::condition_variable condition_variable;
    std::string string_1;
    std::string string_2;
    std::cout << "simplest_subscriber: Waiting for messages..." << std::endl;
    const auto participant = provizio::dds::make_domain_participant();
    const auto subscriber_1 = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
        participant, topic_name_1, [&](const std_msgs::msg::String &message) {
            const std::lock_guard<std::mutex> lock{mutex};
            string_1 = message.data();
            condition_variable.notify_one();
        });

    const auto subscriber_2 = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
        participant, topic_name_2, [&](const std_msgs::msg::String &message) {
            const std::lock_guard<std::mutex> lock{mutex};
            string_2 = message.data();
            condition_variable.notify_one();
        });

    // subscriber_1
    {
        std::unique_lock<std::mutex> lock{mutex};
        condition_variable.wait_for(lock, wait_time, [&]() { return string_1 == expected_value; });

        if (string_1 != expected_value)
        {
            std::cerr << "simplest_subscriber: " << expected_value << " was expected by subscriber_1 but "
                      << (string_1.empty() ? "nothing" : string_1) << " was received!" << std::endl;
            return 1;
        }
    }

    // subscriber_2
    {
        std::unique_lock<std::mutex> lock{mutex};
        condition_variable.wait_for(lock, wait_time, [&]() { return string_2 == expected_value; });

        if (string_2 != expected_value)
        {
            std::cerr << "simplest_subscriber: " << expected_value << " was expected by subscriber_2 but "
                      << (string_2.empty() ? "nothing" : string_2) << " was received!" << std::endl;
            return 1;
        }
    }

    std::cout << "simplest_subscriber: Success" << std::endl;

    return 0;
}
