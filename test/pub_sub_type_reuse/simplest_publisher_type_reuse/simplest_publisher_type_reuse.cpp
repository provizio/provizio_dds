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

#include <iostream>
#include <thread>

#include "provizio/dds/publisher.h"

#include <std_msgs/msg/StringPubSubTypes.h>

int main()
{
    const std::string topic_name_1{"provizio_dds_test_simplest_pub_sub_type_reuse_topic_1"};
    const std::string topic_name_2{"provizio_dds_test_simplest_pub_sub_type_reuse_topic_2"};
    const std::string string{"provizio_dds_test"};
    const std::chrono::milliseconds publish_period{200};
    const std::chrono::milliseconds initial_wait_time{2000}; // Give enough time for subscriber to run
    const int publish_times = 40;

    std::this_thread::sleep_for(initial_wait_time);
    const auto participant = provizio::dds::make_domain_participant();
    auto publisher_1 = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(participant, topic_name_1);
    auto publisher_2 = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(participant, topic_name_2);

    std_msgs::msg::String message;
    message.data(string);
    int successful_times = 0;

    std::cout << "simplest_publisher: Publishing..." << std::endl;
    for (int i = 0; i < publish_times; ++i)
    {
        successful_times += publisher_1->publish(message) ? 1 : 0;
        successful_times += publisher_2->publish(message) ? 1 : 0;
        std::this_thread::sleep_for(publish_period);
    }

    std::cout << "simplest_publisher: Successfully published " << successful_times << " times out of "
              << (publish_times * 2) << " attempts" << std::endl;

    return successful_times > 0 ? 0 : 1;
}
