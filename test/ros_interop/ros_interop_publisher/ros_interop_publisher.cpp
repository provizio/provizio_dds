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

#include <iostream>
#include <thread>

#include "provizio/dds/publisher.h"

#include <std_msgs/msg/StringPubSubTypes.h>

int main()
{
    const std::string topic_name{"rt/chatter"};
    const std::string value{"provizio_test_ros_interop_publisher_says_hi"};
    const std::chrono::milliseconds wait_time{100};
    const int publish_times = 50;

    auto publisher = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(
        provizio::dds::make_domain_participant(), topic_name);

    std_msgs::msg::String str;
    str.data(value);
    int successful_times = 0;
    for (int i = 0; i < publish_times; ++i)
    {
        successful_times += publisher->publish(str) ? 1 : 0;
        std::this_thread::sleep_for(wait_time);
    }

    std::cout << "ros_interop_publisher: Successfully published " << successful_times << " times" << std::endl;

    return successful_times > 0 ? 0 : 1;
}
