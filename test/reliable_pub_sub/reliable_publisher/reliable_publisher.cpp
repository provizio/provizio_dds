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

#include "provizio/dds/publisher.h"

#include <std_msgs/msg/StringPubSubTypes.h>

int main()
{
    const bool reliable_qos = true;
    const std::string topic_name{"provizio_dds_test_reliable_pub_sub_topic"};
    const std::string value{"provizio_dds_test"};
    const std::chrono::seconds wait_time{3};

    std::mutex mutex;
    std::condition_variable condition_variable;
    bool published = false;
    bool unmatched = false;

    auto publisher = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(
        provizio::dds::make_domain_participant(), topic_name,
        [&](provizio::dds::data_publisher<std_msgs::msg::StringPubSubType> &publisher, bool matched) {
            if (matched)
            {
                std_msgs::msg::String data;
                data.data(value);
                std::lock_guard<std::mutex> lock{mutex};
                published = publisher.publish(data);
            }
            else
            {
                std::lock_guard<std::mutex> lock{mutex};
                unmatched = true;
                condition_variable.notify_one();
            }
        },
        reliable_qos);

    std::unique_lock<std::mutex> lock{mutex};
    condition_variable.wait_for(lock, wait_time, [&]() { return published && unmatched; });

    if (!published)
    {
        std::cerr << "reliable_publisher: Never published due to no subscribers matched in time" << std::endl;
        return 1;
    }

    if (!unmatched)
    {
        std::cerr << "reliable_publisher: Though published successfully, the subscriber hasn't unmatched in time"
                  << std::endl;
        return 1;
    }

    std::cout << "reliable_publisher: Success" << std::endl;

    return 0;
}
