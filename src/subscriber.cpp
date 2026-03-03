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

#include "provizio/dds/subscriber.h"

namespace provizio::dds
{
    void data_reader_listener::on_subscription_matched(DataReader *reader, const SubscriptionMatchedStatus &info)
    {
        (void)reader;

        {
            std::lock_guard<std::mutex> lock{num_matched_publishers_mutex};
            num_matched_publishers = info.current_count;
        }
        num_matched_publishers_cv.notify_all();
    }
} // namespace provizio::dds
