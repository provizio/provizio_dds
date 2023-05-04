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

#ifndef DDS_QOS_DEFAULTS
#define DDS_QOS_DEFAULTS

#include <fastdds/rtps/resources/ResourceManagement.h>
#include <fastrtps/qos/QosPolicies.h>

#include "provizio/dds/common.h"

namespace provizio
{
    namespace dds
    {
        /**
         * @brief Defines default QOS policies for a DDS data type. They can be overriden in template specializations
         * for specific types.
         *
         * @tparam data_pub_sub_type DDS data pub/sub type, f.e. std_msgs::msg::StringPubSubType
         * @see https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/policy.html
         */
        template <typename data_pub_sub_type> struct qos_defaults final
        {
            /**
             * @brief Defines whether to use reliable data writer DDS QOS policies. RELIABLE_RELIABILITY_QOS by default
             * in Fast-DDS.
             *
             * @see
             * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
             */
            static constexpr ReliabilityQosPolicyKind datawriter_reliability_kind = RELIABLE_RELIABILITY_QOS;

            /**
             * @brief Defines whether to use reliable data reader DDS QOS policies. BEST_EFFORT_RELIABILITY_QOS by
             * default in Fast-DDS.
             * @see
             * https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#reliabilityqospolicy
             */
            static constexpr ReliabilityQosPolicyKind datareader_reliability_kind = BEST_EFFORT_RELIABILITY_QOS;

            /**
             * @brief Defines the default memory policy for both data reader and data writer.
             * PREALLOCATED_WITH_REALLOC_MEMORY_MODE in Fast-DDS 2.9+.
             * @see
             * https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/rtps/resources/MemoryManagementPolicy.html
             */
            static constexpr auto memory_policy = eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
        };
    } // namespace dds
} // namespace provizio

#endif // DDS_QOS_DEFAULTS
