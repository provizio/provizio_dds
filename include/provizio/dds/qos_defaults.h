#ifndef DDS_QOS_DEFAULTS
#define DDS_QOS_DEFAULTS

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
        };
    } // namespace dds
} // namespace provizio

#endif // DDS_QOS_DEFAULTS
