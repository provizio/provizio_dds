// Copyright 2026 Provizio Ltd.
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
//
// Compile-time source-compat check: exercises the 1.10.x spellings of
// public types that 1.10.x consumer code routinely embedded in callback
// signatures, request/response handlers, and topic-handle declarations.
//
// Fast-DDS 3.x removed the `eprosima::fastrtps` namespace; provizio_dds
// (via `<provizio/dds/common.h>`) reintroduces it as an alias for
// `eprosima::fastdds`. If this translation unit compiles and the
// `static_assert`s below succeed, 1.10.x C++ consumer code that pinned
// these spellings remains source-compatible against 2.x.

#include <cstring>
#include <iostream>
#include <string>
#include <type_traits>
#include <typeinfo>

#include "provizio/dds/common.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/request_response.h"
#include "provizio/dds/subscriber.h"

// PubSubTypes for the wire-name fingerprint section.
#include <provizio/msg/camera_intrinsicsPubSubTypes.hpp>
#include <provizio/msg/radar_infoPubSubTypes.hpp>
#include <std_msgs/msg/BoolPubSubTypes.hpp>
#include <std_msgs/msg/HeaderPubSubTypes.hpp>
#include <std_msgs/msg/StringPubSubTypes.hpp>

// All the types 1.10.x consumer code commonly referenced through the
// `eprosima::fastrtps::rtps::*` path. They must all resolve to the same
// underlying types as the canonical `eprosima::fastdds::rtps::*` ones.
static_assert(std::is_same_v<eprosima::fastrtps::rtps::SampleIdentity, eprosima::fastdds::rtps::SampleIdentity>,
              "fastrtps::rtps::SampleIdentity must alias fastdds::rtps::SampleIdentity");

static_assert(std::is_same_v<eprosima::fastrtps::rtps::GUID_t, eprosima::fastdds::rtps::GUID_t>,
              "fastrtps::rtps::GUID_t must alias fastdds::rtps::GUID_t");

static_assert(std::is_same_v<eprosima::fastrtps::rtps::WriteParams, eprosima::fastdds::rtps::WriteParams>,
              "fastrtps::rtps::WriteParams must alias fastdds::rtps::WriteParams");

// provizio_dds's own short alias for the same GUID type.
static_assert(std::is_same_v<provizio::dds::guid, eprosima::fastrtps::rtps::GUID_t>,
              "provizio::dds::guid must alias fastrtps::rtps::GUID_t");

// Verify the request/response callback signature shape from
// `<provizio/dds/request_response.h>` accepts a SampleIdentity reference
// spelled with the legacy namespace path — this is the canonical 1.10.x
// callback signature.
namespace
{
    [[maybe_unused]] void legacy_on_response_signature(int /* dummy response */,
                                                       const eprosima::fastrtps::rtps::SampleIdentity &identity)
    {
        // The legacy GUID_t spelling must work end-to-end as well.
        const eprosima::fastrtps::rtps::GUID_t &writer = identity.writer_guid();
        (void)writer;
    }
}  // namespace

namespace
{
    // Verify a generated PubSubType advertises the canonical
    // `<pkg>::msg::dds_::<Type>_` wire name 1.10.x publishers and subscribers
    // expect. A regression here means every other deployed node on the wire
    // (radars, fleet pipelines, recorded mcaps) will silently fail to match topics
    // with this build.
    template <typename pub_sub_type> bool check_wire_name(const char *expected)
    {
        pub_sub_type instance;
        const std::string &actual = instance.get_name();
        if (actual != expected)
        {
            std::cerr << "FAIL: " << typeid(pub_sub_type).name() << ".get_name() = '" << actual << "', expected '"
                      << expected << "'\n";
            return false;
        }
        std::cout << "OK: wire-name " << expected << "\n";
        return true;
    }
}  // namespace

int main()
{
    // Wire-name fingerprint: every generated PubSubType must advertise the
    // exact `<pkg>::msg::dds_::<Type>_` form pre-3.x provizio_dds and stock
    // ROS 2 nodes already speak. Per OMG IDL §7.4.4.1, the leading-
    // underscore keyword escape (`struct _Bool`) is stripped at parse time
    // by fastddsgen, so the wire name has a single trailing underscore (from
    // `-typeros2`), not a leading one and not a double-trailing one — both
    // of which broke interop in earlier iterations of this branch.
    bool all_ok = true;
    all_ok &= check_wire_name<std_msgs::msg::BoolPubSubType>("std_msgs::msg::dds_::Bool_");
    all_ok &= check_wire_name<std_msgs::msg::StringPubSubType>("std_msgs::msg::dds_::String_");
    all_ok &= check_wire_name<std_msgs::msg::HeaderPubSubType>("std_msgs::msg::dds_::Header_");
    all_ok &= check_wire_name<provizio::msg::radar_infoPubSubType>("provizio::msg::dds_::radar_info_");
    all_ok &= check_wire_name<provizio::msg::camera_intrinsicsPubSubType>("provizio::msg::dds_::camera_intrinsics_");

    if (all_ok)
    {
        std::cout << "All legacy-API C++ compat checks passed.\n";
    }
    return all_ok ? 0 : 1;
}
