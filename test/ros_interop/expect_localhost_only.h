// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.

#ifndef PROVIZIO_DDS_TEST_ROS_INTEROP_EXPECT_LOCALHOST_ONLY
#define PROVIZIO_DDS_TEST_ROS_INTEROP_EXPECT_LOCALHOST_ONLY

#include <cstdlib>
#include <iostream>
#include <string>

#include "provizio/dds/domain_participant.h"

namespace ros_interop_test
{
    /**
     * @brief Guards the *_localhost_only CTest cases against passing vacuously.
     *
     * Those cases run the provizio side with PROVIZIO_DDS_LOCALHOST_ONLY set and are only worth
     * anything if that made its participants localhost_only; run with the variable ignored they would
     * pass just the same, over the default transports. They set PROVIZIO_DDS_TEST_EXPECT_LOCALHOST_ONLY=1
     * to have the binary check what a default participant resolves to.
     *
     * @return false (after saying why) when the expectation is set and a default participant would not be
     * localhost_only; true otherwise.
     */
    inline bool localhost_only_expectation_met()
    {
        // NOLINTNEXTLINE(concurrency-mt-unsafe): read once, before any thread of the test exists
        const char *const expected = std::getenv("PROVIZIO_DDS_TEST_EXPECT_LOCALHOST_ONLY");
        if (expected == nullptr || std::string{expected} != "1")
        {
            return true;
        }

        if (provizio::dds::resolve_transport_mode(provizio::dds::transport_mode::automatic) !=
            provizio::dds::transport_mode::localhost_only)
        {
            std::cerr << "PROVIZIO_DDS_LOCALHOST_ONLY did not make the default participant localhost_only" << '\n';
            return false;
        }
        return true;
    }
}  // namespace ros_interop_test

#endif  // PROVIZIO_DDS_TEST_ROS_INTEROP_EXPECT_LOCALHOST_ONLY
