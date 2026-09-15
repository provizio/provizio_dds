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

#ifndef PROVIZIO_DDS_TEST_CPU_TIME
#define PROVIZIO_DDS_TEST_CPU_TIME

namespace provizio::dds::test
{
    /**
     * @brief CPU time this process has consumed, in seconds -- user plus system.
     *
     * What the cases that tell a blocking wait from a busy spin measure: a spin consumes the
     * whole of the wall time it covers, a wait consumes almost none of it.
     *
     * NOT std::clock(), which is what those cases used first and why this exists. The C
     * standard leaves its meaning largely to the implementation, and the Microsoft CRT
     * defines it as elapsed WALL time since the process started rather than processor time.
     * A spin detector built on it measures the same quantity twice on Windows, finds them
     * equal, and reports every wait as a spin -- which is exactly how it failed there.
     *
     * Declared here and defined in test_cpu_time.cpp so that the Windows headers it needs
     * stay in that one translation unit. They cannot be let anywhere near a test's other
     * includes: <windows.h> brings in wingdi.h, whose ERROR macro collides with the
     * severity constants in the generated rcl_interfaces/msg/Log.hpp, and its winsock.h
     * conflicts with the winsock2 Fast-DDS uses. Include order could be arranged to avoid
     * both, but only until someone reorders the includes.
     *
     * @return Seconds of CPU consumed, or 0 where the platform cannot be asked -- a test
     * comparing against a budget then sees no spin, which is the non-failing direction: a
     * diagnostic that cannot be taken must not invent a failure.
     */
    double process_cpu_seconds();
}  // namespace provizio::dds::test

#endif  // PROVIZIO_DDS_TEST_CPU_TIME
