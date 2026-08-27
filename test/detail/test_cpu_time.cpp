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

// The platform headers live here and nowhere else -- see test_cpu_time.h for why they may
// not reach a translation unit that also includes the DDS headers.
#if defined(_WIN32)
// winsock2.h ahead of windows.h, the order src/address_snapshot_windows.cpp uses too.
#include <winsock2.h>

#include <windows.h>
#else
// clock_gettime, CLOCK_PROCESS_CPUTIME_ID and the timespec it fills are POSIX, declared by
// <time.h>; <ctime> is only required to carry the ISO C subset, so it is not the header to
// rely on for them.
#include <time.h>  // NOLINT(modernize-deprecated-headers,hicpp-deprecated-headers)
#endif

#include "detail/test_cpu_time.h"

namespace provizio::dds::test
{
    namespace
    {
#if defined(_WIN32)
        /// The two halves of a FILETIME as one count of 100-nanosecond intervals. A free
        /// function rather than a lambda: MSVC insists on capturing a constexpr used inside
        /// one, which is what the shift width would otherwise be.
        unsigned long long filetime_ticks(const FILETIME &value)
        {
            constexpr unsigned int high_word_shift = 32;
            return (static_cast<unsigned long long>(value.dwHighDateTime) << high_word_shift) |
                   static_cast<unsigned long long>(value.dwLowDateTime);
        }
#endif
    }  // namespace

    double process_cpu_seconds()
    {
#if defined(_WIN32)
        FILETIME creation_time{};
        FILETIME exit_time{};
        FILETIME kernel_time{};
        FILETIME user_time{};
        if (GetProcessTimes(GetCurrentProcess(), &creation_time, &exit_time, &kernel_time, &user_time) == 0)
        {
            return 0.0;
        }
        constexpr double ticks_per_second = 10000000.0;
        return static_cast<double>(filetime_ticks(kernel_time) + filetime_ticks(user_time)) / ticks_per_second;
#else
        timespec spent{};
        if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &spent) != 0)
        {
            return 0.0;
        }
        constexpr double nanoseconds_per_second = 1000000000.0;
        return static_cast<double>(spent.tv_sec) + (static_cast<double>(spent.tv_nsec) / nanoseconds_per_second);
#endif
    }
}  // namespace provizio::dds::test
