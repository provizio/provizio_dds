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

#include <cstdio>
#include <iostream>

namespace
{
    // Linked into every C++ test executable (see link_libraries in test/CMakeLists.txt), so
    // that a test killed part-way still shows what it had already said.
    //
    // CTest captures a test's stdout through a pipe, which makes it FULLY buffered: a test
    // that writes its result with '\n' and is then SIGKILLed at its TIMEOUT -- which is
    // exactly how CTest ends a timed-out test -- loses every line it had written. The
    // failure that reads as "***Timeout 45.03 sec" followed by no output at all is
    // indistinguishable from a test that hung before printing anything, and those two need
    // very different explanations.
    //
    // Both halves are needed: unitbuf flushes std::cout after each insertion, and the
    // setvbuf covers anything that reaches stdout through C stdio (the C++ streams are tied
    // to it by default). stderr is already unbuffered by the standard.
    const struct unbuffered_test_output
    {
        unbuffered_test_output() noexcept
        {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) -- setvbuf is the only API for this.
            static_cast<void>(std::setvbuf(stdout, nullptr, _IONBF, 0));
            std::cout.setf(std::ios::unitbuf);
        }
    } installer;
}  // namespace
