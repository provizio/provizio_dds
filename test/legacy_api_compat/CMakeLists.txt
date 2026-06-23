# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# C++ source-compat test: the public types provizio_dds 1.10.x consumer code
# referenced via `eprosima::fastrtps::rtps::*` must still resolve against
# Fast-DDS 3.x. The bulk of the assertions are compile-time static_asserts;
# the executable also runs as a quick ctest smoke check that nothing was
# accidentally inlined away.
add_executable(legacy_api_compat_test legacy_api_compat_test.cpp)
target_link_libraries(legacy_api_compat_test PUBLIC provizio_dds)

add_test(NAME legacy_api_compat_cpp_test COMMAND legacy_api_compat_test)
if(WIN32)
    set_tests_properties(legacy_api_compat_cpp_test PROPERTIES TIMEOUT 30 ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set_tests_properties(legacy_api_compat_cpp_test PROPERTIES TIMEOUT 10)
endif()

# Scale every test TIMEOUT in this directory when a sanitizer is active (no-op otherwise).
provizio_dds_scale_test_timeouts()
