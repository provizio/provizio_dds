# Copyright 2023 Provizio Ltd.
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

add_subdirectory(reliable_publisher)
add_subdirectory(reliable_subscriber)

add_test(NAME reliable_pub_sub COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:reliable_publisher>\""
    "\"$<TARGET_FILE:reliable_subscriber>\""
)
if(WIN32)
    set_tests_properties(reliable_pub_sub PROPERTIES TIMEOUT 90 ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set_tests_properties(reliable_pub_sub PROPERTIES TIMEOUT 30)
endif()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
