# Copyright 2025 Provizio Ltd.
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

add_subdirectory(simplest_publisher_type_reuse)
add_subdirectory(simplest_subscriber_type_reuse)

add_test(NAME pub_sub_type_reuse COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "$<TARGET_FILE:simplest_publisher_type_reuse>"
    "$<TARGET_FILE:simplest_subscriber_type_reuse>"
)
if(WIN32)
    set_tests_properties(pub_sub_type_reuse PROPERTIES ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}")
endif(WIN32)
