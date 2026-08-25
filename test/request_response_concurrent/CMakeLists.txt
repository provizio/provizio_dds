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

add_subdirectory(request_response_concurrent_service)
add_subdirectory(request_response_concurrent_client)

add_test(NAME request_response_concurrent COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_concurrent_service>\""
    "\"$<TARGET_FILE:request_response_concurrent_client>\""
)
if(WIN32)
    set_tests_properties(request_response_concurrent PROPERTIES TIMEOUT 90 ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set_tests_properties(request_response_concurrent PROPERTIES TIMEOUT 30)
endif()

# Scale every test TIMEOUT in this directory when a sanitizer is active (no-op otherwise).
# Shares the "provizio_dds_test_request_response" service name with the entries in
# test/request_response/, so it takes the same lock (see the note there).
set_tests_properties(request_response_concurrent
    PROPERTIES RESOURCE_LOCK provizio_dds_test_request_response)

provizio_dds_finalize_tests()
