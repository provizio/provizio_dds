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

add_subdirectory(request_response_service)
add_subdirectory(request_response_client)
add_subdirectory(request_response_dual_service)

add_test(NAME request_response COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\" serial_requests"
    "\"$<TARGET_FILE:request_response_client>\""
)

# Add inter-iteration delay to let the OS release DDS resources (shared memory
# segments on Linux, UDP sockets/ports on all platforms) between rapid
# participant create/destroy cycles.
set(RELIABILITY_DELAY_ARGS "--delay" "0.5")

add_test(NAME request_response_reliability COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_times.py" ${RELIABILITY_DELAY_ARGS} "128"
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_reliability_service>\" \"\" {i} 1 {i}"
    "\"$<TARGET_FILE:request_response_reliability_client>\" \"\" {i} {i}"
)
set_tests_properties(request_response_reliability PROPERTIES TIMEOUT 2000) # Takes long by design

add_test(NAME request_response_reliability_single_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_reliability_service>\" _single_service 1 128 200"
    "\"${PYTHON_EXECUTABLE}\" \"${CMAKE_SOURCE_DIR}/test/run_times.py\" 128 \"$<TARGET_FILE:request_response_reliability_client>\" _single_service {i} 200"
)
set_tests_properties(request_response_reliability_single_service PROPERTIES TIMEOUT 2000) # Takes long by design

add_test(NAME request_response_concurrent_requests_serial_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\""
    "\"$<TARGET_FILE:request_response_concurrent_requests>\""
)

add_test(NAME request_response_dual_service_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_dual_service>\""
)

add_test(NAME request_response_ignore COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_ignore_service>\""
    "\"$<TARGET_FILE:request_response_ignore_client>\""
)

add_test(NAME request_response_ignore_async COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_ignore_service_async>\""
    "\"$<TARGET_FILE:request_response_ignore_client>\""
)

# Platform-specific timeouts: tighter on Linux/macOS to detect regressions,
# wider on Windows where DDS discovery and process startup are slower.
if(WIN32)
    set_tests_properties(request_response PROPERTIES TIMEOUT 60)
    set_tests_properties(
        request_response_concurrent_requests_serial_service
        request_response_dual_service_test
        PROPERTIES TIMEOUT 60)
    set_tests_properties(
        request_response_ignore
        request_response_ignore_async
        PROPERTIES TIMEOUT 90)
    # PATH: DLLs must be locatable at runtime.
    set_tests_properties(
        request_response
        request_response_reliability
        request_response_reliability_single_service
        request_response_concurrent_requests_serial_service
        request_response_dual_service_test
        request_response_ignore
        request_response_ignore_async
        PROPERTIES ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}"
    )
else()
    set_tests_properties(request_response PROPERTIES TIMEOUT 30)
    set_tests_properties(
        request_response_concurrent_requests_serial_service
        request_response_dual_service_test
        PROPERTIES TIMEOUT 30)
    set_tests_properties(
        request_response_ignore
        request_response_ignore_async
        PROPERTIES TIMEOUT 30)
endif()
