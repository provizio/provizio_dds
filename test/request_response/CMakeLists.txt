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

# service_client (created in advance; many requests, each returning its own future_response): reuse the
# existing x^2 services. Consecutive requests after an explicit wait_for_service():
add_test(NAME request_response_service_client COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\" serial_requests"
    "\"$<TARGET_FILE:request_response_service_client>\""
)
# Parallel requests fired immediately (exercises the auto-defer-until-matched path):
add_test(NAME request_response_concurrent_requests_service_client COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\""
    "\"$<TARGET_FILE:request_response_concurrent_requests_service_client>\""
)
# Parallel requests against an async service, with mid-flight future_response destruction:
add_test(NAME request_response_concurrent_service_client COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_concurrent_service>\" 7"
    "\"$<TARGET_FILE:request_response_concurrent_service_client>\""
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
        request_response_service_client
        request_response_concurrent_requests_service_client
        request_response_concurrent_service_client
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
        request_response_service_client
        request_response_concurrent_requests_service_client
        request_response_concurrent_service_client
        request_response_dual_service_test
        request_response_ignore
        request_response_ignore_async
        PROPERTIES ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}"
    )
else()
    # A test's own budget must stay STRICTLY under its ctest TIMEOUT, or its diagnostic is
    # unreachable. These four wait `30 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE` internally
    # (request_response_client.cpp:34, request_response_service_client.cpp:34,
    # request_response_concurrent_requests_service_client.cpp:35,
    # request_response_concurrent_service_client.cpp:37) against what used to be a TIMEOUT of
    # exactly 30, so CTest's SIGKILL landed at the same instant their own `wait_for_service`
    # would have returned false: the branch that prints WHY never ran, and a failure arrived as
    # a bare ***Timeout with no attribution. That is what a macos-15-intel failure on this
    # branch looked like, and only the harness's own 25 s process dump said anything useful --
    # it showed the client had completed and the SERVICE had not exited.
    #
    # 45 restores the ordering the rest of the suite relies on: internal budget 30 < deadline
    # report at TIMEOUT-5 = 40 < SIGKILL at 45. It also leaves room for the library's own
    # bounded listener detach, which is 30 s (domain_participant.cpp listener_swap_timeout) and
    # so could alone consume the whole of a 30 s budget.
    #
    # The others keep 30: their internal budgets are 10 s and 15 s, already clear of it.
    set_tests_properties(request_response PROPERTIES TIMEOUT 45)
    set_tests_properties(
        request_response_service_client
        request_response_concurrent_requests_service_client
        request_response_concurrent_service_client
        PROPERTIES TIMEOUT 45)
    set_tests_properties(
        request_response_concurrent_requests_serial_service
        request_response_dual_service_test
        PROPERTIES TIMEOUT 30)
    set_tests_properties(
        request_response_ignore
        request_response_ignore_async
        PROPERTIES TIMEOUT 30)
endif()

# ---------------------------------------------------------------------------------------
# Test isolation: ctest entries that share a DDS service/topic name must not run at once.
#
# Several entries below reuse the SAME service name on the SAME domain (14) -- by design,
# since they reuse the same service and client binaries. That is fine when they run one at a
# time, which is how CI runs them (`ctest` with no -j), but not under `ctest -j`: every
# client's request then reaches every service, so a service fills its request quota from a
# FOREIGN test's requests and exits before the paired client is done, and bounded request
# queues start dropping ("The service requests queue is full!"). Measured before this lock:
# 6-7 of the 9 request/response entries failed at -j 8; none failed run individually.
#
# RESOURCE_LOCK makes ctest serialise exactly the entries that share a name, leaving every
# other test free to run alongside them. The alternative -- giving each entry its own topic
# name or domain -- would preserve more parallelism but touches ~20 test programs; the
# reliability tests already do it that way (a per-test postfix plus a per-iteration domain)
# if a future test needs it.
# ---------------------------------------------------------------------------------------
set_tests_properties(
    request_response
    request_response_concurrent_requests_serial_service
    request_response_service_client
    request_response_concurrent_requests_service_client
    request_response_concurrent_service_client
    PROPERTIES RESOURCE_LOCK provizio_dds_test_request_response)
set_tests_properties(
    request_response_ignore
    request_response_ignore_async
    PROPERTIES RESOURCE_LOCK provizio_dds_test_request_response_ignore)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
