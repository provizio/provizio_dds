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

# Subcommand-driven test exe for the deadline arithmetic behind every caller-supplied
# timeout in the public API: that a near-max timeout saturates instead of overflowing into
# the past, that a deadline of time_point::max() blocks rather than spinning, and that an
# ordinary deadline still expires on time.
add_executable(bounded_wait_test bounded_wait_test.cpp "${CMAKE_SOURCE_DIR}/test/detail/test_cpu_time.cpp")
# Threads::Threads because two of the cases need a second thread to satisfy the predicate
# the first one is waiting on.
find_package(Threads REQUIRED)
target_link_libraries(bounded_wait_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)
# test/ on the include path for detail/test_cpu_time.h, whose process_cpu_seconds() is what
# the spin detection measures: std::clock() means elapsed WALL time in the Microsoft CRT.
target_include_directories(bounded_wait_test PRIVATE "${CMAKE_SOURCE_DIR}/test")

# The helper cases are threads, a condition variable and a clock; the two wiring cases
# create a matched pub/sub pair and a request/response service, so the ceiling covers
# discovery on a loaded runner.
if(WIN32)
    set(_BOUNDED_WAIT_TIMEOUT 120)
else()
    set(_BOUNDED_WAIT_TIMEOUT 90)
endif()

set(_BOUNDED_WAIT_CASES saturating_deadline_does_not_overflow
    deadline_reserving_does_not_overflow unbounded_deadline_waits_without_spinning
    coarse_deadline_is_not_already_passed deadline_expires
    # The two below drive real DDS endpoints through the public entry points, so they need
    # discovery time rather than the millisecond budget the helper cases run in.
    matched_counts_accept_an_unbounded_timeout future_response_accepts_an_unbounded_timeout)

foreach(_case ${_BOUNDED_WAIT_CASES})
    provizio_dds_add_subcommand_test(bounded_wait_${_case}
        TARGET bounded_wait_test
        ARGS ${_case}
        TIMEOUT ${_BOUNDED_WAIT_TIMEOUT}
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:bounded_wait_test>")
endforeach()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
