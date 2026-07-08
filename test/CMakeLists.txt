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

# Find Python interpreter for cross-platform test scripts (run_parallel.py, run_times.py)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")

# On Windows, test executables need DLL directories in PATH.
# The ENVIRONMENT test property is a CMake list (semicolon-separated), so bare
# semicolons inside the PATH value would be misinterpreted as list separators.
# We must escape every semicolon in the system PATH before embedding it.
if(WIN32)
    string(REPLACE ";" "\\;" _SYSTEM_PATH_ESCAPED "$ENV{PATH}")
    set(PROVIZIO_DDS_TEST_PATH "$<TARGET_FILE_DIR:provizio_dds>\;$<TARGET_FILE_DIR:provizio_dds_types>")
    if(FAST_DDS_INSTALL_DIR)
        string(APPEND PROVIZIO_DDS_TEST_PATH "\;${FAST_DDS_INSTALL_DIR}/bin\;${FAST_DDS_INSTALL_DIR}/lib")
    endif()
    string(APPEND PROVIZIO_DDS_TEST_PATH "\;${_SYSTEM_PATH_ESCAPED}")
endif(WIN32)

# Sanitizer builds run several times slower, so scale every test's TIMEOUT accordingly — but ONLY
# when a sanitizer is active; uninstrumented builds keep their original (tight, regression-catching)
# timeouts. PROVIZIO_DDS_SANITIZER_FLAGS is set by the top-level CMakeLists from the active
# coding_standards sanitizer. Each test subdirectory calls provizio_dds_scale_test_timeouts() at its
# end — a parent directory cannot modify a subdirectory's test properties, so the scaling must run in
# the directory where the tests were registered — and it multiplies the TIMEOUT of every test there.
set(PROVIZIO_DDS_TEST_TIMEOUT_SCALE 1)
if(PROVIZIO_DDS_SANITIZER_FLAGS)
    set(PROVIZIO_DDS_TEST_TIMEOUT_SCALE 5)
endif()
# Expose the scale to test SOURCE too: some tests have their own internal "wait for an async DDS
# operation to complete" deadlines (e.g. wait_for_service / wait_for(response)) that the outer CTest
# TIMEOUT scaling below does not cover. Those tests multiply ONLY such completion deadlines by this
# macro — never deliberate delays or negative ("must not happen within T") windows. Defined for every
# test target (value 1 unless a sanitizer is active) so the source never needs an #ifndef fallback.
add_compile_definitions(PROVIZIO_DDS_TEST_TIMEOUT_SCALE=${PROVIZIO_DDS_TEST_TIMEOUT_SCALE})
function(provizio_dds_scale_test_timeouts)
    if(NOT PROVIZIO_DDS_TEST_TIMEOUT_SCALE GREATER 1)
        return()
    endif()
    get_property(_provizio_dds_tests DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY TESTS)
    foreach(_provizio_dds_test ${_provizio_dds_tests})
        get_property(_provizio_dds_to TEST "${_provizio_dds_test}" PROPERTY TIMEOUT)
        if(_provizio_dds_to)
            math(EXPR _provizio_dds_nto "${_provizio_dds_to} * ${PROVIZIO_DDS_TEST_TIMEOUT_SCALE}")
            set_property(TEST "${_provizio_dds_test}" PROPERTY TIMEOUT "${_provizio_dds_nto}")
        endif()
    endforeach()
endfunction()

add_subdirectory(simplest_pub_sub)
add_subdirectory(pub_sub_type_reuse)
add_subdirectory(reliable_pub_sub)
add_subdirectory(request_response)
add_subdirectory(request_response_concurrent)
add_subdirectory(ros_interop)
add_subdirectory(legacy_api_compat)
add_subdirectory(network_recovery)
add_subdirectory(discovered_endpoints)
add_subdirectory(match_publisher_default)
add_subdirectory(discovery_tuning)
add_subdirectory(callback_exceptions)
add_subdirectory(point_cloud2)
add_subdirectory(accumulation)

if(PYTHON_BINDINGS)
    message(STATUS "PYTHON_BINDINGS is ON: Python tests will be built and enabled.")
    add_subdirectory(python)
endif(PYTHON_BINDINGS)
