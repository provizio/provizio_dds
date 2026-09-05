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
# coding_standards sanitizer.
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

# How much of its own TIMEOUT a test is left to notice that it is stuck, before CTest steps in.
set(PROVIZIO_DDS_TEST_DEADLINE_MARGIN_SEC 5)

# Every test subdirectory calls this at its end — a parent directory cannot modify a
# subdirectory's test properties, so the finalisation has to run where the tests were
# registered. Two things happen here, both per test in the calling directory:
#
#   1. The TIMEOUT is multiplied by PROVIZIO_DDS_TEST_TIMEOUT_SCALE (a no-op outside
#      sanitizer builds).
#   2. PROVIZIO_DDS_TEST_DEADLINE_SEC is added to the test's environment, a few seconds
#      inside the resulting TIMEOUT. CTest kills a timed-out test with SIGKILL, which no
#      handler can catch (verified: a SIGTERM handler never runs), so a test that wants to
#      say what it was doing has to notice the deadline itself. run_parallel.py reads this
#      and dumps every surviving child's thread states while they can still be read; it
#      changes no verdict -- CTest still decides, because a test whose own budget runs close
#      to its TIMEOUT is slow rather than stuck. Every other test ignores the variable.
function(provizio_dds_finalize_tests)
    get_property(_provizio_dds_tests DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY TESTS)
    foreach(_provizio_dds_test ${_provizio_dds_tests})
        get_property(_provizio_dds_to TEST "${_provizio_dds_test}" PROPERTY TIMEOUT)

        if(_provizio_dds_to AND PROVIZIO_DDS_TEST_TIMEOUT_SCALE GREATER 1)
            math(EXPR _provizio_dds_to "${_provizio_dds_to} * ${PROVIZIO_DDS_TEST_TIMEOUT_SCALE}")
            set_property(TEST "${_provizio_dds_test}" PROPERTY TIMEOUT "${_provizio_dds_to}")
        endif()

        # Appended, never assigned: several tests carry their own ENVIRONMENT (a
        # PROVIZIO_DDS_* override, the Windows DLL PATH) that must survive this.
        get_property(_provizio_dds_env TEST "${_provizio_dds_test}" PROPERTY ENVIRONMENT)
        # The scale itself, because a Python test cannot read the compile definition above
        # and several of them scale their own waits by it. Exported for EVERY test in the
        # directory, whether or not it carries a TIMEOUT -- a test's own budget has to
        # follow the environment the way the ctest TIMEOUT does, and the silent fallback to
        # 1 this replaces is exactly the bug that made python_network_recovery_test's
        # watchdog fire early under a sanitizer. Deliberately outside the TIMEOUT guard
        # below: the five tests registered with no TIMEOUT read no deadline today, but a
        # deadline added to one of them tomorrow would inherit that same silent 1.
        list(APPEND _provizio_dds_env "PROVIZIO_DDS_TEST_TIMEOUT_SCALE=${PROVIZIO_DDS_TEST_TIMEOUT_SCALE}")
        # A deadline, on the other hand, only means something as a fraction of a TIMEOUT:
        # with no TIMEOUT there is nothing for a test to report itself ahead of.
        if(_provizio_dds_to)
            math(EXPR _provizio_dds_deadline "${_provizio_dds_to} - ${PROVIZIO_DDS_TEST_DEADLINE_MARGIN_SEC}")
            if(_provizio_dds_deadline GREATER 0)
                list(APPEND _provizio_dds_env "PROVIZIO_DDS_TEST_DEADLINE_SEC=${_provizio_dds_deadline}")
            endif()
        endif()
        set_property(TEST "${_provizio_dds_test}" PROPERTY ENVIRONMENT "${_provizio_dds_env}")
    endforeach()
endfunction()

# Helper: register one subcommand of a subcommand-driven test executable as a ctest entry.
# Several suites here are built that way — one binary, a subcommand per case, so each case
# gets a fresh process and cached one-shot state (an environment variable parsed once, a
# Meyers' singleton) cannot leak between assertions. They all need the same two things
# around every entry: a TIMEOUT, and on Windows a PATH carrying the provizio_dds /
# provizio_dds_types / Fast-DDS DLL directories, without which the executable does not even
# start. Defined once here rather than per directory so that stays true of all of them.
#
# provizio_dds_add_subcommand_test(<test-name>
#     TARGET <exe target>            # test executable to run
#     [ARGS <arg>...]                # subcommand and its arguments
#     TIMEOUT <seconds>              # ctest TIMEOUT, before sanitizer scaling
#     [ENV <NAME=VALUE>...]          # extra environment, appended to the Windows PATH entry
#     [WORKING_DIRECTORY <dir>])     # defaults to the executable's own directory
function(provizio_dds_add_subcommand_test name)
    set(options)
    set(one_value_args TARGET TIMEOUT WORKING_DIRECTORY)
    set(multi_value_args ARGS ENV)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT ARG_TARGET)
        message(FATAL_ERROR "provizio_dds_add_subcommand_test(${name}): TARGET is required")
    endif()
    if(NOT ARG_TIMEOUT)
        message(FATAL_ERROR "provizio_dds_add_subcommand_test(${name}): TIMEOUT is required")
    endif()

    # Defaulted rather than left to CTest, because "wherever CTest happens to start it" is
    # not a neutral choice for these tests: Fast-DDS' load_profiles() auto-loads any
    # DEFAULT_FASTDDS_PROFILES.xml sitting in the working directory, and this library treats
    # such a file as the caller declaring their own transports -- so the directory a case
    # runs from can decide whether the behaviour under test is even reachable. CTest's
    # default is the calling CMakeLists' binary directory, which coincides with the
    # executable's only under a single-config generator; under MSVC and Xcode the executable
    # lives a per-config level deeper, so the same registration would run from a different
    # directory on Windows than on Linux. Pinning it here makes every entry agree, and makes
    # an omission at a call site harmless rather than silently platform-dependent.
    if(NOT ARG_WORKING_DIRECTORY)
        set(ARG_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${ARG_TARGET}>")
    endif()
    add_test(NAME ${name} COMMAND "$<TARGET_FILE:${ARG_TARGET}>" ${ARG_ARGS}
             WORKING_DIRECTORY "${ARG_WORKING_DIRECTORY}")

    # PROVIZIO_DDS_TEST_PATH is assembled above with pre-escaped semicolons, so it survives
    # being placed in the ENVIRONMENT property (itself a CMake list).
    set(_env "")
    if(WIN32)
        set(_env "PATH=${PROVIZIO_DDS_TEST_PATH}")
    endif()
    foreach(_entry ${ARG_ENV})
        if(_env)
            list(APPEND _env "${_entry}")
        else()
            set(_env "${_entry}")
        endif()
    endforeach()

    if(_env)
        set_tests_properties(${name} PROPERTIES TIMEOUT ${ARG_TIMEOUT} ENVIRONMENT "${_env}")
    else()
        set_tests_properties(${name} PROPERTIES TIMEOUT ${ARG_TIMEOUT})
    endif()
endfunction()

# Every C++ test executable created below gets this object library's one global
# constructor, which turns off stdout buffering. CTest pipes a test's stdout, making it
# fully buffered, and kills a timed-out test with SIGKILL -- so without this, a test that
# had already printed its result loses that output entirely and its failure reads exactly
# like one that hung before printing anything. Applied through link_libraries() rather than
# per target so a test added later cannot forget it, and reset before the Python bindings
# subdirectory, whose targets are libraries loaded into an interpreter that does its own
# buffering.
add_library(provizio_dds_test_unbuffered_output OBJECT detail/unbuffered_test_output.cpp)
link_libraries(provizio_dds_test_unbuffered_output)

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
add_subdirectory(transport_tuning)
add_subdirectory(vpn_interfaces)
add_subdirectory(shm_cleanup)
add_subdirectory(callback_exceptions)
add_subdirectory(listener_drain)
add_subdirectory(bounded_wait)
add_subdirectory(keyless_topic_history)
add_subdirectory(point_cloud2)
add_subdirectory(accumulation)
add_subdirectory(source_text)

# See the link_libraries() above: from here on, targets are not test executables.
set_property(DIRECTORY PROPERTY LINK_LIBRARIES "")

if(PYTHON_BINDINGS)
    message(STATUS "PYTHON_BINDINGS is ON: Python tests will be built and enabled.")
    add_subdirectory(python)
endif(PYTHON_BINDINGS)
