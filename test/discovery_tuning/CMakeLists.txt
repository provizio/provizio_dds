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

# Subcommand-driven test exe for participant auto-discovery tuning: the
# de-escalated discovery defaults, the PROVIZIO_DDS_DISCOVERY_* env overrides,
# and many-participant discovery convergence. Each subcommand is its own ctest
# entry so per-case failure stays isolated — same pattern as the
# match_publisher_default test.
add_executable(discovery_tuning_test discovery_tuning_test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(discovery_tuning_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)

# Per-OS timeouts. The multi_participant case spins up 2*K participants and waits
# for the full discovery mesh to converge (its own 60s internal budget); the
# remaining cases create a single participant and return near-instantly. The
# ctest ceiling sits above the internal convergence budget.
if(WIN32)
    set(_DT_TIMEOUT 120)
    # On Windows the test executable must locate the provizio_dds /
    # provizio_dds_types / Fast-DDS DLLs at launch; PROVIZIO_DDS_TEST_PATH
    # (assembled, with pre-escaped semicolons, in test/CMakeLists.txt) carries
    # those directories — mirror the match_publisher_default test.
    set(_DT_ENV "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_DT_TIMEOUT 90)
    set(_DT_ENV "")
endif()

foreach(_case defaults env_overrides env_invalid multi_participant)
    add_test(NAME discovery_tuning_${_case}
             COMMAND "$<TARGET_FILE:discovery_tuning_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:discovery_tuning_test>")
    if(_DT_ENV)
        set_tests_properties(discovery_tuning_${_case} PROPERTIES TIMEOUT ${_DT_TIMEOUT} ENVIRONMENT "${_DT_ENV}")
    else()
        set_tests_properties(discovery_tuning_${_case} PROPERTIES TIMEOUT ${_DT_TIMEOUT})
    endif()
endforeach()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
