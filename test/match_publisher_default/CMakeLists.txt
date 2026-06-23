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

# Subcommand-driven test exe for the "match publisher" reader-reliability
# default (deferred DataReader creation that adopts the discovered writer's
# reliability). Each subcommand is its own ctest entry so per-case failure
# stays isolated — same pattern as the discovered_endpoints test.
add_executable(match_publisher_default_test match_publisher_default_test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(match_publisher_default_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)

# Per-OS timeouts. Every subcommand either resolves a real cross-participant
# match (internal 15s wait, generous for loaded runners) or, for the deferral
# case, returns near-instantly; all are covered by the DDS ceiling here. The
# deferral case is fast but shares the ceiling for simplicity.
if(WIN32)
    set(_MP_TIMEOUT 60)
    # On Windows the test executable must locate the provizio_dds /
    # provizio_dds_types / Fast-DDS DLLs at launch; PROVIZIO_DDS_TEST_PATH
    # (assembled, with pre-escaped semicolons, in test/CMakeLists.txt) carries
    # those directories — mirror the discovered_endpoints / network_recovery
    # tests.
    set(_MP_ENV "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_MP_TIMEOUT 45)
    set(_MP_ENV "")
endif()

foreach(_case deferral adopts_reliable adopts_best_effort explicit_best_effort explicit_reliable participant_dropped heterogeneous_rederive)
    add_test(NAME match_publisher_default_${_case}
             COMMAND "$<TARGET_FILE:match_publisher_default_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:match_publisher_default_test>")
    if(_MP_ENV)
        set_tests_properties(match_publisher_default_${_case} PROPERTIES TIMEOUT ${_MP_TIMEOUT} ENVIRONMENT "${_MP_ENV}")
    else()
        set_tests_properties(match_publisher_default_${_case} PROPERTIES TIMEOUT ${_MP_TIMEOUT})
    endif()
endforeach()

# Scale every test TIMEOUT in this directory when a sanitizer is active (no-op otherwise).
provizio_dds_scale_test_timeouts()
