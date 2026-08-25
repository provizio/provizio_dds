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

# Subcommand-driven test exe for participant transport tuning: the MTU-sized
# default of the send-side RTPS message-size cap (fastdds.max_message_size),
# the PROVIZIO_DDS_MAX_MESSAGE_SIZE env override, and the fallback to the
# default on malformed input. Each subcommand is its own ctest entry so
# per-case failure stays isolated — same pattern as the discovery_tuning test.
add_executable(transport_tuning_test transport_tuning_test.cpp)
target_link_libraries(transport_tuning_test PUBLIC provizio_dds provizio_dds_types)

# Per-OS timeouts. Every case creates one (or a handful of) participants and
# only reads its QoS back — no pub/sub traffic — so it returns near-instantly;
# the ceiling just tolerates loaded CI runners.
if(WIN32)
    set(_TT_TIMEOUT 90)
    # On Windows the test executable must locate the provizio_dds /
    # provizio_dds_types / Fast-DDS DLLs at launch; PROVIZIO_DDS_TEST_PATH
    # (assembled, with pre-escaped semicolons, in test/CMakeLists.txt) carries
    # those directories — mirror the discovery_tuning test.
    set(_TT_ENV "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_TT_TIMEOUT 60)
    set(_TT_ENV "")
endif()

foreach(_case defaults env_override env_invalid)
    add_test(NAME transport_tuning_${_case}
             COMMAND "$<TARGET_FILE:transport_tuning_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:transport_tuning_test>")
    if(_TT_ENV)
        set_tests_properties(transport_tuning_${_case} PROPERTIES TIMEOUT ${_TT_TIMEOUT} ENVIRONMENT "${_TT_ENV}")
    else()
        set_tests_properties(transport_tuning_${_case} PROPERTIES TIMEOUT ${_TT_TIMEOUT})
    endif()
endforeach()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
