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
# the PROVIZIO_DDS_MAX_MESSAGE_SIZE env override, the fallback to the default on
# malformed input, and the loopback confinement of transport_mode::localhost_only.
# Each subcommand is its own ctest entry so per-case failure stays isolated —
# same pattern as the discovery_tuning test.
add_executable(transport_tuning_test transport_tuning_test.cpp)
# Threads::Threads because the cases synchronise their log-callback captures with a
# std::mutex: on a glibc older than 2.34 (Ubuntu 20.04, which the jetson runners are)
# the pthread symbols libstdc++ needs for that still live outside libc, so a target
# linking provizio_dds alone fails to link there while linking cleanly on every newer
# host. Every other test target here does the same, for the same reason.
find_package(Threads REQUIRED)
target_link_libraries(transport_tuning_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)
# test/ on the include path for detail/test_domain.h (the per-process DDS domain policy).
target_include_directories(transport_tuning_test PRIVATE "${CMAKE_SOURCE_DIR}/test")

# Per-OS timeouts. Most cases create a participant or two and only read the QoS
# back, so they return near-instantly. localhost_only_round_trip and same_host_by_machine_id are the
# exceptions: they wait for real discovery and delivery under their own 30 s deadline, which these
# ceilings must stay clear of so a stuck case reports itself before CTest kills it.
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

foreach(_case defaults env_override env_invalid localhost_only localhost_only_not_applied_warns
        localhost_only_caller_confinement_decides_warning udp_only_shared_memory_decides_warning
        localhost_only_round_trip same_host_by_machine_id dynamic_port_range_domain_warns)
    add_test(NAME transport_tuning_${_case}
             COMMAND "$<TARGET_FILE:transport_tuning_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:transport_tuning_test>")
    if(_TT_ENV)
        set_tests_properties(transport_tuning_${_case} PROPERTIES TIMEOUT ${_TT_TIMEOUT} ENVIRONMENT "${_TT_ENV}")
    else()
        set_tests_properties(transport_tuning_${_case} PROPERTIES TIMEOUT ${_TT_TIMEOUT})
    endif()
endforeach()

# Skipped, not failed, where Fast-DDS can read no machine id (a container without /etc/machine-id): the
# case is about that id, and without it the library falls back to the comparison under test.
set_tests_properties(transport_tuning_same_host_by_machine_id PROPERTIES SKIP_RETURN_CODE 77)

# The discovery-filter case is registered on its own because it is the one case that must run WITH a
# caller's XML profile in force: Fast-DDS reads FASTDDS_DEFAULT_PROFILES_FILE when the factory is first
# used, so setting it inside the process would be too late, and the suite's main() clears an ambient one
# unless PROVIZIO_DDS_TEST_KEEP_XML_PROFILE says this case wants it.
provizio_dds_add_subcommand_test(transport_tuning_localhost_only_discovery_filter_warns
    TARGET transport_tuning_test
    ARGS localhost_only_discovery_filter_warns
    TIMEOUT ${_TT_TIMEOUT}
    ENV "PROVIZIO_DDS_TEST_KEEP_XML_PROFILE=1"
        "FASTDDS_DEFAULT_PROFILES_FILE=${CMAKE_CURRENT_SOURCE_DIR}/../fast_dds_localhost_profile.xml")

# PROVIZIO_DDS_LOCALHOST_ONLY, one process per value because the library reads it once: unset, an
# explicit off, an unrecognised value (all leave transport_mode::automatic alone) and on in two spellings.
provizio_dds_add_subcommand_test(transport_tuning_localhost_only_env_unset
    TARGET transport_tuning_test ARGS resolve_transport_mode automatic TIMEOUT ${_TT_TIMEOUT}
    ENV "PROVIZIO_DDS_LOCALHOST_ONLY=")  # empty counts as unset, and shields the case from an ambient value
provizio_dds_add_subcommand_test(transport_tuning_localhost_only_env_off
    TARGET transport_tuning_test ARGS resolve_transport_mode automatic TIMEOUT ${_TT_TIMEOUT}
    ENV "PROVIZIO_DDS_LOCALHOST_ONLY=off")
provizio_dds_add_subcommand_test(transport_tuning_localhost_only_env_invalid
    TARGET transport_tuning_test ARGS resolve_transport_mode automatic warns TIMEOUT ${_TT_TIMEOUT}
    ENV "PROVIZIO_DDS_LOCALHOST_ONLY=banana")
provizio_dds_add_subcommand_test(transport_tuning_localhost_only_env_on
    TARGET transport_tuning_test ARGS resolve_transport_mode localhost_only TIMEOUT ${_TT_TIMEOUT}
    ENV "PROVIZIO_DDS_LOCALHOST_ONLY=1")
# Surrounding whitespace is not part of the value (an env file with Windows line endings hands over "1\r").
provizio_dds_add_subcommand_test(transport_tuning_localhost_only_env_padded
    TARGET transport_tuning_test ARGS resolve_transport_mode localhost_only TIMEOUT ${_TT_TIMEOUT}
    ENV "PROVIZIO_DDS_LOCALHOST_ONLY=  on  ")
provizio_dds_add_subcommand_test(transport_tuning_localhost_only_env_true
    TARGET transport_tuning_test ARGS resolve_transport_mode localhost_only TIMEOUT ${_TT_TIMEOUT}
    ENV "PROVIZIO_DDS_LOCALHOST_ONLY=True")

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
