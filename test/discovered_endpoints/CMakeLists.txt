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

# Subcommand-driven test exe for domain_participant's discovery callback /
# type-registry / visit_known_type additions. Each subcommand is its own ctest
# entry so per-case failure stays isolated.
add_executable(discovered_endpoints_test discovered_endpoints_test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(discovered_endpoints_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)
# /bigobj for the dispatcher header on MSVC comes in transitively as an
# INTERFACE compile option on provizio_dds_types (which provizio_dds links
# PUBLIC, so consumers of either target inherit it) — see the comment in the
# top-level CMakeLists.txt next to that target_compile_options call.

# Per-OS timeouts: the deterministic subcommands (type_registry,
# visit_known_type) finish in milliseconds; the DDS-discovery subcommands have
# an internal 15s wait and so need a slightly larger ctest timeout to cover
# discovery jitter on loaded runners. `unregister` chains three such waits
# sequentially (pre-unregister, the control participant observing the
# post-unregister publisher, then post-re-register), so it gets double the DDS
# ceiling to preserve the same overhead margin under that 3x worst case.
if(WIN32)
    set(_DE_TIMEOUT_FAST 30)
    set(_DE_TIMEOUT_DDS 60)
    set(_DE_TIMEOUT_UNREGISTER 120)
    # On Windows the test executable must locate the provizio_dds /
    # provizio_dds_types / Fast-DDS DLLs at launch; without them on PATH the
    # process can't start and ctest reports exit code 0xC0000135
    # (STATUS_DLL_NOT_FOUND). PROVIZIO_DDS_TEST_PATH (assembled, with
    # pre-escaped semicolons, in test/CMakeLists.txt) carries those
    # directories — mirror the network_recovery / simplest_pub_sub tests.
    set(_DE_ENV "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_DE_TIMEOUT_FAST 15)
    set(_DE_TIMEOUT_DDS 45)
    set(_DE_TIMEOUT_UNREGISTER 90)
    set(_DE_ENV "")
endif()

foreach(_case type_registry visit_known_type discovers_writer kind_filter constructor_callback callback_receives_participant survives_reset unregister)
    add_test(NAME discovered_endpoints_${_case}
             COMMAND "$<TARGET_FILE:discovered_endpoints_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:discovered_endpoints_test>")
    if(_case STREQUAL "type_registry" OR _case STREQUAL "visit_known_type")
        set(_de_timeout ${_DE_TIMEOUT_FAST})
    elseif(_case STREQUAL "unregister")
        set(_de_timeout ${_DE_TIMEOUT_UNREGISTER})
    else()
        set(_de_timeout ${_DE_TIMEOUT_DDS})
    endif()
    if(_DE_ENV)
        set_tests_properties(discovered_endpoints_${_case} PROPERTIES TIMEOUT ${_de_timeout} ENVIRONMENT "${_DE_ENV}")
    else()
        set_tests_properties(discovered_endpoints_${_case} PROPERTIES TIMEOUT ${_de_timeout})
    endif()
endforeach()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
