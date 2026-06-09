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

# Crash-safety of throwing user callbacks: a user callback that raises must be
# logged through the provizio_dds logging facility and must NOT abort the
# process. Each subcommand is its own ctest entry so per-case failure stays
# isolated — same pattern as the match_publisher_default test.
add_executable(callback_exceptions_test callback_exceptions_test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(callback_exceptions_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)

if(WIN32)
    set(_CBE_TIMEOUT 60)
    # On Windows the test must locate the provizio_dds / provizio_dds_types /
    # Fast-DDS DLLs at launch; PROVIZIO_DDS_TEST_PATH (assembled in
    # test/CMakeLists.txt) carries those directories.
    set(_CBE_ENV "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_CBE_TIMEOUT 45)
    set(_CBE_ENV "")
endif()

foreach(_case on_data on_has_publisher_changed service_handler)
    add_test(NAME callback_exceptions_${_case}
             COMMAND "$<TARGET_FILE:callback_exceptions_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:callback_exceptions_test>")
    if(_CBE_ENV)
        set_tests_properties(callback_exceptions_${_case} PROPERTIES TIMEOUT ${_CBE_TIMEOUT} ENVIRONMENT "${_CBE_ENV}")
    else()
        set_tests_properties(callback_exceptions_${_case} PROPERTIES TIMEOUT ${_CBE_TIMEOUT})
    endif()
endforeach()
