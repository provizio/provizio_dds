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

# Subcommand-driven test exe for the provizio::dds::point_cloud2 utility.
# Pure message-manipulation tests (no DDS discovery), so timeouts are short.
add_executable(point_cloud2_test point_cloud2_test.cpp)
target_link_libraries(point_cloud2_test PUBLIC provizio_dds provizio_dds_types)

if(WIN32)
    set(_PC2_ENV "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_PC2_ENV "")
endif()

set(_PC2_TIMEOUT 30)

foreach(_case basics radar_roundtrip cloud_view_basic cloud_view_endianness cloud_view_bounds read_radar_points create_cloud_tiers entities)
    add_test(NAME point_cloud2_${_case}
             COMMAND "$<TARGET_FILE:point_cloud2_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:point_cloud2_test>")
    if(_PC2_ENV)
        set_tests_properties(point_cloud2_${_case} PROPERTIES TIMEOUT ${_PC2_TIMEOUT} ENVIRONMENT "${_PC2_ENV}")
    else()
        set_tests_properties(point_cloud2_${_case} PROPERTIES TIMEOUT ${_PC2_TIMEOUT})
    endif()
endforeach()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
