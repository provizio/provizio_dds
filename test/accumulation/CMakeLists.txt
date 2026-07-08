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

# Subcommand-driven test exe for provizio::dds::accumulation — a C++ mirror of
# test/python/accumulation_test.py (same golden constants). Non-DDS cases are
# pure maths; DDS-fed cases (_ACC_DDS_CASES) create in-process publisher +
# accumulator participants and need discovery time.
add_executable(accumulation_test accumulation_test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(accumulation_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)

# The accumulation headers auto-detect Eigen via __has_include at compile time (header-only feature): give this
# test target Eigen's include path when available, or the explicit disable macro when DISABLE_EIGEN=ON — so this
# very target is what exercises the chosen maths path.
if(NOT DISABLE_EIGEN AND TARGET Eigen3::Eigen)
    target_link_libraries(accumulation_test PUBLIC Eigen3::Eigen)
elseif(DISABLE_EIGEN)
    target_compile_definitions(accumulation_test PRIVATE PROVIZIO_DDS_DISABLE_EIGEN)
endif()

if(WIN32)
    set(_ACC_ENV "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_ACC_ENV "")
endif()

set(_ACC_CASES rigid_transform_golden zero_accumulated extrinsics move_no_extrinsics move_simple_extrinsics overflow localization_from_sensor_to_ego localization_correction move_simple_extrinsics_snr_filter snr_and_velocity_filters radar_filter rotation_yaw rotation_pitch rotation_roll rotation_yaw_with_extrinsics rotation_and_move_simple_no_extrinsics rotation_and_move_simple_with_extrinsics rotation_and_move_no_extrinsics rotation_and_move_with_extrinsics gps_utils_golden)
set(_ACC_DDS_CASES dds_simple dds_simple_extrinsics dds_simple_extrinsics_including_localization dds_localization_extrinsics_late dds_no_localization_no_extrinsics dds_no_localization_with_extrinsics dds_on_pc2_callback dds_nav_sat_fix_localization dds_kalman_localization dds_timesync_buffered_then_covered dds_timesync_timeout_extrapolation dds_timesync_interpolates_between_fixes dds_timesync_interpolates_orientation dds_timesync_drops_out_of_order_fix dds_timesync_stale_cloud_uses_previous_fix)

add_executable(localization_filter_test localization_filter_test.cpp)
target_link_libraries(localization_filter_test PUBLIC provizio_dds)

# The filter header includes accumulation.h, which includes accumulation_math.h; give the same
# Eigen visibility here as accumulation_test gets (header-only consumer choice).
if(NOT DISABLE_EIGEN AND TARGET Eigen3::Eigen)
    target_link_libraries(localization_filter_test PUBLIC Eigen3::Eigen)
elseif(DISABLE_EIGEN)
    target_compile_definitions(localization_filter_test PRIVATE PROVIZIO_DDS_DISABLE_EIGEN)
endif()

set(_LF_CASES cold_start constant_velocity constant_yaw_rate yaw_wrap)

foreach(_case ${_LF_CASES})
    add_test(NAME localization_filter_${_case}
             COMMAND "$<TARGET_FILE:localization_filter_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:localization_filter_test>")
    if(_ACC_ENV)
        set_tests_properties(localization_filter_${_case} PROPERTIES TIMEOUT 45 ENVIRONMENT "${_ACC_ENV}")
    else()
        set_tests_properties(localization_filter_${_case} PROPERTIES TIMEOUT 45)
    endif()
endforeach()

foreach(_case ${_ACC_CASES} ${_ACC_DDS_CASES})
    add_test(NAME accumulation_${_case}
             COMMAND "$<TARGET_FILE:accumulation_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:accumulation_test>")
    # DDS cases wait for cross-participant discovery (up to ~15s per publisher) — generous ceiling;
    # pure-maths cases share a lower one.
    if("${_case}" IN_LIST _ACC_DDS_CASES)
        set(_ACC_TIMEOUT 90)
    else()
        set(_ACC_TIMEOUT 45)
    endif()
    if(_ACC_ENV)
        set_tests_properties(accumulation_${_case} PROPERTIES TIMEOUT ${_ACC_TIMEOUT} ENVIRONMENT "${_ACC_ENV}")
    else()
        set_tests_properties(accumulation_${_case} PROPERTIES TIMEOUT ${_ACC_TIMEOUT})
    endif()
endforeach()

# Scale every test TIMEOUT in this directory (localization_filter + accumulation) when a sanitizer is
# active (no-op otherwise).
provizio_dds_scale_test_timeouts()
