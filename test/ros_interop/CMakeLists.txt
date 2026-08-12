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

# Check if ROS2 is available in the environment
execute_process(
    COMMAND ros2 -h
    OUTPUT_QUIET
    RESULT_VARIABLE ROS2_RETURN_CODE
)
if(ROS2_RETURN_CODE EQUAL 0)
    set(ENABLE_ROS_TESTS_DEFAULT ON)
else(ROS2_RETURN_CODE EQUAL 0)
    set(ENABLE_ROS_TESTS_DEFAULT OFF)
endif(ROS2_RETURN_CODE EQUAL 0)

# The default (detected) value can be overridden in CMake arguments or CMake cache file
set(ENABLE_ROS_TESTS "${ENABLE_ROS_TESTS_DEFAULT}" CACHE BOOL "Defines whether ROS-related tests to be enabled")

if(ENABLE_ROS_TESTS)
    if(WIN32)
        message(WARNING "ENABLE_ROS_TESTS is ON, but ROS tests don't support Windows at the moment, so ROS-interoperability tests will be excluded")
    else(WIN32)
        message(STATUS "Including ROS-interoperability tests as ENABLE_ROS_TESTS is ON")

        add_subdirectory(ros_interop_subscriber)
        add_subdirectory(ros_interop_publisher)
        add_subdirectory(ros_interop_service)
        add_subdirectory(ros_interop_request)

        # provizio_dds binaries run with ROS env cleaned (default), while
        # ROS commands use the "--ros:" prefix to keep the full ROS environment.
        add_test(NAME ros_interop_subscriber COMMAND
            "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
            "\"$<TARGET_FILE:ros_interop_subscriber>\""
            "--ros:timeout 8 ros2 run demo_nodes_cpp talker || true"
        )
        add_test(NAME ros_interop_publisher COMMAND
            "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
            "\"$<TARGET_FILE:ros_interop_publisher>\""
            "--ros:python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_publisher/ros_subscriber.py"
        )

        # These tests interoperate with a NATIVE ROS 2 node, which uses its default DDS transport
        # (rmw_fastrtps does not apply FASTDDS_DEFAULT_PROFILES_FILE to its QoS). The provizio side must use
        # the default transport too, or it cannot discover the native node. The suite-wide loopback profile
        # (exported in test.sh) would otherwise confine the provizio side to loopback-only and break the
        # interop — verified in the ROS2-compat containers, where the native talker publishes but the provizio
        # side receives nothing. Clearing the variable runs these provizio binaries with the default transport,
        # matching the native node (and matching the pre-profile behaviour these tests were written for).
        set_tests_properties(ros_interop_subscriber ros_interop_publisher
            PROPERTIES ENVIRONMENT "FASTDDS_DEFAULT_PROFILES_FILE=")

        # Request/Response are only compatible with ROS2 Jazzy+
        message(STATUS "Detecting rmw_fastrtps_cpp version...")
        execute_process(
            COMMAND bash -c "ros2 doctor --report | grep rmw_fastrtps_cpp | grep -oP 'local=\\K[0-9.]+'"
            OUTPUT_STRIP_TRAILING_WHITESPACE
            OUTPUT_VARIABLE RMW_FASTRPS_VERSION
            RESULT_VARIABLE RMW_FASTRPS_VERSION_RETURN_CODE
        )
        if(RMW_FASTRPS_VERSION_RETURN_CODE EQUAL 0 AND RMW_FASTRPS_VERSION VERSION_GREATER_EQUAL "8.4.0")
            message("ROS2 Jazzy+ detected, including ROS Interop Request/Response tests")
            add_test(NAME ros_interop_service COMMAND
                "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
                "\"$<TARGET_FILE:ros_interop_service>\""
                "--ros:python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_service/ros_interop_client.py"
            )
            add_test(NAME ros_interop_request COMMAND
                "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
                "--ros:python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_request/ros_interop_service.py"
                "\"$<TARGET_FILE:ros_interop_request>\""
            )
            # Same as subscriber/publisher above: interop with the native ROS 2 node needs the default transport.
            # Both entries use the same service name, and both run with the loopback profile
            # cleared (above) so they are not even confined to localhost -- serialise them.
            set_tests_properties(ros_interop_service ros_interop_request
                PROPERTIES ENVIRONMENT "FASTDDS_DEFAULT_PROFILES_FILE="
                           RESOURCE_LOCK provizio_dds_test_ros_interop)
        else(RMW_FASTRPS_VERSION_RETURN_CODE EQUAL 0 AND RMW_FASTRPS_VERSION VERSION_GREATER_EQUAL "8.4.0")
            message(WARNING "ROS2 Jazzy+ is required for ROS Interop Request/Response but ROS 2 $ENV{ROS_DISTRO} is sourced. Appropriate tests will be skipped.")
        endif(RMW_FASTRPS_VERSION_RETURN_CODE EQUAL 0 AND RMW_FASTRPS_VERSION VERSION_GREATER_EQUAL "8.4.0")
    endif(WIN32)
else(ENABLE_ROS_TESTS)
    message(STATUS "Excluding ROS-interoperability tests as ENABLE_ROS_TESTS is OFF")
endif(ENABLE_ROS_TESTS)

# Scale every test TIMEOUT in this directory when a sanitizer is active (no-op otherwise).
provizio_dds_scale_test_timeouts()
