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

        add_test(NAME ros_interop_subscriber COMMAND
            bash -c "${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && $<TARGET_FILE:ros_interop_subscriber>\" \"timeout 8 ros2 run demo_nodes_cpp talker || true\""
        )
        add_test(NAME ros_interop_publisher COMMAND
            bash -c "${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && $<TARGET_FILE:ros_interop_publisher>\" \"python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_publisher/ros_subscriber.py\""
        )

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
                bash -c "${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && $<TARGET_FILE:ros_interop_service>\" \"python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_service/ros_interop_client.py\""
            )
            add_test(NAME ros_interop_request COMMAND
                bash -c "${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_request/ros_interop_service.py\" \"source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && $<TARGET_FILE:ros_interop_request>\""
            )
        else(RMW_FASTRPS_VERSION_RETURN_CODE EQUAL 0 AND RMW_FASTRPS_VERSION VERSION_GREATER_EQUAL "8.4.0")
            message(WARNING "ROS2 Jazzy+ is required for ROS Interop Request/Response but ROS 2 $ENV{ROS_DISTRO} is sourced. Appropriate tests will be skipped.")
        endif(RMW_FASTRPS_VERSION_RETURN_CODE EQUAL 0 AND RMW_FASTRPS_VERSION VERSION_GREATER_EQUAL "8.4.0")
    endif(WIN32)
else(ENABLE_ROS_TESTS)
    message(STATUS "Excluding ROS-interoperability tests as ENABLE_ROS_TESTS is OFF")
endif(ENABLE_ROS_TESTS)
