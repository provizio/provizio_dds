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

set(ENABLE_ROS_TESTS ON CACHE BOOL "Defines whether ROS-related tests to be enabled")

if(ENABLE_ROS_TESTS)
    if(WIN32)
        message(WARNING "ENABLE_ROS_TESTS is ON, but ROS tests don't support Windows at the moment, so ROS-interoperability tests will be excluded")
    else(WIN32)
        message("Including ROS-interoperability tests as ENABLE_ROS_TESTS is ON")
        
        add_subdirectory(ros_interop_subscriber)
        add_subdirectory(ros_interop_publisher)

        add_test(NAME ros_interop_subscriber COMMAND
            sh -c "(sleep 0.5 && timeout 4 ros2 run demo_nodes_cpp talker) & $<TARGET_FILE:ros_interop_subscriber>"
        )
        add_test(NAME ros_interop_publisher COMMAND
            bash -c "( $<TARGET_FILE:ros_interop_publisher> & (sleep 1.5 && timeout 5 ros2 topic echo chatter>/tmp/provizio_test_ros_interop_publisher.out || true ) ) && grep provizio_test_ros_interop_publisher_says_hi /tmp/provizio_test_ros_interop_publisher.out || (echo \"ROS2 Received:\" && cat /tmp/provizio_test_ros_interop_publisher.out && exit 1)"
        )
    endif(WIN32)
else(ENABLE_ROS_TESTS)
    message("Excluding ROS-interoperability tests as ENABLE_ROS_TESTS is OFF")
endif(ENABLE_ROS_TESTS)
