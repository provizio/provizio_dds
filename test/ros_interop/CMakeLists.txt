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
        # RESOURCE_LOCK: serialised with the localhost-only cases below, which use the same topic -- a
        # default talker still publishing would otherwise be what a localhost-only subscriber hears.
        set_tests_properties(ros_interop_subscriber ros_interop_publisher
            PROPERTIES ENVIRONMENT "FASTDDS_DEFAULT_PROFILES_FILE="
                       RESOURCE_LOCK provizio_dds_test_ros_interop)

        # The same exchanges with ROS 2's own localhost-only mode on the native node and
        # transport_mode::localhost_only on the provizio side (selected by the library's own
        # PROVIZIO_DDS_LOCALHOST_ONLY variable, so the binaries are the ones of the cases above). The
        # native node confines itself without any host filter (rmw_fastrtps: a loopback UDP allowlist on
        # Humble, no multicast plus a 127.0.0.1 peer on Iron and later), the provizio side by transport
        # AND by refusing participants it takes for another host's (FILTER_DIFFERENT_HOST): the two must
        # still find each other on this one.
        # ROS_LOCALHOST_ONLY is what ROS 2 Humble reads and ROS_AUTOMATIC_DISCOVERY_RANGE what Iron and later
        # do (where the former is deprecated): giving both selects localhost-only on every distribution. They
        # are set on the ROS command alone -- the provizio side must not see them. Both processes run in the
        # same container, as in the cases above.
        set(ROS_LOCALHOST_ONLY_ENV "ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
        # Not on ROS 2 Iron. Its localhost mode, like Jazzy's, leaves UDP unconfined, so the node announces only its
        # non-loopback addresses. Jazzy's Fast-DDS (2.14) also announces that it accepts localhost, which lets a
        # loopback-confined peer send to 127.0.0.1 instead; Iron's (2.10, before 2.12) does not. A transport
        # confined to 127.0.0.1 may then send to none of the addresses it is given, and Fast-DDS discards the peer
        # before discovery reports it -- no host filter is involved, a plain loopback XML profile fails the same
        # way. Humble is unaffected: its own loopback allowlist makes it announce 127.0.0.1. (A ROS 2 node of
        # either distribution left in its default mode is no better off than Iron's localhost mode, but those
        # combinations are not part of this suite.)
        if(NOT "$ENV{ROS_DISTRO}" STREQUAL "iron")
            add_test(NAME ros_interop_subscriber_localhost_only COMMAND
                "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
                "\"$<TARGET_FILE:ros_interop_subscriber>\""
                "--ros:${ROS_LOCALHOST_ONLY_ENV} timeout 8 ros2 run demo_nodes_cpp talker || true"
            )
            add_test(NAME ros_interop_publisher_localhost_only COMMAND
                "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
                "\"$<TARGET_FILE:ros_interop_publisher>\""
                "--ros:${ROS_LOCALHOST_ONLY_ENV} python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_publisher/ros_subscriber.py"
            )
            # Serialised with each other and with the request/response cases below: the topic and node names are
            # the ones of the cases above, and a native node of one case must not be discovered by the provizio
            # side of another.
            set_tests_properties(ros_interop_subscriber_localhost_only ros_interop_publisher_localhost_only
                PROPERTIES ENVIRONMENT "FASTDDS_DEFAULT_PROFILES_FILE=;PROVIZIO_DDS_LOCALHOST_ONLY=1;PROVIZIO_DDS_TEST_EXPECT_LOCALHOST_ONLY=1"
                           RESOURCE_LOCK provizio_dds_test_ros_interop)
        else()
            message(STATUS "ROS 2 Iron announces neither a loopback locator nor that it accepts localhost; skipping the localhost-only ROS interoperability cases")
        endif()

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

            # Same again under ROS 2's localhost-only mode (see the subscriber/publisher cases above)
            add_test(NAME ros_interop_service_localhost_only COMMAND
                "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
                "\"$<TARGET_FILE:ros_interop_service>\""
                "--ros:${ROS_LOCALHOST_ONLY_ENV} python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_service/ros_interop_client.py"
            )
            add_test(NAME ros_interop_request_localhost_only COMMAND
                "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
                "--ros:${ROS_LOCALHOST_ONLY_ENV} python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros_interop_request/ros_interop_service.py"
                "\"$<TARGET_FILE:ros_interop_request>\""
            )
            set_tests_properties(ros_interop_service_localhost_only ros_interop_request_localhost_only
                PROPERTIES ENVIRONMENT "FASTDDS_DEFAULT_PROFILES_FILE=;PROVIZIO_DDS_LOCALHOST_ONLY=1;PROVIZIO_DDS_TEST_EXPECT_LOCALHOST_ONLY=1"
                           RESOURCE_LOCK provizio_dds_test_ros_interop)
        else(RMW_FASTRPS_VERSION_RETURN_CODE EQUAL 0 AND RMW_FASTRPS_VERSION VERSION_GREATER_EQUAL "8.4.0")
            message(WARNING "ROS2 Jazzy+ is required for ROS Interop Request/Response but ROS 2 $ENV{ROS_DISTRO} is sourced. Appropriate tests will be skipped.")
        endif(RMW_FASTRPS_VERSION_RETURN_CODE EQUAL 0 AND RMW_FASTRPS_VERSION VERSION_GREATER_EQUAL "8.4.0")
    endif(WIN32)
else(ENABLE_ROS_TESTS)
    message(STATUS "Excluding ROS-interoperability tests as ENABLE_ROS_TESTS is OFF")
endif(ENABLE_ROS_TESTS)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
