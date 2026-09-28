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

# Tests for scripts CI itself runs, from .github/workflows/. They need neither the library nor the
# bindings, only bash and the usual command-line tools -- and Linux, the only place CI runs them:
# the scripts rely on bash 4, which macOS does not ship.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "The CI scripts run on Linux only; skipping their tests here")
    return()
endif()

# resolve_ros_base_image.sh decides which image every job of the ROS 2 compatibility matrix runs in,
# so a mistake in it costs the whole matrix. Run against stand-ins for docker and sleep.
add_test(NAME resolve_ros_base_image
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/resolve_ros_base_image_test.sh"
            "${PROJECT_SOURCE_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}/resolve_ros_base_image"
)
# Seconds. Twenty-four runs of the script against stand-ins that answer at once; the ceiling only has
# to outlast a loaded CI runner.
set_tests_properties(resolve_ros_base_image PROPERTIES TIMEOUT 60)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
