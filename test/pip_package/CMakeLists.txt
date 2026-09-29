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

# A pip package (PYTHON_PIP_PACKAGE, which setup.py sets) always bundles provizio_dds's own, patched
# Fast-DDS, with everything it loads. Both tests are real configures, run in script mode by a driver
# that makes their binary directories afresh, on every platform a pip package is built on, and are
# registered wherever tests are, whatever this tree itself was configured with: what they check is
# the source, not this build.

# The configure of a pip package refuses LOOK_FOR_FAST_DDS, and a configure that is no pip package is
# left alone (see the check near the top of the top-level CMakeLists.txt).
add_test(NAME pip_package_refuses_system_fast_dds
    COMMAND "${CMAKE_COMMAND}"
            "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/refuses_system_fast_dds"
            "-DGENERATOR=${CMAKE_GENERATOR}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/refuses_system_fast_dds_test.cmake")
# Seconds. Three configures, each stopped before or at project(); the ceiling only has to outlast a
# loaded CI runner starting CMake.
set_tests_properties(pip_package_refuses_system_fast_dds PROPERTIES TIMEOUT 60)

# The Fast-DDS of a pip package takes foonathan_memory from the vendored build only
add_test(NAME pip_package_vendored_foonathan_memory
    COMMAND "${CMAKE_COMMAND}"
            "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/vendored_foonathan_memory"
            "-DGENERATOR=${CMAKE_GENERATOR}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/vendored_foonathan_memory_test.cmake")
# Seconds. Two configures of a project that enables no language.
set_tests_properties(pip_package_vendored_foonathan_memory PROPERTIES TIMEOUT 60)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
