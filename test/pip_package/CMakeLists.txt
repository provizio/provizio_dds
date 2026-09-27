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

# A pip package always bundles provizio_dds's own, patched Fast-DDS, so its configure refuses
# LOOK_FOR_FAST_DDS (see the check near the top of the top-level CMakeLists.txt). Exercised as a
# real configure of this source tree, with the options setup.py passes for a pip package plus
# LOOK_FOR_FAST_DDS=TRUE, on every platform a pip package is built on. The check comes ahead of
# project(), so the configure stops within a second: no compiler is looked for, and nothing is
# downloaded or built. Registered wherever tests are, whatever this tree itself was configured
# with -- the check is about the source, not about this build.
add_test(NAME pip_package_refuses_system_fast_dds
    COMMAND "${CMAKE_COMMAND}" -S "${PROJECT_SOURCE_DIR}" -B "${CMAKE_CURRENT_BINARY_DIR}/refused"
            -G "${CMAKE_GENERATOR}"
            -DPYTHON_BINDINGS=ON
            "-DPYTHON_PACKAGES_INSTALL_DIR=${CMAKE_CURRENT_BINARY_DIR}/refused/packages"
            -DLOOK_FOR_FAST_DDS=TRUE)
# The check's own words, a prefix short enough that CMake's wrapping of a long error message cannot
# split it. Matching the output rather than the exit code is the point: a configure that failed for
# any other reason must not count as the refusal.
set_tests_properties(pip_package_refuses_system_fast_dds PROPERTIES
    PASS_REGULAR_EXPRESSION "LOOK_FOR_FAST_DDS cannot be used"
    # Seconds. The configure is refused before project(); the ceiling only has to outlast a
    # loaded CI runner starting CMake.
    TIMEOUT 60)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
