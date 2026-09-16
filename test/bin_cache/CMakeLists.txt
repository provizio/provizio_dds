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

# The binary cache key is derived by bin_cache_config_name.sh, which only Linux uses and runs
# (the Windows key comes from bin_cache_config_name.ps1, which reads the architecture from the
# environment and is not exposed to any of this). Registering it elsewhere would only ever
# report the script's own "Only Linux is currently supported for bin cache".
if(NOT UNIX OR APPLE)
    message(STATUS "The bin cache key is a Linux concern; skipping its tests here")
    return()
endif()

# Reads the repository and runs one shell script, so it needs neither the library nor the
# bindings. Note that a tests-enabled build never USES the cache (see the bin cache section of
# the top-level CMakeLists.txt), which is precisely why the key needs a test of its own: nothing
# else in this suite would notice it naming an archive that cannot exist.
add_test(NAME bin_cache_config_name
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/config_name_architecture_test.sh"
            "${PROJECT_SOURCE_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}/config_name_architecture"
)
# Seconds. It writes one small file, matches a dozen short strings and greps the tracked build
# scripts once; the ceiling only has to outlast a loaded CI runner.
set_tests_properties(bin_cache_config_name PROPERTIES TIMEOUT 60)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
