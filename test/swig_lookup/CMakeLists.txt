# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# provizio_dds_find_swig (cmake/python_bindings/swig_lookup.cmake): configures of a stand-in project
# with stand-in SWIGs, which are shell scripts, so on Linux and macOS only. Registered wherever
# tests are, whatever this tree was configured with: what it checks is the source, not this build.
if(WIN32)
    return()
endif()

add_test(NAME swig_lookup
    COMMAND "${CMAKE_COMMAND}"
            "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/swig_lookup"
            "-DGENERATOR=${CMAKE_GENERATOR}"
            "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/swig_lookup_test.cmake")
# Seconds. Twelve configures of a project that enables no language.
set_tests_properties(swig_lookup PROPERTIES TIMEOUT 60)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
