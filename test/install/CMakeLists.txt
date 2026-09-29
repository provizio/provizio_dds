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

# Coverage for the install steps of provizio_dds's own, which neither the library nor the bindings
# are needed for.

# Where they write and remove files (cmake/install_root.cmake), everywhere: CMake in script mode.
add_test(NAME install_root
    COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/install_root"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/install_root_test.cmake")
# Seconds. A few files installed in script mode.
set_tests_properties(install_root PROPERTIES TIMEOUT 60)

# fully_qualified_fastdds_libs.sh, which runs on Linux only (INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS)
if(UNIX AND NOT APPLE)
    add_test(NAME fully_qualified_fastdds_libs
        COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/fully_qualified_fastdds_libs_test.sh"
                "${PROJECT_SOURCE_DIR}"
                "${CMAKE_CURRENT_BINARY_DIR}/fully_qualified_fastdds_libs"
                "${CMAKE_C_COMPILER}")
    # Seconds. Two tiny libraries compiled, and the script run twice over three files.
    set_tests_properties(fully_qualified_fastdds_libs PROPERTIES TIMEOUT 60)
endif()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
