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

# Find Python interpreter for cross-platform test scripts (run_parallel.py, run_times.py)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")

# On Windows, test executables need DLL directories in PATH.
# The ENVIRONMENT test property is a CMake list (semicolon-separated), so bare
# semicolons inside the PATH value would be misinterpreted as list separators.
# We must escape every semicolon in the system PATH before embedding it.
if(WIN32)
    string(REPLACE ";" "\\;" _SYSTEM_PATH_ESCAPED "$ENV{PATH}")
    set(PROVIZIO_DDS_TEST_PATH "$<TARGET_FILE_DIR:provizio_dds>\;$<TARGET_FILE_DIR:provizio_dds_types>")
    if(FAST_DDS_INSTALL_DIR)
        string(APPEND PROVIZIO_DDS_TEST_PATH "\;${FAST_DDS_INSTALL_DIR}/bin\;${FAST_DDS_INSTALL_DIR}/lib")
    endif()
    string(APPEND PROVIZIO_DDS_TEST_PATH "\;${_SYSTEM_PATH_ESCAPED}")
endif(WIN32)

add_subdirectory(simplest_pub_sub)
add_subdirectory(pub_sub_type_reuse)
add_subdirectory(reliable_pub_sub)
add_subdirectory(request_response)
add_subdirectory(request_response_concurrent)
add_subdirectory(ros_interop)
add_subdirectory(legacy_api_compat)

if(PYTHON_BINDINGS)
    message(STATUS "PYTHON_BINDINGS is ON: Python tests will be built and enabled.")
    add_subdirectory(python)
endif(PYTHON_BINDINGS)
