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

# Coverage for how the Fast-DDS of a pip package finds foonathan_memory: from the vendored build
# only, however visible one of the system's is (see PROVIZIO_DDS_FOONATHAN_MEMORY_VENDORED_ONLY in the
# top-level CMakeLists.txt). A shared one found first would be a library the wheel depends on without
# containing, and nothing would fail on the machine that built it.
#
# foonathan_lookup/ looks it up as the Fast-DDS build does, through cmake/modules, with a stand-in
# for a system foonathan_memory on CMAKE_PREFIX_PATH -- the shared library of a package
# configuration, as a ROS 2 distribution or a preinstalled Fast-DDS provides it -- and one for the
# vendored build, a static library where the find module looks for it. Once without the setting, to
# show the system's is visible there and taken, and once with it, where it must not be.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -DGENERATOR=<generator>
#         -P vendored_foonathan_memory_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT ${_var})
        message(FATAL_ERROR "vendored_foonathan_memory_test.cmake: ${_var} is required")
    endif()
endforeach()

file(REMOVE_RECURSE "${WORK_DIR}")

# The system's: a package configuration naming a shared library
set(_system_prefix "${WORK_DIR}/system")
file(WRITE "${_system_prefix}/lib/cmake/foonathan_memory/foonathan_memory-config.cmake" [=[
set(foonathan_memory_FOUND TRUE)
if(NOT TARGET foonathan_memory)
    get_filename_component(_library "${CMAKE_CURRENT_LIST_DIR}/../../libfoonathan_memory-0.7.3.so" ABSOLUTE)
    add_library(foonathan_memory SHARED IMPORTED)
    set_target_properties(foonathan_memory PROPERTIES IMPORTED_LOCATION "${_library}")
endif()
]=])
file(WRITE "${_system_prefix}/lib/libfoonathan_memory-0.7.3.so" "")

# The vendored build, where the find module looks for it: next to the binary directory of the build
# it runs in, as the Fast-DDS build's is next to cmake/foonathan_memory's install
foreach(_case IN ITEMS default pip_package)
    file(WRITE "${WORK_DIR}/${_case}/foonathan_memory/install/lib/libfoonathan_memory.a" "")
    file(WRITE "${WORK_DIR}/${_case}/foonathan_memory/install/include/foonathan/memory/config.hpp" "")
endforeach()

# Looks foonathan_memory up in <case>, with VENDORED_ONLY set as given, and leaves what the lookup
# said it took in _taken
function(_look_up case vendored_only)
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/foonathan_lookup"
            -B "${WORK_DIR}/${case}/build" -G "${GENERATOR}"
            "-DMODULES_DIR=${SOURCE_DIR}/cmake/modules" "-DCMAKE_PREFIX_PATH=${_system_prefix}"
            "-DPROVIZIO_DDS_FOONATHAN_MEMORY_VENDORED_ONLY=${vendored_only}"
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(NOT _result EQUAL 0 OR NOT _output MATCHES "foonathan_lookup: ([A-Z_]+) ([^\r\n]*)")
        message(FATAL_ERROR "Looking foonathan_memory up in case ${case} failed (exit ${_result}):\n${_output}")
    endif()
    set(_type "${CMAKE_MATCH_1}")
    # The find module names the vendored build relative to the binary directory, ".." and all
    get_filename_component(_location "${CMAKE_MATCH_2}" ABSOLUTE)
    set(_taken "${_type} ${_location}" PARENT_SCOPE)
endfunction()

_look_up(default OFF)
if(NOT _taken MATCHES "^SHARED_LIBRARY .*/system/lib/libfoonathan_memory-0\\.7\\.3\\.so$")
    message(FATAL_ERROR "Without PROVIZIO_DDS_FOONATHAN_MEMORY_VENDORED_ONLY the system's foonathan_memory "
        "should be the one taken, which is what shows this test's stand-in for it is visible, but the "
        "lookup took: ${_taken}")
endif()

_look_up(pip_package ON)
if(NOT _taken MATCHES "^STATIC_LIBRARY .*/pip_package/foonathan_memory/install/lib/libfoonathan_memory\\.a$")
    message(FATAL_ERROR "The lookup of a pip package's Fast-DDS took a foonathan_memory other than the "
        "vendored build's static library: ${_taken}")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "pip_package_vendored_foonathan_memory: the vendored foonathan_memory only, for a pip package")
