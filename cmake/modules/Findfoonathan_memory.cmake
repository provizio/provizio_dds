# Copyright 2023 Provizio Ltd.
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

# The vendored one only, for a pip package: see PYTHON_PIP_PACKAGE in the top-level CMakeLists.txt.
# Whatever an earlier lookup made of foonathan_memory is not consulted either then.
if(PROVIZIO_DDS_FOONATHAN_MEMORY_VENDORED_ONLY)
    set(foonathan_memory_FOUND FALSE)
else()
    find_package(foonathan_memory QUIET NO_MODULE)
endif()

if(NOT foonathan_memory_FOUND)
    include("${CMAKE_CURRENT_LIST_DIR}/../glob_escape.cmake")
    set(foonathan_memory_DIR "${CMAKE_BINARY_DIR}/../foonathan_memory/install")
    provizio_dds_glob_escape(_foonathan_memory_pattern "${foonathan_memory_DIR}")

    # Check for foonathan_memory library in lib, lib64, or lib32
    set(foonathan_memory_LIB_DIR "")
    foreach(lib_subdir lib lib64 lib32)
        # Unix: libfoonathan_memory.a
        if(EXISTS "${foonathan_memory_DIR}/${lib_subdir}/libfoonathan_memory.a")
            set(foonathan_memory_LIB_DIR "${foonathan_memory_DIR}/${lib_subdir}")
            break()
        endif(EXISTS "${foonathan_memory_DIR}/${lib_subdir}/libfoonathan_memory.a")
        # Windows/MSVC: foonathan_memory-*.lib
        file(GLOB foonathan_memory_LIB_FILES "${_foonathan_memory_pattern}/${lib_subdir}/foonathan_memory-*.lib")
        if(foonathan_memory_LIB_FILES)
            set(foonathan_memory_LIB_DIR "${foonathan_memory_DIR}/${lib_subdir}")
            break()
        endif(foonathan_memory_LIB_FILES)
    endforeach(lib_subdir lib lib64 lib32)

    if(foonathan_memory_LIB_DIR)
        set(foonathan_memory_FOUND TRUE)
        # foonathan_memory_vendor v1.4+ installs headers under include/foonathan/memory/.
        # v1.3 used include/foonathan_memory/foonathan/memory/ (with a top-level
        # foonathan_memory parent dir). Detect which layout is present.
        if(EXISTS "${foonathan_memory_DIR}/include/foonathan/memory/config.hpp")
            set(foonathan_memory_INCLUDE_DIRS "${foonathan_memory_DIR}/include")
        else()
            set(foonathan_memory_INCLUDE_DIRS "${foonathan_memory_DIR}/include/foonathan_memory")
        endif()

        # Find the actual library file to set IMPORTED_LOCATION
        provizio_dds_glob_escape(_foonathan_memory_lib_pattern "${foonathan_memory_LIB_DIR}")
        if(WIN32)
            # On Windows, prefer .lib over .a (there may be both; the .a is a symlink)
            file(GLOB foonathan_memory_LIB_FILE "${_foonathan_memory_lib_pattern}/foonathan_memory-*.lib")
            if(NOT foonathan_memory_LIB_FILE)
                file(GLOB foonathan_memory_LIB_FILE "${_foonathan_memory_lib_pattern}/libfoonathan_memory*.a")
            endif(NOT foonathan_memory_LIB_FILE)
        else(WIN32)
            file(GLOB foonathan_memory_LIB_FILE "${_foonathan_memory_lib_pattern}/libfoonathan_memory*.a")
            if(NOT foonathan_memory_LIB_FILE)
                file(GLOB foonathan_memory_LIB_FILE "${_foonathan_memory_lib_pattern}/foonathan_memory-*.lib")
            endif(NOT foonathan_memory_LIB_FILE)
        endif(WIN32)

        # Use the first file found by the glob
        if(NOT foonathan_memory_LIB_FILE)
            message(FATAL_ERROR "foonathan_memory library directory found at ${foonathan_memory_LIB_DIR} but no library file matched")
        endif()
        list(GET foonathan_memory_LIB_FILE 0 foonathan_memory_LIB_FILE)

        # Create an imported target so that target_link_libraries(... foonathan_memory)
        # propagates include directories automatically (required by Fast-DDS)
        if(NOT TARGET foonathan_memory)
            add_library(foonathan_memory STATIC IMPORTED)
            set_target_properties(foonathan_memory PROPERTIES
                IMPORTED_LOCATION "${foonathan_memory_LIB_FILE}"
                INTERFACE_INCLUDE_DIRECTORIES "${foonathan_memory_INCLUDE_DIRS}"
            )
        endif(NOT TARGET foonathan_memory)

        # Keep old-style calls for backwards compatibility
        include_directories("${foonathan_memory_INCLUDE_DIRS}")
        link_directories("${foonathan_memory_LIB_DIR}")
    else(foonathan_memory_LIB_DIR)
        set(foonathan_memory_FOUND FALSE)
    endif(foonathan_memory_LIB_DIR)
endif(NOT foonathan_memory_FOUND)
