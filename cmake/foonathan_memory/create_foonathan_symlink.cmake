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

# Helper script to create symlink for libfoonathan_memory.a in the correct directory (lib, lib64, or lib32)

include("${CMAKE_CURRENT_LIST_DIR}/../glob_escape.cmake")

set(FOONATHAN_MEMORY_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/install")
provizio_dds_glob_escape(FOONATHAN_MEMORY_INSTALL_PATTERN "${FOONATHAN_MEMORY_INSTALL_DIR}")

# Try to find libfoonathan_memory-*.a in lib, lib64, or lib32
foreach(lib_subdir lib lib64 lib32)
    file(GLOB FOONATHAN_LIB_FILES "${FOONATHAN_MEMORY_INSTALL_PATTERN}/${lib_subdir}/libfoonathan_memory-*.a")
    if(FOONATHAN_LIB_FILES)
        list(GET FOONATHAN_LIB_FILES 0 FOONATHAN_LIB_FILE)
        set(FOONATHAN_SYMLINK "${FOONATHAN_MEMORY_INSTALL_DIR}/${lib_subdir}/libfoonathan_memory.a")
        
        # Remove old symlink if exists
        if(EXISTS "${FOONATHAN_SYMLINK}")
            file(REMOVE "${FOONATHAN_SYMLINK}")
        endif(EXISTS "${FOONATHAN_SYMLINK}")
        
        # Create new symlink
        execute_process(
            COMMAND "${CMAKE_COMMAND}" -E create_symlink "${FOONATHAN_LIB_FILE}" "${FOONATHAN_SYMLINK}"
        )
        
        message(STATUS "Created symlink: ${FOONATHAN_SYMLINK} -> ${FOONATHAN_LIB_FILE}")
        break()
    endif(FOONATHAN_LIB_FILES)
endforeach(lib_subdir lib lib64 lib32)
