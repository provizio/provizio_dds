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

# Checks the rpath the Fast-DDS library this tree built was installed with, as the loader will read
# it: every entry in EXPECTED there, and none an absolute path into BUILD_DIR. Relative entries are
# the only ones that mean anything once the library is installed or packaged somewhere -- they are
# how it finds libfastcdr beside it, and the OpenSSL runtime placed next to it, or kept apart under
# lib/provizio_dds by an install -- and an entry into the build tree works on this machine only, for
# as long as the tree is there, which is how a library that cannot load anywhere else passes every
# test run in the tree. On macOS the entries are what install_rpath_as_given.cmake keeps Fast-DDS
# from replacing with exactly such a path.
#
#   cmake -DLIBRARY=<library> -DTOOL=<readelf or otool> -DTOOL_KIND=<readelf|otool>
#         -DEXPECTED=<entry>[|<entry>...] -DBUILD_DIR=<directory> -P install_rpath_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS LIBRARY TOOL TOOL_KIND EXPECTED BUILD_DIR)
    if(NOT ${_var})
        message(FATAL_ERROR "install_rpath_test.cmake: ${_var} is required")
    endif()
endforeach()
string(REPLACE "|" ";" EXPECTED "${EXPECTED}")

if(TOOL_KIND STREQUAL "readelf")
    set(_arguments -d)
elseif(TOOL_KIND STREQUAL "otool")
    set(_arguments -l)
else()
    message(FATAL_ERROR "install_rpath_test.cmake: unknown TOOL_KIND '${TOOL_KIND}'")
endif()
execute_process(COMMAND "${TOOL}" ${_arguments} "${LIBRARY}"
    RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _error)
if(NOT _result EQUAL 0)
    message(FATAL_ERROR "install_rpath_test.cmake: '${TOOL}' could not read ${LIBRARY}: ${_error}")
endif()

set(_entries)
if(TOOL_KIND STREQUAL "readelf")
    # " 0x000000000000001d (RUNPATH)  Library runpath: [$ORIGIN:$ORIGIN/../lib]", or RPATH alike
    if(_output MATCHES "\\((RUNPATH|RPATH)\\)[^\n]*\\[([^]\n]*)\\]")
        string(REPLACE ":" ";" _entries "${CMAKE_MATCH_2}")
    endif()
else()
    # A load command per entry: "cmd LC_RPATH", "cmdsize 48", "path @loader_path (offset 12)"
    string(REPLACE ";" "," _output "${_output}")
    string(REPLACE "\n" ";" _lines "${_output}")
    set(_in_rpath FALSE)
    foreach(_line IN LISTS _lines)
        if(_line MATCHES "^[ \t]*cmd[ \t]+(.+)$")
            set(_in_rpath FALSE)
            if(CMAKE_MATCH_1 STREQUAL "LC_RPATH")
                set(_in_rpath TRUE)
            endif()
        elseif(_in_rpath AND _line MATCHES "^[ \t]*path[ \t]+(.*[^ \t])[ \t]+\\(offset [0-9]+\\)")
            list(APPEND _entries "${CMAKE_MATCH_1}")
        endif()
    endforeach()
endif()

foreach(_entry IN LISTS EXPECTED)
    if(NOT _entry IN_LIST _entries)
        message(FATAL_ERROR "${LIBRARY} was installed with the rpath [${_entries}], which lacks ${_entry}")
    endif()
endforeach()
file(TO_CMAKE_PATH "${BUILD_DIR}" BUILD_DIR)
foreach(_entry IN LISTS _entries)
    string(FIND "${_entry}" "${BUILD_DIR}" _position)
    if(_position EQUAL 0)
        message(FATAL_ERROR "${LIBRARY} was installed with the rpath [${_entries}], whose ${_entry} is this "
            "build tree: it means nothing once the library is installed or packaged anywhere else")
    endif()
endforeach()
message(STATUS "install_rpath: ${LIBRARY} was installed with the rpath [${_entries}]")
