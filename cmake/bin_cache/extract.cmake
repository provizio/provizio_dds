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

# Extracting the prebuilt binaries of the bin cache (see the bin cache section of the top-level
# CMakeLists.txt):
#
#   provizio_dds_bin_cache_extract(<extracted_dir> <out_failure> COMMAND <command>...)
#
# runs <command>, which extracts an archive into the directory holding <extracted_dir>, and sets
# <out_failure> to what went wrong -- the tool not there, or failing, with what it said -- or to ""
# when nothing did. The configure decides whether binaries are there by whether a file of theirs
# exists, so what a failed extraction left behind is removed: a partial one would otherwise pass for
# the binaries at the next configure. And the caller can say that extracting is what failed, rather
# than blame the archive published for missing a file it holds.

function(provizio_dds_bin_cache_extract extracted_dir out_failure)
    cmake_parse_arguments(_extract "" "" "COMMAND" ${ARGN})
    # Both streams, as unzip reports a damaged archive on its standard output
    execute_process(COMMAND ${_extract_COMMAND} RESULT_VARIABLE _result OUTPUT_VARIABLE _error ERROR_VARIABLE _error)
    if(_result STREQUAL "0")
        set(${out_failure} "" PARENT_SCOPE)
        return()
    endif()
    file(REMOVE_RECURSE "${extracted_dir}")
    # Not a number when the tool could not be run at all: then it says why ("No such file or directory")
    list(GET _extract_COMMAND 0 _tool)
    get_filename_component(_tool "${_tool}" NAME)
    if(_result MATCHES "^[0-9]+$")
        set(_failure "${_tool} exited with ${_result}")
    else()
        set(_failure "${_tool} could not be run: ${_result}")
    endif()
    string(STRIP "${_error}" _error)
    string(REGEX REPLACE "[ \t\r\n]+" " " _error "${_error}")
    # The start of what it said is what names the problem; unzip goes on for another paragraph
    string(LENGTH "${_error}" _length)
    if(_length GREATER 300)
        string(SUBSTRING "${_error}" 0 300 _error)
        string(APPEND _error "...")
    endif()
    if(_error)
        string(APPEND _failure ", saying: ${_error}")
    endif()
    set(${out_failure} "${_failure}" PARENT_SCOPE)
endfunction()
