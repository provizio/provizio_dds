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
# runs <command>, which extracts an archive holding <extracted_dir>'s name into the directory
# <DESTINATION> stands for among its arguments, and that the PROVIZIO_DDS_BIN_CACHE_DESTINATION
# environment variable names as well: a staging directory of its own beside <extracted_dir>, from
# which <extracted_dir> alone is taken. Whatever else a malformed archive holds goes with the staging
# directory, rather than land beside <extracted_dir> -- another key's directory there, or a file of
# one, which a later configure would take for those binaries. <out_failure> is set to what went
# wrong -- the tool not there, or failing, with what it said, or the archive holding no such
# directory -- or to "" when nothing did. The configure decides whether binaries are there by whether
# a file of theirs exists, so nothing of a failed extraction is left: a partial one would otherwise
# pass for the binaries at the next configure. And the caller can say that extracting is what
# failed, rather than blame the archive published for missing a file it holds.

function(provizio_dds_bin_cache_extract extracted_dir out_failure)
    cmake_parse_arguments(_extract "" "" "COMMAND" ${ARGN})
    get_filename_component(_parent "${extracted_dir}" DIRECTORY)
    get_filename_component(_name "${extracted_dir}" NAME)
    set(_staging "${_parent}/.staging-${_name}")
    file(REMOVE_RECURSE "${_staging}" "${extracted_dir}")
    # Over what is left of an earlier extraction -- a file in use, on Windows -- it could not be put in
    # place, nor extracted without what is left being taken for part of it
    foreach(_left IN ITEMS "${extracted_dir}" "${_staging}")
        if(EXISTS "${_left}" OR IS_SYMLINK "${_left}")
            set(${out_failure} "what an earlier extraction left in ${_left} could not be removed" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    file(MAKE_DIRECTORY "${_staging}")
    string(REPLACE "<DESTINATION>" "${_staging}" _command "${_extract_COMMAND}")
    set(_destination_before "$ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION}")
    set(ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION} "${_staging}")
    # Both streams, as unzip reports a damaged archive on its standard output
    execute_process(COMMAND ${_command} RESULT_VARIABLE _result OUTPUT_VARIABLE _error ERROR_VARIABLE _error)
    set(ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION} "${_destination_before}")
    if(_result STREQUAL "0")
        # The directory itself, not a link to one elsewhere, which would be taken for the binaries
        if(IS_DIRECTORY "${_staging}/${_name}" AND NOT IS_SYMLINK "${_staging}/${_name}")
            file(RENAME "${_staging}/${_name}" "${extracted_dir}")
            file(REMOVE_RECURSE "${_staging}")
            set(${out_failure} "" PARENT_SCOPE)
        else()
            file(REMOVE_RECURSE "${_staging}")
            set(${out_failure} "the archive holds no ${_name} directory" PARENT_SCOPE)
        endif()
        return()
    endif()
    file(REMOVE_RECURSE "${_staging}")
    # Not a number when the tool could not be run at all: then it says why ("No such file or directory")
    list(GET _command 0 _tool)
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
