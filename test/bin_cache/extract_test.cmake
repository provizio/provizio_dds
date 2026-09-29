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

# Coverage for cmake/bin_cache/extract.cmake: what extracting a bin cache archive leaves beside the
# directory it is to hold -- that directory, and nothing else of the archive, however malformed;
# nothing at all where extracting fails; and every other key's directory there as it was. cmake -E
# tar stands in for unzip and for PowerShell, which take the destination on their command line and
# from the environment.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -P extract_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR)
    if(NOT ${_var})
        message(FATAL_ERROR "extract_test.cmake: ${_var} is required")
    endif()
endforeach()
include("${SOURCE_DIR}/cmake/bin_cache/extract.cmake")

file(REMOVE_RECURSE "${WORK_DIR}")
# Extracts the archive named by the environment into the destination it names, as the PowerShell
# command does
file(WRITE "${WORK_DIR}/extract_by_environment.cmake" "
execute_process(COMMAND \"\${CMAKE_COMMAND}\" -E tar xf \"\$ENV{PROVIZIO_DDS_BIN_CACHE_ARCHIVE}\"
    WORKING_DIRECTORY \"\$ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION}\" RESULT_VARIABLE _result)
if(NOT _result EQUAL 0)
    message(FATAL_ERROR \"could not extract\")
endif()
")

# An archive at <archive> of the files given, relative to <content>, each holding its own name
function(_archive archive content)
    foreach(_file IN LISTS ARGN)
        file(WRITE "${content}/${_file}" "${_file}")
    endforeach()
    execute_process(COMMAND "${CMAKE_COMMAND}" -E tar cf "${archive}" --format=zip ${ARGN}
        WORKING_DIRECTORY "${content}" RESULT_VARIABLE _result)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "extract_test.cmake: could not make ${archive}")
    endif()
endfunction()

# Extracts <archive> for the key cache_key into a cache directory of <case>'s holding another key's
# directory, by <how> (command_line or environment), and checks that it failed as <failure> says
# ("" for not at all) and that the cache directory then holds <held>... and nothing else
function(_check case archive how failure)
    set(_cache "${WORK_DIR}/${case}/cache")
    file(WRITE "${_cache}/other_key/kept.txt" "another key's")
    file(WRITE "${_cache}/cache_key/stale.txt" "an earlier extraction's")
    if(how STREQUAL "environment")
        set(ENV{PROVIZIO_DDS_BIN_CACHE_ARCHIVE} "${archive}")
        # ...the destination it names put back as it was after
        set(ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION} "as it was")
        provizio_dds_bin_cache_extract("${_cache}/cache_key" _failure
            COMMAND "${CMAKE_COMMAND}" -P "${WORK_DIR}/extract_by_environment.cmake")
        unset(ENV{PROVIZIO_DDS_BIN_CACHE_ARCHIVE})
        if(NOT "$ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION}" STREQUAL "as it was")
            message(FATAL_ERROR "extract_test (${case}): the environment's destination is "
                "[$ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION}] after the extraction")
        endif()
        unset(ENV{PROVIZIO_DDS_BIN_CACHE_DESTINATION})
    else()
        provizio_dds_bin_cache_extract("${_cache}/cache_key" _failure
            COMMAND "${CMAKE_COMMAND}" -E chdir "<DESTINATION>" "${CMAKE_COMMAND}" -E tar xf "${archive}")
    endif()
    if(NOT _failure MATCHES "${failure}" OR (failure STREQUAL "" AND NOT _failure STREQUAL ""))
        message(FATAL_ERROR "extract_test (${case}): the extraction failed with [${_failure}], not [${failure}]")
    endif()
    file(GLOB_RECURSE _held LIST_DIRECTORIES false RELATIVE "${_cache}" "${_cache}/*")
    # And no staging directory left, empty or not
    file(GLOB _staging LIST_DIRECTORIES true RELATIVE "${_cache}" "${_cache}/.staging*")
    list(APPEND _held ${_staging})
    list(SORT _held)
    set(_expected other_key/kept.txt ${ARGN})
    list(SORT _expected)
    if(NOT "${_held}" STREQUAL "${_expected}")
        message(FATAL_ERROR "extract_test (${case}): the cache directory holds [${_held}], not [${_expected}]")
    endif()
endfunction()

_archive("${WORK_DIR}/well_formed.zip" "${WORK_DIR}/content/well_formed" cache_key/lib/libprovizio_dds.so)
_archive("${WORK_DIR}/extra_entries.zip" "${WORK_DIR}/content/extra_entries" cache_key/lib/libprovizio_dds.so
    other_key/lib/libprovizio_dds.so stray.txt)
_archive("${WORK_DIR}/no_directory.zip" "${WORK_DIR}/content/no_directory" other_key/lib/libprovizio_dds.so)
file(WRITE "${WORK_DIR}/damaged.zip" "not a zip archive")

foreach(_how IN ITEMS command_line environment)
    # The archive's directory, in place of what an earlier extraction left of it
    _check(well_formed_${_how} "${WORK_DIR}/well_formed.zip" ${_how} "" cache_key/lib/libprovizio_dds.so)
    # Entries beside it, another key's directory among them: none of them lands there
    _check(extra_entries_${_how} "${WORK_DIR}/extra_entries.zip" ${_how} "" cache_key/lib/libprovizio_dds.so)
    # No directory of the key: nothing of it
    _check(no_directory_${_how} "${WORK_DIR}/no_directory.zip" ${_how} "holds no cache_key directory")
    # Not an archive: nothing
    _check(damaged_${_how} "${WORK_DIR}/damaged.zip" ${_how} "exited with")
endforeach()

# An archive holding its directory as a link to one elsewhere, on a host that has links: refused
if(CMAKE_HOST_UNIX)
    file(WRITE "${WORK_DIR}/outside/lib/libprovizio_dds.so" "outside")
    file(MAKE_DIRECTORY "${WORK_DIR}/content/directory_link")
    file(CREATE_LINK "${WORK_DIR}/outside" "${WORK_DIR}/content/directory_link/cache_key" SYMBOLIC)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E tar cf "${WORK_DIR}/directory_link.zip" --format=zip cache_key
        WORKING_DIRECTORY "${WORK_DIR}/content/directory_link" RESULT_VARIABLE _result)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "extract_test.cmake: could not make ${WORK_DIR}/directory_link.zip")
    endif()
    _check(directory_link "${WORK_DIR}/directory_link.zip" command_line "holds no cache_key directory")
    file(GLOB_RECURSE _held LIST_DIRECTORIES false RELATIVE "${WORK_DIR}/outside" "${WORK_DIR}/outside/*")
    if(NOT _held STREQUAL "lib/libprovizio_dds.so")
        message(FATAL_ERROR "extract_test (directory_link): the directory linked to holds [${_held}]")
    endif()
endif()

# What an earlier extraction left in the staging directory that cannot be removed -- its cache
# directory made read-only here, on a POSIX host, as root is not refused -- is reported, and nothing
# extracted over it
execute_process(COMMAND id -u OUTPUT_VARIABLE _uid OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _result)
if(CMAKE_HOST_UNIX AND _result EQUAL 0 AND NOT _uid STREQUAL "0")
    set(_cache "${WORK_DIR}/staging_left/cache")
    file(WRITE "${_cache}/.staging-cache_key/cache_key/lib/stale.so" "an earlier extraction's")
    execute_process(COMMAND chmod 500 "${_cache}")
    provizio_dds_bin_cache_extract("${_cache}/cache_key" _failure
        COMMAND "${CMAKE_COMMAND}" -E chdir "<DESTINATION>" "${CMAKE_COMMAND}" -E tar xf "${WORK_DIR}/well_formed.zip")
    execute_process(COMMAND chmod 700 "${_cache}")
    if(NOT _failure MATCHES "staging-cache_key could not be removed" OR EXISTS "${_cache}/cache_key")
        message(FATAL_ERROR "extract_test (staging_left): the extraction failed with [${_failure}]")
    endif()
endif()

# A tool that is not there: nothing, and why
set(_cache "${WORK_DIR}/no_tool/cache")
file(WRITE "${_cache}/other_key/kept.txt" "another key's")
provizio_dds_bin_cache_extract("${_cache}/cache_key" _failure COMMAND "${WORK_DIR}/no_such_tool" -d "<DESTINATION>")
if(NOT _failure MATCHES "^no_such_tool could not be run")
    message(FATAL_ERROR "extract_test (no_tool): the extraction failed with [${_failure}]")
endif()
file(GLOB _held LIST_DIRECTORIES true RELATIVE "${_cache}" "${_cache}/*" "${_cache}/.staging*")
if(NOT _held STREQUAL "other_key")
    message(FATAL_ERROR "extract_test (no_tool): the cache directory holds [${_held}]")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "bin_cache_extract: the cache directory holds the archive's own directory, and nothing else of it")
