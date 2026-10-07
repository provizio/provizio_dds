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

# Coverage for cmake/bin_cache/missing_file.cmake: which file the prebuilt binaries a bin cache
# holds lack, of those a build hands on -- by a fixed name or by a pattern a versioned name answers
# to, in a directory whose own path holds glob characters.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -P missing_file_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR)
    if(NOT ${_var})
        message(FATAL_ERROR "missing_file_test.cmake: ${_var} is required")
    endif()
endforeach()
include("${SOURCE_DIR}/cmake/bin_cache/missing_file.cmake")

file(REMOVE_RECURSE "${WORK_DIR}")
# Brackets in the path, which a glob would read as a set of characters (and no *, which Windows
# does not allow in a name)
set(_cache "${WORK_DIR}/cache [x]")
set(_required lib/libprovizio_dds.so lib/libprovizio_dds_types.so "lib/libfastdds.so.*" "lib/libfastcdr.so.*")

function(_expect case expected)
    provizio_dds_bin_cache_missing_file(_missing "${_cache}" ${_required})
    if(NOT _missing STREQUAL expected)
        message(FATAL_ERROR "missing_file_test (${case}): said [${_missing}] is missing, not [${expected}]")
    endif()
endfunction()

# Nothing there: the first is missing
_expect(nothing lib/libprovizio_dds.so)
# The libraries a cache holds, Fast-DDS's by their versioned names, but the types library: that one
file(WRITE "${_cache}/lib/libprovizio_dds.so" "")
file(WRITE "${_cache}/lib/libfastdds.so.3.6.2.0" "")
file(WRITE "${_cache}/lib/libfastcdr.so.2.3.5" "")
_expect(no_types_library lib/libprovizio_dds_types.so)
# ...which a directory of its name does not stand for
file(MAKE_DIRECTORY "${_cache}/lib/libprovizio_dds_types.so")
_expect(types_library_directory lib/libprovizio_dds_types.so)
file(REMOVE_RECURSE "${_cache}/lib/libprovizio_dds_types.so")
file(WRITE "${_cache}/lib/libprovizio_dds_types.so" "")
_expect(complete "")
# A versioned library missing, though a directory and the unversioned name answer to its pattern
# (only versioned names do)
file(REMOVE "${_cache}/lib/libfastcdr.so.2.3.5")
file(MAKE_DIRECTORY "${_cache}/lib/libfastcdr.so.2")
file(WRITE "${_cache}/lib/libfastcdr.so" "")
_expect(no_fast_cdr "lib/libfastcdr.so.*")
# Any versioned name answers to it
file(WRITE "${_cache}/lib/libfastcdr.so.2.3.6" "")
_expect(other_fast_cdr_version "")
# Where links can be made: a link to nothing is no library, nor is one beyond the cache that a link
# in it leads to, but one the cache holds stands for it, as the versioned names the install links do
if(NOT WIN32)
    file(REMOVE "${_cache}/lib/libprovizio_dds.so")
    file(CREATE_LINK "${_cache}/nowhere" "${_cache}/lib/libprovizio_dds.so" SYMBOLIC)
    _expect(dangling_link lib/libprovizio_dds.so)
    file(REMOVE "${_cache}/lib/libprovizio_dds.so")
    file(WRITE "${WORK_DIR}/elsewhere/libprovizio_dds.so" "")
    file(CREATE_LINK "${WORK_DIR}/elsewhere/libprovizio_dds.so" "${_cache}/lib/libprovizio_dds.so" SYMBOLIC)
    _expect(link_beyond_the_cache lib/libprovizio_dds.so)
    file(REMOVE "${_cache}/lib/libprovizio_dds.so")
    file(WRITE "${_cache}/lib/libprovizio_dds.so.1" "")
    file(CREATE_LINK "libprovizio_dds.so.1" "${_cache}/lib/libprovizio_dds.so" SYMBOLIC)
    _expect(link_in_the_cache "")
    # ...nor are the libraries of a directory beyond it that the cache's lib/ is a link to
    file(RENAME "${_cache}/lib" "${WORK_DIR}/foreign_lib")
    file(CREATE_LINK "${WORK_DIR}/foreign_lib" "${_cache}/lib" SYMBOLIC)
    _expect(lib_directory_link lib/libprovizio_dds.so)
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "missing_file: a bin cache is complete only with a file for each name the build needs")
