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

# Coverage for cmake/glob_escape.cmake: a path taken through it matches itself alone in a glob,
# though it holds [ or ? (and *, where a name can), or a . or .. after one -- a .. after a link
# resolved as the system resolves it -- and a relative path keeps its dots.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -P glob_escape_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR)
    if(NOT ${_var})
        message(FATAL_ERROR "glob_escape_test.cmake: ${_var} is required")
    endif()
endforeach()
include("${SOURCE_DIR}/cmake/glob_escape.cmake")

file(REMOVE_RECURSE "${WORK_DIR}")

# Globs <directory>/*.txt through the helper and expects the files named <expected>... under it
function(_expect case directory)
    provizio_dds_glob_escape(_pattern "${directory}")
    file(GLOB _found LIST_DIRECTORIES false "${_pattern}/*.txt")
    set(_names)
    foreach(_file IN LISTS _found)
        get_filename_component(_name "${_file}" NAME)
        list(APPEND _names "${_name}")
    endforeach()
    list(SORT _names)
    set(_expected ${ARGN})
    list(SORT _expected)
    if(NOT _names STREQUAL _expected)
        message(FATAL_ERROR
            "glob_escape_test (${case}): [${directory}] matched [${_names}], not [${_expected}]")
    endif()
endfunction()

# A [x] and a ? of the name: the directory itself, not the x and y a pattern would read them as
file(WRITE "${WORK_DIR}/br [x]/own.txt" "")
file(WRITE "${WORK_DIR}/br x/other.txt" "")
_expect(brackets "${WORK_DIR}/br [x]" own.txt)
if(NOT CMAKE_HOST_WIN32)
    file(WRITE "${WORK_DIR}/q?/own.txt" "")
    file(WRITE "${WORK_DIR}/qy/other.txt" "")
    file(WRITE "${WORK_DIR}/st*/own.txt" "")
    file(WRITE "${WORK_DIR}/stz/other.txt" "")
    _expect(question_mark "${WORK_DIR}/q?" own.txt)
    _expect(star "${WORK_DIR}/st*" own.txt)
endif()
# A .. and a . after one: resolved, as matching component by component finds neither
file(MAKE_DIRECTORY "${WORK_DIR}/br [x]/sub")
_expect(dot_dot "${WORK_DIR}/br [x]/sub/../." own.txt)
_expect(dots_between "${WORK_DIR}/br [x]/./sub/./../sub/.." own.txt)
# A .. after a link: the parent of where the link leads, as the system has it, not the directory
# holding the link
if(NOT CMAKE_HOST_WIN32)
    file(WRITE "${WORK_DIR}/real [r]/up.txt" "")
    file(MAKE_DIRECTORY "${WORK_DIR}/real [r]/deep")
    file(CREATE_LINK "${WORK_DIR}/real [r]/deep" "${WORK_DIR}/br [x]/link" SYMBOLIC)
    _expect(link_dot_dot "${WORK_DIR}/br [x]/link/.." up.txt)
    # ...and up to the root, which stays itself: from a directory under it that is no link (/tmp is
    # one on macOS, whose .. is /private), as the first of the work directory's real path is
    get_filename_component(_real_work "${WORK_DIR}" REALPATH)
    string(REGEX MATCH "^/[^/]+" _top "${_real_work}")
    foreach(_root_path IN ITEMS "/." "/.." "${_top}/.." "/../.")
        provizio_dds_glob_escape(_root "${_root_path}")
        if(NOT _root STREQUAL "/")
            message(FATAL_ERROR "glob_escape_test (root): [${_root_path}] became [${_root}], not [/]")
        endif()
    endforeach()
endif()

# A relative path keeps its dots, escaped all the same
provizio_dds_glob_escape(_relative "../br [x]/./sub")
if(NOT _relative STREQUAL "../br [[]x]/./sub")
    message(FATAL_ERROR "glob_escape_test (relative): [../br [x]/./sub] became [${_relative}]")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "glob_escape: a path matches itself alone in a glob, whatever it holds")
