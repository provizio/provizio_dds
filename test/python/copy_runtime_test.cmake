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

# Coverage for copy_runtime.cmake: what it copies into the Python tests' directory, and what of an
# earlier run's copies it removes when the libraries it is to copy are others, or none at all -- and
# that it leaves alone everything else there.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -P copy_runtime_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR)
    if(NOT ${_var})
        message(FATAL_ERROR "copy_runtime_test.cmake: ${_var} is required")
    endif()
endforeach()

file(REMOVE_RECURSE "${WORK_DIR}")
set(_built "${WORK_DIR}/fast_dds/lib")
set(_tests "${WORK_DIR}/tests")
# The directories as they are in the globs below: a [, * or ? in them is no pattern
string(REGEX REPLACE "([[*?])" "[\\1]" _built_pattern "${_built}")
string(REGEX REPLACE "([[*?])" "[\\1]" _work_pattern "${WORK_DIR}")
file(MAKE_DIRECTORY "${_built}" "${_tests}")
file(WRITE "${_built}/libfastdds.so.3.6.2" "fastdds")
file(WRITE "${_built}/libssl.so.3" "ssl")
file(WRITE "${_built}/libcrypto.so.3" "crypto")
file(WRITE "${_tests}/python_publisher.py" "a test's own")
if(CMAKE_HOST_UNIX)
    file(CREATE_LINK "libfastdds.so.3.6.2" "${_built}/libfastdds.so.3.6" SYMBOLIC)
endif()

# Runs copy_runtime.cmake with <globs> (|-separated, empty for none) as the build does: its prune,
# the copies of the build's other steps -- here <name>=<content>, written into the tests' directory,
# if given -- and its copy
function(_copy globs)
    foreach(_step IN ITEMS prune other copy)
        if(_step STREQUAL "other")
            foreach(_other IN LISTS ARGN)
                string(REGEX MATCH "^([^=]*)=(.*)$" _ "${_other}")
                file(WRITE "${_tests}/${CMAKE_MATCH_1}" "${CMAKE_MATCH_2}")
            endforeach()
            continue()
        endif()
        execute_process(COMMAND "${CMAKE_COMMAND}" "-DDESTINATION=${_tests}" "-DGLOBS=${globs}" "-DSTEP=${_step}"
                -P "${SOURCE_DIR}/test/python/copy_runtime.cmake"
            RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
        if(NOT _result EQUAL 0)
            message(FATAL_ERROR "copy_runtime.cmake (${_step}) failed with GLOBS [${globs}]:\n${_output}")
        endif()
    endforeach()
endfunction()

# Checks that the tests' directory holds <names> and nothing else, but the record of what was copied
function(_expect case)
    # The directory as it is: a [, * or ? in it is no pattern
    string(REGEX REPLACE "([[*?])" "[\\1]" _pattern "${_tests}")
    file(GLOB _held LIST_DIRECTORIES false RELATIVE "${_tests}" "${_pattern}/*")
    list(REMOVE_ITEM _held provizio_dds_runtime_copied.txt)
    list(SORT _held)
    set(_expected ${ARGN})
    list(SORT _expected)
    if(NOT "${_held}" STREQUAL "${_expected}")
        message(FATAL_ERROR "copy_runtime_test (${case}): the tests' directory holds [${_held}], not [${_expected}]")
    endif()
endfunction()

set(_linked)
if(CMAKE_HOST_UNIX)
    set(_linked libfastdds.so.3.6)
endif()

# A Fast-DDS built here, with the OpenSSL runtime next to it: all of it copied, a link as a link
_copy("${_built_pattern}/*.so*")
_expect(first python_publisher.py libfastdds.so.3.6.2 libssl.so.3 libcrypto.so.3 ${_linked})
if(CMAKE_HOST_UNIX AND NOT IS_SYMLINK "${_tests}/libfastdds.so.3.6")
    message(FATAL_ERROR "copy_runtime_test (first): the link was not copied as a link")
endif()

# The same Fast-DDS with the system's OpenSSL now: the OpenSSL copied before goes
file(REMOVE "${_built}/libssl.so.3" "${_built}/libcrypto.so.3")
_copy("${_built_pattern}/*.so*")
_expect(openssl_gone python_publisher.py libfastdds.so.3.6.2 ${_linked})

# Another glob as well, matching nothing: as before
_copy("${_built_pattern}/*.so*|${_work_pattern}/nowhere/*.dll")
_expect(nothing_more python_publisher.py libfastdds.so.3.6.2 ${_linked})

# Another step copying a file of a name copied too: the copy beside Fast-DDS is the one left
_copy("${_built_pattern}/*.so*" "libfastdds.so.3.6.2=another step's")
_expect(clash python_publisher.py libfastdds.so.3.6.2 ${_linked})
file(READ "${_tests}/libfastdds.so.3.6.2" _content)
if(NOT _content STREQUAL "fastdds")
    message(FATAL_ERROR "copy_runtime_test (clash): the copy left holds [${_content}], not the one beside Fast-DDS")
endif()

# A system Fast-DDS now, none built here, where another step copies a file of a name copied before
# (the found OpenSSL's DLL, on Windows): every library copied before goes, but that step's copy and
# the test's own stay
_copy("" "libfastdds.so.3.6.2=another step's")
_expect(none_built python_publisher.py libfastdds.so.3.6.2)
file(REMOVE "${_tests}/libfastdds.so.3.6.2")

# A record naming more than files of the tests' directory -- not as copy_runtime.cmake writes it --
# removes nothing outside it
file(WRITE "${WORK_DIR}/outside.so" "")
file(MAKE_DIRECTORY "${_tests}/sub")
file(WRITE "${_tests}/sub/inside.so" "")
file(WRITE "${_tests}/provizio_dds_runtime_copied.txt" "../outside.so\n${WORK_DIR}/outside.so\nsub/inside.so\n..\n.\n")
_copy("")
if(NOT EXISTS "${WORK_DIR}/outside.so" OR NOT EXISTS "${_tests}/sub/inside.so")
    message(FATAL_ERROR "copy_runtime_test (paths): a path in the record removed a file it names")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "copy_runtime: the Python tests' directory holds the runtime chosen last, and nothing of an earlier one")
