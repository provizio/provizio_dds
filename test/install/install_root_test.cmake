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

# Coverage for cmake/install_root.cmake: where the install steps of provizio_dds's own that write or
# remove files by path put them, which must be where file(INSTALL) puts everything else. Each case
# installs a marker file with file(INSTALL) itself, under the DESTDIR and prefix of the case, and
# requires the root the function gives to be where the marker went.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -P install_root_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR)
    if(NOT ${_var})
        message(FATAL_ERROR "install_root_test.cmake: ${_var} is required")
    endif()
endforeach()
include("${SOURCE_DIR}/cmake/install_root.cmake")

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")
file(WRITE "${WORK_DIR}/marker" "")

# <prefix> installed under DESTDIR <destdir> ("" for none): the root must be where file(INSTALL)
# puts the marker, found as <root>/marker
function(_expect_as_installed case prefix destdir)
    set(CMAKE_INSTALL_PREFIX "${prefix}")
    set(ENV{DESTDIR} "${destdir}")
    provizio_dds_install_root(_root)
    file(INSTALL "${WORK_DIR}/marker" DESTINATION "${prefix}/${case}" MESSAGE_NEVER)
    if(NOT EXISTS "${_root}/${case}/marker")
        message(FATAL_ERROR "install_root_test: prefix '${prefix}' with DESTDIR '${destdir}' gave root "
            "'${_root}', where file(INSTALL) put nothing")
    endif()
endfunction()

# The same, in a CMake of its own, run from a directory of this test's, with the environment given
# after (<name>=<value>, or --unset=<name>): for a DESTDIR that stays relative, which file(INSTALL)
# takes from the directory it is run from, and for HOME to be unset
file(WRITE "${WORK_DIR}/as_installed.cmake" "
include([==[${SOURCE_DIR}/cmake/install_root.cmake]==])
set(CMAKE_INSTALL_PREFIX \"\${PREFIX}\")
provizio_dds_install_root(_root)
file(INSTALL \"\${MARKER}\" DESTINATION \"\${PREFIX}/\${CASE}\" MESSAGE_NEVER)
if(NOT EXISTS \"\${_root}/\${CASE}/marker\")
    message(FATAL_ERROR \"gave root '\${_root}', where file(INSTALL) put nothing\")
endif()
")
function(_expect_as_installed_apart case prefix destdir)
    file(MAKE_DIRECTORY "${WORK_DIR}/cwd")
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env "DESTDIR=${destdir}" ${ARGN}
            "${CMAKE_COMMAND}" "-DPREFIX=${prefix}" "-DCASE=${case}" "-DMARKER=${WORK_DIR}/marker"
            -P "${WORK_DIR}/as_installed.cmake"
        WORKING_DIRECTORY "${WORK_DIR}/cwd" RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "install_root_test (${case}): prefix '${prefix}' with DESTDIR '${destdir}' ${ARGN}:\n"
            "${_output}")
    endif()
endfunction()

# The prefix itself, of whichever install is being run, when nothing is staged
_expect_as_installed(plain "${WORK_DIR}/prefix" "")
# A staged install, of a prefix that must not be written: under DESTDIR, trailing slash or not
_expect_as_installed(staged "/provizio_dds_install_root_test" "${WORK_DIR}/stage")
_expect_as_installed(trailing_slash "/provizio_dds_install_root_test" "${WORK_DIR}/stage/")
# Backslashes in DESTDIR are taken as slashes, on every host
string(REPLACE "/" "\\" _backslashed "${WORK_DIR}/stage")
_expect_as_installed(backslashes "/provizio_dds_install_root_test" "${_backslashed}")
# A Windows prefix's drive letter cannot follow another path, so it goes (elsewhere C: is no drive)
if(CMAKE_HOST_WIN32)
    _expect_as_installed(drive_letter "C:/provizio_dds_install_root_test" "${WORK_DIR}/stage")
endif()
# A leading ~ is HOME, where HOME is set
set(ENV{HOME} "${WORK_DIR}/home")
_expect_as_installed(home "/provizio_dds_install_root_test" "~/stage")
# Where it is not, the ~ is left as it is, as file(INSTALL) leaves it: a relative directory of that
# name, never the root of the file system, which would make DESTDIR=~ the live prefix itself
_expect_as_installed_apart(home_unset "/provizio_dds_install_root_test" "~" --unset=HOME)
_expect_as_installed_apart(home_unset_stage "/provizio_dds_install_root_test" "~/stage" --unset=HOME)
# A ~name is the home directory of the user of that name on a POSIX host, and stays a directory of
# that name where there is no such user, and on Windows
_expect_as_installed_apart(no_such_user "/provizio_dds_install_root_test" "~provizio_dds_no_such_user/stage")
if(CMAKE_HOST_UNIX)
    # The user running this, whose home is the start of a way back here, so that nothing is written
    # there; and a first component holding a ':', which is no user's name, left as it is
    execute_process(COMMAND id -un OUTPUT_VARIABLE _user OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _result)
    file(TO_CMAKE_PATH "~${_user}" _user_home)
    if(NOT _result EQUAL 0 OR _user STREQUAL "" OR NOT IS_DIRECTORY "${_user_home}")
        message(STATUS "install_root_test: skipping ~user, as the home of the user running this is unknown")
    else()
        file(MAKE_DIRECTORY "${WORK_DIR}/user_stage")
        get_filename_component(_user_home "${_user_home}" REALPATH)
        get_filename_component(_user_stage "${WORK_DIR}/user_stage" REALPATH)
        file(RELATIVE_PATH _back_here "${_user_home}" "${_user_stage}")
        _expect_as_installed_apart(user_home "/provizio_dds_install_root_test" "~${_user}/${_back_here}")
        _expect_as_installed_apart(not_a_user "/provizio_dds_install_root_test" "~${_user}:x/stage")
    endif()
    # A ':' after the first component, where file(TO_CMAKE_PATH) would split a whole DESTDIR: on a
    # POSIX host only, as no Windows directory can be named with one
    _expect_as_installed_apart(separator_after "/provizio_dds_install_root_test" "~/a:b/c" "HOME=${WORK_DIR}/home")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "install_root: every install root is where file(INSTALL) puts the tree")
