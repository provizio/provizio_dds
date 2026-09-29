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

# <prefix> under DESTDIR <destdir> must give <expected>, for what cannot be installed here
function(_expect case prefix destdir expected)
    set(CMAKE_INSTALL_PREFIX "${prefix}")
    set(ENV{DESTDIR} "${destdir}")
    provizio_dds_install_root(_root)
    if(NOT _root STREQUAL expected)
        message(FATAL_ERROR "install_root_test (${case}): prefix '${prefix}' with DESTDIR '${destdir}' "
            "gave '${_root}', not '${expected}'")
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
unset(ENV{HOME})
_expect(home_unset "/usr/local" "~" "~/usr/local")
_expect(home_unset "/usr/local" "~/stage" "~/stage/usr/local")
# Only a ~ standing for a directory of its own is HOME
set(ENV{HOME} "${WORK_DIR}/home")
_expect(tilde_name "/usr/local" "~stage" "~stage/usr/local")

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "install_root: every install root is where file(INSTALL) puts the tree")
