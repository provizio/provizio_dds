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

# Copies into DESTINATION the files the GLOBS match when it runs, at build time: the libraries of the
# Fast-DDS this tree built, and the OpenSSL runtime placed next to them, as that build left them --
# links as links. A list of them made when the tree was configured would be empty on a first
# configure, before Fast-DDS is built, and on a later one would name what an earlier build left
# there, which that build may since have removed.
#
# What an earlier run copied there is removed first, as what this one does not copy -- a library of
# a Fast-DDS or an OpenSSL this tree no longer builds, after a configure that chose another, or a
# system Fast-DDS, which leaves no GLOBS at all -- would otherwise stay: the tests load their
# libraries from DESTINATION ahead of anywhere else, so it would be loaded in place of the one now
# chosen. That is a step of its own (STEP=prune), run ahead of every other copy into DESTINATION, so
# that none of theirs is what it removes; the copy (STEP=copy, the default) runs after them, so that
# a file of the same name as one of theirs is this one. The copy records the names of what it copied
# in DESTINATION/provizio_dds_runtime_copied.txt for the next prune to go by.
#
#   cmake -DDESTINATION=<directory> -DSTEP=prune -P copy_runtime.cmake
#   cmake -DDESTINATION=<directory> [-DGLOBS=<glob>[|<glob>...]] [-DSTEP=copy] -P copy_runtime.cmake

cmake_minimum_required(VERSION 3.15)

if(NOT DESTINATION)
    message(FATAL_ERROR "copy_runtime.cmake: DESTINATION is required")
endif()
if(NOT STEP)
    set(STEP copy)
endif()
if(NOT STEP MATCHES "^(prune|copy)$")
    message(FATAL_ERROR "copy_runtime.cmake: STEP is prune or copy, not ${STEP}")
endif()
string(REPLACE "|" ";" GLOBS "${GLOBS}")

set(_record "${DESTINATION}/provizio_dds_runtime_copied.txt")
if(STEP STREQUAL "prune")
    if(EXISTS "${_record}")
        file(STRINGS "${_record}" _copied)
        foreach(_name IN LISTS _copied)
            # A name of a file in DESTINATION, as the copy writes them, and nothing else: no path,
            # nor . or ..
            if(NOT _name MATCHES "[/\\]" AND NOT _name MATCHES "^\\.\\.?$")
                file(REMOVE "${DESTINATION}/${_name}")
            endif()
        endforeach()
    endif()
    return()
endif()

set(_files)
foreach(_glob IN LISTS GLOBS)
    file(GLOB _matched LIST_DIRECTORIES false "${_glob}")
    list(APPEND _files ${_matched})
endforeach()
set(_names)
foreach(_file IN LISTS _files)
    get_filename_component(_name "${_file}" NAME)
    list(APPEND _names "${_name}")
endforeach()
# Over whatever of the same name is there, however recent: file(COPY) leaves a file it takes for the
# same by its time, to the second
foreach(_name IN LISTS _names)
    file(REMOVE "${DESTINATION}/${_name}")
endforeach()
if(_files)
    file(COPY ${_files} DESTINATION "${DESTINATION}")
endif()
string(REPLACE ";" "\n" _record_text "${_names}")
file(WRITE "${_record}" "${_record_text}\n")
