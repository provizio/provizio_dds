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
#   cmake -DDESTINATION=<directory> -DGLOBS=<glob>[|<glob>...] -P copy_runtime.cmake

cmake_minimum_required(VERSION 3.15)

if(NOT DESTINATION OR NOT GLOBS)
    message(FATAL_ERROR "copy_runtime.cmake: DESTINATION and GLOBS are required")
endif()
string(REPLACE "|" ";" GLOBS "${GLOBS}")

set(_files)
foreach(_glob IN LISTS GLOBS)
    file(GLOB _matched LIST_DIRECTORIES false "${_glob}")
    list(APPEND _files ${_matched})
endforeach()
if(_files)
    file(COPY ${_files} DESTINATION "${DESTINATION}")
endif()
