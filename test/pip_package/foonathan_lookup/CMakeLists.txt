# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# Stands in for the Fast-DDS build in the pip_package_vendored_foonathan_memory test: looks
# foonathan_memory up as Fast-DDS does, through provizio_dds's own find module (MODULES_DIR), and
# says what it got. Configured by vendored_foonathan_memory_test.cmake, never built.

cmake_minimum_required(VERSION 3.15)
project(foonathan_lookup LANGUAGES NONE)

list(PREPEND CMAKE_MODULE_PATH "${MODULES_DIR}")
find_package(foonathan_memory REQUIRED)
if(NOT TARGET foonathan_memory)
    message(FATAL_ERROR "foonathan_lookup: no foonathan_memory target was defined")
endif()
get_target_property(_type foonathan_memory TYPE)
get_target_property(_location foonathan_memory IMPORTED_LOCATION)
message(STATUS "foonathan_lookup: ${_type} ${_location}")
