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

# Stands in for provizio_dds in the fast_dds_openssl_package test: finds OpenSSL as provizio_dds does,
# and when that is as a package, writes the FindOpenSSL given to Fast-DDS into OUT_DIR with
# provizio_dds's own function, and next to it, in runtime_dirs.txt, where provizio_dds would look for
# the OpenSSL runtime of a Release Fast-DDS. Configured by openssl_package_test.cmake, never built.

cmake_minimum_required(VERSION 3.15)
project(openssl_package_provizio LANGUAGES NONE)

# CMAKE_PREFIX_PATH as |-separated PREFIXES, a list being hard to hand through the test's command lines
if(DEFINED PREFIXES)
    string(REPLACE "|" ";" CMAKE_PREFIX_PATH "${PREFIXES}")
endif()

# A package location holding a list, as |-separated LIST_ROOT, under a name that sorts ahead of every
# other location: split at its ;, it would shift every location written after it
if(DEFINED LIST_ROOT)
    string(REPLACE "|" ";" _list_root "${LIST_ROOT}")
    set(AProvizioTestList_ROOT "${_list_root}" CACHE STRING "Two directories")
endif()

# A variable whose name holds an unpaired square bracket, which, in a list of the names, joins every
# name sorted after it into one: every location and result must be found all the same
set("AAProvizioTest[unpaired" "")

# A package location as an ordinary variable rather than a cache entry, as a toolchain or a project
# around provizio_dds may set one: PLAIN_LOCATION as <name>=<value>
if(DEFINED PLAIN_LOCATION)
    string(FIND "${PLAIN_LOCATION}" "=" _equals)
    string(SUBSTRING "${PLAIN_LOCATION}" 0 ${_equals} _plain_name)
    math(EXPR _equals "${_equals} + 1")
    string(SUBSTRING "${PLAIN_LOCATION}" ${_equals} -1 _plain_value)
    set(${_plain_name} "${_plain_value}")
endif()

find_package(OpenSSL REQUIRED)
if(NOT OpenSSL_CONFIG)
    message(FATAL_ERROR "openssl_package_provizio: OpenSSL was not found as a package")
endif()
include("${REPOSITORY}/cmake/fast_dds/openssl_package.cmake")
provizio_dds_write_openssl_find_module("${OUT_DIR}" "")
# Of a Release Fast-DDS, which the module given it maps onto MAPPED_CONFIG where that is set
provizio_dds_openssl_runtime_dirs(_runtime_dirs Release "${MAPPED_CONFIG}")
file(WRITE "${OUT_DIR}/runtime_dirs.txt" "${_runtime_dirs}")
# ...and as for an OpenSSL FindOpenSSL found, the maps of whose targets reach no Fast-DDS
set(_openssl_config "${OpenSSL_CONFIG}")
unset(OpenSSL_CONFIG)
provizio_dds_openssl_runtime_dirs(_runtime_dirs Release)
file(WRITE "${OUT_DIR}/runtime_dirs_findopenssl.txt" "${_runtime_dirs}")
set(OpenSSL_CONFIG "${_openssl_config}")
