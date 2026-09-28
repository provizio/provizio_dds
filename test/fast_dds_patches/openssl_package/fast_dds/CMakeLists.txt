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

# Stands in for the Fast-DDS build in the fast_dds_openssl_package test: looks OpenSSL up as Fast-DDS
# v3.6 does, with the FindOpenSSL provizio_dds wrote first on its CMAKE_MODULE_PATH, and says what it
# got. Configured by openssl_package_test.cmake, never built.

# As Fast-DDS's own, for the same policies
cmake_minimum_required(VERSION 3.22)
project(openssl_package_fast_dds LANGUAGES NONE)

# Not REQUIRED, as Fast-DDS without SECURITY has it: provizio_dds makes it so with
# CMAKE_REQUIRE_FIND_PACKAGE_OpenSSL, which the test passes as well
find_package(OpenSSL)
if(NOT OPENSSL_FOUND OR NOT TARGET OpenSSL::SSL OR NOT TARGET OpenSSL::Crypto)
    message(FATAL_ERROR "openssl_package_fast_dds: OpenSSL was not found as Fast-DDS needs it")
endif()

# Which stand-in each dependency the package's configuration looked up was, as its target says
set(_dependencies)
foreach(_target IN ITEMS ProvizioTestZ::ProvizioTestZ ProvizioTestFoo::ProvizioTestFoo)
    if(TARGET ${_target})
        get_target_property(_which ${_target} INTERFACE_COMPILE_DEFINITIONS)
        list(APPEND _dependencies "${_target}=${_which}")
    endif()
endforeach()
message(STATUS "openssl_package_fast_dds: OpenSSL ${OPENSSL_VERSION} with [${_dependencies}]")
