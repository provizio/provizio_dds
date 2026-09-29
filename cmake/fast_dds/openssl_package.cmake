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

# The FindOpenSSL given to the Fast-DDS that provizio_dds builds when provizio_dds found OpenSSL as a
# package (OpenSSL_CONFIG set -- Conan's, say): see FindOpenSSL.cmake.in for what it does and why.
#
#   provizio_dds_write_openssl_find_module(<directory> <package_config>)
#
# writes <directory>/FindOpenSSL.cmake from what the calling scope has: the package found
# (OpenSSL_CONFIG, OPENSSL_VERSION, OPENSSL_INCLUDE_DIR), the package search settings it was found
# with, and <package_config>, the configuration (build type) of the package's imported targets that
# Fast-DDS is to take, empty for no mapping. configure_file() rewrites it only when that changes,
# which is when Fast-DDS has to configure again. Package locations are among it, so one first cached
# after this call -- by a package that provizio_dds, or a project around it, finds later on -- changes
# it on the next configure of a new build tree, which configures Fast-DDS once more; from then on it
# settles.
#
# Called by the top-level CMakeLists.txt, and by the fast_dds_openssl_package test, with stand-in
# packages, to generate the module exactly as the build does.

# Where the template is, as a function knows only its caller's directory before CMake 3.17
set(_PROVIZIO_DDS_OPENSSL_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(provizio_dds_write_openssl_find_module directory package_config)
    get_filename_component(PROVIZIO_DDS_OPENSSL_CONFIG_DIR "${OpenSSL_CONFIG}" DIRECTORY)
    set(PROVIZIO_DDS_OPENSSL_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
    set(PROVIZIO_DDS_OPENSSL_MODULE_PATH "${CMAKE_MODULE_PATH}")
    set(PROVIZIO_DDS_OPENSSL_PACKAGE_CONFIG "${package_config}")
    list(GET OPENSSL_INCLUDE_DIR 0 _OPENSSL_INCLUDE_DIR)
    # For the header only: not every package's configuration sets the upper-case one
    if(DEFINED OPENSSL_VERSION)
        set(PROVIZIO_DDS_OPENSSL_VERSION "${OPENSSL_VERSION}")
    else()
        set(PROVIZIO_DDS_OPENSSL_VERSION "${OpenSSL_VERSION}")
    endif()

    # Every package location this configure was given or found, as a package's own lookups of its
    # dependencies may take them from: a <Package>_DIR that holds that package's configuration, and
    # a <Package>_ROOT. Both are cache entries -- find_package records the first, and either can be
    # given on the command line, of no particular type then -- and CMake's own are no package's.
    # Each is written as a set() of its own rather than as a pair in one list: a value can be a list
    # itself (a <Package>_ROOT of two directories), and its ; would then split it across the pairs.
    set(PROVIZIO_DDS_OPENSSL_LOCATIONS "")
    get_cmake_property(_entries CACHE_VARIABLES)
    list(SORT _entries)
    foreach(_entry IN LISTS _entries)
        if(_entry MATCHES "^CMAKE_")
            continue()
        endif()
        get_property(_value CACHE "${_entry}" PROPERTY VALUE)
        if(_entry MATCHES "^(.+)_DIR$")
            set(_package "${CMAKE_MATCH_1}")
            string(TOLOWER "${_package}" _package_lower)
            if(NOT IS_DIRECTORY "${_value}"
                    OR NOT (EXISTS "${_value}/${_package}Config.cmake" OR EXISTS "${_value}/${_package_lower}-config.cmake"))
                continue()
            endif()
        elseif(NOT _entry MATCHES "_ROOT$" OR "${_value}" STREQUAL "")
            continue()
        endif()
        string(APPEND PROVIZIO_DDS_OPENSSL_LOCATIONS
            "    set([==[${_entry}]==] [==[${_value}]==])\n"
            "    list(APPEND _provizio_dds_openssl_names [==[${_entry}]==])\n")
    endforeach()

    configure_file("${_PROVIZIO_DDS_OPENSSL_PACKAGE_DIR}/FindOpenSSL.cmake.in" "${directory}/FindOpenSSL.cmake" @ONLY)
endfunction()
