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

# The install of the OpenSSL runtime Fast-DDS loads (see PROVIZIO_DDS_PRIVATE_LIB_DIR in the top-level
# CMakeLists.txt):
#
#   provizio_dds_install_openssl_runtime(<directory>)
#
# adds to the install the OpenSSL libraries in <directory> as they are at install time -- whatever
# the openssl_runtime step placed there after the configure, or whatever the prebuilt binaries carry
# -- rather than as a configure-time list of them, into lib/${PROVIZIO_DDS_PRIVATE_LIB_DIR} under the
# prefix of the install being run and the DESTDIR of a staged one. It removes from there then the
# OpenSSL libraries an earlier install put there that this one does not install: of a build that has
# since been given another OpenSSL, or the system's -- after the install, so that an install that
# fails removes nothing. That directory is provizio_dds's own, and
# Fast-DDS's libraries load from it ahead of the system's directories, so a copy left there would be
# loaded in place of the OpenSSL now chosen.
#
# Included by the top-level CMakeLists.txt, for the function, and by the install code the function
# writes, with PROVIZIO_DDS_OPENSSL_RUNTIME_FROM and PROVIZIO_DDS_OPENSSL_RUNTIME_TO set, to run that
# install; and by the openssl_runtime_install test, through both.

if(CMAKE_SCRIPT_MODE_FILE AND DEFINED PROVIZIO_DDS_OPENSSL_RUNTIME_FROM)
    # At install time, which runs in script mode, as no configure does
    include("${CMAKE_CURRENT_LIST_DIR}/install_root.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/glob_escape.cmake")

    function(_provizio_dds_install_openssl_runtime from to)
        # The directories as they are: a [, * or ? in either is no pattern of the globs below, which
        # would otherwise match, and remove, the files of other directories
        provizio_dds_glob_escape(_from_pattern "${from}")
        file(GLOB _runtime LIST_DIRECTORIES false "${_from_pattern}/libssl*" "${_from_pattern}/libcrypto*")
        set(_names)
        foreach(_file IN LISTS _runtime)
            get_filename_component(_name "${_file}" NAME)
            list(APPEND _names "${_name}")
        endforeach()

        if(_runtime)
            file(INSTALL ${_runtime} DESTINATION "${CMAKE_INSTALL_PREFIX}/${to}")
        endif()

        provizio_dds_install_root(_root)
        provizio_dds_glob_escape(_to_pattern "${_root}/${to}")
        file(GLOB _installed LIST_DIRECTORIES false "${_to_pattern}/libssl*" "${_to_pattern}/libcrypto*")
        foreach(_file IN LISTS _installed)
            get_filename_component(_name "${_file}" NAME)
            list(FIND _names "${_name}" _at)
            if(_at EQUAL -1)
                message(STATUS "Removing: ${_file}")
                file(REMOVE "${_file}")
            endif()
        endforeach()
    endfunction()

    _provizio_dds_install_openssl_runtime("${PROVIZIO_DDS_OPENSSL_RUNTIME_FROM}" "${PROVIZIO_DDS_OPENSSL_RUNTIME_TO}")
    unset(PROVIZIO_DDS_OPENSSL_RUNTIME_FROM)
    unset(PROVIZIO_DDS_OPENSSL_RUNTIME_TO)
    return()
endif()

set(_PROVIZIO_DDS_INSTALL_OPENSSL_RUNTIME "${CMAKE_CURRENT_LIST_FILE}")

function(provizio_dds_install_openssl_runtime directory)
    # The paths go into the install code as bracket arguments, of a level none of them can close early
    set(_to "lib/${PROVIZIO_DDS_PRIVATE_LIB_DIR}")
    set(_level "==")
    string(FIND "${directory}]${_to}]${_PROVIZIO_DDS_INSTALL_OPENSSL_RUNTIME}]" "]${_level}]" _at)
    while(NOT _at EQUAL -1)
        string(APPEND _level "=")
        string(FIND "${directory}]${_to}]${_PROVIZIO_DDS_INSTALL_OPENSSL_RUNTIME}]" "]${_level}]" _at)
    endwhile()
    install(CODE "set(PROVIZIO_DDS_OPENSSL_RUNTIME_FROM [${_level}[${directory}]${_level}])
        set(PROVIZIO_DDS_OPENSSL_RUNTIME_TO [${_level}[${_to}]${_level}])
        include([${_level}[${_PROVIZIO_DDS_INSTALL_OPENSSL_RUNTIME}]${_level}])")
endfunction()
