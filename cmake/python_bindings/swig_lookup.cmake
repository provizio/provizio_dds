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


#   provizio_dds_find_swig()
#
# find_package(SWIG REQUIRED COMPONENTS python), with two things FindSWIG gets wrong done first.
#
# Which SWIG: before CMake 3.30, FindSWIG looks every swig<N>.0 up before a plain swig, wherever
# each is, so a distribution's swig4.0 in /usr/bin -- which Ubuntu keeps when its swig package is
# removed -- wins over a newer SWIG in /usr/local/bin, such as the one install_dependencies.sh
# builds. With those CMake versions the executable is looked up here instead, directory by
# directory as later FindSWIG does, so the first SWIG on the search path is taken whatever it is
# named; under SWIG_ROOT (or the SWIG_ROOT environment variable) first, which FindSWIG would search
# first and a lookup outside find_package does not. A SWIG_EXECUTABLE given is taken as it is.
#
# Which library: FindSWIG caches SWIG_DIR and SWIG_VERSION as found for the executable, and does not
# look again while they are set, so pointing SWIG_EXECUTABLE at another SWIG -- or upgrading the one
# there in place, as install_dependencies.sh does -- kept the first one's version and library
# directory, which UseSWIG then has the other SWIG generate code from. Both are found again
# whenever the executable is not the one they were found for, by its path and its content -- not its
# time, which an installer may keep, or which a change within the second keeps; but not in a tree's
# first configure, where nothing was found before, nor a SWIG_DIR that
# was given rather than found. And a SWIG_EXECUTABLE cached that is gone -- a distribution's
# swig4.0, which install_dependencies.sh removes -- is looked up again rather than kept.
#
# A macro, for FindSWIG's results (SWIG_FOUND, SWIG_USE_FILE, SWIG_VERSION...) to be the caller's.

# The executable named in SWIG_EXECUTABLE as <out> is to tell it from another: its path, and the
# hash of its content where it is a file there of any size -- not a device or a pipe, which have none
# and could be read without end
macro(_provizio_dds_swig_identity out)
    set(${out} "${SWIG_EXECUTABLE}")
    if(SWIG_EXECUTABLE AND EXISTS "${SWIG_EXECUTABLE}" AND NOT IS_DIRECTORY "${SWIG_EXECUTABLE}")
        file(SIZE "${SWIG_EXECUTABLE}" _provizio_dds_swig_size)
        if(_provizio_dds_swig_size GREATER 0)
            file(SHA256 "${SWIG_EXECUTABLE}" _provizio_dds_swig_hash)
            string(APPEND ${out} "@${_provizio_dds_swig_hash}")
        endif()
        unset(_provizio_dds_swig_size)
        unset(_provizio_dds_swig_hash)
    endif()
endmacro()

macro(provizio_dds_find_swig)
    if(SWIG_EXECUTABLE AND IS_ABSOLUTE "${SWIG_EXECUTABLE}" AND NOT EXISTS "${SWIG_EXECUTABLE}")
        unset(SWIG_EXECUTABLE CACHE)
    endif()
    if(NOT SWIG_EXECUTABLE AND CMAKE_VERSION VERSION_LESS "3.30")
        set(_provizio_dds_swig_roots)
        foreach(_provizio_dds_swig_root IN ITEMS "${SWIG_ROOT}" "$ENV{SWIG_ROOT}")
            # Only one that is set: an empty root would make its bin /bin, ahead of the search path
            if(NOT "${_provizio_dds_swig_root}" STREQUAL "")
                list(APPEND _provizio_dds_swig_roots "${_provizio_dds_swig_root}/bin" "${_provizio_dds_swig_root}")
            endif()
        endforeach()
        if(_provizio_dds_swig_roots)
            find_program(SWIG_EXECUTABLE NAMES swig4.0 swig3.0 swig2.0 swig NAMES_PER_DIR
                PATHS ${_provizio_dds_swig_roots} NO_DEFAULT_PATH)
        endif()
        if(NOT SWIG_EXECUTABLE)
            find_program(SWIG_EXECUTABLE NAMES swig4.0 swig3.0 swig2.0 swig NAMES_PER_DIR)
        endif()
        unset(_provizio_dds_swig_root)
        unset(_provizio_dds_swig_roots)
    endif()

    _provizio_dds_swig_identity(_provizio_dds_swig_identity_now)
    # The cache file is written once the configure is done, so a first configure has none yet
    if(EXISTS "${CMAKE_BINARY_DIR}/CMakeCache.txt"
            AND NOT "${_PROVIZIO_DDS_SWIG_FOUND_FOR}" STREQUAL "${_provizio_dds_swig_identity_now}")
        unset(SWIG_VERSION CACHE)
        # A tree configured before this was found for records nothing, and found its SWIG_DIR
        if(NOT DEFINED CACHE{_PROVIZIO_DDS_SWIG_DIR_FOUND}
                OR (NOT "${_PROVIZIO_DDS_SWIG_DIR_FOUND}" STREQUAL ""
                    AND "${SWIG_DIR}" STREQUAL "${_PROVIZIO_DDS_SWIG_DIR_FOUND}"))
            unset(SWIG_DIR CACHE)
        endif()
    endif()
    set(_provizio_dds_swig_dir_given FALSE)
    if(SWIG_DIR)
        set(_provizio_dds_swig_dir_given TRUE)
    endif()

    find_package(SWIG REQUIRED COMPONENTS python)

    _provizio_dds_swig_identity(_provizio_dds_swig_identity_now)
    set(_PROVIZIO_DDS_SWIG_FOUND_FOR "${_provizio_dds_swig_identity_now}" CACHE INTERNAL
        "The SWIG executable, and the hash of its content, SWIG_DIR and SWIG_VERSION were found for")
    # Kept from one configure to the next while SWIG_DIR stays: given then, or found then
    if(NOT _provizio_dds_swig_dir_given)
        set(_PROVIZIO_DDS_SWIG_DIR_FOUND "${SWIG_DIR}" CACHE INTERNAL "The SWIG_DIR FindSWIG found")
    elseif(NOT DEFINED CACHE{_PROVIZIO_DDS_SWIG_DIR_FOUND}
            OR NOT "${SWIG_DIR}" STREQUAL "${_PROVIZIO_DDS_SWIG_DIR_FOUND}")
        set(_PROVIZIO_DDS_SWIG_DIR_FOUND "" CACHE INTERNAL "The SWIG_DIR FindSWIG found")
    endif()
    unset(_provizio_dds_swig_identity_now)
    unset(_provizio_dds_swig_dir_given)
endmacro()
