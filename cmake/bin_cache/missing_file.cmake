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

# Whether the prebuilt binaries of the bin cache hold what a build hands on (see the bin cache
# section of the top-level CMakeLists.txt):
#
#   provizio_dds_bin_cache_missing_file(<out_missing> <directory> <name>...)
#
# sets <out_missing> to the first <name> -- a path relative to <directory>, which may be a glob
# pattern (lib/libfastdds.so.*), as the libraries a cache bundles carry their versions in their
# names -- that no file under <directory> answers to, or to "" when every one is there. A
# directory, a link to nothing, or a file a link leads to outside <directory> answers to no name.
# An archive published truncated or malformed extracts all the same: taken for usable without one
# of these, the imported targets name a file that is not there, or the install hands on a library
# that cannot load, where a build from source would still serve.

include("${CMAKE_CURRENT_LIST_DIR}/../glob_escape.cmake")

function(provizio_dds_bin_cache_missing_file out_missing directory)
    provizio_dds_glob_escape(_directory_pattern "${directory}")
    get_filename_component(_real_directory "${directory}" REALPATH)
    foreach(_name IN LISTS ARGN)
        file(GLOB _matches LIST_DIRECTORIES false "${_directory_pattern}/${_name}")
        set(_found FALSE)
        foreach(_match IN LISTS _matches)
            # A file the cache holds, not one a link in it leads to elsewhere
            get_filename_component(_real_match "${_match}" REALPATH)
            string(FIND "${_real_match}" "${_real_directory}/" _inside)
            if(_inside EQUAL 0 AND EXISTS "${_real_match}" AND NOT IS_DIRECTORY "${_real_match}")
                set(_found TRUE)
                break()
            endif()
        endforeach()
        if(NOT _found)
            set(${out_missing} "${_name}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    set(${out_missing} "" PARENT_SCOPE)
endfunction()
