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

# A path taken as it is in a glob pattern:
#
#   provizio_dds_glob_escape(<out> <path>)
#
# sets <out> to <path> with every [, * and ? it holds escaped. A checkout, build or install
# directory may have one in its name, which file(GLOB) would otherwise read as part of the
# pattern: matching nothing, which reads as an empty directory and passes in silence, or the files
# of another directory.
#
# The . and .. of an absolute <path> are resolved too. Once one component of a pattern is matched by
# listing a directory, as an escaped one is, each later component is looked for among the entries
# of the directory before, which never include . or ..: a/../b would match nothing. A . is dropped,
# as it names the directory it is in however that is reached. A .. is resolved as the system
# resolves it: on Windows as text, and elsewhere to the parent of wherever the path before it really
# is -- through a link, not the directory the text before the link names, which a glob that removes
# what it matches must not take for it. The root resolves to itself.

function(provizio_dds_glob_escape out path)
    if(IS_ABSOLUTE "${path}" AND CMAKE_HOST_WIN32)
        get_filename_component(path "${path}" ABSOLUTE)
    elseif(IS_ABSOLUTE "${path}")
        string(REGEX REPLACE "/(\\./)+" "/" path "${path}")
        string(REGEX REPLACE "/\\.$" "" path "${path}")
        while(TRUE)
            string(FIND "${path}/" "/../" _at)
            if(_at EQUAL -1)
                break()
            endif()
            string(SUBSTRING "${path}" 0 ${_at} _before)
            math(EXPR _rest_at "${_at} + 3")
            string(SUBSTRING "${path}" ${_rest_at} -1 _rest)
            # The path before holds no .. of its own, which REALPATH would collapse as text first
            get_filename_component(_before "${_before}/" REALPATH)
            get_filename_component(_before "${_before}" DIRECTORY)
            string(REGEX REPLACE "/$" "" _before "${_before}")
            set(path "${_before}${_rest}")
        endwhile()
        if(path STREQUAL "")
            set(path "/")
        endif()
    endif()
    string(REGEX REPLACE "([[*?])" "[\\1]" _escaped "${path}")
    set(${out} "${_escaped}" PARENT_SCOPE)
endfunction()
