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

# Where the tree being installed goes, for the install steps of provizio_dds's own that write or
# remove files by path, which only file(INSTALL) places under DESTDIR by itself:
#
#   provizio_dds_install_root(<out_var>)
#
# sets <out_var> to the prefix of the install being run (a "cmake --install --prefix" one among
# them), read when it runs rather than at the configure, under the DESTDIR of a staged install --
# taken as file(INSTALL) takes it: on Windows without the prefix's drive letter, which no path can
# follow, with DESTDIR's backslashes as slashes, and a leading ~ as HOME where HOME is set (where it
# is not, the ~ stays, a directory of that name, as it does for file(INSTALL)). A ~user is not
# expanded, which file(INSTALL) does on POSIX hosts.
#
# Included by the top-level CMakeLists.txt's install code, and by the install_root test.

function(provizio_dds_install_root out_var)
    set(_root "${CMAKE_INSTALL_PREFIX}")
    set(_destdir "$ENV{DESTDIR}")
    if(NOT _destdir STREQUAL "")
        string(REPLACE "\\" "/" _destdir "${_destdir}")
        if(_destdir MATCHES "^~(/|$)" AND DEFINED ENV{HOME})
            string(SUBSTRING "${_destdir}" 1 -1 _destdir)
            set(_destdir "$ENV{HOME}${_destdir}")
        endif()
        if(CMAKE_HOST_WIN32)
            string(REGEX REPLACE "^[A-Za-z]:" "" _root "${_root}")
        endif()
        set(_root "${_destdir}${_root}")
    endif()
    set(${out_var} "${_root}" PARENT_SCOPE)
endfunction()
