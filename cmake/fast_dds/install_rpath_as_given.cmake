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

# Makes the Fast-DDS library keep, on macOS too, the install rpath provizio_dds builds it with.
#
# provizio_dds gives the Fast-DDS build CMAKE_INSTALL_RPATH: @loader_path, @loader_path/../lib and
# @loader_path/provizio_dds on macOS, as $ORIGIN and the rest are on Linux. Relative entries are the
# only ones that mean anything once the libraries are installed or packaged somewhere: a pip
# package, a consumer's install prefix. Fast-DDS overrides the value on Apple, though, in
# src/cpp/CMakeLists.txt, ahead of its add_library():
#
#   if(APPLE)
#       ...
#       set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#
# a normal variable, so it shadows the cache entry given on the command line. libfastdds then carries
# a single LC_RPATH, the absolute path of Fast-DDS's install directory in this build tree, which is
# shipped as it is. It names its dependencies by @rpath -- libfastcdr, and an OpenSSL that is
# @rpath-named (Conan's, vcpkg's, conda's) -- so once the build tree is gone, or on any other
# machine, dyld finds them only where something else happens to supply them: a library that loaded
# libfastdds and carries an rpath of its own, or one already loaded under the same install name. The
# OpenSSL provizio_dds places next to Fast-DDS (see openssl_runtime.cmake) is then found nowhere --
# where the installed layout keeps it apart, under lib/provizio_dds -- and `import provizio_dds`,
# which loads libfastdds by path, fails on it.
#
# Patched to set that value only when none was given, as it is for any build of Fast-DDS that
# provizio_dds does not make. Nothing changes off Apple, where the block does not run.
#
# This runs as the Fast-DDS ExternalProject PATCH_COMMAND. Idempotent, and self-checking like the
# other scripts here: if the block has moved, it FAILs rather than leave libfastdds unrelocatable.
#
# Invoked as:
#   cmake -DFAST_DDS_CPP_CMAKELISTS=<path-to-src/cpp/CMakeLists.txt> -P install_rpath_as_given.cmake

if(NOT DEFINED FAST_DDS_CPP_CMAKELISTS)
    message(FATAL_ERROR "install_rpath_as_given.cmake: FAST_DDS_CPP_CMAKELISTS must be defined")
endif()

if(NOT EXISTS "${FAST_DDS_CPP_CMAKELISTS}")
    message(FATAL_ERROR "install_rpath_as_given.cmake: file not found: ${FAST_DDS_CPP_CMAKELISTS}")
endif()

set(_marker "# [provizio_dds] install rpath as given")
set(_overriding [=[
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
]=])
set(_as_given [=[
    # [provizio_dds] install rpath as given: the relative one provizio_dds builds Fast-DDS with,
    # rather than this build's absolute install directory (see cmake/fast_dds/install_rpath_as_given.cmake)
    if(NOT CMAKE_INSTALL_RPATH)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    endif()
]=])

# Sources are read and written through patch_io.cmake, which keeps the line endings of the
# checkout and of the host from mattering (see there).
include("${CMAKE_CURRENT_LIST_DIR}/patch_io.cmake")
provizio_dds_patch_read("${FAST_DDS_CPP_CMAKELISTS}" _contents)

# NO REVISION MARKER HERE, deliberately, and there is a rule attached to that. A tree carries no
# record of WHICH revision of a patch script wrote it, so a bare "already patched" marker means only
# "some revision did" -- and the moment the replacement text above changes, every existing build
# tree keeps the OLD text while reporting itself patched. resource_event_per_timer_wait.cmake hit
# exactly that and now carries a _revision / _revision_marker pair plus a migration.
#
# So: IF YOU CHANGE THE REPLACEMENT TEXT ABOVE, add that mechanism first, and make the migration
# decide from what the file CONTAINS rather than from which marker it carries.

string(FIND "${_contents}" "${_marker}" _already_pos)
if(NOT _already_pos EQUAL -1)
    message(STATUS "install_rpath_as_given: Fast-DDS already keeps the install rpath given -- no-op")
    return()
endif()

# The override exactly once, inside the Apple block that precedes the library's add_library()
string(FIND "${_contents}" "if(APPLE)\n    set(CMAKE_MACOSX_RPATH ON)" _apple_pos)
string(FIND "${_contents}" "${_overriding}" _pos)
string(FIND "${_contents}" "${_overriding}" _last_pos REVERSE)
string(FIND "${_contents}" "add_library(" _add_library_pos)
if(_apple_pos EQUAL -1 OR _pos EQUAL -1 OR NOT _pos EQUAL _last_pos OR _pos LESS _apple_pos
        OR (NOT _add_library_pos EQUAL -1 AND _add_library_pos LESS _pos))
    message(FATAL_ERROR
        "install_rpath_as_given: could not find, once and ahead of add_library(), inside the\n"
        "  'if(APPLE)' block of ${FAST_DDS_CPP_CMAKELISTS}, the line\n"
        "${_overriding}"
        "Fast-DDS may have changed how it sets its install rpath on macOS; update this patch so that "
        "the relative one provizio_dds builds it with is kept there -- without it libfastdds finds "
        "its @rpath dependencies, the OpenSSL placed next to it among them, only in this build tree.")
endif()

string(REPLACE "${_overriding}" "${_as_given}" _contents "${_contents}")
provizio_dds_patch_write("${FAST_DDS_CPP_CMAKELISTS}" "${_contents}")
message(STATUS "install_rpath_as_given: Fast-DDS keeps the install rpath given on macOS as well")
