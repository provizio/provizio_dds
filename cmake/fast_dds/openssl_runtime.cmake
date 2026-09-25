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

# Places the shared OpenSSL libraries that Fast-DDS's installed libraries load next to them, and
# removes the ones placed there before. Run as a step of the Fast-DDS ExternalProject, after its
# install, by provizio_dds's CMakeLists.txt, which says why they go there.
#
# Which libraries is read from the libraries in DESTINATION with TOOL, as TOOL_KIND says:
#   dumpbin  (Windows)  the OpenSSL DLLs they import;
#   readelf  (Linux)    the OpenSSL SONAMEs they need, but those found in one of SYSTEM_DIRS, the
#                       system's own directories, where the loader looks anyway;
#   otool    (macOS)    the OpenSSL libraries they name by an install name relative to their rpath
#                       (@rpath/) or to themselves (@loader_path/, next to them), dyld loading one
#                       named by an absolute path from that path;
# and so on for the libraries placed, as libssl imports libcrypto. Each is looked for in SEARCH_DIRS,
# the directories of the OpenSSL that Fast-DDS was built against, and copied under the name it is
# loaded by. A static OpenSSL is loaded as none, so nothing is placed for it.
#
# An @loader_path/ name is found next to the library naming it only, rpath or no rpath, and an
# install puts the OpenSSL runtime apart from Fast-DDS's own libraries (see
# PROVIZIO_DDS_PRIVATE_LIB_DIR), so each of Fast-DDS's libraries naming one placed here is made to
# name it @rpath/ instead, with RELINK_TOOL (install_name_tool), which their rpath finds wherever
# the two are put; and any of them loading one placed here whose signature no longer holds is
# signed again with CODESIGN_TOOL (codesign), ad hoc, as dyld refuses such a library on Apple
# silicon. The placed libraries name one another as they did: they are always put together.
#
#   cmake -DDESTINATION=<directory> -DSEARCH_DIRS=<directory>[|<directory>...]
#         [-DSYSTEM_DIRS=<directory>[|<directory>...]] [-DTOOL=<command>] [-DTOOL_KIND=<kind>]
#         [-DRELINK_TOOL=<command>] [-DCODESIGN_TOOL=<command>] -P openssl_runtime.cmake
#
# Lists arrive |-separated, or ;-separated where ExternalProject has put back its LIST_SEPARATOR.
# Each tool can be a command with arguments of its own, which its own arguments follow. Without a
# TOOL the script only removes.

cmake_minimum_required(VERSION 3.15)

if(NOT DESTINATION)
    message(FATAL_ERROR "openssl_runtime.cmake: DESTINATION is required")
endif()
foreach(_list IN ITEMS SEARCH_DIRS SYSTEM_DIRS TOOL RELINK_TOOL CODESIGN_TOOL)
    string(REPLACE "|" ";" ${_list} "${${_list}}")
endforeach()

# DESTINATION as it is in the globs below: a [, * or ? in it is no pattern, which would otherwise
# match, and remove, the files of other directories
string(REGEX REPLACE "([[*?])" "[\\1]" _destination_pattern "${DESTINATION}")

# Fast-DDS installs no OpenSSL of its own, so any here is one this script placed
file(GLOB _stale "${_destination_pattern}/libssl*" "${_destination_pattern}/libcrypto*")
if(_stale)
    file(REMOVE ${_stale})
endif()
if(NOT TOOL)
    return()
endif()

if(TOOL_KIND STREQUAL "dumpbin")
    # dumpbin takes its options with - as well as /, and - is the one no shell mistakes for a path
    set(_tool_args -NOLOGO -DEPENDENTS)
    set(_library_regex "\\.[dD][lL][lL]$")
elseif(TOOL_KIND STREQUAL "readelf")
    set(_tool_args -d)
    set(_library_regex "\\.so(\\.[0-9]+)*$")
elseif(TOOL_KIND STREQUAL "otool")
    set(_tool_args -L)
    set(_library_regex "\\.dylib$")
else()
    message(FATAL_ERROR "openssl_runtime.cmake: unknown TOOL_KIND '${TOOL_KIND}'")
endif()

# The OpenSSL libraries a file loads, by the names the loader looks them up by, in <out>, and of
# those, the ones it names @loader_path/, in <loader_path_out>
function(_openssl_imports file out loader_path_out)
    execute_process(COMMAND ${TOOL} ${_tool_args} "${file}"
        OUTPUT_VARIABLE _output RESULT_VARIABLE _result ERROR_VARIABLE _error)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "openssl_runtime.cmake: '${TOOL}' could not read ${file}: ${_error}")
    endif()
    string(REPLACE "\r" "" _output "${_output}")
    string(REPLACE ";" "," _output "${_output}")
    string(REPLACE "\n" ";" _lines "${_output}")
    set(_names)
    set(_loader_path_names)
    set(_unplaced)
    foreach(_line IN LISTS _lines)
        if(TOOL_KIND STREQUAL "dumpbin")
            # One per line, indented, under "Image has the following dependencies:"
            string(TOLOWER "${_line}" _line)
            if(_line MATCHES "^[ \t]+(lib(ssl|crypto)[^ \t]*\\.dll)[ \t]*$")
                list(APPEND _names "${CMAKE_MATCH_1}")
            endif()
        elseif(TOOL_KIND STREQUAL "readelf")
            # " 0x0000000000000001 (NEEDED)  Shared library: [libssl.so.3]"
            if(_line MATCHES "\\(NEEDED\\).*\\[(lib(ssl|crypto)\\.so[^]]*)\\]")
                list(APPEND _names "${CMAKE_MATCH_1}")
            endif()
        else()
            # A tab, then "@rpath/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)",
            # or "@loader_path/libssl.3.dylib (...)" for one next to the library loading it
            if(_line MATCHES "^[ \t]+@(rpath|loader_path)/(lib(ssl|crypto)[^ \t/(]*\\.dylib)")
                list(APPEND _names "${CMAKE_MATCH_2}")
                if(CMAKE_MATCH_1 STREQUAL "loader_path")
                    list(APPEND _loader_path_names "${CMAKE_MATCH_2}")
                endif()
            elseif(_line MATCHES "^[ \t]+(@[^ \t]*/lib(ssl|crypto)[^ \t/(]*\\.dylib)")
                # Relative to the executable, or to a directory of its own: nowhere a copy can be put
                # for every program and layout, so said at build time rather than by dyld at run time
                list(APPEND _unplaced "${CMAKE_MATCH_1}")
            endif()
        endif()
    endforeach()
    # Each once, where a universal library names it once per architecture
    foreach(_list IN ITEMS _names _loader_path_names _unplaced)
        if(${_list})
            list(REMOVE_DUPLICATES ${_list})
        endif()
    endforeach()
    foreach(_name IN LISTS _unplaced)
        message(WARNING "openssl_runtime.cmake: ${file} loads ${_name}, which is placed nowhere, so it is found "
            "only where that name leads for the program running")
    endforeach()
    set(${out} "${_names}" PARENT_SCOPE)
    set(${loader_path_out} "${_loader_path_names}" PARENT_SCOPE)
endfunction()

set(_system_dirs)
foreach(_dir IN LISTS SYSTEM_DIRS)
    get_filename_component(_dir "${_dir}" REALPATH)
    list(APPEND _system_dirs "${_dir}")
endforeach()

# The libraries installed here, each once: a link names the file it points at
set(_queue)
file(GLOB _candidates "${_destination_pattern}/*")
foreach(_candidate IN LISTS _candidates)
    if(NOT IS_DIRECTORY "${_candidate}" AND _candidate MATCHES "${_library_regex}")
        get_filename_component(_candidate "${_candidate}" REALPATH)
        list(APPEND _queue "${_candidate}")
    endif()
endforeach()
list(REMOVE_DUPLICATES _queue)
set(_fast_dds_libraries ${_queue})

set(_seen)
# Each OpenSSL library each of Fast-DDS's libraries loads, and of those, each it names @loader_path/,
# as <file>|<name>
set(_fast_dds_imports)
set(_relinks)
while(_queue)
    list(POP_FRONT _queue _file)
    _openssl_imports("${_file}" _names _loader_path_names)
    if(_file IN_LIST _fast_dds_libraries)
        foreach(_name IN LISTS _names)
            list(APPEND _fast_dds_imports "${_file}|${_name}")
        endforeach()
        foreach(_name IN LISTS _loader_path_names)
            list(APPEND _relinks "${_file}|${_name}")
        endforeach()
    endif()
    foreach(_name IN LISTS _names)
        if(_name IN_LIST _seen)
            continue()
        endif()
        list(APPEND _seen "${_name}")
        set(_found)
        foreach(_dir IN LISTS SEARCH_DIRS)
            if(EXISTS "${_dir}/${_name}")
                set(_found "${_dir}/${_name}")
                break()
            endif()
        endforeach()
        if(NOT _found)
            message(WARNING "openssl_runtime.cmake: ${_name}, which ${_file} loads, is in none of "
                "${SEARCH_DIRS}, the directories of the OpenSSL Fast-DDS was built against, so it is not "
                "placed next to Fast-DDS, and the loader will take whichever ${_name} it finds elsewhere")
            continue()
        endif()
        get_filename_component(_found_dir "${_found}" DIRECTORY)
        get_filename_component(_found_dir "${_found_dir}" REALPATH)
        if(_found_dir IN_LIST _system_dirs)
            continue()
        endif()
        # The file itself under the name it is loaded by, which on Linux can be a link to a file
        # named for the full version: cmake -E copy copies what a link points at
        execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${_found}" "${DESTINATION}/${_name}"
            RESULT_VARIABLE _result)
        if(NOT _result EQUAL 0)
            message(FATAL_ERROR "openssl_runtime.cmake: failed to copy ${_found} to ${DESTINATION}")
        endif()
        message(STATUS "Placed ${_name} from ${_found_dir}")
        list(APPEND _queue "${DESTINATION}/${_name}")
    endforeach()
endwhile()

if(_relinks)
    # Once per architecture of a universal library, as otool names each architecture's imports
    list(REMOVE_DUPLICATES _relinks)
endif()
set(_relinked)
foreach(_relink IN LISTS _relinks)
    string(REPLACE "|" ";" _relink "${_relink}")
    list(GET _relink 0 _file)
    list(GET _relink 1 _name)
    if(NOT EXISTS "${DESTINATION}/${_name}")
        continue()
    endif()
    if(NOT RELINK_TOOL)
        message(WARNING "openssl_runtime.cmake: ${_file} names ${_name} @loader_path/, and no install_name_tool "
            "was given to make that @rpath/, so an install, which puts it apart from ${_name}, cannot load it")
        continue()
    endif()
    execute_process(COMMAND ${RELINK_TOOL} -change "@loader_path/${_name}" "@rpath/${_name}" "${_file}"
        RESULT_VARIABLE _result ERROR_VARIABLE _error)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "openssl_runtime.cmake: '${RELINK_TOOL}' could not make ${_file} name ${_name} "
            "@rpath/: ${_error}")
    endif()
    message(STATUS "Made ${_file} name ${_name} @rpath/")
    list(APPEND _relinked "${_file}")
endforeach()

# Each of Fast-DDS's libraries loading an OpenSSL library placed here is signed again where its
# signature no longer holds -- not merely where it was changed this time, as a run stopped between
# the change and the signing leaves nothing to change the next, and not where it holds, as
# install_name_tool keeps the linker's own signature valid where it can, and replacing that would
# leave a later change by another tool breaking it rather than updating it. One with no signature
# at all, as an x86_64 library may have, is left so.
set(_signed_candidates)
foreach(_import IN LISTS _fast_dds_imports)
    string(REPLACE "|" ";" _import "${_import}")
    list(GET _import 0 _file)
    list(GET _import 1 _name)
    if(EXISTS "${DESTINATION}/${_name}")
        list(APPEND _signed_candidates "${_file}")
    endif()
endforeach()
if(_signed_candidates)
    list(REMOVE_DUPLICATES _signed_candidates)
endif()
if(_relinked AND NOT CODESIGN_TOOL)
    message(WARNING "openssl_runtime.cmake: no codesign was given to check the signature of ${_relinked} once "
        "changed, which dyld refuses to load on Apple silicon unless it holds")
endif()
foreach(_file IN LISTS _signed_candidates)
    if(NOT CODESIGN_TOOL)
        break()
    endif()
    execute_process(COMMAND ${CODESIGN_TOOL} --verify "${_file}" RESULT_VARIABLE _result
        OUTPUT_VARIABLE _error ERROR_VARIABLE _error)
    if(_result EQUAL 0 OR _error MATCHES "not signed at all")
        continue()
    endif()
    execute_process(COMMAND ${CODESIGN_TOOL} --force --sign - "${_file}" RESULT_VARIABLE _result ERROR_VARIABLE _error)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "openssl_runtime.cmake: '${CODESIGN_TOOL}' could not sign ${_file} again: ${_error}")
    endif()
    message(STATUS "Signed ${_file} again, ad hoc")
endforeach()
