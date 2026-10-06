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

# Coverage for cmake/fast_dds/openssl_runtime.cmake, the step placing next to Fast-DDS's installed
# libraries the shared OpenSSL libraries they load.
#
# The script decides from what dumpbin, readelf or otool say a library imports, so it runs here with
# the tool replaced by openssl_runtime_fake_tool.cmake, printing what the real one prints for the
# libraries of each case: the parsing for every platform is covered wherever the test runs, and no
# binary is needed. Each case has a DESTINATION holding a Fast-DDS library and OpenSSL libraries an
# earlier run placed there, and directories of an OpenSSL, and says what DESTINATION holds after.
#
# Invoked as:
#   cmake -DRUNTIME_SCRIPT=<path> -DFAKE_TOOL=<path> -DFAKE_COMMAND=<path> -DWORK_DIR=<scratch dir>
#         -P openssl_runtime_test.cmake

# 3.22, as the Fast-DDS built here needs it: this test is registered only where that is built, and
# the stand-in tool relies on CMake 3.18 (cmake -E cat, and options after -- for a -P script).
cmake_minimum_required(VERSION 3.22)

foreach(_var IN ITEMS RUNTIME_SCRIPT FAKE_TOOL FAKE_COMMAND WORK_DIR)
    if(NOT ${_var})
        message(FATAL_ERROR "openssl_runtime_test.cmake: ${_var} is required")
    endif()
endforeach()
file(REMOVE_RECURSE "${WORK_DIR}")

# Files of a case: <directory>/<name> holding <name>, to tell a copy by its content
function(_files dir)
    foreach(_name IN LISTS ARGN)
        file(WRITE "${dir}/${_name}" "${_name}")
    endforeach()
endfunction()

# What the tool prints for <name> in case <case>
function(_prints case name output)
    file(WRITE "${WORK_DIR}/${case}/imports/${name}.txt" "${output}")
endfunction()

# Runs the script over case <case>, with the directories given relative to the case's, and checks
# that DESTINATION then holds exactly <expected>, OpenSSL files with the content of the originals.
# With _RELINK set, install_name_tool is given too, and with _CODESIGN, codesign, as stand-ins
# logging their calls to <case>/commands.log, a codesign --verify failing with _VERIFY_ERROR where
# that is set.
function(_check case kind search_dirs system_dirs expected)
    set(_case_dir "${WORK_DIR}/${case}")
    set(_search)
    foreach(_dir IN LISTS search_dirs)
        list(APPEND _search "${_case_dir}/${_dir}")
    endforeach()
    set(_system)
    foreach(_dir IN LISTS system_dirs)
        list(APPEND _system "${_case_dir}/${_dir}")
    endforeach()
    string(REPLACE ";" "|" _search "${_search}")
    string(REPLACE ";" "|" _system "${_system}")
    set(_tool)
    if(kind)
        set(_tool "${CMAKE_COMMAND}|-DFAKE_IMPORTS_DIR=${_case_dir}/imports|-P|${FAKE_TOOL}|--")
    endif()
    set(_relink_tool)
    set(_codesign_tool)
    set(_log "-DFAKE_LOG=${_case_dir}/commands.log")
    if(_RELINK)
        set(_relink_tool "${CMAKE_COMMAND}|${_log}|-DFAKE_NAME=install_name_tool|-P|${FAKE_COMMAND}|--")
    endif()
    if(_CODESIGN)
        set(_codesign_tool "${CMAKE_COMMAND}|${_log}|-DFAKE_NAME=codesign|-DFAKE_VERIFY_ERROR=${_VERIFY_ERROR}|-P|${FAKE_COMMAND}|--")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" "-DDESTINATION=${_case_dir}/dest" "-DSEARCH_DIRS=${_search}"
            "-DSYSTEM_DIRS=${_system}" "-DTOOL=${_tool}" "-DTOOL_KIND=${kind}" "-DRELINK_TOOL=${_relink_tool}"
            "-DCODESIGN_TOOL=${_codesign_tool}" -P "${RUNTIME_SCRIPT}"
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "Case ${case}: openssl_runtime.cmake failed:\n${_output}")
    endif()
    string(REGEX REPLACE "([[*?])" "[\\1]" _pattern "${_case_dir}/dest")
    file(GLOB _held RELATIVE "${_case_dir}/dest" "${_pattern}/*")
    list(SORT _held)
    list(SORT expected)
    if(NOT _held STREQUAL expected)
        message(FATAL_ERROR "Case ${case}: expected [${expected}] in DESTINATION, found [${_held}]\n${_output}")
    endif()
    foreach(_name IN LISTS _held)
        if(_name MATCHES "^lib(ssl|crypto)")
            file(READ "${_case_dir}/dest/${_name}" _content)
            string(TOLOWER "${_content}" _content)
            if(NOT _content STREQUAL _name)
                message(FATAL_ERROR "Case ${case}: ${_name} is not a copy of the original, it holds '${_content}'")
            endif()
        endif()
    endforeach()
    set(_last_output "${_output}" PARENT_SCOPE)
endfunction()

# Windows: the DLLs imported, whatever their case, and those they import; not the others. A stale
# placement of another version goes.
_files("${WORK_DIR}/dumpbin/dest" fastdds-3.6.dll libssl-1_1-x64.dll libcrypto-1_1-x64.dll)
_files("${WORK_DIR}/dumpbin/openssl/bin" libssl-3-x64.dll libcrypto-3-x64.dll capi.dll)
_prints(dumpbin fastdds-3.6.dll [=[
Dump of file C:\build\fast_dds_build\install\bin\fastdds-3.6.dll

File Type: DLL

  Image has the following dependencies:

    fastcdr-2.3.dll
    libssl-3-x64.dll
    KERNEL32.dll

  Summary

        1000 .data
]=])
_prints(dumpbin libssl-3-x64.dll [=[
Dump of file C:\build\fast_dds_build\install\bin\libssl-3-x64.dll

File Type: DLL

  Image has the following dependencies:

    LIBCRYPTO-3-X64.DLL
    KERNEL32.dll

  Summary
]=])
_prints(dumpbin libcrypto-3-x64.dll [=[
Dump of file C:\build\fast_dds_build\install\bin\libcrypto-3-x64.dll

File Type: DLL

  Image has the following dependencies:

    WS2_32.dll
    KERNEL32.dll

  Summary
]=])
_check(dumpbin dumpbin "openssl/bin" "" "fastdds-3.6.dll;libssl-3-x64.dll;libcrypto-3-x64.dll")

# Linux: the SONAMEs needed, of more than one number too, from a directory that is not the system's
_files("${WORK_DIR}/readelf/dest" libfastdds.so.3.6.2.0 libssl.so.3)
_files("${WORK_DIR}/readelf/openssl/lib" libssl.so.1.1 libcrypto.so.1.1 libssl.so.3)
set(_fastdds_dynamic_section [=[

Dynamic section at offset 0x5a3c08 contains 38 entries:
  Tag        Type                         Name/Value
 0x0000000000000001 (NEEDED)             Shared library: [libfastcdr.so.2]
 0x0000000000000001 (NEEDED)             Shared library: [libssl.so.1.1]
 0x0000000000000001 (NEEDED)             Shared library: [libcrypto.so.1.1]
 0x0000000000000001 (NEEDED)             Shared library: [libc.so.6]
 0x000000000000000e (SONAME)             Library soname: [libfastdds.so.3.6]
 0x000000000000001d (RUNPATH)            Library runpath: [$ORIGIN:$ORIGIN/../lib]
]=])
_prints(readelf libfastdds.so.3.6.2.0 "${_fastdds_dynamic_section}")
_prints(readelf libssl.so.1.1 [=[

Dynamic section at offset 0xa1d30 contains 27 entries:
  Tag        Type                         Name/Value
 0x0000000000000001 (NEEDED)             Shared library: [libcrypto.so.1.1]
 0x0000000000000001 (NEEDED)             Shared library: [libc.so.6]
 0x000000000000000e (SONAME)             Library soname: [libssl.so.1.1]
]=])
_prints(readelf libcrypto.so.1.1 [=[

Dynamic section at offset 0x3b6c10 contains 26 entries:
  Tag        Type                         Name/Value
 0x0000000000000001 (NEEDED)             Shared library: [libc.so.6]
 0x000000000000000e (SONAME)             Library soname: [libcrypto.so.1.1]
]=])
_check(readelf readelf "openssl/lib" "" "libfastdds.so.3.6.2.0;libssl.so.1.1;libcrypto.so.1.1")

# Linux: an OpenSSL of the system's own is left where the loader looks anyway
_files("${WORK_DIR}/readelf_system/dest" libfastdds.so.3.6.2.0 libssl.so.1.1)
_files("${WORK_DIR}/readelf_system/openssl/lib" libssl.so.1.1 libcrypto.so.1.1)
_prints(readelf_system libfastdds.so.3.6.2.0 "${_fastdds_dynamic_section}")
_check(readelf_system readelf "openssl/lib" "openssl/lib" "libfastdds.so.3.6.2.0")

# A DESTINATION whose name a pattern would match another's by: as above, and the other untouched
_files("${WORK_DIR}/readelf_bracket[1]/dest" libfastdds.so.3.6.2.0 libssl.so.1.1)
_files("${WORK_DIR}/readelf_bracket[1]/openssl/lib" libssl.so.1.1 libcrypto.so.1.1)
_files("${WORK_DIR}/readelf_bracket1/dest" libfastdds.so.3.5.0.0 libssl.so.1.0.2)
_prints("readelf_bracket[1]" libfastdds.so.3.6.2.0 "${_fastdds_dynamic_section}")
_check("readelf_bracket[1]" readelf "openssl/lib" "openssl/lib" "libfastdds.so.3.6.2.0")
string(REGEX REPLACE "([[*?])" "[\\1]" _pattern "${WORK_DIR}/readelf_bracket1/dest")
file(GLOB _held RELATIVE "${WORK_DIR}/readelf_bracket1/dest" "${_pattern}/*")
list(SORT _held)
if(NOT _held STREQUAL "libfastdds.so.3.5.0.0;libssl.so.1.0.2")
    message(FATAL_ERROR "Case readelf_bracket[1]: another DESTINATION now holds [${_held}]")
endif()

# macOS: those named by an @rpath install name, from Fast-DDS or from another placed, but not one
# named by an absolute path, which dyld loads from there
_files("${WORK_DIR}/otool/dest" libfastdds.3.6.2.dylib)
_files("${WORK_DIR}/otool/openssl/lib" libssl.3.dylib libcrypto.3.dylib)
_prints(otool libfastdds.3.6.2.dylib [=[
/build/fast_dds_build/install/lib/libfastdds.3.6.2.dylib:
	@rpath/libfastdds.3.6.dylib (compatibility version 3.6.0, current version 3.6.2)
	@rpath/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	/opt/homebrew/opt/openssl@3/lib/libcrypto.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	/usr/lib/libc++.1.dylib (compatibility version 1.0.0, current version 1800.101.0)
]=])
_prints(otool libssl.3.dylib [=[
/build/fast_dds_build/install/lib/libssl.3.dylib:
	@rpath/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	@rpath/libcrypto.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	/usr/lib/libSystem.B.dylib (compatibility version 1.0.0, current version 1345.100.2)
]=])
_prints(otool libcrypto.3.dylib [=[
/build/fast_dds_build/install/lib/libcrypto.3.dylib:
	@rpath/libcrypto.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	/usr/lib/libSystem.B.dylib (compatibility version 1.0.0, current version 1345.100.2)
]=])
_check(otool otool "openssl/lib" "" "libfastdds.3.6.2.dylib;libssl.3.dylib;libcrypto.3.dylib")
# Nothing to warn of where libraries are named @rpath/ or by absolute paths only
if(_last_output MATCHES "CMake Warning")
    message(FATAL_ERROR "Case otool: warned of nothing amiss:\n${_last_output}")
endif()

# macOS: those named relative to the library loading them, next to it, as for an @rpath one; and
# Fast-DDS made to name them @rpath/ -- once, though universal -- as an install puts the two apart,
# and signed again where its signature no longer holds -- not where it does, nor where it has
# none -- while the libraries placed, always put together, are left naming one another as they did.
# Without install_name_tool or codesign, said what that leaves undone.
set(_loader_path_cases otool_loader_path_invalid otool_loader_path_valid otool_loader_path_unsigned
    otool_loader_path_no_relink otool_loader_path_no_codesign)
foreach(_case IN LISTS _loader_path_cases)
    _files("${WORK_DIR}/${_case}/dest" libfastdds.3.6.2.dylib)
    _files("${WORK_DIR}/${_case}/openssl/lib" libssl.3.dylib libcrypto.3.dylib)
    # A universal library, which otool names the imports of once per architecture
    _prints(${_case} libfastdds.3.6.2.dylib [=[
/build/fast_dds_build/install/lib/libfastdds.3.6.2.dylib (architecture x86_64):
	@rpath/libfastdds.3.6.dylib (compatibility version 3.6.0, current version 3.6.2)
	@loader_path/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)
/build/fast_dds_build/install/lib/libfastdds.3.6.2.dylib (architecture arm64):
	@rpath/libfastdds.3.6.dylib (compatibility version 3.6.0, current version 3.6.2)
	@loader_path/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)
]=])
    _prints(${_case} libssl.3.dylib [=[
/build/fast_dds_build/install/lib/libssl.3.dylib:
	@loader_path/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	@loader_path/libcrypto.3.dylib (compatibility version 3.0.0, current version 3.0.0)
]=])
    _prints(${_case} libcrypto.3.dylib [=[
/build/fast_dds_build/install/lib/libcrypto.3.dylib:
	@loader_path/libcrypto.3.dylib (compatibility version 3.0.0, current version 3.0.0)
]=])
endforeach()
# Checks that case <case> ran the commands <expected>, one per line, <file> standing for Fast-DDS's
function(_expect_commands case expected)
    get_filename_component(_fast_dds "${WORK_DIR}/${case}/dest/libfastdds.3.6.2.dylib" REALPATH)
    string(REPLACE "<file>" "${_fast_dds}" expected "${expected}")
    set(_commands "")
    if(EXISTS "${WORK_DIR}/${case}/commands.log")
        file(READ "${WORK_DIR}/${case}/commands.log" _commands)
    endif()
    if(NOT _commands STREQUAL expected)
        message(FATAL_ERROR "Case ${case}: expected the commands\n${expected}found\n${_commands}")
    endif()
endfunction()
set(_expected_placed "libfastdds.3.6.2.dylib;libssl.3.dylib;libcrypto.3.dylib")
set(_relink_command "install_name_tool -change @loader_path/libssl.3.dylib @rpath/libssl.3.dylib <file>\n")
set(_RELINK TRUE)
set(_CODESIGN TRUE)
set(_VERIFY_ERROR "<file>: invalid signature (code or signature have been modified)")
_check(otool_loader_path_invalid otool "openssl/lib" "" "${_expected_placed}")
_expect_commands(otool_loader_path_invalid
    "${_relink_command}codesign --verify <file>\ncodesign --force --sign - <file>\n")
set(_VERIFY_ERROR "")
_check(otool_loader_path_valid otool "openssl/lib" "" "${_expected_placed}")
_expect_commands(otool_loader_path_valid "${_relink_command}codesign --verify <file>\n")
set(_VERIFY_ERROR "<file>: code object is not signed at all")
_check(otool_loader_path_unsigned otool "openssl/lib" "" "${_expected_placed}")
_expect_commands(otool_loader_path_unsigned "${_relink_command}codesign --verify <file>\n")
set(_VERIFY_ERROR "")
set(_RELINK FALSE)
_check(otool_loader_path_no_relink otool "openssl/lib" "" "${_expected_placed}")
_expect_commands(otool_loader_path_no_relink "codesign --verify <file>\n")
# A warning's words, which CMake wraps, apart
if(NOT _last_output MATCHES "no[ \t\r\n]+install_name_tool[ \t\r\n]+was[ \t\r\n]+given")
    message(FATAL_ERROR "Case otool_loader_path_no_relink: no warning that an install cannot load it:\n${_last_output}")
endif()
set(_RELINK TRUE)
set(_CODESIGN FALSE)
_check(otool_loader_path_no_codesign otool "openssl/lib" "" "${_expected_placed}")
_expect_commands(otool_loader_path_no_codesign "${_relink_command}")
if(NOT _last_output MATCHES "no[ \t\r\n]+codesign[ \t\r\n]+was[ \t\r\n]+given")
    message(FATAL_ERROR "Case otool_loader_path_no_codesign: no warning of the signature left unchecked:\n"
        "${_last_output}")
endif()
set(_RELINK FALSE)

# macOS: Homebrew's, named by absolute paths only, is placed nowhere; nor is one named relative to
# the executable, which is warned about
_files("${WORK_DIR}/otool_absolute/dest" libfastdds.3.6.2.dylib)
_files("${WORK_DIR}/otool_absolute/openssl/lib" libssl.3.dylib libcrypto.3.dylib)
_prints(otool_absolute libfastdds.3.6.2.dylib [=[
/build/fast_dds_build/install/lib/libfastdds.3.6.2.dylib:
	@rpath/libfastdds.3.6.dylib (compatibility version 3.6.0, current version 3.6.2)
	/opt/homebrew/opt/openssl@3/lib/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	/opt/homebrew/opt/openssl@3/lib/libcrypto.3.dylib (compatibility version 3.0.0, current version 3.0.0)
	@executable_path/../Frameworks/libcrypto.3.dylib (compatibility version 3.0.0, current version 3.0.0)
]=])
_check(otool_absolute otool "openssl/lib" "" "libfastdds.3.6.2.dylib")
# The name only, as CMake wraps a warning's lines
string(FIND "${_last_output}" "@executable_path/../Frameworks/libcrypto.3.dylib" _at)
if(_at EQUAL -1)
    message(FATAL_ERROR "Case otool_absolute: no warning of the library named relative to the executable:\n"
        "${_last_output}")
endif()

# A library loaded that the OpenSSL's directories do not have is warned about, not failed on
_files("${WORK_DIR}/missing/dest" fastdds-3.6.dll)
_files("${WORK_DIR}/missing/openssl/bin" capi.dll)
_prints(missing fastdds-3.6.dll [=[
  Image has the following dependencies:

    libssl-3-x64.dll
]=])
_check(missing dumpbin "openssl/bin" "" "fastdds-3.6.dll")
# CMake wraps a warning's text, so any whitespace can stand between two words
if(NOT _last_output MATCHES "libssl-3-x64\\.dll,[ \t\r\n]+which")
    message(FATAL_ERROR "Case missing: no warning about libssl-3-x64.dll:\n${_last_output}")
endif()

# Without a tool, what an earlier run placed still goes
_files("${WORK_DIR}/no_tool/dest" fastdds-3.6.dll libssl-3-x64.dll libcrypto-3-x64.dll)
_check(no_tool "" "" "" "fastdds-3.6.dll")

message(STATUS "openssl_runtime.cmake: all cases pass")
