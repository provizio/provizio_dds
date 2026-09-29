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

# Coverage for cmake/install_openssl_runtime.cmake: the OpenSSL runtime an install puts into
# provizio_dds's own directory under lib/ is the one the build at hand has -- none, where it has none
# -- whatever an earlier install into the same prefix put there, under the DESTDIR of a staged install
# alone, with nothing else in that directory touched, and nothing of another directory a [ in the
# name of one would make a pattern match. Each case installs a project of its own that writes the
# install code with provizio_dds_install_openssl_runtime(), as the top-level CMakeLists.txt does.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -DGENERATOR=<generator> -P openssl_runtime_install_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT ${_var})
        message(FATAL_ERROR "openssl_runtime_install_test.cmake: ${_var} is required")
    endif()
endforeach()

file(REMOVE_RECURSE "${WORK_DIR}")
file(WRITE "${WORK_DIR}/project/CMakeLists.txt" "
cmake_minimum_required(VERSION 3.15)
project(openssl_runtime_install LANGUAGES NONE)
set(PROVIZIO_DDS_PRIVATE_LIB_DIR provizio_dds)
include([==[${SOURCE_DIR}/cmake/install_openssl_runtime.cmake]==])
provizio_dds_install_openssl_runtime(\"\${RUNTIME}\")
")

# Installs the project configured for the runtime in <from> into <prefix>, under DESTDIR <destdir> ("" for none)
function(_install case from prefix destdir)
    set(_build "${WORK_DIR}/builds/${case}")
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${WORK_DIR}/project" -B "${_build}" -G "${GENERATOR}"
            "-DRUNTIME=${from}"
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(_result EQUAL 0)
        execute_process(COMMAND "${CMAKE_COMMAND}" -E env "DESTDIR=${destdir}"
                "${CMAKE_COMMAND}" --install "${_build}" --prefix "${prefix}"
            RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    endif()
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "openssl_runtime_install_test (${case}): configuring or installing failed:\n${_output}")
    endif()
endfunction()

# Checks that <directory> holds <name>=<content>... and nothing else
function(_expect case directory)
    string(REGEX REPLACE "([[*?])" "[\\1]" _pattern "${directory}")
    file(GLOB _held LIST_DIRECTORIES false RELATIVE "${directory}" "${_pattern}/*")
    set(_found)
    foreach(_name IN LISTS _held)
        file(READ "${directory}/${_name}" _content)
        list(APPEND _found "${_name}=${_content}")
    endforeach()
    list(SORT _found)
    set(_expected ${ARGN})
    list(SORT _expected)
    if(NOT "${_found}" STREQUAL "${_expected}")
        message(FATAL_ERROR "openssl_runtime_install_test (${case}): ${directory} holds [${_found}], not [${_expected}]")
    endif()
endfunction()

# What an earlier install, of a build with another OpenSSL, left in <prefix>'s directory
function(_installed_before prefix)
    set(_private "${prefix}/lib/provizio_dds")
    file(MAKE_DIRECTORY "${_private}")
    file(WRITE "${_private}/libssl.so.1.1" "old")
    file(WRITE "${_private}/libcrypto.so.1.1" "old")
    file(WRITE "${_private}/notes.txt" "not an OpenSSL library")
endfunction()
set(_before libssl.so.1.1=old libcrypto.so.1.1=old "notes.txt=not an OpenSSL library")

# The runtime of the build at hand, of another build a [ in the name of this one's directory would
# make a pattern match, and none, of a build taking the system's OpenSSL
set(_runtime "${WORK_DIR}/build[1]/lib")
file(WRITE "${_runtime}/libssl.so.3" "new")
file(WRITE "${_runtime}/libcrypto.so.3" "new")
file(WRITE "${_runtime}/libfastdds.so.3.6" "not OpenSSL")
file(WRITE "${WORK_DIR}/build1/lib/libssl.so.9" "another build's")
set(_none "${WORK_DIR}/system_openssl/lib")
file(MAKE_DIRECTORY "${_none}")

# Another OpenSSL than the one installed before: the old one goes, the new one is installed
_installed_before("${WORK_DIR}/replaced")
_install(replaced "${_runtime}" "${WORK_DIR}/replaced" "")
_expect(replaced "${WORK_DIR}/replaced/lib/provizio_dds" libssl.so.3=new libcrypto.so.3=new "notes.txt=not an OpenSSL library")

# The system's OpenSSL now, of which nothing is installed: every one installed before goes
_installed_before("${WORK_DIR}/system")
_install(system "${_none}" "${WORK_DIR}/system" "")
_expect(system "${WORK_DIR}/system/lib/provizio_dds" "notes.txt=not an OpenSSL library")

# A staged install: what is under DESTDIR, and nothing of the live prefix itself -- under DESTDIR
# without the prefix's drive letter on Windows, as file(INSTALL) puts it there
set(_staged "${WORK_DIR}/live")
if(CMAKE_HOST_WIN32)
    string(REGEX REPLACE "^[A-Za-z]:" "" _staged "${_staged}")
endif()
set(_staged "${WORK_DIR}/stage${_staged}")
_installed_before("${_staged}")
_installed_before("${WORK_DIR}/live")
_install(staged "${_none}" "${WORK_DIR}/live" "${WORK_DIR}/stage")
_expect(staged "${_staged}/lib/provizio_dds" "notes.txt=not an OpenSSL library")
_expect(staged_live "${WORK_DIR}/live/lib/provizio_dds" ${_before})

# A prefix whose name a pattern would match the names of others by: those are not touched
_installed_before("${WORK_DIR}/app[12]")
_installed_before("${WORK_DIR}/app1")
_installed_before("${WORK_DIR}/app2")
_install(bracketed "${_none}" "${WORK_DIR}/app[12]" "")
_expect(bracketed "${WORK_DIR}/app[12]/lib/provizio_dds" "notes.txt=not an OpenSSL library")
_expect(bracketed_other "${WORK_DIR}/app1/lib/provizio_dds" ${_before})
_expect(bracketed_other "${WORK_DIR}/app2/lib/provizio_dds" ${_before})

# Into a new prefix, where nothing was installed before
_install(new "${_runtime}" "${WORK_DIR}/new" "")
_expect(new "${WORK_DIR}/new/lib/provizio_dds" libssl.so.3=new libcrypto.so.3=new)

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "openssl_runtime_install: an install leaves the OpenSSL runtime of the build at hand, and none of an earlier one")
