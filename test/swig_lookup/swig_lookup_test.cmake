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

# Coverage for provizio_dds_find_swig (cmake/python_bindings/swig_lookup.cmake): the SWIG taken is
# the first on the search path whatever it is named -- a newer swig in one directory over an older
# swig4.0 in a later one, as a distribution's swig4.0 in /usr/bin and one built into /usr/local/bin
# are -- or the one under SWIG_ROOT, given as a variable or in the environment. And in a configured
# tree, pointing SWIG_EXECUTABLE at another SWIG, or upgrading the one there in place, takes that
# one's version and library too; a SWIG cached that is gone is looked up again; and a SWIG_DIR
# given is kept.
#
# The SWIGs answer -version, -swiglib and -help as SWIG does, each with a library directory of its
# own, and the stand-in project is configured with them alone on its search path, so that no SWIG of
# the host's can answer for them. They are of two kinds, each run through every case below: shell
# scripts, on a POSIX host, and copies of the swig_stand_in program (STAND_IN) with their answers
# appended -- the only kind that runs on Windows, where FindSWIG's execute_process of -version and
# -swiglib goes through CreateProcess, which starts a program image and nothing else (no #! script).
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -DGENERATOR=<generator>
#         [-DMAKE_PROGRAM=<program>] [-DSTAND_IN=<swig_stand_in>] -P swig_lookup_test.cmake
#
# MAKE_PROGRAM is needed for a generator of Ninja or Makefiles: nothing but the SWIGs is on the
# search path the stand-in is configured with, where CMake would otherwise look for it. STAND_IN is
# needed on Windows, the scripts being of no use there.

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT ${_var})
        message(FATAL_ERROR "swig_lookup_test.cmake: ${_var} is required")
    endif()
endforeach()
if(GENERATOR MATCHES "Ninja|Makefiles" AND NOT MAKE_PROGRAM)
    message(FATAL_ERROR "swig_lookup_test.cmake: MAKE_PROGRAM is required with the ${GENERATOR} generator")
endif()
set(_kinds)
if(CMAKE_HOST_UNIX)
    list(APPEND _kinds script)
endif()
if(STAND_IN)
    list(APPEND _kinds built)
elseif(CMAKE_HOST_WIN32)
    message(FATAL_ERROR "swig_lookup_test.cmake: STAND_IN is required on Windows, which runs no script as a SWIG")
endif()
# The search path's own separator, between the directories each case puts on it
if(CMAKE_HOST_WIN32)
    set(_sep ";")
else()
    set(_sep ":")
endif()

set(_top "${WORK_DIR}")
file(REMOVE_RECURSE "${_top}")

set(_make_program)
if(MAKE_PROGRAM)
    set(_make_program "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}")
endif()

# A SWIG of <version> named <name> in <dir>, of the kind being run, with a library directory of its
# own, or saying the one given after. Sets _exe to the file it is, which a built one names with the
# host's suffix for programs.
function(_swig dir name version)
    set(_swiglib "${WORK_DIR}/lib_${version}")
    if(ARGC GREATER 3)
        set(_swiglib "${ARGV3}")
    else()
        file(WRITE "${_swiglib}/swig.swg" "")
    endif()
    if(_kind STREQUAL "script")
        file(WRITE "${dir}/${name}" "#!/bin/sh
case \"$1\" in
-version) echo; echo \"SWIG Version ${version}\" ;;
-swiglib) echo \"${_swiglib}\" ;;
-help) echo \"     -python         - Generate Python wrappers\" ;;
esac
")
        execute_process(COMMAND chmod 755 "${dir}/${name}" RESULT_VARIABLE _result)
        if(NOT _result EQUAL 0)
            message(FATAL_ERROR "swig_lookup_test.cmake: could not make ${dir}/${name} executable")
        endif()
        return()
    endif()
    # A copy of the program, then its answers and their length in eight digits (see swig_stand_in.cpp).
    # Text with no line break, which file(APPEND) writes as it is on every host.
    set(_file "${dir}/${name}${_suffix}")
    file(MAKE_DIRECTORY "${dir}")
    execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${STAND_IN}" "${_file}" RESULT_VARIABLE _result)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "swig_lookup_test.cmake: could not copy ${STAND_IN} to ${_file}")
    endif()
    set(_answers "${version}|${_swiglib}")
    string(LENGTH "${_answers}" _length)
    string(LENGTH "${_length}" _digits)
    while(_digits LESS 8)
        string(PREPEND _length "0")
        math(EXPR _digits "${_digits} + 1")
    endwhile()
    file(APPEND "${_file}" "${_answers}${_length}")
endfunction()

# Configures the stand-in project in <case>, with the directories <path> (separated by _sep) alone on
# the search path and the options and environment given after (each environment entry as ENV
# <name>=<value>), and checks it found <expected>, as <executable>|<version>|<library directory>.
# The search path is its own argument: one holding a ; would be split as a list in the others.
function(_check case path expected)
    set(_env)
    set(_options)
    set(_next_is_env FALSE)
    foreach(_arg IN LISTS ARGN)
        if(_next_is_env)
            list(APPEND _env "${_arg}")
            set(_next_is_env FALSE)
        elseif(_arg STREQUAL "ENV")
            set(_next_is_env TRUE)
        else()
            list(APPEND _options "${_arg}")
        endif()
    endforeach()
    # Nothing of the caller's that CMake would search ahead of the search path
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=SWIG_ROOT --unset=CMAKE_PREFIX_PATH
            --unset=CMAKE_PROGRAM_PATH --unset=CMAKE_APPBUNDLE_PATH --unset=CMAKE_TOOLCHAIN_FILE "PATH=${path}"
            ${_env} "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/project" -B "${WORK_DIR}/${case}"
            -G "${GENERATOR}" ${_make_program} "-DREPOSITORY=${SOURCE_DIR}" "-DOUT_FILE=${WORK_DIR}/${case}.txt"
            ${_options}
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "Case ${case} (${_kind}): configuring the stand-in failed:\n${_output}")
    endif()
    file(READ "${WORK_DIR}/${case}.txt" _found)
    if(NOT _found STREQUAL expected)
        message(FATAL_ERROR "Case ${case} (${_kind}): found [${_found}], expected [${expected}]")
    endif()
endfunction()

# Gives <file> a time long past, failing the test where it cannot: archived with that time, and
# extracted over itself, which every host's CMake can do (unlike touch -t)
function(_touch_old file)
    get_filename_component(_dir "${file}" DIRECTORY)
    get_filename_component(_name "${file}" NAME)
    set(_archive "${WORK_DIR}/old_time.tar")
    execute_process(COMMAND "${CMAKE_COMMAND}" -E tar cf "${_archive}" "--mtime=2020-01-01 00:00:00 UTC" "${_name}"
        WORKING_DIRECTORY "${_dir}" RESULT_VARIABLE _archived ERROR_VARIABLE _error)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E tar xf "${_archive}"
        WORKING_DIRECTORY "${_dir}" RESULT_VARIABLE _extracted ERROR_VARIABLE _error)
    file(REMOVE "${_archive}")
    file(TIMESTAMP "${file}" _year "%Y" UTC)
    if(NOT _archived EQUAL 0 OR NOT _extracted EQUAL 0 OR NOT _year STREQUAL "2020")
        message(FATAL_ERROR "swig_lookup_test.cmake: could not give ${file} a time long past: it is of "
            "[${_year}] (${_error})")
    endif()
endfunction()

# Every case below, with the SWIGs of <kind>
function(_run_cases kind)
    set(_kind "${kind}")
    set(WORK_DIR "${_top}/${kind}")
    set(_suffix "")
    if(kind STREQUAL "built" AND CMAKE_HOST_WIN32)
        set(_suffix ".exe")
    endif()

    _swig("${WORK_DIR}/local/bin" swig 4.4.1)
    _swig("${WORK_DIR}/system/bin" swig4.0 4.0.1)
    _swig("${WORK_DIR}/root/bin" swig 4.4.2)

    # 1. The first on the search path, though another name comes before swig in FindSWIG's list
    _check(first "${WORK_DIR}/local/bin${_sep}${WORK_DIR}/system/bin"
        "${WORK_DIR}/local/bin/swig${_suffix}|4.4.1|${WORK_DIR}/lib_4.4.1")

    # 2. The one under SWIG_ROOT, as a variable and in the environment, ahead of the search path
    _check(root_variable "${WORK_DIR}/system/bin" "${WORK_DIR}/root/bin/swig${_suffix}|4.4.2|${WORK_DIR}/lib_4.4.2"
        "-DSWIG_ROOT=${WORK_DIR}/root")
    _check(root_environment "${WORK_DIR}/system/bin" "${WORK_DIR}/root/bin/swig${_suffix}|4.4.2|${WORK_DIR}/lib_4.4.2"
        ENV "SWIG_ROOT=${WORK_DIR}/root")

    # 3. Another SWIG given to the tree of case 1: its version and library, not those of the first
    _check(first "${WORK_DIR}/local/bin${_sep}${WORK_DIR}/system/bin"
        "${WORK_DIR}/system/bin/swig4.0${_suffix}|4.0.1|${WORK_DIR}/lib_4.0.1"
        "-DSWIG_EXECUTABLE=${WORK_DIR}/system/bin/swig4.0${_suffix}")

    # 4. The SWIG upgraded in place, as install_dependencies.sh and install_dependencies.ps1 do: the new
    #    one's version and library -- and so again with the time of the old one kept, as an installer
    #    may keep a file's time
    _swig("${WORK_DIR}/upgrade/bin" swig 4.3.0)
    _check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig${_suffix}|4.3.0|${WORK_DIR}/lib_4.3.0")
    _swig("${WORK_DIR}/upgrade/bin" swig 4.4.1)
    _check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig${_suffix}|4.4.1|${WORK_DIR}/lib_4.4.1")
    _touch_old("${WORK_DIR}/upgrade/bin/swig${_suffix}")
    _check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig${_suffix}|4.4.1|${WORK_DIR}/lib_4.4.1")
    _swig("${WORK_DIR}/upgrade/bin" swig 4.5.0)
    _touch_old("${WORK_DIR}/upgrade/bin/swig${_suffix}")
    _check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig${_suffix}|4.5.0|${WORK_DIR}/lib_4.5.0")

    # 5. A SWIG cached that is gone, as a distribution's swig4.0 install_dependencies.sh removes: the
    #    one the search path has now
    _swig("${WORK_DIR}/gone/bin" swig4.0 4.0.2)
    _check(gone "${WORK_DIR}/local/bin${_sep}${WORK_DIR}/gone/bin"
        "${WORK_DIR}/gone/bin/swig4.0${_suffix}|4.0.2|${WORK_DIR}/lib_4.0.2"
        "-DSWIG_EXECUTABLE=${WORK_DIR}/gone/bin/swig4.0${_suffix}")
    file(REMOVE "${WORK_DIR}/gone/bin/swig4.0${_suffix}")
    _check(gone "${WORK_DIR}/local/bin${_sep}${WORK_DIR}/gone/bin"
        "${WORK_DIR}/local/bin/swig${_suffix}|4.4.1|${WORK_DIR}/lib_4.4.1")

    # 6. A SWIG_DIR given, for a SWIG that says a library it does not have: kept in the first configure
    #    and the next
    file(WRITE "${WORK_DIR}/given_lib/swig.swg" "")
    _swig("${WORK_DIR}/relocated/bin" swig 4.4.3 "${WORK_DIR}/nowhere")
    _check(given_dir "${WORK_DIR}/local/bin" "${WORK_DIR}/relocated/bin/swig${_suffix}|4.4.3|${WORK_DIR}/given_lib"
        "-DSWIG_EXECUTABLE=${WORK_DIR}/relocated/bin/swig${_suffix}" "-DSWIG_DIR=${WORK_DIR}/given_lib")
    _check(given_dir "${WORK_DIR}/local/bin" "${WORK_DIR}/relocated/bin/swig${_suffix}|4.4.3|${WORK_DIR}/given_lib")
endfunction()

foreach(_kind_to_run IN LISTS _kinds)
    _run_cases(${_kind_to_run})
endforeach()

file(REMOVE_RECURSE "${_top}")
message(STATUS "swig_lookup: the SWIG found is the one meant, with its own version and library (${_kinds})")
