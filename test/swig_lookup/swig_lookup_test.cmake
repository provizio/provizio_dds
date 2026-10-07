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
# The SWIGs are shell scripts answering -version, -swiglib and -help as SWIG does, each with a
# library directory of its own, and the stand-in project is configured with them alone on its
# search path, so that no SWIG of the host's can answer for them.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -DGENERATOR=<generator>
#         [-DMAKE_PROGRAM=<program>] -P swig_lookup_test.cmake
#
# MAKE_PROGRAM is needed for a generator of Ninja or Makefiles: nothing but the SWIGs is on the
# search path the stand-in is configured with, where CMake would otherwise look for it.

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT ${_var})
        message(FATAL_ERROR "swig_lookup_test.cmake: ${_var} is required")
    endif()
endforeach()
if(GENERATOR MATCHES "Ninja|Makefiles" AND NOT MAKE_PROGRAM)
    message(FATAL_ERROR "swig_lookup_test.cmake: MAKE_PROGRAM is required with the ${GENERATOR} generator")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")

# A SWIG of <version> named <name> in <dir>, with a library directory of its own, or saying the one
# given after
function(_swig dir name version)
    set(_swiglib "${WORK_DIR}/lib_${version}")
    if(ARGC GREATER 3)
        set(_swiglib "${ARGV3}")
    else()
        file(WRITE "${_swiglib}/swig.swg" "")
    endif()
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
endfunction()
_swig("${WORK_DIR}/local/bin" swig 4.4.1)
_swig("${WORK_DIR}/system/bin" swig4.0 4.0.1)
_swig("${WORK_DIR}/root/bin" swig 4.4.2)

set(_make_program)
if(MAKE_PROGRAM)
    set(_make_program "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}")
endif()

# Configures the stand-in project in <case>, with the search path <path> and the options and
# environment given after (each environment entry as ENV <name>=<value>), and checks it found
# <expected>, as <executable>|<version>|<library directory>
function(_check case path expected)
    set(_env "PATH=${path}")
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
            --unset=CMAKE_PROGRAM_PATH --unset=CMAKE_APPBUNDLE_PATH --unset=CMAKE_TOOLCHAIN_FILE ${_env}
            "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/project" -B "${WORK_DIR}/${case}" -G "${GENERATOR}"
            ${_make_program} "-DREPOSITORY=${SOURCE_DIR}" "-DOUT_FILE=${WORK_DIR}/${case}.txt" ${_options}
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "Case ${case}: configuring the stand-in failed:\n${_output}")
    endif()
    file(READ "${WORK_DIR}/${case}.txt" _found)
    if(NOT _found STREQUAL expected)
        message(FATAL_ERROR "Case ${case}: found [${_found}], expected [${expected}]")
    endif()
endfunction()

# 1. The first on the search path, though another name comes before swig in FindSWIG's list
_check(first "${WORK_DIR}/local/bin:${WORK_DIR}/system/bin"
    "${WORK_DIR}/local/bin/swig|4.4.1|${WORK_DIR}/lib_4.4.1")

# 2. The one under SWIG_ROOT, as a variable and in the environment, ahead of the search path
_check(root_variable "${WORK_DIR}/system/bin" "${WORK_DIR}/root/bin/swig|4.4.2|${WORK_DIR}/lib_4.4.2"
    "-DSWIG_ROOT=${WORK_DIR}/root")
_check(root_environment "${WORK_DIR}/system/bin" "${WORK_DIR}/root/bin/swig|4.4.2|${WORK_DIR}/lib_4.4.2"
    ENV "SWIG_ROOT=${WORK_DIR}/root")

# 3. Another SWIG given to the tree of case 1: its version and library, not those of the first
_check(first "${WORK_DIR}/local/bin:${WORK_DIR}/system/bin"
    "${WORK_DIR}/system/bin/swig4.0|4.0.1|${WORK_DIR}/lib_4.0.1" "-DSWIG_EXECUTABLE=${WORK_DIR}/system/bin/swig4.0")

# Gives <file> a time long past, failing the test where it cannot
function(_touch_old file)
    execute_process(COMMAND touch -t 202001010000 "${file}" RESULT_VARIABLE _result)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "swig_lookup_test.cmake: could not set the time of ${file}")
    endif()
endfunction()

# 4. The SWIG upgraded in place, as install_dependencies.sh does: the new one's version and library
#    -- and so again with the time of the old one kept, as an installer may keep a file's time
_swig("${WORK_DIR}/upgrade/bin" swig 4.3.0)
_check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig|4.3.0|${WORK_DIR}/lib_4.3.0")
_swig("${WORK_DIR}/upgrade/bin" swig 4.4.1)
_check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig|4.4.1|${WORK_DIR}/lib_4.4.1")
_touch_old("${WORK_DIR}/upgrade/bin/swig")
_check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig|4.4.1|${WORK_DIR}/lib_4.4.1")
_swig("${WORK_DIR}/upgrade/bin" swig 4.5.0)
_touch_old("${WORK_DIR}/upgrade/bin/swig")
_check(upgrade "${WORK_DIR}/upgrade/bin" "${WORK_DIR}/upgrade/bin/swig|4.5.0|${WORK_DIR}/lib_4.5.0")

# 5. A SWIG cached that is gone, as a distribution's swig4.0 install_dependencies.sh removes: the
#    one the search path has now
_swig("${WORK_DIR}/gone/bin" swig4.0 4.0.2)
_check(gone "${WORK_DIR}/local/bin:${WORK_DIR}/gone/bin" "${WORK_DIR}/gone/bin/swig4.0|4.0.2|${WORK_DIR}/lib_4.0.2"
    "-DSWIG_EXECUTABLE=${WORK_DIR}/gone/bin/swig4.0")
file(REMOVE "${WORK_DIR}/gone/bin/swig4.0")
_check(gone "${WORK_DIR}/local/bin:${WORK_DIR}/gone/bin" "${WORK_DIR}/local/bin/swig|4.4.1|${WORK_DIR}/lib_4.4.1")

# 6. A SWIG_DIR given, for a SWIG that says a library it does not have: kept in the first configure
#    and the next
file(WRITE "${WORK_DIR}/given_lib/swig.swg" "")
_swig("${WORK_DIR}/relocated/bin" swig 4.4.3 "${WORK_DIR}/nowhere")
_check(given_dir "${WORK_DIR}/local/bin" "${WORK_DIR}/relocated/bin/swig|4.4.3|${WORK_DIR}/given_lib"
    "-DSWIG_EXECUTABLE=${WORK_DIR}/relocated/bin/swig" "-DSWIG_DIR=${WORK_DIR}/given_lib")
_check(given_dir "${WORK_DIR}/local/bin" "${WORK_DIR}/relocated/bin/swig|4.4.3|${WORK_DIR}/given_lib")

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "swig_lookup: the SWIG found is the one meant, with its own version and library")
