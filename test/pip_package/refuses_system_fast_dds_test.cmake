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

# Coverage for the check near the top of the top-level CMakeLists.txt that keeps a pip package on
# provizio_dds's own Fast-DDS: real configures of this source tree, each in a binary directory made
# afresh and removed after, so that nothing an earlier run -- or a tree configured with another
# generator -- left there decides the outcome. The check comes ahead of project(), so each configure
# but the last two stop within a second: no compiler is looked for, and nothing is downloaded or
# built. The last two get as far as project(), which reads a toolchain file, and the check made again
# after it.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -DGENERATOR=<generator>
#         -P refuses_system_fast_dds_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT ${_var})
        message(FATAL_ERROR "refuses_system_fast_dds_test.cmake: ${_var} is required")
    endif()
endforeach()

# Configures the source tree in a fresh <WORK_DIR>/<case> with the options given, leaving its output
# in _output and its exit status in _result.
function(_configure case)
    set(_binary_dir "${WORK_DIR}/${case}")
    file(REMOVE_RECURSE "${_binary_dir}")
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${SOURCE_DIR}" -B "${_binary_dir}" -G "${GENERATOR}" ${ARGN}
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    set(_result "${_result}" PARENT_SCOPE)
    set(_output "${_output}" PARENT_SCOPE)
endfunction()

# The check's own words, prefixes short enough that CMake's wrapping of a long error message cannot
# split them. Matching the output rather than the exit status is the point: a configure that failed
# for any other reason must not count as the check's doing.
set(_refusal "LOOK_FOR_FAST_DDS cannot be used")
set(_requirement "PYTHON_PIP_PACKAGE needs PYTHON_BINDINGS=ON")

file(REMOVE_RECURSE "${WORK_DIR}")

# 1. A pip package that asks for a system Fast-DDS, as setup.py configures one plus
#    CMAKE_ARGUMENTS=-DLOOK_FOR_FAST_DDS=TRUE: refused -- and the value refused is not left in the
#    cache, or the next configure of the tree, the one following the message's advice, would be
#    refused all over again.
_configure(refused -DPYTHON_BINDINGS=ON -DPYTHON_PIP_PACKAGE=ON
    "-DPYTHON_PACKAGES_INSTALL_DIR=${WORK_DIR}/refused/packages" -DLOOK_FOR_FAST_DDS=TRUE)
if(_result EQUAL 0 OR NOT _output MATCHES "${_refusal}")
    message(FATAL_ERROR "A pip package asking for a system Fast-DDS was not refused (exit ${_result}):\n${_output}")
endif()
if(EXISTS "${WORK_DIR}/refused/CMakeCache.txt")
    file(STRINGS "${WORK_DIR}/refused/CMakeCache.txt" _remembered REGEX "^LOOK_FOR_FAST_DDS:")
    if(_remembered)
        message(FATAL_ERROR "The refused configure left '${_remembered}' in its cache, so configuring the "
            "tree again without it would be refused again.")
    endif()
endif()

# 2. PYTHON_PIP_PACKAGE without the bindings installed as packages is no pip package at all
_configure(incomplete -DPYTHON_PIP_PACKAGE=ON)
if(_result EQUAL 0 OR NOT _output MATCHES "${_requirement}")
    message(FATAL_ERROR "PYTHON_PIP_PACKAGE without PYTHON_BINDINGS / PYTHON_PACKAGES_INSTALL_DIR was not "
        "refused (exit ${_result}):\n${_output}")
endif()

# 3. The same options but for PYTHON_PIP_PACKAGE -- the bindings installed as packages against a
#    Fast-DDS of the system's, as a distribution's recipe does -- are not the check's business. So
#    that this case too stops within a second, a toolchain file that does not exist ends the
#    configure at project(), just after the check; what matters is which of the two stopped it.
_configure(distribution -DPYTHON_BINDINGS=ON "-DPYTHON_PACKAGES_INSTALL_DIR=${WORK_DIR}/distribution/packages"
    -DLOOK_FOR_FAST_DDS=TRUE "-DCMAKE_TOOLCHAIN_FILE=${WORK_DIR}/no_such_toolchain.cmake")
if(_output MATCHES "${_refusal}|${_requirement}")
    message(FATAL_ERROR "The Python bindings installed as packages against a system Fast-DDS, with no "
        "PYTHON_PIP_PACKAGE, were refused:\n${_output}")
endif()
if(NOT _output MATCHES "no_such_toolchain")
    message(FATAL_ERROR "The configure of case 3 did not get as far as project(), so it cannot show that "
        "the check let it through (exit ${_result}):\n${_output}")
endif()

# 4. A pip package whose toolchain file asks for a system Fast-DDS, which project() reads, after the
#    check ahead of it: refused all the same, by the check made again after project()
file(WRITE "${WORK_DIR}/look_for_fast_dds_toolchain.cmake" "set(LOOK_FOR_FAST_DDS TRUE)\n")
_configure(toolchain -DPYTHON_BINDINGS=ON -DPYTHON_PIP_PACKAGE=ON
    "-DPYTHON_PACKAGES_INSTALL_DIR=${WORK_DIR}/toolchain/packages"
    "-DCMAKE_TOOLCHAIN_FILE=${WORK_DIR}/look_for_fast_dds_toolchain.cmake")
if(_result EQUAL 0 OR NOT _output MATCHES "${_refusal}")
    message(FATAL_ERROR "A pip package whose toolchain file asks for a system Fast-DDS was not refused "
        "(exit ${_result}):\n${_output}")
endif()

# 5. A pip package whose toolchain file turns PYTHON_PIP_PACKAGE off, as well as asking for a system
#    Fast-DDS, which would take the check after project() with it: refused for turning it off
file(WRITE "${WORK_DIR}/no_pip_package_toolchain.cmake" "set(PYTHON_PIP_PACKAGE OFF)\nset(LOOK_FOR_FAST_DDS TRUE)\n")
_configure(toolchain_off -DPYTHON_BINDINGS=ON -DPYTHON_PIP_PACKAGE=ON
    "-DPYTHON_PACKAGES_INSTALL_DIR=${WORK_DIR}/toolchain_off/packages"
    "-DCMAKE_TOOLCHAIN_FILE=${WORK_DIR}/no_pip_package_toolchain.cmake")
if(_result EQUAL 0 OR NOT _output MATCHES "PYTHON_PIP_PACKAGE was asked for")
    message(FATAL_ERROR "A pip package whose toolchain file turns PYTHON_PIP_PACKAGE off was not refused "
        "(exit ${_result}):\n${_output}")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "pip_package_refuses_system_fast_dds: all cases pass")
