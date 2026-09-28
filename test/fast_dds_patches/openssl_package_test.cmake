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

# Coverage for the FindOpenSSL given to Fast-DDS when provizio_dds finds OpenSSL as a package
# (cmake/fast_dds/openssl_package.cmake, FindOpenSSL.cmake.in): the package's own lookups of its
# dependencies, run inside Fast-DDS's configure, must find what provizio_dds's did -- through the
# package locations and module path provizio_dds was given, not only its CMAKE_PREFIX_PATH -- and must
# not be decided by what an earlier configure of that build cached.
#
# openssl_package/provizio stands in for provizio_dds: it finds a stand-in OpenSSL package as a
# package-manager toolchain has it found (CMAKE_FIND_PACKAGE_PREFER_CONFIG) and writes the module with
# provizio_dds's own function. openssl_package/fast_dds stands in for the Fast-DDS build: it looks
# OpenSSL up through that module and says which stand-in each dependency was. The stand-ins are named
# for this test alone, so that no package of the host's, and no find module of CMake's, can answer
# for them.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -DGENERATOR=<generator>
#         -P openssl_package_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT ${_var})
        message(FATAL_ERROR "openssl_package_test.cmake: ${_var} is required")
    endif()
endforeach()

file(REMOVE_RECURSE "${WORK_DIR}")
set(_packages "${WORK_DIR}/packages")

# An OpenSSL package whose configuration looks up <dependency> (with <how>, "" or MODULE) and links it
function(_openssl_package name dependency how)
    file(WRITE "${_packages}/${name}/lib/cmake/OpenSSL/OpenSSLConfig.cmake" "
include(CMakeFindDependencyMacro)
find_dependency(${dependency} ${how})
if(NOT TARGET OpenSSL::Crypto)
    add_library(OpenSSL::Crypto INTERFACE IMPORTED)
    set_target_properties(OpenSSL::Crypto PROPERTIES INTERFACE_LINK_LIBRARIES ${dependency}::${dependency})
    add_library(OpenSSL::SSL INTERFACE IMPORTED)
    set_target_properties(OpenSSL::SSL PROPERTIES INTERFACE_LINK_LIBRARIES OpenSSL::Crypto)
endif()
set(OpenSSL_VERSION 3.9.9)
set(OPENSSL_VERSION 3.9.9)
set(OPENSSL_INCLUDE_DIR \"\${CMAKE_CURRENT_LIST_DIR}/../../../include\")
")
endfunction()
_openssl_package(openssl_z ProvizioTestZ "")
_openssl_package(openssl_foo ProvizioTestFoo MODULE)

# Two copies of a dependency found as a package, told apart by what their targets carry
foreach(_copy IN ITEMS A B)
    file(WRITE "${_packages}/z${_copy}/lib/cmake/ProvizioTestZ/ProvizioTestZConfig.cmake" "
if(NOT TARGET ProvizioTestZ::ProvizioTestZ)
    add_library(ProvizioTestZ::ProvizioTestZ INTERFACE IMPORTED)
    set_target_properties(ProvizioTestZ::ProvizioTestZ PROPERTIES INTERFACE_COMPILE_DEFINITIONS ${_copy})
endif()
")
endforeach()

# And one found by a find module of the consumer's own
file(WRITE "${WORK_DIR}/modules/FindProvizioTestFoo.cmake" "
set(ProvizioTestFoo_FOUND TRUE)
if(NOT TARGET ProvizioTestFoo::ProvizioTestFoo)
    add_library(ProvizioTestFoo::ProvizioTestFoo INTERFACE IMPORTED)
    set_target_properties(ProvizioTestFoo::ProvizioTestFoo PROPERTIES INTERFACE_COMPILE_DEFINITIONS module)
endif()
")

# Configures <project> (provizio or fast_dds) in <binary_dir> with the options given, failing the test
# on a failed configure, and leaves its output in _output
function(_configure project binary_dir)
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/openssl_package/${project}"
            -B "${binary_dir}" -G "${GENERATOR}" "-DREPOSITORY=${SOURCE_DIR}" ${ARGN}
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "Configuring the ${project} stand-in in ${binary_dir} failed:\n${_output}")
    endif()
    set(_output "${_output}" PARENT_SCOPE)
endfunction()

# Runs provizio_dds's side with <options> in a fresh directory of <case>, then Fast-DDS's in
# <case>/fast_dds -- made afresh unless <reuse> -- as provizio_dds configures it, and checks that its
# dependencies were <expected>
function(_check case reuse expected)
    file(REMOVE_RECURSE "${WORK_DIR}/${case}/provizio")
    _configure(provizio "${WORK_DIR}/${case}/provizio" "-DOUT_DIR=${WORK_DIR}/${case}/module"
        -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON ${ARGN})
    if(NOT reuse)
        file(REMOVE_RECURSE "${WORK_DIR}/${case}/fast_dds")
    endif()
    _configure(fast_dds "${WORK_DIR}/${case}/fast_dds" "-DCMAKE_MODULE_PATH=${WORK_DIR}/${case}/module"
        -DCMAKE_REQUIRE_FIND_PACKAGE_OpenSSL=ON)
    if(NOT _output MATCHES "openssl_package_fast_dds: OpenSSL 3\\.9\\.9 with \\[([^]]*)\\]")
        message(FATAL_ERROR "Case ${case}: the Fast-DDS stand-in did not report its OpenSSL:\n${_output}")
    endif()
    if(NOT CMAKE_MATCH_1 STREQUAL expected)
        message(FATAL_ERROR "Case ${case}: the package's dependencies were [${CMAKE_MATCH_1}] in the Fast-DDS "
            "build, where provizio_dds's were [${expected}]")
    endif()
endfunction()

# 1. A dependency found through its <Package>_DIR
_check(package_dir FALSE "ProvizioTestZ::ProvizioTestZ=A"
    "-DOpenSSL_DIR=${_packages}/openssl_z/lib/cmake/OpenSSL"
    "-DProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ")

# 2. A dependency found through its <Package>_ROOT
_check(package_root FALSE "ProvizioTestZ::ProvizioTestZ=B"
    "-DOpenSSL_DIR=${_packages}/openssl_z/lib/cmake/OpenSSL"
    "-DProvizioTestZ_ROOT=${_packages}/zB")

# 3. A dependency found by a module on the consumer's CMAKE_MODULE_PATH
_check(module_path FALSE "ProvizioTestFoo::ProvizioTestFoo=module"
    "-DOpenSSL_DIR=${_packages}/openssl_foo/lib/cmake/OpenSSL"
    "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules")

# 4. A dependency that has moved since the Fast-DDS build was configured: provizio_dds finds the new
#    one, and so must the Fast-DDS build configured again -- not the one its cache last held
_check(moved FALSE "ProvizioTestZ::ProvizioTestZ=A" "-DPREFIXES=${_packages}/openssl_z|${_packages}/zA")
_check(moved TRUE "ProvizioTestZ::ProvizioTestZ=B" "-DPREFIXES=${_packages}/openssl_z|${_packages}/zB")
file(STRINGS "${WORK_DIR}/moved/fast_dds/CMakeCache.txt" _cached REGEX "^(ProvizioTestZ|OpenSSL)_DIR:")
if(_cached)
    message(FATAL_ERROR "The OpenSSL lookup left [${_cached}] in the Fast-DDS build's cache, for its next "
        "configure to take over what provizio_dds finds then")
endif()

# 5. A package location whose value is a list, ahead of the rest: each location keeps its own value,
#    so the dependency still comes from its _DIR rather than from the other copy on the prefix path
_check(list_value FALSE "ProvizioTestZ::ProvizioTestZ=A"
    "-DOpenSSL_DIR=${_packages}/openssl_z/lib/cmake/OpenSSL"
    "-DProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ"
    "-DPREFIXES=${_packages}/zB" "-DLIST_ROOT=${WORK_DIR}/nowhere/a|${WORK_DIR}/nowhere/b")

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "fast_dds_openssl_package: the Fast-DDS build finds what provizio_dds found, in every case")
