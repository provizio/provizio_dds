# Copyright 2022 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.10.0)

include(ExternalProject)

# Versions
set(PROVIZIO_DDS_VERSION "23.3")
set(PROVIZIO_DDS_IDLS_VERSION "develop" CACHE STRING "provizio_dds_idls version")
set(FAST_DDS_VERSION "v2.7.2" CACHE STRING "Fast-DDS version")
set(FOONATHAN_MEMORY_VENDOR_VERSION "v1.3.0" CACHE STRING "provizio_dds_idls foonathan_memory_vendor version")
set(PROVIZIO_CODING_STANDARDS_VERSION "v22.07.26" CACHE STRING "Provizio Coding Standards version")

# Project
project(provizio_dds VERSION ${PROVIZIO_DDS_VERSION})

# Enable CTest
include(CTest)
enable_testing()

# Provizio Config Standards - Enables multiple quality checks
file(DOWNLOAD
    "https://raw.githubusercontent.com/provizio/coding_standards/${PROVIZIO_CODING_STANDARDS_VERSION}/cpp/cmake/StandardConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/StandardConfig.cmake" TLS_VERIFY ON)
include(${CMAKE_CURRENT_BINARY_DIR}/StandardConfig.cmake)
StandardConfig(SAFETY_CRITICAL)

# Check if Fast-DDS installation is present in the system
if(NOT fastcdr_FOUND)
    find_package(fastcdr QUIET)
endif()
if(NOT fastrtps_FOUND)
    find_package(fastrtps QUIET)
endif()
if(fastcdr_FOUND AND fastrtps_FOUND)
    message("Fast-DDS version ${fastrtps_VERSION} found at ${fastrtps_DIR} and will be used.")
    set(FAST_DDS_LIBS fastrtps fastcdr)
    set(FAST_DDS_DEPENDENCIES)
    set(FAST_DDS_INCLUDE_DIRS)
else(fastcdr_FOUND AND fastrtps_FOUND)
    # Let's build Fast-DDS
    message("Fast-DDS not found in the system and will be built. Version ${FAST_DDS_VERSION} with foonathan_memory_vendor version ${FOONATHAN_MEMORY_VENDOR_VERSION}.")

    # OpenSSL is required for Fast-DDS
    find_package(OpenSSL REQUIRED)

    # foonathan_memory_vendor (Required for Fast-DDS)
    set(FOONATHAN_MEMORY_DIR "${CMAKE_CURRENT_BINARY_DIR}/foonathan_memory")
    file(MAKE_DIRECTORY "${FOONATHAN_MEMORY_DIR}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/foonathan_memory" "-DFOONATHAN_MEMORY_VENDOR_VERSION=${FOONATHAN_MEMORY_VENDOR_VERSION}"
        WORKING_DIRECTORY "${FOONATHAN_MEMORY_DIR}"
    )
    execute_process(
        COMMAND "${CMAKE_COMMAND}" --build .
        WORKING_DIRECTORY "${FOONATHAN_MEMORY_DIR}"
    )
    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})
    find_package(foonathan_memory REQUIRED)
    link_directories("${FOONATHAN_MEMORY_INSTALL_DIR}/lib")

    # Fast-DDS (https://github.com/provizio/Fast-DDS)
    set(FAST_DDS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/fast_dds_build")
    set(FAST_DDS_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/fast_dds")
    set(FAST_DDS_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
    set(FAST_DDS_GITHUB_PROJECT "provizio/Fast-DDS")
    set(FAST_DDS_INSTALL_DIR "${FAST_DDS_BINARY_DIR}/install")
    ExternalProject_Add(libfast_dds
        GIT_REPOSITORY "https://github.com/${FAST_DDS_GITHUB_PROJECT}.git"
        GIT_TAG "${FAST_DDS_VERSION}"
        PREFIX "${FAST_DDS_PREFIX}"
        SOURCE_DIR "${FAST_DDS_SOURCE_DIR}"
        BINARY_DIR "${FAST_DDS_BINARY_DIR}"
        CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${FAST_DDS_INSTALL_DIR}" "-DBUILD_TESTING=OFF" "-DBUILD_SHARED_LIBS=OFF" "-DTHIRDPARTY=ON" "-DCMAKE_MODULE_PATH=${PROJECT_SOURCE_DIR}/cmake/modules"
    )
    link_directories("${FAST_DDS_INSTALL_DIR}/lib/")

    set(FAST_DDS_DEPENDENCIES libfast_dds)
    set(FAST_DDS_INCLUDE_DIRS "${FAST_DDS_INSTALL_DIR}/include/")
    set(FAST_DDS_LIBS
        fastrtps
        fastcdr
        foonathan_memory
        OpenSSL::SSL
        OpenSSL::Crypto
        $<$<BOOL:${WIN32}>:crypt32.lib>
    )
endif(fastcdr_FOUND AND fastrtps_FOUND)

# Resolve provizio_dds_idls (https://github.com/provizio/provizio_dds_idls)
set(PROVIZIO_DDS_IDLS_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_idls")
file(MAKE_DIRECTORY "${PROVIZIO_DDS_IDLS_DIR}")
execute_process(
    COMMAND "${CMAKE_COMMAND}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/provizio_dds_idls" "-DPROVIZIO_DDS_IDLS_VERSION=${PROVIZIO_DDS_IDLS_VERSION}"
    WORKING_DIRECTORY "${PROVIZIO_DDS_IDLS_DIR}"
)
execute_process(
    COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${PROVIZIO_DDS_IDLS_DIR}"
)
set(PROVIZIO_DDS_IDLS_SRC_DIR "${PROVIZIO_DDS_IDLS_DIR}/src/provizio_dds_idls_fastdds_cpp")
file(GLOB_RECURSE PROVIZIO_DDS_IDLS "${PROVIZIO_DDS_IDLS_SRC_DIR}/*.cxx")

# provizio_dds library
add_library(provizio_dds src/provizio_dds.cpp ${PROVIZIO_DDS_IDLS})

if(FAST_DDS_DEPENDENCIES)
add_dependencies(provizio_dds ${FAST_DDS_DEPENDENCIES})
endif(FAST_DDS_DEPENDENCIES)
target_include_directories(provizio_dds PUBLIC ${PROVIZIO_DDS_IDLS_SRC_DIR} ${FAST_DDS_INCLUDE_DIRS})
target_link_libraries(provizio_dds PUBLIC ${FAST_DDS_LIBS})
