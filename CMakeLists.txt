# Copyright 2023 Provizio Ltd.
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

cmake_minimum_required(VERSION 3.11.0)

include(ExternalProject)
include(FetchContent)

# Configuration
set(PROVIZIO_DDS_IDLS_VERSION "feature/APT-2168-Python" CACHE STRING "provizio_dds_idls version")
set(FAST_DDS_VERSION "v2.8.2" CACHE STRING "Fast-DDS version, when not already present in the system")
set(LOOK_FOR_FAST_DDS TRUE CACHE BOOL "Try to find Fast-DDS installation, and only build one when not found")
set(FOONATHAN_MEMORY_VENDOR_VERSION "v1.3.0" CACHE STRING "provizio_dds_idls foonathan_memory_vendor version")
set(FAST_DDS_PYTHON_VERSION "1.2.1" CACHE STRING "Fast-DDS-python version, without v prefix")
set(PROVIZIO_CODING_STANDARDS_VERSION "v23.04.20" CACHE STRING "Provizio Coding Standards version")
set(FORMAT_CMAKE_VERSION "1.7.3" CACHE STRING "Provizio Format.cmake version, without v prefix")
set(ENABLE_TESTS ON CACHE BOOL "Defines whether tests are built and enabled")
set(PYTHON_BINDINGS OFF CACHE BOOL "Defines whether Python bindings are to be generated")
set(PYTHON_PACKAGES_INSTALL_DIR "" CACHE STRING "Defines an install directory for Python artifacts (or use default if empty)")

# Detect the version of provizio_dds from git tag if present, otherwise use yy.mm.dd
find_package(Git REQUIRED)
execute_process(
        COMMAND "${GIT_EXECUTABLE}" tag --points-at HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE VERSION
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
if (VERSION STREQUAL "")
    # Tag not found, use the date instead (without trailing zeros)
    string(TIMESTAMP CURRENT_DATE "%y.%m.%d")
    string(REPLACE ".0" "." PROVIZIO_DDS_VERSION "${CURRENT_DATE}")
else(VERSION STREQUAL "")
    # Remove the "v" prefix and trailing zeros
    string(REPLACE "v" "" PROVIZIO_DDS_VERSION "${VERSION}")
    string(REPLACE ".0" "." PROVIZIO_DDS_VERSION "${PROVIZIO_DDS_VERSION}")
endif (VERSION STREQUAL "")
message("Version: ${PROVIZIO_DDS_VERSION}")

# Project
project(provizio_dds VERSION ${PROVIZIO_DDS_VERSION})

# No static analysis in macOS+clang due to https://bugs.llvm.org/show_bug.cgi?id=47460
if(STATIC_ANALYSIS AND APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(WARNING "macOS clang-tidy is broken when compiling with clang. Disabling STATIC_ANALYSIS.")
    set(STATIC_ANALYSIS
        "OFF"
        CACHE STRING "Static analysis" FORCE)
endif(STATIC_ANALYSIS AND APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")

# Provizio Config Standards - Enables multiple quality checks
file(DOWNLOAD
    "https://raw.githubusercontent.com/provizio/coding_standards/${PROVIZIO_CODING_STANDARDS_VERSION}/cpp/cmake/StandardConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/StandardConfig.cmake" TLS_VERIFY ON)
include(${CMAKE_CURRENT_BINARY_DIR}/StandardConfig.cmake)
StandardConfig(SAFETY_CRITICAL)

# Disable runtime analysis tools
if(CMAKE_CXX_FLAGS)
    message(WARNING "Runtime analysis tools will be disabled due to known issues in Fast-DDS")
    set(ENABLE_ASAN FALSE)
    set(ENABLE_TSAN FALSE)
    set(ENABLE_MSAN FALSE)
    set(CMAKE_CXX_FLAGS "")
    set(CMAKE_C_FLAGS "")
endif(CMAKE_CXX_FLAGS)

# RPATH to look for libraries in expected locations
if(NOT WIN32)
    # RPATH to look for shared libraries in expected locations
    if(APPLE)
        set(RPATH_ORIGIN "@loader_path")
        set(RPATH_DIV ";")
    elseif(UNIX)
        set(RPATH_ORIGIN "$ORIGIN")
        set(RPATH_DIV ":")
    endif(APPLE)

    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}${RPATH_DIV}${RPATH_ORIGIN}${RPATH_DIV}${RPATH_ORIGIN}/../lib")

    if(CMAKE_INSTALL_PREFIX)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}${RPATH_DIV}${CMAKE_INSTALL_PREFIX}/lib")
    endif(CMAKE_INSTALL_PREFIX)
endif(NOT WIN32)

# Help it find OpenSSL in macOS
if(APPLE AND NOT OPENSSL_ROOT_DIR)
    find_package(openssl QUIET)
    if(NOT openssl_FOUND)
        set(OPENSSL_ROOT_DIR "/usr/local/opt/openssl") # As in the brew package
        find_package(openssl REQUIRED)
    endif(NOT openssl_FOUND)
endif(APPLE AND NOT OPENSSL_ROOT_DIR)

# clang-tidy requires at least one enabled check to succeed, hence using an exotic check type to disable actual checks in auto-generated code
set(CLANG_TIDY_DISABLE "Checks: 'cppcoreguidelines-avoid-reference-coroutine-parameters'\n")

# Check if Fast-DDS installation is present in the system
if(LOOK_FOR_FAST_DDS)
    if(NOT fastcdr_FOUND)
        find_package(fastcdr QUIET)
    endif()
    if(NOT fastrtps_FOUND)
        find_package(fastrtps QUIET)
    endif()
endif(LOOK_FOR_FAST_DDS)
if(LOOK_FOR_FAST_DDS AND fastcdr_FOUND AND fastrtps_FOUND)
    message("Fast-DDS version ${fastrtps_VERSION} found at ${fastrtps_DIR} and will be used.")
    set(FAST_DDS_DEPENDENCIES)
    set(FAST_DDS_INCLUDE_DIRS)
else(LOOK_FOR_FAST_DDS AND fastcdr_FOUND AND fastrtps_FOUND)
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
        CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${FAST_DDS_INSTALL_DIR}" "-DBUILD_TESTING=OFF" "-DTHIRDPARTY=ON" "-DCMAKE_MODULE_PATH=${PROJECT_SOURCE_DIR}/cmake/modules" "-DCMAKE_INSTALL_RPATH=${CMAKE_INSTALL_RPATH}${RPATH_DIV}${FAST_DDS_INSTALL_DIR}/lib"
    )
    link_directories("${FAST_DDS_INSTALL_DIR}/lib/")

    set(FAST_DDS_DEPENDENCIES libfast_dds)
    set(FAST_DDS_INCLUDE_DIRS "${FAST_DDS_INSTALL_DIR}/include/")
endif(LOOK_FOR_FAST_DDS AND fastcdr_FOUND AND fastrtps_FOUND)

# Resolve provizio_dds_idls (https://github.com/provizio/provizio_dds_idls)
FetchContent_Declare(provizio_dds_idls
    GIT_REPOSITORY "https://github.com/provizio/provizio_dds_idls"
    GIT_TAG "${PROVIZIO_DDS_IDLS_VERSION}"
)
FetchContent_Populate(provizio_dds_idls)

set(PROVIZIO_DDS_IDLS_SRC_DIR "${provizio_dds_idls_SOURCE_DIR}/provizio_dds_idls_fastdds")
file(GLOB_RECURSE PROVIZIO_DDS_IDLS "${PROVIZIO_DDS_IDLS_SRC_DIR}/*.cxx")
file(WRITE "${PROVIZIO_DDS_IDLS_SRC_DIR}/.clang-tidy" "${CLANG_TIDY_DISABLE}") # No static analysis in autogenerated code

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# provizio_dds_types library
add_library(provizio_dds_types SHARED ${PROVIZIO_DDS_IDLS})
if(WIN32)
    target_compile_definitions(provizio_dds_types PRIVATE EPROSIMA_USER_DLL_EXPORT)
endif(WIN32)
if(FAST_DDS_DEPENDENCIES)
    add_dependencies(provizio_dds_types ${FAST_DDS_DEPENDENCIES})
endif(FAST_DDS_DEPENDENCIES)
target_include_directories(provizio_dds_types SYSTEM PUBLIC include ${PROVIZIO_DDS_IDLS_SRC_DIR} ${FAST_DDS_INCLUDE_DIRS})
target_link_libraries(provizio_dds_types PUBLIC fastrtps fastcdr)
set_target_properties(provizio_dds_types PROPERTIES DEFINE_SYMBOL "PROVIZIO_DDS_TYPES_EXPORTS")

# provizio_dds library
set(PROVIZIO_DDS_SOURCES
    src/domain_participant.cpp
)
add_library(provizio_dds SHARED ${PROVIZIO_DDS_SOURCES})
target_link_libraries(provizio_dds PUBLIC provizio_dds_types fastrtps fastcdr)
set_target_properties(provizio_dds PROPERTIES DEFINE_SYMBOL "PROVIZIO_DDS_EXPORTS")

# Installation config
install(
    TARGETS provizio_dds provizio_dds_types
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(DIRECTORY include/provizio DESTINATION include)
install(DIRECTORY "${PROVIZIO_DDS_IDLS_SRC_DIR}/" DESTINATION include FILES_MATCHING PATTERN "*.h")
if(FAST_DDS_INSTALL_DIR)
    install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/include/" DESTINATION include)
    install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/lib/" DESTINATION lib)
endif(FAST_DDS_INSTALL_DIR)

# Python
if(PYTHON_BINDINGS)
    message(STATUS "Configuring Python bindings...")

    # SWIG: use standard target name.
    if(POLICY CMP0078)
        cmake_policy(SET CMP0078 NEW)
    endif()
    # SWIG: use SWIG_MODULE_NAME property.
    if(POLICY CMP0086)
        cmake_policy(SET CMP0086 NEW)
    endif()

    # SWIG
    find_package(SWIG REQUIRED COMPONENTS python)
    include(${SWIG_USE_FILE})
    set(CMAKE_SWIG_FLAGS "")
    if("${SWIG_VERSION}" VERSION_LESS "4.0")
        message(FATAL_ERROR "SWIG 4.0 or newer is required, but ${SWIG_VERSION} found!")
    endif("${SWIG_VERSION}" VERSION_LESS "4.0")

    # Python3
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    set(PYTHON_INCLUDE_PATH ${Python3_INCLUDE_DIRS})
    set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
    set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
    include_directories(SYSTEM ${PYTHON_INCLUDE_PATH})

    include_directories(SYSTEM ${FAST_DDS_INCLUDE_DIRS})

    # Resolve Fast-DDS-python
    FetchContent_Declare(fast_dds_python
        GIT_REPOSITORY "https://github.com/provizio/Fast-DDS-python"
        GIT_TAG "v${FAST_DDS_PYTHON_VERSION}"
    )
    FetchContent_Populate(fast_dds_python)
    set(FASTDDS_PYTHON_CMAKE_DIR "${fast_dds_python_SOURCE_DIR}/fastdds_python")
    set(FASTDDS_PYTHON_CMAKE_FILE "${FASTDDS_PYTHON_CMAKE_DIR}/CMakeLists.txt")
    file(WRITE
        "${FASTDDS_PYTHON_CMAKE_FILE}"

        "project(fastdds_python VERSION ${FAST_DDS_PYTHON_VERSION})
        
        # Disable treating warnings as errors in Fast-DDS-python
        add_compile_options(-Wno-error -Wno-unused-parameter)
        
        add_subdirectory(src/swig)
        "
    )
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/.clang-tidy" "${CLANG_TIDY_DISABLE}") # No static analysis in autogenerated code
    add_subdirectory("${FASTDDS_PYTHON_CMAKE_DIR}" "${fast_dds_python_BINARY_DIR}")

    file(GLOB_RECURSE PROVIZIO_DDS_IDL_I_S "${PROVIZIO_DDS_IDLS_SRC_DIR}/*.i")
    list(FILTER PROVIZIO_DDS_IDL_I_S EXCLUDE REGEX ".*PubSub.*")

    # Remove %module as we'll package all type wrappers into single module instead
    set(MAIN_I "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_python_types.i")
    file(WRITE "${MAIN_I}" "%module provizio_dds_python_types\n\n")
    foreach(IDL_I ${PROVIZIO_DDS_IDL_I_S})
        file(READ "${IDL_I}" FILE_CONTENTS)
        string(REPLACE "%module" "//" FILE_CONTENTS "${FILE_CONTENTS}")
        file(WRITE "${IDL_I}" "${FILE_CONTENTS}")
        file(APPEND "${MAIN_I}" "%include ${IDL_I}\n")
    endforeach(IDL_I ${PROVIZIO_DDS_IDL_I_S})

    set_source_files_properties(${MAIN_I} PROPERTIES
        CPLUSPLUS ON
        USE_TARGET_INCLUDE_DIRECTORIES TRUE
    )
    swig_add_library(provizio_dds_python_types
        TYPE SHARED
        LANGUAGE python
        SOURCES ${MAIN_I}
    )

    # Add all IDL directories to include directories, as generated .cxx files include their headers just as #include "filename.h"
    file(GLOB_RECURSE IDLS_SRC_DIRS LIST_DIRECTORIES true "${PROVIZIO_DDS_IDLS_SRC_DIR}/*")
    foreach(DIR ${IDLS_SRC_DIRS})
        if(IS_DIRECTORY "${DIR}")
            if(NOT APPLE AND NOT WIN32)
                target_include_directories(provizio_dds_python_types SYSTEM PRIVATE "${DIR}")
            else(NOT APPLE AND NOT WIN32)
                # macOS and Windows use case-insensitive file system, so ROS2 Time.h and String.h are confused with system time.h and string.h
                # So instead of including their directories, we have to include the paths in the generated code
                file(GLOB H_FILES RELATIVE "${DIR}" "${DIR}/*.h")
                file(GLOB I_FILES RELATIVE "${DIR}" "${DIR}/*.i")
                file(GLOB CXX_FILES RELATIVE "${DIR}" "${DIR}/*.cxx")
                foreach(GENERATED_FILE ${CXX_FILES} ${H_FILES} ${I_FILES})
                    file(READ "${DIR}/${GENERATED_FILE}" FILE_CONTENT)
                    foreach(H_FILE ${H_FILES})
                        string(REPLACE "#include \"${H_FILE}\"" "#include \"${DIR}/${H_FILE}\"" FILE_CONTENT "${FILE_CONTENT}")
                    endforeach(H_FILE ${H_FILES})
                    file(WRITE "${DIR}/${GENERATED_FILE}" "${FILE_CONTENT}")
                endforeach(GENERATED_FILE ${CXX_FILES} ${H_FILES} ${I_FILES})
            endif(NOT APPLE AND NOT WIN32)
        endif(IS_DIRECTORY "${DIR}")
    endforeach(DIR ${IDLS_SRC_DIRS})

    if(UNIX AND NOT APPLE AND CMAKE_SIZEOF_VOID_P EQUAL 8)
        set_property(TARGET provizio_dds_python_types PROPERTY SWIG_COMPILE_DEFINITIONS SWIGWORDSIZE64)
    endif(UNIX AND NOT APPLE AND CMAKE_SIZEOF_VOID_P EQUAL 8)

    # We have to disable some of the warnings in the generated code
    target_compile_options(provizio_dds_python_types PRIVATE -Wno-delete-non-virtual-dtor -Wno-unused-parameter -Wno-missing-field-initializers -Wno-deprecated-declarations -Wno-error)

    swig_link_libraries(provizio_dds_python_types
        Python3::Module
        provizio_dds_types
        fastrtps
        fastcdr
    )

    # Make sure Fast-DDS is built prior to SWIG code generation
    if(FAST_DDS_DEPENDENCIES)
        add_dependencies(fastdds_python_swig_compilation ${FAST_DDS_DEPENDENCIES})
        add_dependencies(provizio_dds_python_types_swig_compilation ${FAST_DDS_DEPENDENCIES})
    endif(FAST_DDS_DEPENDENCIES)

    set(FASTDDS_PYTHON_LIBS
        "${fast_dds_python_BINARY_DIR}/src/swig/fastdds.py"
        "${fast_dds_python_BINARY_DIR}/src/swig/_fastdds_python${CMAKE_SHARED_LIBRARY_SUFFIX}"
    )
    set(PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS
        "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_python_types.py"
        "${CMAKE_CURRENT_BINARY_DIR}/_provizio_dds_python_types${CMAKE_SHARED_LIBRARY_SUFFIX}"
    )
    set(PROVIZIO_DDS_PYTHON_LIBS
        "${CMAKE_CURRENT_SOURCE_DIR}/python/provizio_dds.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/python/point_cloud2.py"
    )
    set(PROVIZIO_DDS_ALL_PYTHON_LIBS
        ${FASTDDS_PYTHON_LIBS}
        ${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS}
        ${PROVIZIO_DDS_PYTHON_LIBS}
    )
    if(FAST_DDS_INSTALL_DIR)
        file(GLOB FASTDDS_LIBS "${FAST_DDS_INSTALL_DIR}/lib/*${CMAKE_SHARED_LIBRARY_SUFFIX}*")
        set(PROVIZIO_DDS_ALL_PYTHON_LIBS ${PROVIZIO_DDS_ALL_PYTHON_LIBS} ${FASTDDS_LIBS})
    endif(FAST_DDS_INSTALL_DIR)

    # Find the installation path
    if(PYTHON_PACKAGES_INSTALL_DIR)
        # Installation as Python packages with their __init__.py
        set(FASTDDS_PYTHON_LIBS ${FASTDDS_PYTHON_LIBS} "${CMAKE_CURRENT_SOURCE_DIR}/python/inits/fastdds/__init__.py")
        set(PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS ${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS} "${CMAKE_CURRENT_SOURCE_DIR}/python/inits/provizio_dds_python_types/__init__.py")
        set(PROVIZIO_DDS_PYTHON_LIBS ${PROVIZIO_DDS_PYTHON_LIBS} "${CMAKE_CURRENT_SOURCE_DIR}/python/inits/provizio_dds/__init__.py")

        set(FASTDDS_PYTHON_LIBS_DIR "${PYTHON_PACKAGES_INSTALL_DIR}/fastdds")
        set(PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS_DIR "${PYTHON_PACKAGES_INSTALL_DIR}/provizio_dds_python_types")
        set(PROVIZIO_DDS_PYTHON_LIBS_DIR "${PYTHON_PACKAGES_INSTALL_DIR}/provizio_dds")

        # Output the version to the version file, to be used by setup.py
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/version.txt" "${PROVIZIO_DDS_VERSION}")
    else(PYTHON_PACKAGES_INSTALL_DIR)
        # Installation as .py files
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(plat_specific=True, prefix='${CMAKE_INSTALL_PREFIX}' ) )"
            OUTPUT_VARIABLE _ABS_PYTHON_MODULE_PATH
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        get_filename_component(_ABS_PYTHON_MODULE_PATH ${_ABS_PYTHON_MODULE_PATH} ABSOLUTE)
        file(RELATIVE_PATH _REL_PYTHON_MODULE_PATH ${CMAKE_INSTALL_PREFIX} ${_ABS_PYTHON_MODULE_PATH})
        set(FASTDDS_PYTHON_LIBS_DIR ${_REL_PYTHON_MODULE_PATH})
        set(PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS_DIR ${_REL_PYTHON_MODULE_PATH})
        set(PROVIZIO_DDS_PYTHON_LIBS_DIR ${_REL_PYTHON_MODULE_PATH})
    endif(PYTHON_PACKAGES_INSTALL_DIR)

    # Install Python artifacts
    install(FILES
        ${FASTDDS_PYTHON_LIBS}
        DESTINATION "${FASTDDS_PYTHON_LIBS_DIR}"
    )
    install(FILES
        ${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS}
        DESTINATION "${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS_DIR}"
    )
    install(TARGETS provizio_dds_types
        LIBRARY DESTINATION "${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS_DIR}"
    )
    install(FILES
        ${PROVIZIO_DDS_PYTHON_LIBS}
        DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}"
    )
    if(FAST_DDS_INSTALL_DIR)
        # Local build of Fast-DDS, have to be included with the package then
        install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/lib/" DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}")
    endif(FAST_DDS_INSTALL_DIR)

    if("${CMAKE_SHARED_LIBRARY_SUFFIX}" STREQUAL ".dylib")
        # Python expects .so and .so only in import
        install(FILES
            "${fast_dds_python_BINARY_DIR}/src/swig/_fastdds_python${CMAKE_SHARED_LIBRARY_SUFFIX}"
            DESTINATION "${FASTDDS_PYTHON_LIBS_DIR}"
            RENAME _fastdds_python.so)
        install(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/_provizio_dds_python_types${CMAKE_SHARED_LIBRARY_SUFFIX}"
            DESTINATION "${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS_DIR}"
            RENAME _provizio_dds_python_types.so)
    endif("${CMAKE_SHARED_LIBRARY_SUFFIX}" STREQUAL ".dylib")

endif(PYTHON_BINDINGS)

if(ENABLE_TESTS)
add_subdirectory(test)
endif(ENABLE_TESTS)
