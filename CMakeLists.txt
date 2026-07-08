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

cmake_minimum_required(VERSION 3.15.0)

include(ExternalProject)
include(FetchContent)
include("${CMAKE_CURRENT_LIST_DIR}/cmake/fast_dds/imported_targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/python_bindings/swig_main_i.cmake")

# Configuration
set(PROVIZIO_DDS_IDLS_VERSION "2.1.0" CACHE STRING "provizio_dds_idls version")
set(FAST_DDS_VERSION "v3.6.1" CACHE STRING "Fast-DDS git ref (upstream tag, mirrored by the provizio fork)")
set(FAST_DDS_NUMERIC_VERSION "3.6.1" CACHE STRING "Fast-DDS numeric version (used for library naming on Windows); must match what FAST_DDS_VERSION resolves to")
set(FAST_CDR_VERSION "2.3" CACHE STRING "Fast-CDR major.minor version (for Windows versioned library naming). Must match the Fast-CDR bundled with FAST_DDS_VERSION")
set(LOOK_FOR_FAST_DDS FALSE CACHE BOOL "Try to find Fast-DDS installation, and only build one when not found")
set(FOONATHAN_MEMORY_VENDOR_VERSION "v1.4.1" CACHE STRING "foonathan_memory_vendor git ref (upstream tag, mirrored by the provizio fork)")
set(FAST_DDS_PYTHON_VERSION "v2.6.1" CACHE STRING "Fast-DDS-python git ref (upstream tag, mirrored by the provizio fork)")
set(FAST_DDS_PYTHON_NUMERIC_VERSION "2.6.1" CACHE STRING "Fast-DDS-python numeric version (used for CMake project() VERSION); must match what FAST_DDS_PYTHON_VERSION resolves to")
set(PROVIZIO_CODING_STANDARDS_VERSION "1.10.5" CACHE STRING "Provizio Coding Standards version")
set(DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS OFF CACHE BOOL "Disable all provizio/coding_standards checks")
set(FORMAT_CMAKE_VERSION "1.7.3" CACHE STRING "Provizio Format.cmake version, without v prefix")
set(ENABLE_TESTS OFF CACHE BOOL "Defines whether tests are built and enabled")
set(PYTHON_BINDINGS OFF CACHE BOOL "Defines whether Python bindings are to be generated")
set(PYTHON_PACKAGES_INSTALL_DIR "" CACHE STRING "Defines an install directory for Python artifacts (or use default if empty)")
if(PYTHON_PACKAGES_INSTALL_DIR)
    # Normalize to forward slashes. An absolute destination is baked verbatim into
    # the generated cmake_install.cmake, so a native Windows path with backslashes
    # (e.g. "D:\a\...") is re-parsed there as a quoted string and rejected with
    # "Invalid character escape" on sequences like \a, aborting `cmake --install`.
    # Forward slashes are valid path separators on every platform CMake supports.
    string(REPLACE "\\" "/" PYTHON_PACKAGES_INSTALL_DIR "${PYTHON_PACKAGES_INSTALL_DIR}")
endif()
set(IGNORE_BIN_CACHE OFF CACHE BOOL "Enable to force build from sources instead of using prebuilt cache")
set(DONT_INSTALL_STDCPP_LIBS ON CACHE BOOL "When installing from prebuilt binaries, skip standard C++ libraries")
set(INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS OFF CACHE BOOL "Install and link to fully-qualified Fast-DDS libraries to avoid runtime incompatibilities across different .so.major.minor.patch versions, Linux only")
set(DISABLE_EIGEN OFF CACHE BOOL "Force plain-CPU linear algebra in point clouds accumulation for this build even when Eigen3 is installed (consumers make their own choice at compile time — the accumulation maths path is header-only)")

# Force Release CRT when building Python bindings in Debug mode on Windows.
# Python interpreters (actions/setup-python, system installs) are Release-built
# and use /MD. Debug builds use /MDd, creating a CRT mismatch that causes access
# violations when loading .pyd modules. Force /MD for all targets so the entire
# DLL chain is CRT-compatible with the Release Python interpreter.
set(PROVIZIO_DDS_FORCE_RELEASE_CRT OFF)
# Note: CMAKE_BUILD_TYPE check works because the project uses Ninja (single-config).
# Multi-config generators (Visual Studio) would need generator expressions instead.
if(WIN32 AND PYTHON_BINDINGS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(POLICY CMP0091)
        cmake_policy(SET CMP0091 NEW)
    endif()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
    set(PROVIZIO_DDS_FORCE_RELEASE_CRT ON)
    message(STATUS "Debug+Python on Windows: using Release CRT (/MD) for Python compatibility")
endif()

# Detect the version of provizio_dds from git tag if present, otherwise use yy.mm.dd
find_package(Git REQUIRED)
execute_process(
    COMMAND "${GIT_EXECUTABLE}" tag --points-at HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE VERSION
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

if(VERSION STREQUAL "")
    # Tag not found, derive a deterministic version (0.yyyymmdd.hhmmss) from the HEAD commit timestamp
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" show -s --format=%cd --date=format:%Y%m%d
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE COMMIT_DATE
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" show -s --format=%cd --date=format:%H%M%S
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE COMMIT_TIME
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(PROVIZIO_DDS_VERSION "0.${COMMIT_DATE}.${COMMIT_TIME}")
else(VERSION STREQUAL "")
    string(REGEX MATCH "^[0-9]+\\.[0-9]+\\.[0-9]+" VERSION "${VERSION}")
    set(PROVIZIO_DDS_VERSION "${VERSION}")
endif(VERSION STREQUAL "")

message(STATUS "Version: ${PROVIZIO_DDS_VERSION}")

# Project
project(provizio_dds VERSION ${PROVIZIO_DDS_VERSION})

# Eigen3 (optional): accelerates the point-clouds-accumulation linear algebra (include/provizio/dds/accumulation.h).
# The maths path is HEADER-ONLY: provizio/dds/detail/accumulation_math.h auto-detects Eigen at COMPILE time via
# __has_include, so this find_package only serves provizio_dds's own test builds (include path + the explicit
# PROVIZIO_DDS_DISABLE_EIGEN macro) and the configure-time heads-up below. Consumers decide for themselves by
# making Eigen visible (or not) to their compiler; libprovizio_dds and the bin cache are identical either way.
if(NOT DISABLE_EIGEN)
    find_package(Eigen3 QUIET)
endif(NOT DISABLE_EIGEN)
if(NOT DISABLE_EIGEN AND TARGET Eigen3::Eigen)
    message(STATUS "Eigen3 found: using Eigen for linear algebra operations in point clouds accumulation")
elseif(DISABLE_EIGEN)
    message(STATUS "DISABLE_EIGEN=ON: plain-CPU processing will be used for linear algebra operations in point clouds accumulation")
else()
    message(WARNING "Eigen3 not found: less efficient plain-CPU processing will be used for linear algebra operations in point clouds accumulation")
endif()

# Trying locating bin cache, unless disabled
set(HAS_BIN_CACHE FALSE)

if(NOT IGNORE_BIN_CACHE AND(NOT DEFINED ENV{IGNORE_BIN_CACHE} OR "$ENV{IGNORE_BIN_CACHE}" STREQUAL "FALSE"))
    if(UNIX AND NOT APPLE) # Linux
        # On Linux, 3.8-3.13 share ABI (tag "3"), 3.14+ broke ABI (tag "3_14")
        if(PYTHON_BINDINGS AND PYTHON_PACKAGES_INSTALL_DIR)
            find_package(Python3 COMPONENTS Interpreter QUIET)
            if(Python3_FOUND)
                execute_process(COMMAND ${Python3_EXECUTABLE} -c "import sys; print('3_14' if sys.version_info >= (3, 14) else '3')"
                    OUTPUT_VARIABLE PYTHON_VERSION_TAG OUTPUT_STRIP_TRAILING_WHITESPACE)
            endif(Python3_FOUND)
        endif(PYTHON_BINDINGS AND PYTHON_PACKAGES_INSTALL_DIR)

        execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/bin_cache_config_name.sh" "${CMAKE_BUILD_TYPE}" "${PROVIZIO_DDS_IDLS_VERSION}" OUTPUT_VARIABLE BIN_CACHE_CONFIG_NAME)
        string(STRIP "${BIN_CACHE_CONFIG_NAME}" BIN_CACHE_CONFIG_NAME)
        set(BIN_CACHE_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cache/${BIN_CACHE_CONFIG_NAME}.zip")
        set(BIN_CACHE_PARENT_PATH "${CMAKE_CURRENT_BINARY_DIR}/cache")
        set(BIN_CACHE_PATH "${BIN_CACHE_PARENT_PATH}/${BIN_CACHE_CONFIG_NAME}")
        set(CACHED_PROVIZIO_DDS_SO "${BIN_CACHE_PATH}/lib/libprovizio_dds.so")
        set(CACHED_PROVIZIO_DDS_TYPES_SO "${BIN_CACHE_PATH}/lib/libprovizio_dds_types.so")

        if(PYTHON_VERSION_TAG)
            set(PYTHON_BIN_CACHE_CONFIG_NAME "${BIN_CACHE_CONFIG_NAME}.python${PYTHON_VERSION_TAG}")
            set(PYTHON_BIN_CACHE_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cache/${PYTHON_BIN_CACHE_CONFIG_NAME}.zip")
            set(PYTHON_BIN_CACHE_PATH "${BIN_CACHE_PARENT_PATH}/${PYTHON_BIN_CACHE_CONFIG_NAME}")
            set(PYTHON_CACHE_PATH "${PYTHON_BIN_CACHE_PATH}/python")
            set(CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO "${PYTHON_CACHE_PATH}/provizio_dds_python_types/_provizio_dds_python_types.so")
        else()
            set(PYTHON_CACHE_PATH "${BIN_CACHE_PATH}/python")
            set(CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO "${PYTHON_CACHE_PATH}/provizio_dds_python_types/_provizio_dds_python_types.so")
        endif(PYTHON_VERSION_TAG)

        # Extract general cache if needed
        if(EXISTS "${BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_SO}")
            execute_process(COMMAND unzip -q "${BIN_CACHE_SOURCE_PATH}" -d "${BIN_CACHE_PARENT_PATH}")
        endif(EXISTS "${BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_SO}")

        # Extract Python-versioned cache if needed
        if(PYTHON_VERSION_TAG AND EXISTS "${PYTHON_BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO}")
            execute_process(COMMAND unzip -q "${PYTHON_BIN_CACHE_SOURCE_PATH}" -d "${BIN_CACHE_PARENT_PATH}")
        endif(PYTHON_VERSION_TAG AND EXISTS "${PYTHON_BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO}")

        if(EXISTS "${CACHED_PROVIZIO_DDS_SO}" AND(NOT PYTHON_BINDINGS OR EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO}"))
            execute_process(COMMAND uname -r OUTPUT_VARIABLE HOST_KERNEL_VERSION)
            file(READ "${BIN_CACHE_PATH}/kernel_version" CACHE_KERNEL_VERSION)
            if("${HOST_KERNEL_VERSION}" VERSION_GREATER_EQUAL "${CACHE_KERNEL_VERSION}")
                set(HAS_BIN_CACHE TRUE)

                if(NOT INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS)
                    # Softlinks need to be added, as cache doesn't include them
                    execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/fully_qualified_fastdds_libs.sh" "${BIN_CACHE_PATH}/lib" "revert")
                endif(NOT INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS)

            else("${HOST_KERNEL_VERSION}" VERSION_GREATER_EQUAL "${CACHE_KERNEL_VERSION}")
                message(STATUS "Prebuilt binaries located, but won't be used as they were built in a newer Linux kernel")
            endif("${HOST_KERNEL_VERSION}" VERSION_GREATER_EQUAL "${CACHE_KERNEL_VERSION}")
        endif(EXISTS "${CACHED_PROVIZIO_DDS_SO}" AND(NOT PYTHON_BINDINGS OR EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO}"))
    elseif(WIN32)
        # Find PowerShell
        find_program(POWERSHELL_EXECUTABLE NAMES pwsh powershell)
        if(POWERSHELL_EXECUTABLE)
            execute_process(
                COMMAND "${POWERSHELL_EXECUTABLE}" -ExecutionPolicy Bypass -File "${CMAKE_CURRENT_SOURCE_DIR}/bin_cache_config_name.ps1" "${CMAKE_BUILD_TYPE}" "${PROVIZIO_DDS_IDLS_VERSION}"
                OUTPUT_VARIABLE BIN_CACHE_CONFIG_NAME
                RESULT_VARIABLE BIN_CACHE_CONFIG_RESULT
            )
            string(STRIP "${BIN_CACHE_CONFIG_NAME}" BIN_CACHE_CONFIG_NAME)

            if(BIN_CACHE_CONFIG_RESULT EQUAL 0 AND NOT BIN_CACHE_CONFIG_NAME STREQUAL "")
                set(BIN_CACHE_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cache/${BIN_CACHE_CONFIG_NAME}.zip")
                set(BIN_CACHE_PARENT_PATH "${CMAKE_CURRENT_BINARY_DIR}/cache")
                set(BIN_CACHE_PATH "${BIN_CACHE_PARENT_PATH}/${BIN_CACHE_CONFIG_NAME}")
                set(CACHED_PROVIZIO_DDS_DLL "${BIN_CACHE_PATH}/bin/provizio_dds.dll")
                set(CACHED_PROVIZIO_DDS_LIB "${BIN_CACHE_PATH}/lib/provizio_dds.lib")
                set(CACHED_PROVIZIO_DDS_TYPES_DLL "${BIN_CACHE_PATH}/bin/provizio_dds_types.dll")
                set(CACHED_PROVIZIO_DDS_TYPES_LIB "${BIN_CACHE_PATH}/lib/provizio_dds_types.lib")

                # On Windows, .pyd files link against specific pythonXY.dll, so each version needs its own cache
                if(PYTHON_BINDINGS AND PYTHON_PACKAGES_INSTALL_DIR)
                    find_package(Python3 COMPONENTS Interpreter QUIET)
                    if(Python3_FOUND)
                        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')"
                            OUTPUT_VARIABLE WIN_PYTHON_VERSION_TAG OUTPUT_STRIP_TRAILING_WHITESPACE)
                    endif(Python3_FOUND)
                endif(PYTHON_BINDINGS AND PYTHON_PACKAGES_INSTALL_DIR)

                if(WIN_PYTHON_VERSION_TAG)
                    set(PYTHON_BIN_CACHE_CONFIG_NAME "${BIN_CACHE_CONFIG_NAME}.python${WIN_PYTHON_VERSION_TAG}")
                    set(PYTHON_BIN_CACHE_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cache/${PYTHON_BIN_CACHE_CONFIG_NAME}.zip")
                    set(PYTHON_BIN_CACHE_PATH "${BIN_CACHE_PARENT_PATH}/${PYTHON_BIN_CACHE_CONFIG_NAME}")
                    set(PYTHON_CACHE_PATH "${PYTHON_BIN_CACHE_PATH}/python")
                else()
                    set(PYTHON_CACHE_PATH "${BIN_CACHE_PATH}/python")
                endif(WIN_PYTHON_VERSION_TAG)
                set(CACHED_PROVIZIO_DDS_PYTHON_TYPES_PYD "${PYTHON_CACHE_PATH}/provizio_dds_python_types/_provizio_dds_python_types.pyd")

                # Extract general cache if needed
                if(EXISTS "${BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_DLL}")
                    execute_process(
                        COMMAND "${POWERSHELL_EXECUTABLE}" -Command "Expand-Archive -Path '${BIN_CACHE_SOURCE_PATH}' -DestinationPath '${BIN_CACHE_PARENT_PATH}' -Force"
                    )
                endif(EXISTS "${BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_DLL}")

                # Extract Python-versioned cache if needed
                if(WIN_PYTHON_VERSION_TAG AND EXISTS "${PYTHON_BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_PYD}")
                    execute_process(
                        COMMAND "${POWERSHELL_EXECUTABLE}" -Command "Expand-Archive -Path '${PYTHON_BIN_CACHE_SOURCE_PATH}' -DestinationPath '${BIN_CACHE_PARENT_PATH}' -Force"
                    )
                endif(WIN_PYTHON_VERSION_TAG AND EXISTS "${PYTHON_BIN_CACHE_SOURCE_PATH}" AND NOT EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_PYD}")

                if(EXISTS "${CACHED_PROVIZIO_DDS_DLL}" AND EXISTS "${CACHED_PROVIZIO_DDS_LIB}" AND(NOT PYTHON_BINDINGS OR EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_PYD}"))
                    set(HAS_BIN_CACHE TRUE)
                endif(EXISTS "${CACHED_PROVIZIO_DDS_DLL}" AND EXISTS "${CACHED_PROVIZIO_DDS_LIB}" AND(NOT PYTHON_BINDINGS OR EXISTS "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_PYD}"))
            endif(BIN_CACHE_CONFIG_RESULT EQUAL 0 AND NOT BIN_CACHE_CONFIG_NAME STREQUAL "")
        endif(POWERSHELL_EXECUTABLE)
    endif(UNIX AND NOT APPLE)
endif(NOT IGNORE_BIN_CACHE AND(NOT DEFINED ENV{IGNORE_BIN_CACHE} OR "$ENV{IGNORE_BIN_CACHE}" STREQUAL "FALSE"))

# No static analysis in macOS+clang due to https://bugs.llvm.org/show_bug.cgi?id=47460
if(STATIC_ANALYSIS AND APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(WARNING "macOS clang-tidy is broken when compiling with clang. Disabling STATIC_ANALYSIS.")
    set(STATIC_ANALYSIS
        "OFF"
        CACHE STRING "Static analysis" FORCE)
endif(STATIC_ANALYSIS AND APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")

if(STATIC_ANALYSIS AND HAS_BIN_CACHE)
    message(STATUS "Prebuilt binaries located, but won't be used as STATIC_ANALYSIS is enabled")
    set(HAS_BIN_CACHE FALSE)
endif(STATIC_ANALYSIS AND HAS_BIN_CACHE)

if(ENABLE_TESTS AND HAS_BIN_CACHE)
    message(STATUS "Prebuilt binaries located, but won't be used as tests are enabled")
    set(HAS_BIN_CACHE FALSE)
endif(ENABLE_TESTS AND HAS_BIN_CACHE)

if(PYTHON_BINDINGS AND NOT PYTHON_PACKAGES_INSTALL_DIR AND HAS_BIN_CACHE)
    message(STATUS "Prebuilt binaries located, but won't be used as PYTHON_PACKAGES_INSTALL_DIR is not specified")
    set(HAS_BIN_CACHE FALSE)
endif(PYTHON_BINDINGS AND NOT PYTHON_PACKAGES_INSTALL_DIR AND HAS_BIN_CACHE)

# Provizio Config Standards - Enables multiple quality checks (see https://github.com/provizio/coding_standards)
if(NOT DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS)
    set(_provizio_standards_url
        "https://raw.githubusercontent.com/provizio/coding_standards/${PROVIZIO_CODING_STANDARDS_VERSION}/cpp/cmake/StandardConfig.cmake")
    set(_provizio_standards_file "${CMAKE_BINARY_DIR}/StandardConfig.cmake")
    # Retry — Docker / CI runners hit raw.githubusercontent.com transient
    # failures (TLS handshake reset, DNS hiccup, GitHub Raw 5xx) often enough
    # that a single attempt is flaky. Without an explicit STATUS check, a
    # silent download failure surfaces as "Unknown CMake command StandardConfig"
    # on the include() + StandardConfig() lines below, which is misleading.
    set(_provizio_standards_attempts 3)
    set(_provizio_standards_downloaded FALSE)
    foreach(_attempt RANGE 1 ${_provizio_standards_attempts})
        file(REMOVE "${_provizio_standards_file}")
        file(DOWNLOAD
            "${_provizio_standards_url}"
            "${_provizio_standards_file}"
            TLS_VERIFY ON
            TIMEOUT 60
            STATUS _provizio_standards_status
            LOG _provizio_standards_log)
        list(GET _provizio_standards_status 0 _provizio_standards_code)
        list(GET _provizio_standards_status 1 _provizio_standards_msg)
        if(_provizio_standards_code EQUAL 0)
            # CMake reports success even on a 0-byte body, so verify the file
            # actually has content before calling include() on it.
            file(SIZE "${_provizio_standards_file}" _provizio_standards_size)
            if(_provizio_standards_size GREATER 0)
                set(_provizio_standards_downloaded TRUE)
                break()
            endif()
            set(_provizio_standards_msg "downloaded file is empty (0 bytes)")
        endif()
        message(STATUS
            "Provizio coding_standards download attempt ${_attempt}/${_provizio_standards_attempts} "
            "failed: ${_provizio_standards_msg} (status ${_provizio_standards_code})")
    endforeach()
    if(NOT _provizio_standards_downloaded)
        message(FATAL_ERROR
            "Failed to download Provizio coding_standards after ${_provizio_standards_attempts} attempts.\n"
            "  URL:    ${_provizio_standards_url}\n"
            "  Status: ${_provizio_standards_msg} (code ${_provizio_standards_code})\n"
            "Likely causes: transient network / DNS / TLS issue on this runner, "
            "GitHub Raw outage, or an invalid PROVIZIO_CODING_STANDARDS_VERSION "
            "(\"${PROVIZIO_CODING_STANDARDS_VERSION}\").\n"
            "Last download log:\n${_provizio_standards_log}")
    endif()
    include("${_provizio_standards_file}")
    StandardConfig(SAFETY_CRITICAL)
else(NOT DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS)
    set(CMAKE_CXX_STANDARD 17)
    if(MSVC)
        # Applied globally because Fast-DDS headers trigger these in all consuming targets:
        # /wd4068: unknown pragma (Fast-DDS uses GCC pragmas)
        # /wd4996: deprecated API (Fast-DDS uses deprecated POSIX functions)
        add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:/EHsc>" /wd4068 /wd4996)
    endif(MSVC)
    if(STATIC_ANALYSIS)
        message(WARNING "STATIC_ANALYSIS setting doesn't have any effect when DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS is ON")
    endif(STATIC_ANALYSIS)
    # The runtime sanitizers are wired through coding_standards' StandardConfig (it sets the -fsanitize
    # flags). With the standards checks disabled StandardConfig never runs, so ENABLE_ASAN / ENABLE_TSAN
    # / ENABLE_MSAN silently produce an UNinstrumented build — warn rather than mislead.
    if(ENABLE_ASAN OR ENABLE_TSAN OR ENABLE_MSAN)
        message(WARNING "ENABLE_ASAN/ENABLE_TSAN/ENABLE_MSAN have no effect when DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS is ON: "
            "the sanitizer flags come from coding_standards, which is not loaded. The build will NOT be instrumented.")
    endif(ENABLE_ASAN OR ENABLE_TSAN OR ENABLE_MSAN)
endif(NOT DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS)

if(WIN32)
    # NOMINMAX: prevents <windows.h> from macro-defining `min` / `max`,
    # which clobber std::numeric_limits<...>::min() / ::max() in Fast-DDS
    # headers (e.g. ThreadSettings.hpp).
    # WIN32_LEAN_AND_MEAN: skip rarely used pieces of <windows.h>.
    # _WIN32_WINNT=0x0600 / NTDDI_VERSION=NTDDI_VISTA: required by
    # netioapi.h for NotifyUnicastIpAddressChange / CancelMibChangeNotify2
    # used by the network-auto-recovery Windows monitor.
    # Applied globally as compiler -D flags so the values are seen before
    # ANY include — and so downstream consumers transitively pulling
    # Fast-DDS via provizio_dds headers also get the same surface. Set
    # OUTSIDE the coding-standards if/else above so it activates on every
    # Windows build, including the StandardConfig(SAFETY_CRITICAL) branch
    # that downstream consumers (apt_gui) take.
    add_compile_definitions(
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        _WIN32_WINNT=0x0600
        NTDDI_VERSION=0x06000000
    )
endif(WIN32)

# Runtime sanitizers are driven by provizio/coding_standards (StandardConfig.cmake): ENABLE_ASAN
# (defaulted on in Debug), ENABLE_TSAN or ENABLE_MSAN, which append the matching -fsanitize flags to
# CMAKE_CXX_FLAGS / CMAKE_C_FLAGS. They were historically force-disabled here due to issues in older
# Fast-DDS; those are now handled at runtime via suppression files (see test/sanitizers/, applied
# through ASAN_OPTIONS / UBSAN_OPTIONS / TSAN_OPTIONS) rather than by disabling, so on GCC/Clang the
# flags are honored.
set(PROVIZIO_DDS_SANITIZER_FLAGS "")
string(REGEX MATCHALL "-fsanitize=[A-Za-z,]+" _provizio_dds_san_tokens "${CMAKE_CXX_FLAGS}")
if(MSVC)
    # coding_standards emits GCC/Clang `-fsanitize=` syntax unconditionally, which MSVC's cl.exe
    # rejects; strip it on Windows (sanitizer builds here target Linux/macOS only). Mirrors the
    # original force-disable that kept Windows Debug builds working.
    set(ENABLE_ASAN FALSE)
    set(ENABLE_TSAN FALSE)
    set(ENABLE_MSAN FALSE)
    string(REGEX REPLACE " *-fsanitize=[A-Za-z,]+" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REGEX REPLACE " *-fsanitize=[A-Za-z,]+" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
elseif(PYTHON_BINDINGS AND _provizio_dds_san_tokens)
    # The instrumented provizio_dds / Fast-DDS / Fast-CDR shared libraries are dlopen'd by an
    # UNINSTRUMENTED Python interpreter, which cannot resolve the sanitizer runtime symbols
    # (e.g. __ubsan_vptr_type_cache, __asan_*) and fails at import. Build the Python-binding
    # configuration uninstrumented; the C++ sanitizer coverage comes from the PYTHON_BINDINGS=OFF Debug
    # jobs that build the identical library from source.
    message(STATUS "PYTHON_BINDINGS is ON: disabling runtime sanitizers (an uninstrumented Python interpreter cannot load instrumented modules)")
    set(ENABLE_ASAN FALSE)
    set(ENABLE_TSAN FALSE)
    set(ENABLE_MSAN FALSE)
    string(REGEX REPLACE " *-fsanitize=[A-Za-z,]+" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REGEX REPLACE " *-fsanitize=[A-Za-z,]+" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
elseif(_provizio_dds_san_tokens)
    # GCC/Clang (Linux/macOS), no Python bindings: honor the active sanitizer and capture just its
    # -fsanitize flags so the Fast-DDS and foonathan_memory sub-builds — separate ExternalProjects that
    # do NOT inherit CMAKE_CXX_FLAGS — are instrumented too; a half-instrumented stack misses real
    # races AND reports false ones.
    string(REPLACE ";" " " PROVIZIO_DDS_SANITIZER_FLAGS "${_provizio_dds_san_tokens}")
    message(STATUS "Runtime sanitizer active (coding_standards): ${PROVIZIO_DDS_SANITIZER_FLAGS}; instrumenting foonathan_memory and Fast-DDS to match")
endif()

if(HAS_BIN_CACHE)
    message(STATUS "Prebuilt binaries are located and will be used...")

    add_library(provizio_dds SHARED IMPORTED)
    add_library(provizio_dds_types SHARED IMPORTED)

    if(WIN32)
        set_target_properties(provizio_dds PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${BIN_CACHE_PATH}/include"
            IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
            IMPORTED_IMPLIB "${CACHED_PROVIZIO_DDS_LIB}"
            IMPORTED_LOCATION "${CACHED_PROVIZIO_DDS_DLL}"
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO ${CMAKE_BUILD_TYPE}
            MAP_IMPORTED_CONFIG_MINSIZEREL ${CMAKE_BUILD_TYPE}
        )
        set_target_properties(provizio_dds_types PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${BIN_CACHE_PATH}/include"
            IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
            IMPORTED_IMPLIB "${CACHED_PROVIZIO_DDS_TYPES_LIB}"
            IMPORTED_LOCATION "${CACHED_PROVIZIO_DDS_TYPES_DLL}"
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO ${CMAKE_BUILD_TYPE}
            MAP_IMPORTED_CONFIG_MINSIZEREL ${CMAKE_BUILD_TYPE}
        )
    else(WIN32)
        set_target_properties(provizio_dds PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${BIN_CACHE_PATH}/include"
            IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
            IMPORTED_LOCATION "${CACHED_PROVIZIO_DDS_SO}"
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO ${CMAKE_BUILD_TYPE}
            MAP_IMPORTED_CONFIG_MINSIZEREL ${CMAKE_BUILD_TYPE}
        )
        set_target_properties(provizio_dds_types PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${BIN_CACHE_PATH}/include"
            IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
            IMPORTED_LOCATION "${CACHED_PROVIZIO_DDS_TYPES_SO}"
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO ${CMAKE_BUILD_TYPE}
            MAP_IMPORTED_CONFIG_MINSIZEREL ${CMAKE_BUILD_TYPE}
        )
    endif(WIN32)

    install(DIRECTORY "${BIN_CACHE_PATH}/include/" DESTINATION include)

    if(WIN32)
        install(DIRECTORY "${BIN_CACHE_PATH}/lib/" DESTINATION lib
            PATTERN "cmake" EXCLUDE)
        install(DIRECTORY "${BIN_CACHE_PATH}/bin/" DESTINATION bin)
    elseif(DONT_INSTALL_STDCPP_LIBS)
        install(DIRECTORY "${BIN_CACHE_PATH}/lib/" DESTINATION lib
            PATTERN "libstdc++.so*" EXCLUDE
            PATTERN "libgcc_s.so*" EXCLUDE
            PATTERN "cmake" EXCLUDE
        )
    else()
        install(DIRECTORY "${BIN_CACHE_PATH}/lib/" DESTINATION lib
            PATTERN "cmake" EXCLUDE)
    endif()

    if(PYTHON_BINDINGS)
        # Output the version to the version file, to be used by setup.py
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/version.txt" "${PROVIZIO_DDS_VERSION}")

        if(NOT PYTHON_PACKAGES_INSTALL_DIR)
            message(FATAL_ERROR "Using prebuilt Python binaries requires PYTHON_PACKAGES_INSTALL_DIR to be specified")
        endif(NOT PYTHON_PACKAGES_INSTALL_DIR)

        file(GLOB PYTHON_DIRS "${PYTHON_CACHE_PATH}/*")

        foreach(entry ${PYTHON_DIRS})
            if(IS_DIRECTORY ${entry})
                install(DIRECTORY "${entry}" DESTINATION "${PYTHON_PACKAGES_INSTALL_DIR}")
            else(IS_DIRECTORY ${entry})
                install(FILES "${entry}" DESTINATION "${PYTHON_PACKAGES_INSTALL_DIR}")
            endif(IS_DIRECTORY ${entry})
        endforeach(entry ${PYTHON_DIRS})
    endif(PYTHON_BINDINGS)
else(HAS_BIN_CACHE)
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

        if(NOT fastdds_FOUND)
            find_package(fastdds QUIET)
        endif()
    endif(LOOK_FOR_FAST_DDS)

    if(LOOK_FOR_FAST_DDS AND fastcdr_FOUND AND fastdds_FOUND)
        message(STATUS "Fast-DDS version ${fastdds_VERSION} found at ${fastdds_DIR} and will be used.")
        set(FAST_DDS_DEPENDENCIES)
        set(FAST_DDS_INCLUDE_DIRS)
        set(FASTDDS_LIB "fastdds")
        set(FASTCDR_LIB "fastcdr")
    else(LOOK_FOR_FAST_DDS AND fastcdr_FOUND AND fastdds_FOUND)
        # Let's build Fast-DDS
        message(STATUS "Fast-DDS will be built. Version ${FAST_DDS_VERSION} with foonathan_memory_vendor version ${FOONATHAN_MEMORY_VENDOR_VERSION}.")

        # OpenSSL is used by Fast-DDS for its security plugin
        find_package(OpenSSL REQUIRED)

        # foonathan_memory_vendor (Required for Fast-DDS)
        set(FOONATHAN_MEMORY_DIR "${CMAKE_CURRENT_BINARY_DIR}/foonathan_memory")
        file(MAKE_DIRECTORY "${FOONATHAN_MEMORY_DIR}")
        # When forcing Release CRT for Python compatibility, build foonathan_memory
        # in Release so its static .lib uses /MD. The Findfoonathan_memory.cmake
        # glob matches regardless of debug suffix.
        set(_FOONATHAN_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
        if(PROVIZIO_DDS_FORCE_RELEASE_CRT)
            set(_FOONATHAN_BUILD_TYPE "Release")
        endif()
        set(_FOONATHAN_CMAKE_ARGS -G "${CMAKE_GENERATOR}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/foonathan_memory" "-DFOONATHAN_MEMORY_VENDOR_VERSION=${FOONATHAN_MEMORY_VENDOR_VERSION}" "-DCMAKE_BUILD_TYPE=${_FOONATHAN_BUILD_TYPE}")
        if(PROVIZIO_DDS_SANITIZER_FLAGS)
            list(APPEND _FOONATHAN_CMAKE_ARGS "-DCMAKE_CXX_FLAGS=${PROVIZIO_DDS_SANITIZER_FLAGS}" "-DCMAKE_C_FLAGS=${PROVIZIO_DDS_SANITIZER_FLAGS}")
        endif()
        execute_process(
            COMMAND "${CMAKE_COMMAND}" ${_FOONATHAN_CMAKE_ARGS}
            WORKING_DIRECTORY "${FOONATHAN_MEMORY_DIR}"
        )
        execute_process(
            COMMAND "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY "${FOONATHAN_MEMORY_DIR}"
        )
        set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})
        find_package(foonathan_memory REQUIRED)
        # Link directory is set by Findfoonathan_memory.cmake based on actual location (lib/lib64/lib32)

        # Fast-DDS (https://github.com/provizio/Fast-DDS)
        set(FAST_DDS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/fast_dds_build")
        set(FAST_DDS_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/fast_dds")
        set(FAST_DDS_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
        set(FAST_DDS_GITHUB_PROJECT "provizio/Fast-DDS")
        set(FAST_DDS_INSTALL_DIR "${FAST_DDS_BINARY_DIR}/install")
        # When forcing Release CRT for Python compatibility, build Fast-DDS in
        # Release to ensure the entire DLL chain uses /MD consistently.
        set(_FAST_DDS_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
        if(PROVIZIO_DDS_FORCE_RELEASE_CRT)
            set(_FAST_DDS_BUILD_TYPE "Release")
        endif()
        set(FAST_DDS_CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${_FAST_DDS_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${FAST_DDS_INSTALL_DIR}" "-DBUILD_TESTING=OFF" "-DTHIRDPARTY=FORCE" "-DCMAKE_MODULE_PATH=${PROJECT_SOURCE_DIR}/cmake/modules")
        # Fast-DDS turns on CMAKE_COMPILE_WARNING_AS_ERROR (its own -Werror / MSVC /WX) for every build
        # except TSan. As a vendored third-party dependency, its internal warnings must never gate our
        # CI: newer toolchains warn on code we don't own (e.g. macos-latest's Apple clang flags a
        # -Wnonnull in Fast-DDS's own TypeObjectRegistry.cpp). Tell the sub-build's CMake to ignore
        # CMAKE_COMPILE_WARNING_AS_ERROR (overrides the project's set(); CMake >= 3.24). provizio_dds's
        # own build is a separate CMake invocation and keeps its -Werror.
        if(NOT CMAKE_VERSION VERSION_LESS "3.24")
            list(APPEND FAST_DDS_CMAKE_ARGS "--compile-no-warning-as-error")
        endif()
        if(NOT WIN32)
            list(APPEND FAST_DDS_CMAKE_ARGS "-DCMAKE_INSTALL_RPATH=${CMAKE_INSTALL_RPATH}${RPATH_DIV}${FAST_DDS_INSTALL_DIR}/lib")
        endif()
        if(PROVIZIO_DDS_SANITIZER_FLAGS)
            list(APPEND FAST_DDS_CMAKE_ARGS "-DCMAKE_CXX_FLAGS=${PROVIZIO_DDS_SANITIZER_FLAGS}" "-DCMAKE_C_FLAGS=${PROVIZIO_DDS_SANITIZER_FLAGS}")
        endif()
        # OpenSSL is always available here (find_package REQUIRED above)

        # Parse Fast-DDS version once for byproducts and library naming below
        # Prefer FAST_DDS_NUMERIC_VERSION (for branch refs); fall back to parsing
        # FAST_DDS_VERSION (when it's a vX.Y.Z tag).
        string(REGEX REPLACE "^v" "" FAST_DDS_VERSION_STRIPPED "${FAST_DDS_VERSION}")
        string(REGEX MATCH "^[0-9]+\\.[0-9]+" FAST_DDS_MAJOR_MINOR "${FAST_DDS_VERSION_STRIPPED}")
        if(NOT FAST_DDS_MAJOR_MINOR AND FAST_DDS_NUMERIC_VERSION)
            string(REGEX MATCH "^[0-9]+\\.[0-9]+" FAST_DDS_MAJOR_MINOR "${FAST_DDS_NUMERIC_VERSION}")
        endif()
        if(_FAST_DDS_BUILD_TYPE STREQUAL "Debug")
            set(FAST_DDS_DEBUG_SUFFIX "d")
        else()
            set(FAST_DDS_DEBUG_SUFFIX "")
        endif()

        # Declare library byproducts so the Ninja generator knows which build
        # rule produces them (required for imported-target file-level deps).
        if(WIN32)
            set(_FAST_DDS_BYPRODUCTS
                "${FAST_DDS_INSTALL_DIR}/lib/fastcdr${FAST_DDS_DEBUG_SUFFIX}-${FAST_CDR_VERSION}.lib"
                "${FAST_DDS_INSTALL_DIR}/lib/fastdds${FAST_DDS_DEBUG_SUFFIX}-${FAST_DDS_MAJOR_MINOR}.lib"
            )
        else()
            set(_FAST_DDS_BYPRODUCTS
                "${FAST_DDS_INSTALL_DIR}/lib/libfastcdr${CMAKE_SHARED_LIBRARY_SUFFIX}"
                "${FAST_DDS_INSTALL_DIR}/lib/libfastdds${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
        endif()
        ExternalProject_Add(libfast_dds
            GIT_REPOSITORY "https://github.com/${FAST_DDS_GITHUB_PROJECT}.git"
            GIT_TAG "${FAST_DDS_VERSION}"
            UPDATE_COMMAND ""
            # Decorate eprosima::SystemInfo::update_interfaces() with
            # FASTDDS_EXPORTED_API so it is reachable in the Windows DLL
            # export table (provizio_dds calls it to refresh Fast-DDS's
            # interface cache on network-recovery resets — APT-11792). The
            # script is a no-op on Linux / macOS (default visibility already
            # exports it) and idempotent. See export_system_info.cmake for
            # why this targeted patch is used instead of
            # CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS (the latter overflows the
            # 65535-symbol Windows DLL export limit on a library this large).
            PATCH_COMMAND ${CMAKE_COMMAND}
                "-DSYSTEMINFO_HPP=<SOURCE_DIR>/src/cpp/utils/SystemInfo.hpp"
                -P "${PROJECT_SOURCE_DIR}/cmake/fast_dds/export_system_info.cmake"
            PREFIX "${FAST_DDS_PREFIX}"
            SOURCE_DIR "${FAST_DDS_SOURCE_DIR}"
            BINARY_DIR "${FAST_DDS_BINARY_DIR}"
            CMAKE_ARGS ${FAST_DDS_CMAKE_ARGS}
            BUILD_BYPRODUCTS ${_FAST_DDS_BYPRODUCTS}
        )
        link_directories("${FAST_DDS_INSTALL_DIR}/lib/")

        # On Windows, installed libraries have versioned names (e.g. fastdds-3.6.lib).
        # Debug builds add a 'd' suffix (e.g. fastddsd-3.6.lib).
        # Version variables (FAST_DDS_VERSION_STRIPPED, FAST_DDS_MAJOR_MINOR, FAST_DDS_DEBUG_SUFFIX)
        # are parsed above, shared with the byproducts block.
        provizio_dds_create_fast_dds_imported_targets(
            INSTALL_DIR "${FAST_DDS_INSTALL_DIR}"
            FAST_DDS_MAJOR_MINOR "${FAST_DDS_MAJOR_MINOR}"
            FAST_CDR_VERSION "${FAST_CDR_VERSION}"
            DEBUG_SUFFIX "${FAST_DDS_DEBUG_SUFFIX}"
        )

        set(FAST_DDS_DEPENDENCIES libfast_dds)
        set(FAST_DDS_INCLUDE_DIRS "${FAST_DDS_INSTALL_DIR}/include/")
    endif(LOOK_FOR_FAST_DDS AND fastcdr_FOUND AND fastdds_FOUND)

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
        # Disable eProsima auto-linking (#pragma comment(lib, ...)) — linking is handled by CMake.
        # EPROSIMA_ALL_DYN_LINK enables __declspec(dllimport) for fastdds/fastcdr shared libraries.
        target_compile_definitions(provizio_dds_types PUBLIC EPROSIMA_ALL_NO_LIB EPROSIMA_ALL_DYN_LINK)
    endif(WIN32)

    if(FAST_DDS_DEPENDENCIES)
        add_dependencies(provizio_dds_types ${FAST_DDS_DEPENDENCIES})
    endif(FAST_DDS_DEPENDENCIES)

    # generated_include/ hosts headers configure_file'd from cmake/*.h.in below
    # (e.g. provizio/dds/known_types_dispatcher.h, autogenerated from the
    # consumed provizio_dds_idls's *PubSubTypes.hpp set). Expose it on the same
    # PUBLIC include path as include/ so consumers see one logical
    # provizio/dds/ namespace regardless of which headers are generated.
    target_include_directories(provizio_dds_types SYSTEM PUBLIC
        include
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated_include>"
        ${PROVIZIO_DDS_IDLS_SRC_DIR}
        ${FAST_DDS_INCLUDE_DIRS})
    target_link_libraries(provizio_dds_types PUBLIC ${FASTDDS_LIB} ${FASTCDR_LIB})
    # PROVIZIO_DDS_TYPES_EXPORTS is not currently consumed by any header macro
    # (generated IDL code uses EPROSIMA_USER_DLL_EXPORT instead), but is set for
    # forward compatibility if a PROVIZIO_DDS_TYPES_API macro is added later.
    set_target_properties(provizio_dds_types PROPERTIES DEFINE_SYMBOL "PROVIZIO_DDS_TYPES_EXPORTS")

    # Generate the visit_known_type<V>() dispatcher from whichever *PubSubTypes.hpp
    # headers the consumed provizio_dds_idls version ships, so the dispatcher
    # auto-tracks IDL additions without a maintained hand-list. CONFIGURE_DEPENDS
    # regenerates on next build when the set of shipped headers changes.
    file(GLOB_RECURSE _PROVIZIO_DDS_PUBSUBTYPE_HEADERS_REL
         RELATIVE "${PROVIZIO_DDS_IDLS_SRC_DIR}"
         CONFIGURE_DEPENDS
         "${PROVIZIO_DDS_IDLS_SRC_DIR}/*PubSubTypes.hpp")
    list(SORT _PROVIZIO_DDS_PUBSUBTYPE_HEADERS_REL)
    set(KNOWN_TYPES_INCLUDES "")
    set(KNOWN_TYPES_DISPATCH_BRANCHES "")
    foreach(_HDR IN LISTS _PROVIZIO_DDS_PUBSUBTYPE_HEADERS_REL)
        # The dispatcher is generated from the *contents* of these headers (class names are
        # read out via file(READ) below), but CONFIGURE_DEPENDS on the glob only re-runs CMake
        # when the *set* of matching files changes — not when a header's contents change (e.g.
        # an IDL regeneration that edits an existing header without adding/removing files). List
        # each header as a configure dependency so a content change also triggers a reconfigure
        # and regenerates the dispatcher, keeping incremental builds correct.
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                     "${PROVIZIO_DDS_IDLS_SRC_DIR}/${_HDR}")
        string(APPEND KNOWN_TYPES_INCLUDES "#include <${_HDR}>\n")
        # Namespace comes from the directory part of the path:
        # std_msgs/msg/StringPubSubTypes.hpp -> "std_msgs::msg".
        get_filename_component(_HDR_DIR "${_HDR}" DIRECTORY)
        string(REPLACE "/" "::" _HDR_NAMESPACE "${_HDR_DIR}")
        # Class names come from reading the header — one .msg header declares
        # one `class XxxPubSubType`, one .srv header declares two
        # (`Xxx_RequestPubSubType` + `Xxx_ResponsePubSubType`), so filename-to-
        # classname doesn't work. The regex matches every top-level PubSubType
        # class declaration regardless of count.
        file(READ "${PROVIZIO_DDS_IDLS_SRC_DIR}/${_HDR}" _HDR_CONTENT)
        string(REGEX MATCHALL "class[ \t]+[A-Za-z0-9_]+PubSubType[ \t]*:"
               _CLASS_DECLS "${_HDR_CONTENT}")
        foreach(_DECL IN LISTS _CLASS_DECLS)
            string(REGEX REPLACE "class[ \t]+([A-Za-z0-9_]+PubSubType)[ \t]*:"
                   "\\1" _CLASS_NAME "${_DECL}")
            # Comparison is against the cached wire-format name (computed once
            # per type by detail::known_type_wire_name), not against a literal
            # — service types and namespace-mangled names follow conventions
            # that aren't trivially derivable from the C++ class name, so we
            # let the type itself answer.
            string(APPEND KNOWN_TYPES_DISPATCH_BRANCHES
                   "        if (type_name == detail::known_type_wire_name<::${_HDR_NAMESPACE}::${_CLASS_NAME}>()) {\n"
                   "            std::forward<visitor>(v).template operator()<::${_HDR_NAMESPACE}::${_CLASS_NAME}>();\n"
                   "            return true;\n"
                   "        }\n")
        endforeach()
    endforeach()
    # Modern CMake's configure_file() creates non-existent output directories, but the
    # declared minimum here is 3.15 where that isn't guaranteed — create the directory
    # explicitly so a clean build tree configures reliably on every supported CMake.
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated_include/provizio/dds")
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/known_types_dispatcher.h.in"
        "${CMAKE_CURRENT_BINARY_DIR}/generated_include/provizio/dds/known_types_dispatcher.h"
        @ONLY)

    # provizio_dds library
    set(PROVIZIO_DDS_SOURCES
        src/network_recovery_coordinator.cpp
        src/accumulation.cpp
        src/detail/localization_filter.cpp
        src/domain_participant.cpp
        src/logging.cpp
        src/network_recovery.cpp
        src/point_cloud2.cpp
        src/request_response.cpp
        src/topic.cpp
    )
    # Per-platform network monitoring + address-snapshot backends. Each .cpp is
    # gated internally by the matching #ifdef so the build can also tolerate the
    # files being unconditionally listed; we still split per platform here so a
    # CMake regeneration on the wrong OS surfaces an obvious missing-file message.
    if(WIN32)
        list(APPEND PROVIZIO_DDS_SOURCES
            src/address_snapshot_windows.cpp
            src/network_monitor_windows.cpp
        )
    elseif(APPLE)
        list(APPEND PROVIZIO_DDS_SOURCES
            src/address_snapshot_macos.cpp
            src/network_monitor_macos.cpp
        )
    else()
        list(APPEND PROVIZIO_DDS_SOURCES
            src/address_snapshot_linux.cpp
            src/network_monitor_linux.cpp
        )
    endif()
    add_library(provizio_dds SHARED ${PROVIZIO_DDS_SOURCES})
    target_link_libraries(provizio_dds PUBLIC provizio_dds_types ${FASTDDS_LIB} ${FASTCDR_LIB})
    # /bigobj is required on MSVC for any consumer translation unit that
    # includes the generated provizio/dds/known_types_dispatcher.h — the
    # dispatcher templates over every PubSubType the consumed
    # provizio_dds_idls version ships and easily pushes the default
    # 65535-section COFF object-file limit (fatal error C1128). Apply via
    # INTERFACE on provizio_dds_types — the target whose PUBLIC include path
    # carries the dispatcher header — so EVERY consumer that can include it gets
    # the option transparently, whether it links provizio_dds_types directly or
    # links provizio_dds (which links provizio_dds_types PUBLIC, propagating the
    # option). Harmless on TUs that don't include the dispatcher; /bigobj only
    # permits a larger object file.
    target_compile_options(provizio_dds_types INTERFACE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
    set_target_properties(provizio_dds PROPERTIES DEFINE_SYMBOL "PROVIZIO_DDS_EXPORTS")
    # PROVIZIO_DDS_ENABLE_TESTS is no longer needed as a compile definition:
    # the test-only entry points on the network_recovery_coordinator are now
    # always part of the ABI (gated behind a different compile flag would
    # change the class layout between consumers built with and without it,
    # causing memory corruption on mismatch). The methods are documented as
    # test-only and unconditional.

    # Installation config
    install(
        TARGETS provizio_dds provizio_dds_types
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
    install(DIRECTORY include/provizio DESTINATION include)
    # The generated dispatcher header lives outside include/ (it depends on the
    # consumed provizio_dds_idls catalogue, regenerated at configure time), but
    # ships at the same install path consumers expect:
    # <prefix>/include/provizio/dds/known_types_dispatcher.h.
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/generated_include/provizio/dds/known_types_dispatcher.h"
        DESTINATION include/provizio/dds)
    install(DIRECTORY "${PROVIZIO_DDS_IDLS_SRC_DIR}/" DESTINATION include
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.ipp")

    # IDL .h -> .hpp backward-compat shims.
    # provizio_dds_idls v2.0.0 (Fast-DDS-Gen v4) renamed generated headers from
    # *.h to *.hpp. Downstream C++ code that still includes the pre-v2.0.0 .h
    # spelling would otherwise fail to compile. Generate matching .h files at
    # install time, each containing a single #include of the .hpp sibling, so
    # the rename is transparent to consumers without modifying provizio_dds_idls
    # itself. Files (not symlinks) for cross-platform reliability — Windows
    # symbolic-link creation requires admin / developer-mode privileges that
    # an end-user install cannot assume, and CMake's file(CREATE_LINK) does
    # not silently fall back to a copy. The shims are tiny (~150 bytes each)
    # and live only in the installed tree; the upstream IDL repo and our
    # build tree are untouched.
    install(CODE "set(_provizio_dds_idl_src_dir \"${PROVIZIO_DDS_IDLS_SRC_DIR}\")")
    install(CODE [=[
        file(GLOB_RECURSE _provizio_dds_idl_hpp
             RELATIVE "${_provizio_dds_idl_src_dir}"
             "${_provizio_dds_idl_src_dir}/*.hpp")
        list(LENGTH _provizio_dds_idl_hpp _provizio_dds_idl_count)
        message(STATUS
            "provizio_dds: generating ${_provizio_dds_idl_count} .h compat "
            "shims for renamed IDL headers under ${CMAKE_INSTALL_PREFIX}/include")
        foreach(_rel ${_provizio_dds_idl_hpp})
            string(REGEX REPLACE "\\.hpp$" ".h" _rel_h "${_rel}")
            get_filename_component(_hpp_basename "${_rel}" NAME)
            set(_out "${CMAKE_INSTALL_PREFIX}/include/${_rel_h}")
            if(NOT EXISTS "${_out}")
                file(WRITE "${_out}"
                    "// Auto-generated by provizio_dds install for backwards source\n"
                    "// compatibility — provizio_dds_idls v2.0.0 renamed generated\n"
                    "// headers from *.h to *.hpp when Fast-DDS-Gen v4 changed its\n"
                    "// default extension. Existing code that includes the old .h\n"
                    "// spelling keeps compiling via this shim.\n"
                    "#include \"${_hpp_basename}\"\n")
            endif()
        endforeach()
    ]=])

    if(FAST_DDS_INSTALL_DIR)
        install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/include/" DESTINATION include)
        # Install only library files — exclude cmake configs, scripts, and executables.
        # PATTERN "cmake" EXCLUDE prevents empty lib/cmake/ directories from being created
        # (FILES_MATCHING alone still creates the directory structure).
        if(WIN32)
            install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/lib/" DESTINATION lib
                FILES_MATCHING PATTERN "*.lib" PATTERN "cmake" EXCLUDE)
            install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/bin/" DESTINATION bin
                FILES_MATCHING PATTERN "*.dll")
        else()
            install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/lib/" DESTINATION lib
                FILES_MATCHING PATTERN "*.so*" PATTERN "*.dylib*" PATTERN "cmake" EXCLUDE)
        endif(WIN32)
    endif(FAST_DDS_INSTALL_DIR)

    # Python
    if(PYTHON_BINDINGS)
        message(STATUS "Configuring Python bindings...")

        # Python3
        if(POLICY CMP0094)
            cmake_policy(SET CMP0094 NEW)
        endif()

        find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
        set(PYTHON_INCLUDE_PATH ${Python3_INCLUDE_DIRS})
        set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
        set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
        include_directories(SYSTEM ${PYTHON_INCLUDE_PATH})

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

        # Python 3.14 changed PyHeapTypeObject layout (ht_token slot); SWIG 4.4+ is required
        if("${Python3_VERSION}" VERSION_GREATER_EQUAL "3.14" AND "${SWIG_VERSION}" VERSION_LESS "4.4")
            message(FATAL_ERROR "SWIG 4.4 or newer is required for Python ${Python3_VERSION}, but ${SWIG_VERSION} found!")
        endif()

        include_directories(SYSTEM ${FAST_DDS_INCLUDE_DIRS})

        # Resolve Fast-DDS-python
        FetchContent_Declare(fast_dds_python
            GIT_REPOSITORY "https://github.com/provizio/Fast-DDS-python"
            GIT_TAG "${FAST_DDS_PYTHON_VERSION}"
        )
        FetchContent_Populate(fast_dds_python)

        # Patch fast_dds_python to use stdlib sysconfig instead of removed distutils (Python 3.12+)
        set(FASTDDS_PYTHON_SWIG_CMAKE_FILE "${fast_dds_python_SOURCE_DIR}/fastdds_python/src/swig/CMakeLists.txt")
        file(READ "${FASTDDS_PYTHON_SWIG_CMAKE_FILE}" FASTDDS_PYTHON_SWIG_CMAKE_CONTENT)
        string(REGEX REPLACE
            [=[from distutils import sysconfig; print\(sysconfig\.get_python_lib\(plat_specific=True, prefix='([^']*)' \) \)]=]
            [=[import sysconfig; print(sysconfig.get_path('platlib', vars={'base': '\1', 'platbase': '\1'}))]=]
            FASTDDS_PYTHON_SWIG_CMAKE_CONTENT "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}")
        # Strip the whitespace inside the upstream
        #   $<IF:$<BOOL:${USE_PYTHON_STABLE_ABI}>, Python3::SABIModule, Python3::Module>
        # generator expression. CMake < 3.27 keeps the leading spaces in the
        # branch results, producing literal target names like " Python3::Module"
        # that fail to resolve ("Target ... links to target 'Python3::Module>'").
        # ROS 2 humble's docker image ships CMake 3.22, so the unfixed form
        # blocks the compatibility build.
        #
        # `\${USE_PYTHON_STABLE_ABI}` is escaped so the patched downstream
        # CMakeLists.txt still evaluates *its own* variable when Fast-DDS-
        # python is configured. Without the escape, `${USE_PYTHON_STABLE_ABI}`
        # would expand here (where the variable is undefined), permanently
        # baking in the `Python3::Module` branch and disabling the stable-ABI
        # option for downstream builds.
        string(REGEX REPLACE
            "\\$<IF:\\$<BOOL:\\$\\{USE_PYTHON_STABLE_ABI\\}>, *Python3::SABIModule, *Python3::Module>"
            "\$<IF:\$<BOOL:\${USE_PYTHON_STABLE_ABI}>,Python3::SABIModule,Python3::Module>"
            FASTDDS_PYTHON_SWIG_CMAKE_CONTENT "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}")
        # Fail closed: if a future FAST_DDS_PYTHON_VERSION bump shifts this text, the
        # REGEX above silently no-ops and the CMake<3.27 / ROS 2 humble build regresses
        # to the unresolvable " Python3::Module>" target. Assert the collapsed form is
        # present afterwards (true whether we just produced it or upstream already ships
        # it) rather than that the replace changed something: the patched file is written
        # back and persists, so a "did it change?" check would FATAL on every re-configure.
        string(FIND "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}"
            "\$<IF:\$<BOOL:\${USE_PYTHON_STABLE_ABI}>,Python3::SABIModule,Python3::Module>"
            _stable_abi_genexpr_pos)
        if(_stable_abi_genexpr_pos EQUAL -1)
            message(FATAL_ERROR
                "Fast-DDS-python ${FAST_DDS_PYTHON_VERSION}: the Python3 SABIModule/Module generator-expression "
                "patch did not produce the expected collapsed form — upstream text has likely changed. Update the "
                "REGEX REPLACE in CMakeLists.txt or the CMake 3.22 / ROS 2 humble compatibility build will break.")
        endif()
        # Don't set SWIGWORDSIZE64 on macOS. Upstream defines it on every
        # 64-bit UNIX, which makes SWIG treat `long` as the native 64-bit
        # integer — fine on Linux where `uint64_t` is `unsigned long`, but
        # broken on macOS where `uint64_t` is `unsigned long long`. With
        # the macro set on macOS, SWIG generates wrapper code that
        # alternates between `std::vector<uint64_t>` (==
        # `std::vector<unsigned long long>`) and `std::vector<unsigned
        # long>`, and the C++ compiler then rejects the mismatched
        # pointer/iterator conversions (no implicit conversion between
        # the two `vector<>` instantiations, even though the elements are
        # the same size). Skipping the macro on APPLE matches what we
        # already do for our own provizio_dds_python_types target.
        string(REGEX REPLACE
            "if\\(UNIX AND CMAKE_SIZEOF_VOID_P EQUAL 8\\)"
            "if(UNIX AND NOT APPLE AND CMAKE_SIZEOF_VOID_P EQUAL 8)"
            FASTDDS_PYTHON_SWIG_CMAKE_CONTENT "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}")
        # Fail closed (see the generator-expression patch above): assert the macOS guard
        # is present after patching so a future upstream text shift surfaces here instead
        # of as a macOS-only std::vector<unsigned long> vs <unsigned long long> wrapper-
        # compile failure.
        string(FIND "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}"
            "if(UNIX AND NOT APPLE AND CMAKE_SIZEOF_VOID_P EQUAL 8)"
            _swigwordsize_guard_pos)
        if(_swigwordsize_guard_pos EQUAL -1)
            message(FATAL_ERROR
                "Fast-DDS-python ${FAST_DDS_PYTHON_VERSION}: the SWIGWORDSIZE64 macOS-guard patch did not "
                "produce the expected 'NOT APPLE' form — upstream text has likely changed. Update the "
                "REGEX REPLACE in CMakeLists.txt or macOS wrapper compilation will regress.")
        endif()
        # Remove cmake config/export install rules from fastdds_python (not needed at runtime, and
        # they install cmake files under lib/cmake/ that we don't want in the package)
        # 1. Remove "EXPORT <name>-targets" line from install(TARGETS) so it doesn't generate export sets
        string(REGEX REPLACE "    EXPORT [^\n]*\n" "" FASTDDS_PYTHON_SWIG_CMAKE_CONTENT "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}")
        # 2. Truncate everything from "export(TARGETS" onward (cmake packaging boilerplate)
        string(FIND "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}" "export(TARGETS" _FASTDDS_PYTHON_EXPORT_POS)
        if(NOT _FASTDDS_PYTHON_EXPORT_POS EQUAL -1)
            string(SUBSTRING "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}" 0 ${_FASTDDS_PYTHON_EXPORT_POS} FASTDDS_PYTHON_SWIG_CMAKE_CONTENT)
        endif()
        file(WRITE "${FASTDDS_PYTHON_SWIG_CMAKE_FILE}" "${FASTDDS_PYTHON_SWIG_CMAKE_CONTENT}")

        set(FASTDDS_PYTHON_CMAKE_DIR "${fast_dds_python_SOURCE_DIR}/fastdds_python")
        set(FASTDDS_PYTHON_CMAKE_FILE "${FASTDDS_PYTHON_CMAKE_DIR}/CMakeLists.txt")
        set(FASTDDS_PYTHON_MISSING_DEFINITIONS_FIX_HEADER "${CMAKE_CURRENT_BINARY_DIR}/FASTDDS_PYTHON_MISSING_DEFINITIONS_FIX_HEADER.h")
        file(WRITE
            "${FASTDDS_PYTHON_MISSING_DEFINITIONS_FIX_HEADER}"

            "#include \"fastdds/rtps/attributes/ThreadSettings.hpp\"
         using namespace eprosima::fastdds::rtps;"
        )
        file(WRITE
            "${FASTDDS_PYTHON_CMAKE_FILE}"

            "project(fastdds_python VERSION ${FAST_DDS_PYTHON_NUMERIC_VERSION})
        
        # Disable treating warnings as errors in Fast-DDS-python
        if(MSVC)
            add_compile_options(/W0)
        else()
            add_compile_options(-Wno-error -Wno-unused-parameter -Wno-sign-compare -Wno-deprecated-declarations)
        endif()

        # Adds missing ThreadSettings definition
        if(MSVC)
            add_definitions(/FI\"${FASTDDS_PYTHON_MISSING_DEFINITIONS_FIX_HEADER}\")
        else(MSVC)
            # GCC or Clang
            add_definitions(-include \"${FASTDDS_PYTHON_MISSING_DEFINITIONS_FIX_HEADER}\")
        endif(MSVC)
        
        # Disable eProsima auto-linking and enable DLL imports on Windows
        if(WIN32)
            add_definitions(-DEPROSIMA_ALL_DYN_LINK -DEPROSIMA_ALL_NO_LIB)
            # Prevent SWIG from auto-linking python3XX_d.lib in Debug builds
            add_definitions(-DSWIG_PYTHON_INTERPRETER_NO_DEBUG)
        endif()

        add_subdirectory(src/swig)
        "
        )
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/.clang-tidy" "${CLANG_TIDY_DISABLE}") # No static analysis in autogenerated code
        add_subdirectory("${FASTDDS_PYTHON_CMAKE_DIR}" "${fast_dds_python_BINARY_DIR}")

        file(GLOB_RECURSE PROVIZIO_DDS_IDL_I_S "${PROVIZIO_DDS_IDLS_SRC_DIR}/*.i")
        list(FILTER PROVIZIO_DDS_IDL_I_S EXCLUDE REGEX ".*PubSub.*")

        set(MAIN_I "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_python_types.i")
        provizio_dds_write_swig_main_i(
            OUTPUT "${MAIN_I}"
            MODULE_NAME provizio_dds_python_types
            IDL_FILES ${PROVIZIO_DDS_IDL_I_S}
            IDLS_ROOT "${PROVIZIO_DDS_IDLS_SRC_DIR}"
            PYTHON3_EXECUTABLE "${Python3_EXECUTABLE}"
        )

        set_source_files_properties(${MAIN_I} PROPERTIES
            CPLUSPLUS ON
            USE_TARGET_INCLUDE_DIRECTORIES TRUE
        )
        swig_add_library(provizio_dds_python_types
            TYPE SHARED
            LANGUAGE python
            SOURCES ${MAIN_I}
        )

        # Add all IDL directories to include directories, as generated files
        # include their headers just as #include "filename.{h,hpp}". Fast-DDS-Gen v4
        # emits .hpp + .ipp alongside (or instead of) the v3 .h flavor, so
        # both header suffixes must be covered here.
        file(GLOB_RECURSE IDLS_SRC_DIRS LIST_DIRECTORIES true "${PROVIZIO_DDS_IDLS_SRC_DIR}/*")

        foreach(DIR ${IDLS_SRC_DIRS})
            if(IS_DIRECTORY "${DIR}")
                if(NOT APPLE AND NOT WIN32)
                    target_include_directories(provizio_dds_python_types SYSTEM PRIVATE "${DIR}")
                else(NOT APPLE AND NOT WIN32)
                    # macOS and Windows use case-insensitive file system, so ROS2 Time.h and String.h are confused with system time.h and string.h
                    # So instead of including their directories, we have to include the paths in the generated code
                    file(GLOB H_FILES RELATIVE "${DIR}" "${DIR}/*.h")
                    file(GLOB HPP_FILES RELATIVE "${DIR}" "${DIR}/*.hpp")
                    file(GLOB IPP_FILES RELATIVE "${DIR}" "${DIR}/*.ipp")
                    file(GLOB I_FILES RELATIVE "${DIR}" "${DIR}/*.i")
                    file(GLOB CXX_FILES RELATIVE "${DIR}" "${DIR}/*.cxx")

                    set(HEADER_FILES ${H_FILES} ${HPP_FILES} ${IPP_FILES})

                    foreach(GENERATED_FILE ${CXX_FILES} ${HEADER_FILES} ${I_FILES})
                        file(READ "${DIR}/${GENERATED_FILE}" FILE_CONTENT)

                        foreach(HEADER_FILE ${HEADER_FILES})
                            string(REPLACE "#include \"${HEADER_FILE}\"" "#include \"${DIR}/${HEADER_FILE}\"" FILE_CONTENT "${FILE_CONTENT}")
                        endforeach(HEADER_FILE ${HEADER_FILES})

                        file(WRITE "${DIR}/${GENERATED_FILE}" "${FILE_CONTENT}")
                    endforeach(GENERATED_FILE ${CXX_FILES} ${HEADER_FILES} ${I_FILES})
                endif(NOT APPLE AND NOT WIN32)
            endif(IS_DIRECTORY "${DIR}")
        endforeach(DIR ${IDLS_SRC_DIRS})

        if(UNIX AND NOT APPLE AND CMAKE_SIZEOF_VOID_P EQUAL 8)
            set_property(TARGET provizio_dds_python_types PROPERTY SWIG_COMPILE_DEFINITIONS SWIGWORDSIZE64)
        endif(UNIX AND NOT APPLE AND CMAKE_SIZEOF_VOID_P EQUAL 8)

        # We have to disable some of the warnings in the generated code
        if(MSVC)
            target_compile_options(provizio_dds_python_types PRIVATE /W0 /bigobj)
            # Prevent SWIG from auto-linking python3XX_d.lib in Debug builds
            target_compile_definitions(provizio_dds_python_types PRIVATE SWIG_PYTHON_INTERPRETER_NO_DEBUG)
        else()
            target_compile_options(provizio_dds_python_types PRIVATE -Wno-delete-non-virtual-dtor -Wno-unused-parameter -Wno-missing-field-initializers -Wno-deprecated-declarations -Wno-error)
        endif()

        swig_link_libraries(provizio_dds_python_types
            Python3::Module
            provizio_dds_types
            ${FASTDDS_LIB}
            ${FASTCDR_LIB}
            fastdds_python
        )

        # Make sure Fast-DDS is built (and its headers installed) before ANY SWIG code
        # generation runs: the .i preprocessing #includes Fast-DDS/fastcdr headers, so a
        # parallel build that starts SWIG before the libfast_dds ExternalProject has installed
        # them fails non-deterministically with "Unable to find 'fastcdr/config.h'".
        #
        # UseSWIG's target layout varies by CMake version: some versions fold the generation
        # into the <name> target, others split it into a separate <name>_swig_compilation
        # target that runs the generation step. The two SWIG libraries here do NOT necessarily
        # share the same layout — e.g. under CMake 4.x provizio_dds_python_types gets a
        # _swig_compilation target while fastdds_python does not — so we must NOT infer one
        # library's layout from the other's (the previous heuristic probed
        # fastdds_python_swig_compilation to choose a single postfix for both, which left
        # provizio_dds_python_types_swig_compilation ungated under CMake 4.x and raced).
        # Attach the dependency to whichever target spelling actually exists, per library.
        if(FAST_DDS_DEPENDENCIES)
            foreach(_swig_target fastdds_python provizio_dds_python_types)
                foreach(_swig_suffix "" "_swig_compilation")
                    if(TARGET ${_swig_target}${_swig_suffix})
                        add_dependencies(${_swig_target}${_swig_suffix} ${FAST_DDS_DEPENDENCIES})
                    endif()
                endforeach()
            endforeach()
        endif(FAST_DDS_DEPENDENCIES)

        # On Windows, SWIG Python modules are .pyd, not .dll
        if(WIN32)
            set(PYTHON_EXTENSION_SUFFIX ".pyd")
        else()
            set(PYTHON_EXTENSION_SUFFIX "${CMAKE_SHARED_LIBRARY_SUFFIX}")
        endif()

        set(FASTDDS_PYTHON_LIBS
            "${fast_dds_python_BINARY_DIR}/src/swig/fastdds.py"
            "${fast_dds_python_BINARY_DIR}/src/swig/_fastdds_python${PYTHON_EXTENSION_SUFFIX}"
        )
        set(PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS
            "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_python_types.py"
            "${CMAKE_CURRENT_BINARY_DIR}/_provizio_dds_python_types${PYTHON_EXTENSION_SUFFIX}"
        )
        set(PROVIZIO_DDS_PYTHON_LIBS
            "${CMAKE_CURRENT_SOURCE_DIR}/python/provizio_dds.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/python/point_cloud2.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/python/accumulation.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/python/gps_utils.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/python/network_recovery.py"
        )
        set(PROVIZIO_DDS_ALL_PYTHON_LIBS
            ${FASTDDS_PYTHON_LIBS}
            ${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS}
            ${PROVIZIO_DDS_PYTHON_LIBS}
        )

        if(FAST_DDS_INSTALL_DIR)
            file(GLOB FASTDDS_LIBS "${FAST_DDS_INSTALL_DIR}/lib/*${CMAKE_SHARED_LIBRARY_SUFFIX}*")
            if(WIN32)
                # On Windows, DLLs are in bin/, not lib/
                file(GLOB FASTDDS_BIN_LIBS "${FAST_DDS_INSTALL_DIR}/bin/*.dll")
                list(APPEND FASTDDS_LIBS ${FASTDDS_BIN_LIBS})
            endif()
            set(PROVIZIO_DDS_ALL_PYTHON_LIBS ${PROVIZIO_DDS_ALL_PYTHON_LIBS} ${FASTDDS_LIBS})
        endif(FAST_DDS_INSTALL_DIR)

        if(WIN32)
            # On Windows, Python extension modules (.pyd) need the provizio_dds DLLs at runtime.
            # These are added via generator expressions in test/python/CMakeLists.txt.
            # Fast-DDS DLLs must also be copied because file(GLOB) above runs at configure
            # time before ExternalProject builds them, so FASTDDS_LIBS is empty.
            set(PROVIZIO_DDS_PYTHON_EXTRA_DLLS provizio_dds provizio_dds_types)
            if(TARGET fastcdr AND TARGET fastdds)
                list(APPEND PROVIZIO_DDS_PYTHON_EXTRA_DLLS fastcdr fastdds)
            endif()

            # Copy OpenSSL DLLs so they are found before any ABI-incompatible copies
            # (e.g. MinGW-built) that may appear in PATH via os.add_dll_directory().
            if(OPENSSL_FOUND)
                get_filename_component(_OPENSSL_ROOT "${OPENSSL_INCLUDE_DIR}" DIRECTORY)
                file(GLOB _OPENSSL_DLLS "${_OPENSSL_ROOT}/bin/libssl*.dll" "${_OPENSSL_ROOT}/bin/libcrypto*.dll")
                if(NOT _OPENSSL_DLLS)
                    message(WARNING "OpenSSL DLLs not found under ${_OPENSSL_ROOT}/bin/ (expected libssl*.dll, libcrypto*.dll)")
                endif()
                list(APPEND PROVIZIO_DDS_ALL_PYTHON_LIBS ${_OPENSSL_DLLS})
            endif()
        endif()

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
            execute_process(COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib', vars={'base': '${CMAKE_INSTALL_PREFIX}', 'platbase': '${CMAKE_INSTALL_PREFIX}'}))"
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
            RUNTIME DESTINATION "${PROVIZIO_DDS_PYTHON_TYPES_PYTHON_LIBS_DIR}"
        )
        install(FILES
            ${PROVIZIO_DDS_PYTHON_LIBS}
            DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}"
        )

        if(PYTHON_PACKAGES_INSTALL_DIR)
            install(FILES
                "${CMAKE_CURRENT_BINARY_DIR}/version.txt"
                DESTINATION "${PYTHON_PACKAGES_INSTALL_DIR}"
            )
        endif(PYTHON_PACKAGES_INSTALL_DIR)

        if(FAST_DDS_INSTALL_DIR)
            # Local build of Fast-DDS, have to be included with the package then
            install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/lib/" DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}"
                PATTERN "cmake" EXCLUDE)
            if(WIN32)
                # On Windows, runtime DLLs are in bin/, not lib/
                install(DIRECTORY "${FAST_DDS_INSTALL_DIR}/bin/" DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}"
                    FILES_MATCHING PATTERN "*.dll")
            endif(WIN32)
        endif(FAST_DDS_INSTALL_DIR)

        # On Windows, Fast-DDS links against OpenSSL when SECURITY=ON.  Bundle the
        # correct MSVC-built OpenSSL DLLs so they are found before any ABI-incompatible
        # copies (MinGW, MySQL, PHP, etc.) that may exist elsewhere on the system.
        if(WIN32 AND OPENSSL_FOUND)
            get_filename_component(_OPENSSL_ROOT "${OPENSSL_INCLUDE_DIR}" DIRECTORY)
            install(DIRECTORY "${_OPENSSL_ROOT}/bin/"
                DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}"
                FILES_MATCHING PATTERN "libssl*.dll" PATTERN "libcrypto*.dll")
        endif()

        # provizio_dds DLL is needed by the Python extension modules at runtime
        install(TARGETS provizio_dds
            LIBRARY DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}"
            RUNTIME DESTINATION "${PROVIZIO_DDS_PYTHON_LIBS_DIR}"
        )

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

    if(INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS AND NOT LOOK_FOR_FAST_DDS AND UNIX AND NOT APPLE)
        # Linux: link fastdds libraries by the fully-qualified name (.so.major.minor.patch) as they're incompatible between different .patch versions
        install(CODE "execute_process(COMMAND \"${CMAKE_CURRENT_SOURCE_DIR}/fully_qualified_fastdds_libs.sh\" \"${CMAKE_INSTALL_PREFIX}/lib\")")
    endif(INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS AND NOT LOOK_FOR_FAST_DDS AND UNIX AND NOT APPLE)

    # Remove cmake config dirs installed by third-party targets.
    # These are not needed at runtime and should not be included in packages or caches.
    # Scoped to specific subdirectories to avoid destroying unrelated cmake configs
    # when installed to a shared prefix.
    # Also remove the pre-3.x `fastrtps` config dir in case this install
    # is layering on top of a prefix that already had a Fast-DDS 1.x/2.x
    # tree there — leaving it behind would let a later
    # `find_package(fastrtps)` pick up the obsolete config from the same
    # prefix.
    install(CODE "
        file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/fastrtps\")
        file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/fastdds\")
        file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/fastcdr\")
        file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/fastdds_python\")
        file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/foonathan_memory\")
    ")

    if(ENABLE_TESTS)
        message(STATUS "ENABLE_TESTS is ON: tests will be built and enabled.")
        include(CTest)
        enable_testing()
        add_subdirectory(test)
    endif(ENABLE_TESTS)
endif(HAS_BIN_CACHE)
