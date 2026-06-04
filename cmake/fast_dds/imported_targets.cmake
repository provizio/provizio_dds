# Defines provizio_dds_create_fast_dds_imported_targets(), which creates
# imported CMake targets named `fastdds` and `fastcdr` that point at the
# libraries produced by the in-tree Fast-DDS ExternalProject build.
#
# Why imported targets are needed even when we already know FASTDDS_LIB /
# FASTCDR_LIB as strings:
#
# - On Windows, installed Fast-DDS libraries have versioned names
#   (`fastdds-3.6.lib`, debug builds add a `d` suffix). Third-party CMake code
#   (e.g. Fast-DDS-Python's fastdds_python target) calls
#   `target_link_libraries(... fastcdr fastdds)` with the unversioned names —
#   without imported targets bridging that to the versioned files, linking
#   fails. The Windows branch maps both IMPORTED_IMPLIB (the .lib import
#   library) and IMPORTED_LOCATION (the .dll runtime library).
#
# - On Linux/macOS, libraries are unversioned in the build tree
#   (`libfastdds.so`, `libfastdds.dylib`; the install step may rename them to
#   `.so.MAJOR.MINOR.PATCH` when INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=ON,
#   but the imported targets here point at the build tree), so plain
#   `-lfastdds` would normally work. The imported targets are still
#   required because upstream Fast-DDS-Python 2.x emits a
#   `$<TARGET_FILE_NAME:fastdds>` generator expression inside its
#   `fastdds.i`'s `moduleimport` directive (consumed at SWIG-generate time on
#   ALL platforms — the resolved string is only used at runtime on Windows,
#   but CMake evaluates the expression unconditionally). Without an imported
#   `fastdds` target, that file(GENERATE) call fails with `No target "fastdds"`.
#
# Both Windows and non-Windows branches set FASTDDS_LIB / FASTCDR_LIB in the
# caller's scope so the existing call sites (target_link_libraries, etc.)
# don't need to change.

function(provizio_dds_create_fast_dds_imported_targets)
    set(_options "")
    set(_one_value_args INSTALL_DIR FAST_DDS_MAJOR_MINOR FAST_CDR_VERSION DEBUG_SUFFIX)
    set(_multi_value_args "")
    cmake_parse_arguments(_ARG "${_options}" "${_one_value_args}" "${_multi_value_args}" ${ARGN})

    if(NOT _ARG_INSTALL_DIR)
        message(FATAL_ERROR "provizio_dds_create_fast_dds_imported_targets: INSTALL_DIR is required")
    endif()

    if(WIN32)
        if(NOT _ARG_FAST_DDS_MAJOR_MINOR OR NOT _ARG_FAST_CDR_VERSION)
            message(FATAL_ERROR
                "provizio_dds_create_fast_dds_imported_targets: FAST_DDS_MAJOR_MINOR and FAST_CDR_VERSION are required on Windows")
        endif()
        set(_fastdds_libname "fastdds${_ARG_DEBUG_SUFFIX}-${_ARG_FAST_DDS_MAJOR_MINOR}")
        set(_fastcdr_libname "fastcdr${_ARG_DEBUG_SUFFIX}-${_ARG_FAST_CDR_VERSION}")
        if(NOT TARGET fastcdr)
            add_library(fastcdr SHARED IMPORTED)
            set_target_properties(fastcdr PROPERTIES
                IMPORTED_IMPLIB "${_ARG_INSTALL_DIR}/lib/${_fastcdr_libname}.lib"
                IMPORTED_LOCATION "${_ARG_INSTALL_DIR}/bin/${_fastcdr_libname}.dll"
            )
        endif()
        if(NOT TARGET fastdds)
            add_library(fastdds SHARED IMPORTED)
            set_target_properties(fastdds PROPERTIES
                IMPORTED_IMPLIB "${_ARG_INSTALL_DIR}/lib/${_fastdds_libname}.lib"
                IMPORTED_LOCATION "${_ARG_INSTALL_DIR}/bin/${_fastdds_libname}.dll"
            )
        endif()
        set(FASTDDS_LIB "${_fastdds_libname}" PARENT_SCOPE)
        set(FASTCDR_LIB "${_fastcdr_libname}" PARENT_SCOPE)
    else()
        if(NOT TARGET fastcdr)
            add_library(fastcdr SHARED IMPORTED)
            set_target_properties(fastcdr PROPERTIES
                IMPORTED_LOCATION "${_ARG_INSTALL_DIR}/lib/libfastcdr${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
        endif()
        if(NOT TARGET fastdds)
            add_library(fastdds SHARED IMPORTED)
            set_target_properties(fastdds PROPERTIES
                IMPORTED_LOCATION "${_ARG_INSTALL_DIR}/lib/libfastdds${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
        endif()
        set(FASTDDS_LIB "fastdds" PARENT_SCOPE)
        set(FASTCDR_LIB "fastcdr" PARENT_SCOPE)
    endif()
endfunction()
