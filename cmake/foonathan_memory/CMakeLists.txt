cmake_minimum_required(VERSION 3.10.0)

project(foonathan_memory)

find_package(foonathan_memory QUIET)

set(FOONATHAN_MEMORY_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/install")

if(NOT foonathan_memory_FOUND)
    # Not found, let's build it
    include(ExternalProject)

    if(NOT FOONATHAN_MEMORY_VENDOR_VERSION)
        set(FOONATHAN_MEMORY_VENDOR_VERSION "v1.3.1")
    endif(NOT FOONATHAN_MEMORY_VENDOR_VERSION)

    set(FOONATHAN_MEMORY_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/build")
    set(FOONATHAN_MEMORY_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/source")
    set(FOONATHAN_MEMORY_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
    set(FOONATHAN_MEMORY_GITHUB_PROJECT "provizio/foonathan_memory_vendor")
    ExternalProject_Add(libfoonathan_memory
        GIT_REPOSITORY "https://github.com/${FOONATHAN_MEMORY_GITHUB_PROJECT}.git"
        GIT_TAG "${FOONATHAN_MEMORY_VENDOR_VERSION}"
        # Both refs this can resolve to (v1.4.1, and v1.3.1 for the legacy path above) are
        # tags, and nothing here derives anything from git history — see the note on
        # libfast_dds in the top-level CMakeLists.txt, including why a SHA pin would have to
        # drop this.
        GIT_SHALLOW TRUE
        UPDATE_COMMAND ""
        PREFIX "${FOONATHAN_MEMORY_PREFIX}"
        SOURCE_DIR "${FOONATHAN_MEMORY_SOURCE_DIR}"
        BINARY_DIR "${FOONATHAN_MEMORY_BINARY_DIR}"
        CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${FOONATHAN_MEMORY_INSTALL_DIR}"
    )
    add_custom_target(get_foonathan_memory ALL
        COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_SOURCE_DIR}/create_foonathan_symlink.cmake"
    )
    add_dependencies(get_foonathan_memory libfoonathan_memory)
endif(NOT foonathan_memory_FOUND)
