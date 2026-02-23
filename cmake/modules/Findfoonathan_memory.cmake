find_package(foonathan_memory QUIET NO_MODULE)

if(NOT foonathan_memory_FOUND)
    set(foonathan_memory_DIR "${CMAKE_BINARY_DIR}/../foonathan_memory/install")

    # Check for foonathan_memory library in lib, lib64, or lib32
    set(foonathan_memory_LIB_DIR "")
    foreach(lib_subdir lib lib64 lib32)
        # Unix: libfoonathan_memory.a
        if(EXISTS "${foonathan_memory_DIR}/${lib_subdir}/libfoonathan_memory.a")
            set(foonathan_memory_LIB_DIR "${foonathan_memory_DIR}/${lib_subdir}")
            break()
        endif(EXISTS "${foonathan_memory_DIR}/${lib_subdir}/libfoonathan_memory.a")
        # Windows/MSVC: foonathan_memory-*.lib
        file(GLOB foonathan_memory_LIB_FILES "${foonathan_memory_DIR}/${lib_subdir}/foonathan_memory-*.lib")
        if(foonathan_memory_LIB_FILES)
            set(foonathan_memory_LIB_DIR "${foonathan_memory_DIR}/${lib_subdir}")
            break()
        endif(foonathan_memory_LIB_FILES)
    endforeach(lib_subdir lib lib64 lib32)

    if(foonathan_memory_LIB_DIR)
        set(foonathan_memory_FOUND TRUE)
        set(foonathan_memory_INCLUDE_DIRS "${foonathan_memory_DIR}/include/foonathan_memory")

        # Find the actual library file to set IMPORTED_LOCATION
        if(WIN32)
            # On Windows, prefer .lib over .a (there may be both; the .a is a symlink)
            file(GLOB foonathan_memory_LIB_FILE "${foonathan_memory_LIB_DIR}/foonathan_memory-*.lib")
            if(NOT foonathan_memory_LIB_FILE)
                file(GLOB foonathan_memory_LIB_FILE "${foonathan_memory_LIB_DIR}/libfoonathan_memory*.a")
            endif(NOT foonathan_memory_LIB_FILE)
        else(WIN32)
            file(GLOB foonathan_memory_LIB_FILE "${foonathan_memory_LIB_DIR}/libfoonathan_memory*.a")
            if(NOT foonathan_memory_LIB_FILE)
                file(GLOB foonathan_memory_LIB_FILE "${foonathan_memory_LIB_DIR}/foonathan_memory-*.lib")
            endif(NOT foonathan_memory_LIB_FILE)
        endif(WIN32)

        # Use the first file found by the glob
        list(GET foonathan_memory_LIB_FILE 0 foonathan_memory_LIB_FILE)

        # Create an imported target so that target_link_libraries(... foonathan_memory)
        # propagates include directories automatically (required by Fast-DDS)
        if(NOT TARGET foonathan_memory)
            add_library(foonathan_memory STATIC IMPORTED)
            set_target_properties(foonathan_memory PROPERTIES
                IMPORTED_LOCATION "${foonathan_memory_LIB_FILE}"
                INTERFACE_INCLUDE_DIRECTORIES "${foonathan_memory_INCLUDE_DIRS}"
            )
        endif(NOT TARGET foonathan_memory)

        # Keep old-style calls for backwards compatibility
        include_directories("${foonathan_memory_INCLUDE_DIRS}")
        link_directories("${foonathan_memory_LIB_DIR}")
    else(foonathan_memory_LIB_DIR)
        set(foonathan_memory_FOUND FALSE)
    endif(foonathan_memory_LIB_DIR)
endif(NOT foonathan_memory_FOUND)
