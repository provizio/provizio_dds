find_package(foonathan_memory QUIET NO_MODULE)

if(NOT foonathan_memory_FOUND)
    set(foonathan_memory_DIR "${CMAKE_BINARY_DIR}/../foonathan_memory/install")

    # Check for libfoonathan_memory.a in lib, lib64, or lib32
    set(foonathan_memory_LIB_DIR "")
    foreach(lib_subdir lib lib64 lib32)
        if(EXISTS "${foonathan_memory_DIR}/${lib_subdir}/libfoonathan_memory.a")
            set(foonathan_memory_LIB_DIR "${foonathan_memory_DIR}/${lib_subdir}")
            break()
        endif(EXISTS "${foonathan_memory_DIR}/${lib_subdir}/libfoonathan_memory.a")
    endforeach(lib_subdir lib lib64 lib32)

    if(foonathan_memory_LIB_DIR)
        set(foonathan_memory_FOUND TRUE)
        set(foonathan_memory_INCLUDE_DIRS "${foonathan_memory_DIR}/include/foonathan_memory")
        include_directories("${foonathan_memory_INCLUDE_DIRS}")
        link_directories("${foonathan_memory_LIB_DIR}")
    else(foonathan_memory_LIB_DIR)
        set(foonathan_memory_FOUND FALSE)
    endif(foonathan_memory_LIB_DIR)
endif(NOT foonathan_memory_FOUND)
