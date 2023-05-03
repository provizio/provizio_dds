find_package(foonathan_memory QUIET NO_MODULE)

if(NOT foonathan_memory_FOUND)
    set(foonathan_memory_DIR "${CMAKE_BINARY_DIR}/../foonathan_memory/install")

    if(EXISTS "${foonathan_memory_DIR}/lib/libfoonathan_memory.a")
        set(foonathan_memory_FOUND TRUE)
        set(foonathan_memory_INCLUDE_DIRS "${foonathan_memory_DIR}/include/foonathan_memory")
        include_directories("${foonathan_memory_INCLUDE_DIRS}")
        link_directories("${foonathan_memory_DIR}/lib")
    else(EXISTS "${foonathan_memory_DIR}/lib/libfoonathan_memory.a")
        set(foonathan_memory_FOUND FALSE)
    endif(EXISTS "${foonathan_memory_DIR}/lib/libfoonathan_memory.a")
endif(NOT foonathan_memory_FOUND)
