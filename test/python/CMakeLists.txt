set(TEST_PATH "${CMAKE_CURRENT_BINARY_DIR}")

# Install numpy, copy tests and all dependencies to the same test directory
add_custom_target(python_tests ALL "${PYTHON_EXECUTABLE}" -m pip install \"numpy>=1.16\")
file(GLOB TEST_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/*.py")
foreach(FILE ${PROVIZIO_DDS_ALL_PYTHON_LIBS} ${TEST_SCRIPTS})
    add_custom_command(TARGET python_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${FILE}" "${TEST_PATH}"
    )
endforeach()
if("${CMAKE_SHARED_LIBRARY_SUFFIX}" STREQUAL ".dylib")
    # Python expects .so and .so only in import
    add_custom_command(TARGET python_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink "${TEST_PATH}/_provizio_dds_python_types${CMAKE_SHARED_LIBRARY_SUFFIX}" "${TEST_PATH}/_provizio_dds_python_types.so")
    add_custom_command(TARGET python_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink "${TEST_PATH}/_fastdds_python${CMAKE_SHARED_LIBRARY_SUFFIX}" "${TEST_PATH}/_fastdds_python.so")
endif("${CMAKE_SHARED_LIBRARY_SUFFIX}" STREQUAL ".dylib")
add_dependencies(python_tests provizio_dds_python_types)

# TODO: Windows versions
add_test(NAME python_pub_python_sub
    COMMAND sh -c "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py & \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 20)

add_test(NAME python_pub_cpp_sub
    COMMAND sh -c "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py & $<TARGET_FILE:simplest_subscriber>"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_cpp_sub PROPERTIES TIMEOUT 20)

add_test(NAME cpp_pub_python_sub
    COMMAND sh -c "$<TARGET_FILE:simplest_publisher> & \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(cpp_pub_python_sub PROPERTIES TIMEOUT 20)

add_test(NAME pointcloud2_test
    COMMAND sh -c "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./pointcloud2_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_publisher_no_leaks_test
    COMMAND sh -c "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher_no_leaks_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_subscriber_no_leaks_test
    COMMAND sh -c "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber_no_leaks_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
