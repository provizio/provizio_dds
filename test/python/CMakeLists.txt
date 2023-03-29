set(TEST_PATH "${CMAKE_CURRENT_BINARY_DIR}")

# Copy tests and all dependencies to the same test directory
add_custom_target(python_tests ALL)
file(GLOB TEST_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/*.py")
foreach(FILE ${PROVIZIO_DDS_PYTHON_LIBS} ${TEST_SCRIPTS})
    add_custom_command(TARGET python_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${FILE}" "${TEST_PATH}"
    )
endforeach()
add_dependencies(python_tests provizio_dds_python_types)

# TODO: Windows versions
add_test(NAME python_pub_python_sub
    COMMAND sh -c "${PYTHON_EXECUTABLE} ./python_publisher.py & ${PYTHON_EXECUTABLE} ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 4)

add_test(NAME python_pub_cpp_sub
    COMMAND sh -c "${PYTHON_EXECUTABLE} ./python_publisher.py & $<TARGET_FILE:simplest_subscriber>"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_cpp_sub PROPERTIES TIMEOUT 4)

add_test(NAME cpp_pub_python_sub
    COMMAND sh -c "$<TARGET_FILE:simplest_publisher> & ${PYTHON_EXECUTABLE} ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(cpp_pub_python_sub PROPERTIES TIMEOUT 4)
