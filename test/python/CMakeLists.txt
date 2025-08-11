set(TEST_PATH "${CMAKE_CURRENT_BINARY_DIR}")

# Install dependencies, copy tests and all dependencies to the same test directory
add_custom_target(python_tests ALL bash -c \"\'${PYTHON_EXECUTABLE}\' -m pip install 'numpy>=1.16' 'transforms3d>=0.4.1' >/dev/null 2>&1 || true\") # Supressing errors as packages can be installed manually, as the case in CI
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
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py & \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 20)

add_test(NAME python_pub_python_sub_type_reuse
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher_type_reuse.py & \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber_type_reuse.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 20)

add_test(NAME python_pub_cpp_sub
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py & $<TARGET_FILE:simplest_subscriber>; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_cpp_sub PROPERTIES TIMEOUT 20)

add_test(NAME cpp_pub_python_sub
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { $<TARGET_FILE:simplest_publisher> & \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(cpp_pub_python_sub PROPERTIES TIMEOUT 20)

add_test(NAME python_request_response
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_service.py\" \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_client.py\"; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response PROPERTIES TIMEOUT 30)

add_test(NAME python_request_response_async
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_service_concurrent.py\" \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_client.py\"; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_async PROPERTIES TIMEOUT 30)

add_test(NAME python_request_response_ignore
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_ignore_service.py\" \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_ignore_client.py\"; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_ignore PROPERTIES TIMEOUT 30)

add_test(NAME python_request_response_ignore_async
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_ignore_service_async.py\" \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_ignore_client.py\"; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_ignore_async PROPERTIES TIMEOUT 30)

add_test(NAME python_request_response_dual_service
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_dual_service.py; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_dual_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_service
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_service>\" \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_client.py\"; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_response_concurrent_requests_serial_service
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_service>\" \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_concurrent_client.py\"; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_response_concurrent_requests_serial_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_service_concurrent
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_concurrent_service> 5\" \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_concurrent_client.py\"; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_service_concurrent PROPERTIES TIMEOUT 30)

add_test(NAME python_service_cpp_client
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_service.py\" \"$<TARGET_FILE:request_response_client>\"; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_service_cpp_client PROPERTIES TIMEOUT 30)

add_test(NAME python_serial_service_cpp_concurrent_requests
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_service.py\" \"$<TARGET_FILE:request_response_concurrent_requests>\"; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_serial_service_cpp_concurrent_requests PROPERTIES TIMEOUT 30)

add_test(NAME python_service_cpp_requests_concurrent
   COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"${PYTHON_EXECUTABLE} -q -X faulthandler ./python_request_response_service_concurrent.py\" \"$<TARGET_FILE:request_response_concurrent_requests>\"; }"
   WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_service_cpp_requests_concurrent PROPERTIES TIMEOUT 30)

add_test(NAME pointcloud2_test
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./pointcloud2_test.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_publisher_no_leaks_test
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher_no_leaks_test.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_subscriber_no_leaks_test
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber_no_leaks_test.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_stop_subscriber_test
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_stop_subscriber_test.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME accumulation_test
    COMMAND bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./accumulation_test.py; }"
    WORKING_DIRECTORY "${TEST_PATH}"
)
