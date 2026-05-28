set(TEST_PATH "${CMAKE_CURRENT_BINARY_DIR}")

# Install dependencies, copy tests and all dependencies to the same test directory.
# Suppressing errors as packages can be installed manually, as the case in CI.
# On Windows, pywin32 is also required at runtime: Fast-DDS-Python 2.x's generated
# `fastdds.py` calls `win32api.LoadLibrary('fastdds-X.Y.dll')` at import time on
# Windows, and without it every Python test fails with ModuleNotFoundError.
if(WIN32)
    set(PYTHON_TEST_DEPS "'numpy>=1.16', 'transforms3d>=0.4.1', 'pywin32'")
else()
    set(PYTHON_TEST_DEPS "'numpy>=1.16', 'transforms3d>=0.4.1'")
endif()
add_custom_target(python_tests ALL
    COMMAND ${PYTHON_EXECUTABLE} -c "import subprocess, sys$<SEMICOLON> subprocess.call([sys.executable, '-m', 'pip', 'install', ${PYTHON_TEST_DEPS}, '--quiet'])"
    VERBATIM
)
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
if(WIN32 AND PROVIZIO_DDS_PYTHON_EXTRA_DLLS)
    foreach(TGT ${PROVIZIO_DDS_PYTHON_EXTRA_DLLS})
        add_custom_command(TARGET python_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${TGT}>" "${TEST_PATH}"
        )
    endforeach()
endif()
add_dependencies(python_tests provizio_dds_python_types)
if(FAST_DDS_DEPENDENCIES)
    add_dependencies(python_tests ${FAST_DDS_DEPENDENCIES})
endif(FAST_DDS_DEPENDENCIES)

add_test(NAME python_pub_python_sub COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
add_test(NAME python_pub_python_sub_type_reuse COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher_type_reuse.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber_type_reuse.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_pub_cpp_sub COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py"
    "\"$<TARGET_FILE:simplest_subscriber>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_cpp_sub PROPERTIES TIMEOUT 20)

add_test(NAME cpp_pub_python_sub COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:simplest_publisher>\""
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(cpp_pub_python_sub PROPERTIES TIMEOUT 20)

add_test(NAME python_request_response COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response PROPERTIES TIMEOUT 30)

add_test(NAME python_request_response_async COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service_concurrent.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_async PROPERTIES TIMEOUT 30)

# Add inter-iteration delay to let the OS release DDS resources between rapid
# participant create/destroy cycles.
set(PYTHON_RELIABILITY_DELAY_ARGS "--delay" "0.5")

add_test(NAME python_request_response_reliability COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_times.py" ${PYTHON_RELIABILITY_DELAY_ARGS} "--timeout" "15" "128"
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_service.py \"\" {i} 1 {i}"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_client.py \"\" {i} {i}"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_reliability PROPERTIES TIMEOUT 2000) # Takes long by design

add_test(NAME python_request_response_reliability_single_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_service.py _single_service 1 128 200"
    "\"${PYTHON_EXECUTABLE}\" \"${CMAKE_SOURCE_DIR}/test/run_times.py\" 128 \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_client.py _single_service {i} 200"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_reliability_single_service PROPERTIES TIMEOUT 3500) # Takes long by design

add_test(NAME python_request_response_ignore COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_service.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
add_test(NAME python_request_response_ignore_async COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_service_async.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_request_response_dual_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_dual_service.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_dual_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\""
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_response_concurrent_requests_serial_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\""
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_concurrent_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_response_concurrent_requests_serial_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_service_concurrent COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_concurrent_service>\" 5"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_concurrent_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_service_concurrent PROPERTIES TIMEOUT 30)

add_test(NAME python_service_cpp_client COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"$<TARGET_FILE:request_response_client>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_service_cpp_client PROPERTIES TIMEOUT 30)

add_test(NAME python_serial_service_cpp_concurrent_requests COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"$<TARGET_FILE:request_response_concurrent_requests>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_serial_service_cpp_concurrent_requests PROPERTIES TIMEOUT 30)

add_test(NAME python_service_cpp_requests_concurrent COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service_concurrent.py"
    "\"$<TARGET_FILE:request_response_concurrent_requests>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_service_cpp_requests_concurrent PROPERTIES TIMEOUT 30)

add_test(NAME pointcloud2_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./pointcloud2_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_publisher_no_leaks_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher_no_leaks_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_subscriber_no_leaks_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber_no_leaks_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_stop_subscriber_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_stop_subscriber_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME accumulation_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./accumulation_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

# 1.10.x Python API source-compat: ensures `ReturnCode_t.RETCODE_OK`,
# `pub_sub_type.getName()`, and other camelCase / enum-class spellings
# carried over from Fast-DDS-python 1.x still resolve after the 2.x
# migration. Pure import-level checks, no DDS network activity.
add_test(NAME legacy_api_compat_test COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler ./legacy_api_compat_test.py
    WORKING_DIRECTORY "${TEST_PATH}"
)

# Cross-version wire-interop: spin up the current (2.x) build and the
# pip-installed 1.10.1 reference side-by-side and exchange traffic in
# every direction (pub<->sub, req<->svc). Skips itself when the legacy
# venv isn't installed, so dev machines without it stay green; CI sets
# PROVIZIO_DDS_LEGACY_PYTHON in its provizio_dds.yml workflow after
# running setup_legacy_provizio_dds_venv.sh. See the runner script's
# docstring for the full matrix.
add_test(NAME cross_version_compat_test COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler ./cross_version_compat_test.py
    WORKING_DIRECTORY "${TEST_PATH}"
)
# `SKIPPED:` is the marker the test prints when `PROVIZIO_DDS_LEGACY_PYTHON`
# is unset or the legacy interpreter can't import provizio_dds. With this
# regex set, ctest reports the test as "Skipped" rather than "Passed" so a
# missing setup step on a CI runner shows up loudly in the output.
set_tests_properties(cross_version_compat_test PROPERTIES
    TIMEOUT 240
    SKIP_REGULAR_EXPRESSION "^SKIPPED:")

# Platform-specific timeouts: tighter on Linux/macOS to detect regressions,
# wider on Windows where DDS discovery and process startup are slower.
# "Takes long by design" tests keep the same timeout on all platforms.
if(WIN32)
    set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 45)
    set_tests_properties(python_pub_python_sub_type_reuse PROPERTIES TIMEOUT 30)
    set_tests_properties(
        python_request_response_ignore
        python_request_response_ignore_async
        PROPERTIES TIMEOUT 60)
else()
    set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 20)
    set_tests_properties(python_pub_python_sub_type_reuse PROPERTIES TIMEOUT 20)
    set_tests_properties(
        python_request_response_ignore
        python_request_response_ignore_async
        PROPERTIES TIMEOUT 30)
endif()

if(WIN32)
    set(PYTHON_TEST_NAMES
        python_pub_python_sub python_pub_python_sub_type_reuse
        python_pub_cpp_sub cpp_pub_python_sub
        python_request_response python_request_response_async
        python_request_response_reliability python_request_response_reliability_single_service
        python_request_response_ignore python_request_response_ignore_async
        python_request_response_dual_service
        python_request_cpp_service python_request_cpp_response_concurrent_requests_serial_service
        python_request_cpp_service_concurrent python_service_cpp_client
        python_serial_service_cpp_concurrent_requests python_service_cpp_requests_concurrent
        pointcloud2_test python_publisher_no_leaks_test python_subscriber_no_leaks_test
        python_stop_subscriber_test accumulation_test
        legacy_api_compat_test cross_version_compat_test
    )
    # PATH: DLLs must be locatable at runtime.
    set_tests_properties(${PYTHON_TEST_NAMES} PROPERTIES ENVIRONMENT
        "PATH=${PROVIZIO_DDS_TEST_PATH}")
endif(WIN32)
