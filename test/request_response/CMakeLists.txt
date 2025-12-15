# Copyright 2025 Provizio Ltd.
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

add_subdirectory(request_response_service)
add_subdirectory(request_response_client)
add_subdirectory(request_response_dual_service)

# TODO: Windows tests

add_test(NAME request_response COMMAND
    bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_service> serial_requests\" \"$<TARGET_FILE:request_response_client>\"; }"
)

add_test(NAME request_response_reliability COMMAND
    bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_times.sh 128 ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \\\"$<TARGET_FILE:request_response_reliability_service> \\\\\\\"\\\\\\\" \\\$i 1\\\" \\\"$<TARGET_FILE:request_response_reliability_client> \\\\\\\"\\\\\\\" \\\$i\\\"; }"
)
set_tests_properties(request_response_reliability PROPERTIES TIMEOUT 1500) # Takes long by design

add_test(NAME request_response_reliability_single_service COMMAND
    bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_reliability_service> _single_service 1 128\" \"${CMAKE_SOURCE_DIR}/test/run_times.sh 128 $<TARGET_FILE:request_response_reliability_client> _single_service \\\\\\$i\"; }"
)
set_tests_properties(request_response_reliability_single_service PROPERTIES TIMEOUT 1500) # Takes long by design

add_test(NAME request_response_concurrent_requests_serial_service COMMAND
    bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_service>\" \"$<TARGET_FILE:request_response_concurrent_requests>\"; }"
)

add_test(NAME request_response_dual_service_test COMMAND
    bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && $<TARGET_FILE:request_response_dual_service>"
)

add_test(NAME request_response_ignore COMMAND
    bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_ignore_service>\" \"$<TARGET_FILE:request_response_ignore_client>\"; }"
)

add_test(NAME request_response_ignore_async COMMAND
    bash -c "source ${CMAKE_SOURCE_DIR}/test/unset_ros.sh && { ${CMAKE_SOURCE_DIR}/test/run_multiple.sh \"$<TARGET_FILE:request_response_ignore_service_async>\" \"$<TARGET_FILE:request_response_ignore_client>\"; }"
)
