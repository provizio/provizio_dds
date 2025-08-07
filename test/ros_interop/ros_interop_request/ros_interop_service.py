# Copyright 2025 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
import sys


class TestRosInteropService(Node):
    NAME = "test_ros_interop_service"
    TIMEOUT = 5.0
    TIME_TO_SEND_LAST_RESPONSE = 0.5
    REQUESTS_EXPECTED = 2

    def __init__(self):
        super().__init__(TestRosInteropService.NAME)
        self.response_count = 0
        self.srv = self.create_service(
            SetBool, "provizio_dds_test_ros_interop_request", self.service_callback
        )
        self.get_logger().info(f"{TestRosInteropService.NAME}: Waiting for requests...")

    def service_callback(self, request, response):
        response.success = request.data
        response.message = f"{request.data} for you from {TestRosInteropService.NAME}"

        self.response_count += 1

        self.get_logger().info(
            f"{TestRosInteropService.NAME}: Sent {response.success}!"
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    service_node = TestRosInteropService()
    start_time = time.time()
    single_spin_timeout = 0.1

    while (
        rclpy.ok()
        and service_node.response_count < TestRosInteropService.REQUESTS_EXPECTED
        and (time.time() - start_time) < TestRosInteropService.TIMEOUT
    ):
        rclpy.spin_once(service_node, timeout_sec=single_spin_timeout)

    error_code = 0
    if service_node.response_count >= TestRosInteropService.REQUESTS_EXPECTED:
        # First, let it send the last response
        start_time = time.time()
        while (
            rclpy.ok()
            and (time.time() - start_time)
            < TestRosInteropService.TIME_TO_SEND_LAST_RESPONSE
        ):
            rclpy.spin_once(service_node, timeout_sec=single_spin_timeout)

        service_node.get_logger().info(
            f"{TestRosInteropService.NAME}: Successfully sent all responses"
        )
    else:
        service_node.get_logger().error(f"{TestRosInteropService.NAME}: Timeout!")
        error_code = 0

    service_node.destroy_node()
    rclpy.shutdown()
    sys.exit(error_code)


main()
