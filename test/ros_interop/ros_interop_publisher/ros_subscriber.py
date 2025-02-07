# Copyright 2024 Provizio Ltd.
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

"""
Tests if ros_interop_publisher-sent messages can be received by ROS2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

TIMEOUT_SEC = 8
TOPIC = "chatter"
EXPECTED_MESSAGE = "provizio_test_ros_interop_publisher_says_hi"
NODE_NAME = "ros_interop_publisher_ros_subscriber"


class Subscriber(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            String, TOPIC, self.listener_callback, 1
        )

    def listener_callback(self, msg):
        if msg.data == EXPECTED_MESSAGE:
            print(f"{NODE_NAME} has received the expected message: {msg.data}")
            sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()
    time_started = time.time()
    while rclpy.ok() and (time.time() - time_started) < TIMEOUT_SEC:
        rclpy.spin_once(subscriber, timeout_sec=0.1)

    subscriber.destroy_node()
    rclpy.shutdown()

    print(f"{NODE_NAME} has NOT received the expected message!")
    sys.exit(1)


if __name__ == "__main__":
    main()
