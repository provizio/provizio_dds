import rclpy
import asyncio
from rclpy.node import Node
from std_srvs.srv import SetBool


class ros_interop_client(Node):
    NAME = "ros_interop_client"
    SERVICE_NAME = "provizio_dds_test_ros_interop_service"
    MATCHING_ITERATION_TIMEOUT = 0.4
    RESPONSE_ITERATION_TIMEOUT = 0.4
    RETRIES = 10

    def __init__(self):
        super().__init__(ros_interop_client.NAME)
        self.client = self.create_client(SetBool, ros_interop_client.SERVICE_NAME)

        # Wait till the service is matched
        for i in range(ros_interop_client.RETRIES):
            if not self.client.wait_for_service(
                timeout_sec=ros_interop_client.MATCHING_ITERATION_TIMEOUT
            ):
                if i == ros_interop_client.RETRIES - 1:
                    error_message = f"{ros_interop_client.NAME}: Service not available!"
                    self.get_logger().error(error_message)
                    raise Exception(error_message)

        self.request_true = SetBool.Request()
        self.request_true.data = True

        self.request_false = SetBool.Request()
        self.request_false.data = False

    async def send_requests(self):
        await self.request_and_check(self.request_true)
        await self.request_and_check(self.request_false)

        self.get_logger().info(
            f"{ros_interop_client.NAME}: Successfully got all the expected responses!"
        )

    async def request_and_check(self, request):
        self.get_logger().info(
            f"{ros_interop_client.NAME}: Requesting with {request.data}"
        )

        future_response = self.client.call_async(request)

        for i in range(ros_interop_client.RETRIES):
            rclpy.spin_until_future_complete(
                self,
                future_response,
                timeout_sec=ros_interop_client.RESPONSE_ITERATION_TIMEOUT,
            )
            if future_response.done():
                break

            if i == ros_interop_client.RETRIES - 1:
                error_message = (
                    f"{ros_interop_client.NAME}: Timeout waiting for the response!"
                )
                self.get_logger().error(error_message)
                raise Exception(error_message)

        if future_response.result() is not None:
            response = future_response.result()
            self.get_logger().info(
                f"{ros_interop_client.NAME}: Received response: success={response.success}, message={response.message}"
            )

            if response.success != request.data:
                raise Exception(
                    f"{ros_interop_client.NAME}: unexpected response.success={response.success}!"
                )

        else:
            raise Exception(f"{ros_interop_client.NAME}: Got no response")


async def main(args=None):
    rclpy.init(args=args)

    set_bool_client = ros_interop_client()

    await set_bool_client.send_requests()

    del set_bool_client

    rclpy.shutdown()


asyncio.run(main())
