#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node
from functools import partial

# Import type
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_oop_client")  # Name of the node
        self.call_add_two_ints_server(a=6, b=7)

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(
            srv_type=AddTwoInts,
            srv_name="add_two_ints",
        )

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Waiting for server Add Two Ints..."
            )  # Print statement

        request = AddTwoInts.Request()  # Request type
        request.a = a  # Request parameter 1
        request.b = b  # Request parameter 2

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")  # Print statement

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")  # Print statement


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = AddTwoIntsClientNode()
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
