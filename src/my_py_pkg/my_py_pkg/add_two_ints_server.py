#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import type
from example_interfaces.srv import AddTwoInts


class MyNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")  # Name of the node
        self.server_ = self.create_service(
            srv_type=AddTwoInts,
            srv_name="add_two_ints",
            callback=self.callback_add_two_ints,
        )  # Create a service

        self.get_logger().info("Add two ints server has started")  # Print statement

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b  # Service operation
        self.get_logger().info(
            f"{request.a} + {request.b} = {response.sum}"
        )  # Print statement
        return response


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = MyNode()  # Create an object from the class
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
