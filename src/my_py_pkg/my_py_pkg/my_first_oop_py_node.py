#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("my_first_oop_py_node")  # Name of the node
        self.get_logger().info("ROS2: Hello from oop python node")  # Print statement


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = MyNode()
    ##################################################

    rclpy.spin(node)  # Keep alive till CTRL + C

    ##################################################
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
