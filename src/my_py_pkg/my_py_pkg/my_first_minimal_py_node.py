#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)                        # Init ROS 2 communication
    node = Node("my_first_minimal_py_node")         # Name of the node
    ##################################################

    node.get_logger().info("ROS2: Hello from minimal python node")   # Print statement

    rclpy.spin(node)                             # Keep alive till CTRL + C

    ##################################################
    rclpy.shutdown()                             # Shudtdown ROS 2 communication

if __name__ == "__main__":
    main()