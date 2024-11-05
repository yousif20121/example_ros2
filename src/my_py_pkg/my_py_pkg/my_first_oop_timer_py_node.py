#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("my_first_oop_timer_py_node")                        # Name of the node

        self.get_logger().info("ROS2: Hello from oop & timer python node")   # Print statement

        self.create_timer(timer_period_sec=0.5, callback=self.timer_callback)   # Create a timer specifying frequency & callback function

        self.counter = 0    # Create a counter



    def timer_callback(self):
        self.get_logger().info(f"Counter = {self.counter}")
        self.counter +=1


def main(args=None):
    rclpy.init(args=args)                        # Init ROS 2 communication
    node = MyNode()
    ##################################################

    rclpy.spin(node)                             # Keep alive till CTRL + C

    ##################################################
    rclpy.shutdown()                             # Shudtdown ROS 2 communication

if __name__ == "__main__":
    main()