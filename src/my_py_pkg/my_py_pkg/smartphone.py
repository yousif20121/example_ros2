#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import message types
from example_interfaces.msg import String


class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")  # Name of the node

        self.subscriber_ = self.create_subscription(
            msg_type=String,
            topic="robot_news",
            callback=self.callback_robot_news,
            qos_profile=10,
        )  # Create a subscriber
        self.get_logger().info("Smartphone has been started.")  # Print statement

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)  # Print statement


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = SmartphoneNode()  # Create an object from the class
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
