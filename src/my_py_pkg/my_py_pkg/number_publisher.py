#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import messages types
from std_msgs.msg import Int64


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")  # Name of the node

        self.publisher_ = self.create_publisher(
            msg_type=Int64, topic="number", qos_profile=10
        )  # Create a publisher
        self.timer_ = self.create_timer(
            timer_period_sec=1, callback=self.publish_number
        )  # Create a timer
        self.number_ = 0

        self.get_logger().info("Number publisher has started")  # Print statement

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)
        self.number_ += 1


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = NumberPublisherNode()  # Create an object from the class
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
