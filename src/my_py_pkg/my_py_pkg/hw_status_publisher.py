#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import message types
from my_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hw_status_publisher")  # Name of the node

        self.hw_status_publisher_ = self.create_publisher(
            msg_type=HardwareStatus, topic="hardware_status", qos_profile=10
        )

        self.timer_ = self.create_timer(
            timer_period_sec=1.0, callback=self.publish_hw_status
        )

        self.get_logger().info(
            "Hardware status publisher has been started."
        )  # Print statement

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special here"
        self.hw_status_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = HardwareStatusPublisherNode()
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
