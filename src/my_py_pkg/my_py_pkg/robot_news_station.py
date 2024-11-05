#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import message types
from example_interfaces.msg import String


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")  # Name of the node

        self.robot_default_name_ = "C3P0"

        self.declare_parameter(name="robot_name", value=self.robot_default_name_)

        self.robot_name_ = self.get_parameter(name="robot_name").value

        self.publisher_ = self.create_publisher(
            msg_type=String, topic="robot_news", qos_profile=10
        )  # Create a publisher
        self.timer_ = self.create_timer(
            timer_period_sec=0.5, callback=self.publish_news
        )  # Create a timer
        self.get_logger().info("Robot News Station has been started")  # Print statement

    def publish_news(self):
        msg = String()  # Create an object from the message
        msg.data = f"Hi, this is {self.robot_name_} from the robot news station."  # Message to be transmitted
        self.publisher_.publish(msg)  # Publish message


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = RobotNewsStationNode()  # Create an object from the class
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
