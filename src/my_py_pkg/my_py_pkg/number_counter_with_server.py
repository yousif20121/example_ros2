#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import message and service types
from std_msgs.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")  # Name of the node

        # Create a subscriber
        self.subscriber_ = self.create_subscription(
            msg_type=Int64,
            topic="number",
            callback=self.number_callback,
            qos_profile=10,
        )
        self.number_ = 0

        # Create a publisher
        self.publisher_ = self.create_publisher(
            msg_type=Int64, topic="number_count", qos_profile=10
        )

        # Create a service
        self.server_ = self.create_service(
            srv_type=SetBool,
            srv_name="reset_counter",
            callback=self.callback_reset_counter,
        )

        self.get_logger().info("Number counter has started.")  # Log startup

    def number_callback(self, msg):
        self.number_ += 1
        self.number_publisher()

    def number_publisher(self):
        msg_1 = Int64()
        msg_1.data = self.number_
        self.publisher_.publish(msg_1)

    def callback_reset_counter(self, request, response):
        self.get_logger().info(
            f"Reset counter service called. Reset flag: {request.data}"
        )
        if request.data:  # If request is true, reset the counter
            self.number_ = -1
            self.get_logger().info("Number counter has been reset.")
            response.success = True
            
        else:
            self.get_logger().info("Number counter reset not triggered.")
            response.success = False


        response.message = f"Counter reset status: {request.data}"
        return response


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = NumberCounterNode()  # Create an instance of the node
    rclpy.spin(node)  # Keep the node alive until shutdown
    rclpy.shutdown()  # Shutdown ROS 2 communication


if __name__ == "__main__":
    main()
