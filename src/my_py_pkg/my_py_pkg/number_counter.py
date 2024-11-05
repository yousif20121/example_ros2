#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import messages types
from std_msgs.msg import Int64

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")      # Name of the node
        
        self.subscriber_ = self.create_subscription(msg_type=Int64, topic="number", callback=self.number_callback, qos_profile=10)  # Create a subscriber
        self.number_ = 0

        self.publisher_ = self.create_publisher(msg_type=Int64, topic="number_count", qos_profile=10)

        self.get_logger().info("Number counter has started.")    # Print statement
        
    def number_callback(self, msg):
        self.number_ += 1
        self.number_publisher()
        
    def number_publisher(self):
        msg_1 = Int64()
        msg_1.data = self.number_
        self.publisher_.publish(msg_1)

def main(args=None):
    rclpy.init(args=args)                        # Init ROS 2 communication
    node = NumberCounterNode()                   # Create an object from the class
    rclpy.spin(node)                             # Keep alive till CTRL + C
    rclpy.shutdown()                             # Shudtdown ROS 2 communication

if __name__ == "__main__":
    main()