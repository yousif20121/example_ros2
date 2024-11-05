#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import message types
from my_robot_interfaces.msg import LedStateArray
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")  # Name of the node

        self.default_led_states = [0, 0, 0]

        self.declare_parameter(name="led_states", value=self.default_led_states)

        self.led_states_ = self.get_parameter("led_states").value

        self.led_states_publisher_ = self.create_publisher(
            LedStateArray, "led_states", 10
        )
        self.led_states_timer_ = self.create_timer(4.0, self.publish_led_states)

        self.get_logger().info("Led panel node has started.")

        self.led_states_server = self.create_service(
            SetLed, "set_led", self.set_led_callback
        )

    def publish_led_states(self):
        msg = LedStateArray()
        msg.led_states = self.led_states_
        self.led_states_publisher_.publish(msg)

    def set_led_callback(self, request, response):
        self.get_logger().info("Set Led service has been called.")
        led_id = request.led_id
        state = request.state

        if led_id > len(self.led_states_) or led_id <= 0 or state not in [0, 1]:
            response.success = False
            return response

        self.led_states_[led_id - 1] = state
        response.success = True
        self.publish_led_states()
        return response


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = LedPanelNode()
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
