#!/usr/bin/env python3

# Import ROS2 dependencies
import rclpy
from rclpy.node import Node

# Import importatnt libs
from functools import partial

# Import message/services
from my_robot_interfaces.srv import SetLed


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")  # Name of the node

        self.battery_state_ = "full"

        self.last_battery_change_ = self.get_current_time_seconds()

        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)

        self.get_logger().info("Battery node has started.")

    def get_current_time_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now - self.last_battery_change_ > 4.0:
                self.battery_state_ = "empty"
                self.get_logger().info("Battery is empty! Charging battery...")
                self.last_battery_change_ = time_now
                self.call_set_led_server(3, 1)
        else:
            if time_now - self.last_battery_change_ > 6.0:
                self.battery_state_ = "full"
                self.get_logger().info("Battery is full again.")
                self.last_battery_change_ = time_now
                self.call_set_led_server(3, 0)

    def call_set_led_server(self, led_id, state):
        client = self.create_client(
            srv_type=SetLed,
            srv_name="set_led",
        )

        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for server Set Led")  # Print statement

        request = SetLed.Request()  # Request type
        request.led_id = led_id  # Request parameter 1
        request.state = state  # Request parameter 2

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_set_led, led_id=led_id, state=state))

    def callback_call_set_led(self, future, led_id, state):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))  # Print statement

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")  # Print statement


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = BatteryNode()
    rclpy.spin(node)  # Keep alive till CTRL + C
    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
