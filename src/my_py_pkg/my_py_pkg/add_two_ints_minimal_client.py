#!/usr/bin/env python3

# Import dependencies
import rclpy
from rclpy.node import Node

# Import type
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)  # Init ROS 2 communication
    node = Node("add_two_ints_minimal_client")  # Name of the node

    client = node.create_client(
        srv_type=AddTwoInts,
        srv_name="add_two_ints",
    )

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn("Waiting for server Add Two Ints...")  # Print statement

    request = AddTwoInts.Request()  # Request type
    request.a = 3  # Request parameter 1
    request.b = 8  # Request parameter 2

    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        node, future
    )  # Keep alive till response is recieved

    try:
        response = future.result()
        node.get_logger().info(
            f"{request.a} + {request.b} = {response.sum}"
        )  # Print statement

    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")  # Print statement

    rclpy.shutdown()  # Shudtdown ROS 2 communication


if __name__ == "__main__":
    main()
