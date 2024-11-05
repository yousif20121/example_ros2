from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    number_publisher_node = Node(
        package="my_py_pkg", 
        executable="number_publisher_with_params",
        name="my_num_param",
        remappings=[
            ("number", "my_num_topic")
        ],
        parameters=[
            {"number_to_publish": 100},
            {"publish_frequency": 0.5}
        ]
    )

    ld.add_action(number_publisher_node)

    return ld
