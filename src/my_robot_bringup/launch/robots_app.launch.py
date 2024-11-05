from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_news_node_1 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="my_robo_1",
        parameters=[{"robot_name": "1"}],
    )

    robot_news_node_2 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="my_robo_2",
        parameters=[{"robot_name": "2"}],
    )

    robot_news_node_3 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="my_robo_3",
        parameters=[{"robot_name": "3"}],
    )

    robot_news_node_4 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="my_robo_4",
        parameters=[{"robot_name": "4"}],
    )

    robot_news_node_5 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="my_robo_5",
        parameters=[{"robot_name": "5"}],
    )
    
    smartphone = Node(
        package="my_py_pkg",
        executable="smartphone",
        name="smartphone",
    )    

    ld.add_action(robot_news_node_1)
    ld.add_action(robot_news_node_2)
    ld.add_action(robot_news_node_3)
    ld.add_action(robot_news_node_4)
    ld.add_action(robot_news_node_5)
    ld.add_action(smartphone)

    return ld
