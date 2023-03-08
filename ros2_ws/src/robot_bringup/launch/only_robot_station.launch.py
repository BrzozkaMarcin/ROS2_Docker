from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_station = Node(
        package="py_pkg",
        executable="robot_station",
        name="my_robot_station",
        parameters=[
            {"number_of_points": 5}
        ]
        )

    ld.add_action(robot_station) 
    return ld