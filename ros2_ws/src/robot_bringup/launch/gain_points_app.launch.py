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

    robot = Node(
        package="py_pkg",
        executable="robot",
        name="my_robot",
        parameters=[
            {"robot_station_x": 0.0},
            {"robot_station_y": 0.0},
            {"max_speed": 10.0},
            {"simu_frequency": 5.0}
        ]
        )

    ld.add_action(robot_station) 
    ld.add_action(robot)
    return ld