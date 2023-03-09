from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    points_spawner = Node(
        package="py_pkg",
        executable="points_spawner",
        name="my_robot_station",
        parameters=[
            {"simultaneously_numbers": 2},
            {"spawn_time_period": 10.0}
        ]
        )

    ld.add_action(points_spawner) 
    return ld