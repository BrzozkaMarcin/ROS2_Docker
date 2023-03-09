#!/usr/bin/env python3
from functools import partial
import random
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Point, PointArray
from robot_interfaces.srv import SpawnPoints

 
class PointsSpawner(Node):
    def __init__(self):
        super().__init__("points_spawner")
        self.declare_parameter("simultaneously_numbers", 2)
        self.declare_parameter("spawn_time_period", 10.0)
        self.simultaneously_numbers_ = self.get_parameter("simultaneously_numbers").value
        self.spawn_time_period_ = self.get_parameter("spawn_time_period").value
        self.next_point_id_ = 0
        
        self.get_logger().info("PointsSpawner has been started.")
        self.generate_points()
        self.spawn_timer_ = self.create_timer(self.spawn_time_period_, self.generate_points)

    # Generate random points to gain.
    def generate_points(self):
        point_list = []
        for i in range(self.simultaneously_numbers_):
            new_point = Point()
            new_point.x = random.uniform(0.0, 100.0)
            new_point.y = random.uniform(0.0, 100.0)
            new_point.name = "Point_" + str(self.next_point_id_)
            self.next_point_id_ += 1
            point_list.append(new_point)
            points = PointArray()
            points = point_list
        self.call_spawn_points_server(points)

    # Establish a connection with the server and spawn new points.
    def call_spawn_points_server(self, point_list):
        client = self.create_client(SpawnPoints, "spawn_points")
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for RobotStation...')

        request = SpawnPoints.Request()
        request.points = point_list
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_points_server, points=point_list))

    # Receive confirmation that the points has been added.
    def callback_spawn_points_server(self, future, points):
        try:
            response = future.result()
            self.get_logger().info("Added new point(s).")
            if not response.success:
                self.get_logger().error("Points could not be added.")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = PointsSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()