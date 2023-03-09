#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Point, PointArray
from robot_interfaces.srv import GainPoint, SpawnPoints

 
class RobotStation(Node):
    def __init__(self):
        super().__init__("robot_station")
        self.declare_parameter("number_of_points", 5)
        self.spawn_number_of_points_ = self.get_parameter("number_of_points").value
        self.point_list_ = []
        self.points_publisher_ = self.create_publisher(
            PointArray, "unattained_points", 10)
        
        self.gain_point_service_ = self.create_service(
            GainPoint, "gain_point", self.callback_gained_point)
        self.spawn_points_service_ = self.create_service(
            SpawnPoints, "spawn_points", self.callback_spawn_points)

        self.get_logger().info("RobotStation has been started.")
        # self.generate_points()
        # self.timer_ = self.create_timer(10.0, self.publish_points)

    # Spawn new points
    def callback_spawn_points(self, request, response):
        for point in request.points:
            self.point_list_.append(point)
        self.publish_points()
        response.success = True
        return response

    # Generate random points to gain.
    def generate_points(self):
        for i in range(self.spawn_number_of_points_):
            new_point = Point()
            new_point.x = random.uniform(0.0, 100.0)
            new_point.y = random.uniform(0.0, 100.0)
            new_point.name = "Point_" + str(i+1)
            self.point_list_.append(new_point)
        self.publish_points()
    
    # Publish a topic with a list of points.
    def publish_points(self):
        msg = PointArray()
        msg.points = self.point_list_
        if len(self.point_list_) > 0:
            self.get_logger().info("Publishing " + str(len(self.point_list_)) + " unattained points.")
        else:
            self.get_logger().info("All points gained.")
        self.points_publisher_.publish(msg)

    # Remove the point from the list and confirm the operation.
    def callback_gained_point(self, request, response):
        for (i, point) in enumerate(self.point_list_):
                if point.name == request.name:
                    del self.point_list_[i]
                    self.publish_points()
                    break
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotStation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()