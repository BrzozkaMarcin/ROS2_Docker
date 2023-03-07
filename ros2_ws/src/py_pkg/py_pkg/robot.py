#!/usr/bin/env python3
from functools import partial
import math
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Point, PointArray, Pose
from robot_interfaces.srv import GainPoint
 
 
class Robot(Node):
    def __init__(self):
        super().__init__("robot")
        self.declare_parameter("robot_station_x", 0.0)
        self.declare_parameter("robot_station_y", 0.0)
        self.declare_parameter("max_speed", 10.0)
        self.declare_parameter("simu_frequency", 5.0)

        self.station_location = Point()
        self.station_location.x = self.get_parameter("robot_station_x").value
        self.station_location.y = self.get_parameter("robot_station_y").value
        self.station_location.name = 'Robot_Station'

        self.pose_ = Pose()
        self.pose_.x = self.station_location.x
        self.pose_.y = self.station_location.y

        self.closest_point_ = None
        self.points_subscriber_ = self.create_subscription(
            PointArray, "unattained_points", self.callback_points, 10)

        self.max_speed_ = self.get_parameter("max_speed").value
        self.simu_frequency_ = self.get_parameter("simu_frequency").value
        self.d_t = 1.0 / self.simu_frequency_
        self.simu_timer_ = self.create_timer(1.0 / self.simu_frequency_, self.movement_simulate)
        self.get_logger().info("Robot has been started.")

    def movement_simulate(self):
        if self.pose_ == None or self.closest_point_ == None:
            self.get_logger().info("At the RobotStation")
            return
        
        # Determining the distance
        d_x = self.closest_point_.x - self.pose_.x
        d_y = self.closest_point_.y - self.pose_.y
        distance_to_point = math.sqrt(d_x ** 2 + d_y ** 2)
        
        # Change position or gain a point
        if distance_to_point > 0.5:
            v = 2 * distance_to_point
            if v >= self.max_speed_:
                v = self.max_speed_
            distance = self.d_t * v
            if d_x >= 0:
                self.pose_.x += distance * math.cos(d_x/distance_to_point)
            else:
                self.pose_.x -= distance * math.cos(d_x/distance_to_point)
            self.pose_.y += distance * math.sin(d_y/distance_to_point)
            self.get_logger().info("Actual pose:\nx= " + str(self.pose_.x) +
                                   "\ny= " + str(self.pose_.y)) 
        elif self.closest_point_.name != 'Robot_Station':
            self.call_gained_point_server(self.closest_point_.name)
        elif self.closest_point_.name == 'Robot_Station':
            self.closest_point_ = None
            self.get_logger().info("Returned to RobotStation.")

        

    # From the list of points, select the closest one as the target 
    # (or the station in case of an empty list of points).
    def callback_points(self, msg):
        if len(msg.points) > 0:
            closest_point = None
            closest_point_distance = None
            for p in msg.points:
                d_x = p.x - self.pose_.x
                d_y = p.y - self.pose_.y
                dist = math.sqrt(d_x ** 2 + d_y ** 2)
                if closest_point is None or dist < closest_point_distance:
                    closest_point = p
                    closest_point_distance = dist
            self.closest_point_ = closest_point
            self.get_logger().info("Selected Point: " + str(self.closest_point_.name) + ".")
            # self.call_gained_point_server(self.closest_point_.name)
        else:
            self.closest_point_ = self.station_location

    # Establish a connection with the server and report that a point has been gained.
    def call_gained_point_server(self, point_name):
        client = self.create_client(GainPoint, "gain_point")
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for RobotStation...')

        request = GainPoint.Request()
        request.name = point_name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_gained_point_server, name=point_name))

    # Receive confirmation that the point has been removed from the list.
    def callback_gained_point_server(self, future, name):
        try:
            response = future.result()
            self.get_logger().info("Gained a " + str(name) + ".")
            if not response.success:
                self.get_logger().error(str(name) + " could not be gain.")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()