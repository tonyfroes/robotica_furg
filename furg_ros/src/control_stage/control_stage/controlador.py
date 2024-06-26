#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.laser_callback,
            10)
        
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.waypoints = [(9.6, -6.7), (16.3, -1.1)]
        #self.waypoints = [(16.3, -1.1), (9.6, -6.7)]
        
        self.laser_data = None
        self.odom_data = None
        
        self.min_distance_to_obstacle = 0.3
        
        self.timer = self.create_timer(0.1, self.control)
        
        self.position_robot_x = 0
        self.position_robot_y = 0
        self.orientation_robot_yaw = 0.0
        self.current_waypoint_index = 0
        self.all_waypoints_visited = False
        
    def get_yaw_from_orientation(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg) -> None:
        self.odom_data = msg
        self.position_robot_x = msg.pose.pose.position.x
        self.position_robot_y = msg.pose.pose.position.y
        self.orientation_robot_yaw = self.get_yaw_from_orientation(msg.pose.pose.orientation)        
    def laser_callback(self, msg) -> None:
        self.laser_data = msg.ranges
        
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control(self) -> None:
        if self.laser_data is None or self.odom_data is None or self.all_waypoints_visited:
            return

        cmd_vel = Twist()
        
        distance_to_waypoint = math.sqrt(
            (self.position_robot_x - self.waypoints[self.current_waypoint_index][0])**2 + 
            (self.position_robot_y - self.waypoints[self.current_waypoint_index][1])**2
        )
        
        goal_angle = math.atan2(
            self.waypoints[self.current_waypoint_index][1] - self.position_robot_y, 
            self.waypoints[self.current_waypoint_index][0] - self.position_robot_x
        )

        if distance_to_waypoint < 0.5:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('Deu bom!')
                self.all_waypoints_visited = True
                self.publisher_cmd_vel.publish(Twist())
                return

        min_distance_ahead = min(self.laser_data[50:155])
        
        if min_distance_ahead < self.min_distance_to_obstacle:
            cmd_vel.angular.z = 0.5
            cmd_vel.linear.x = 0.0
        else:
            cmd_vel.linear.x = 0.5  # Move forward
            angle_diff = self.normalize_angle(goal_angle - self.orientation_robot_yaw)
            cmd_vel.angular.z = 1.0 * angle_diff

        self.publisher_cmd_vel.publish(cmd_vel)
                
def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
