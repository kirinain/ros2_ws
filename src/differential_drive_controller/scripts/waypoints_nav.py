#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from tf_transformations import euler_from_quaternion

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_1_x', 2.0),
                ('waypoint_1_y', 1.0),
                ('waypoint_2_x', 4.0),
                ('waypoint_2_y', 3.0),
                ('kp', 0.5),
                ('ki', 0.0),
                ('kd', 0.1),
                ('safe_distance', 0.5)  # Stop if obstacle is within 0.5 meters
            ]
        )
        
        # Get waypoints
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]

        # self.waypoints = [
        #     (-1.3359988, -0.4163493),  # Current Position from Odometry
        #     (-1.0, -0.3),  # Example: A point slightly ahead
        #     (0.0, 0.0),  # Example: Another waypoint in the house
        # ]

        # PID gains
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        # for avoiding
        self.safe_distance = self.get_parameter('safe_distance').value
        self.obstacle_detected = False
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.current_waypoint = 0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.distance_threshold = 0.1  #
        
    
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        #timer loop
        self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('Waypoint navigator initialized with obstacle avoidance!')
        self.get_logger().info(f'Waypoints: {self.waypoints}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

    def scan_callback(self, msg):
        """Detect obstacles in front of the robot."""
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < self.safe_distance

        if self.obstacle_detected:
            self.get_logger().info(f"Obstacle detected at {min_distance:.2f}m! Stopping.")

    def get_distance_to_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            return 0.0
        dx = self.waypoints[self.current_waypoint][0] - self.current_x
        dy = self.waypoints[self.current_waypoint][1] - self.current_y
        return math.sqrt(dx*dx + dy*dy)

    def get_angle_to_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            return 0.0
        dx = self.waypoints[self.current_waypoint][0] - self.current_x
        dy = self.waypoints[self.current_waypoint][1] - self.current_y
        target_yaw = math.atan2(dy, dx)
        error = target_yaw - self.current_yaw
        
        
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        return error

    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            self.stop_robot()
            return

        # Stop if an obstacle is detected
        # if self.obstacle_detected:
        #     self.stop_robot()
        #     return

        distance = self.get_distance_to_waypoint()
        angle_error = self.get_angle_to_waypoint()

        if distance < self.distance_threshold:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint + 1}!')
            self.current_waypoint += 1
            self.error_sum = 0.0
            self.last_error = 0.0
            if self.current_waypoint >= len(self.waypoints):
                self.get_logger().info('All waypoints reached!')
                self.stop_robot()
                return

        
        self.error_sum += angle_error
        error_diff = angle_error - self.last_error
        angular_velocity = (self.kp * angle_error + 
                          self.ki * self.error_sum + 
                          self.kd * error_diff)
        self.last_error = angle_error

        # aligned linear velocity
        linear_velocity = 0.3 * (1 - abs(angle_error) / math.pi)
        if abs(angle_error) > math.pi/4:  #angle too large
            linear_velocity = 0.0

        
        cmd_vel = Twist()
    
        cmd_vel.linear.x = max(0.0, min(linear_velocity, 0.15))  # Reduce max speed to 0.15 m/s
        cmd_vel.angular.z = max(0.0, min(angular_velocity, 0.0))
        self.cmd_vel_pub.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
