#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped
# from tf_transformations import euler_from_quaternion


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('target_speed', 4.0) # get speed from autonomous demo
        self.declare_parameter('wheelbase', 1.534)
        self.declare_parameter('max_steering_angle', 0.366)
        
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        
        # State variables
        self.current_pose = None
        self.current_path = None
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.ackermann_pub = self.create_publisher(
            AckermannDrive,
            '/ackermann_cmd',
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Pure Pursuit Controller initialized')
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def path_callback(self, msg):
        """Store the received path"""
        self.current_path = msg
        self.get_logger().info(f'Received path with {len(msg.poses)} poses')
    
    def odom_callback(self, msg):
        """Store current pose from odometry"""
        self.current_pose = msg.pose.pose
    
    def get_lookahead_point(self):
        """Find the lookahead point on the path"""
        if not self.current_path or not self.current_pose:
            return None
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Find the closest point on the path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(self.current_path.poses):
            dist = math.sqrt(
                (pose.pose.position.x - current_x)**2 + 
                (pose.pose.position.y - current_y)**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Find lookahead point starting from closest point
        for i in range(closest_idx, len(self.current_path.poses)):
            pose = self.current_path.poses[i]
            dist = math.sqrt(
                (pose.pose.position.x - current_x)**2 + 
                (pose.pose.position.y - current_y)**2
            )
            
            if dist >= self.lookahead_distance:
                return pose.pose.position
        
        # If no point found at lookahead distance, return the last point
        if len(self.current_path.poses) > 0:
            return self.current_path.poses[-1].pose.position
        
        return None
    
    def calculate_steering_angle(self, lookahead_point):
        """Calculate steering angle using pure pursuit algorithm"""
        if not self.current_pose or not lookahead_point:
            return 0.0
        
        # Get current position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Convert quaternion to yaw angle
        orientation_q = self.current_pose.orientation
        _, _, yaw = self.quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        
        # Calculate vector from current position to lookahead point
        dx = lookahead_point.x - current_x
        dy = lookahead_point.y - current_y
        
        # Transform to vehicle coordinate system
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        # Calculate distance to lookahead point
        lookahead_dist = math.sqrt(local_x**2 + local_y**2)
        
        if lookahead_dist < 0.1:  # Avoid division by zero
            return 0.0
        
        # Pure pursuit formula
        curvature = 2 * local_y / (lookahead_dist**2)
        steering_angle = math.atan(self.wheelbase * curvature)
        
        # Clamp steering angle
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        return 1*steering_angle
    
    def control_loop(self):
        """Main control loop"""
        if not self.current_pose or not self.current_path:
            return
        
        # Check if path is empty
        if len(self.current_path.poses) == 0:
            self.get_logger().warn('Empty path received')
            return
        
        # Get lookahead point
        lookahead_point = self.get_lookahead_point()
        
        if lookahead_point is None:
            self.get_logger().warn('Could not find lookahead point')
            return
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(lookahead_point)
        
        # Check if we've reached the end of the path
        last_pose = self.current_path.poses[-1].pose.position
        dist_to_goal = math.sqrt(
            (last_pose.x - self.current_pose.position.x)**2 + 
            (last_pose.y - self.current_pose.position.y)**2
        )
        
        # Reduce speed when approaching goal
        speed = self.target_speed
        if dist_to_goal < 1.0:
            speed = max(0.5, self.target_speed * (dist_to_goal / 1.0))
        
        if dist_to_goal < 0.2:
            speed = 0.0
        
        # Create and publish Ackermann command
        ackermann_msg = AckermannDrive()
        # ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        # ackermann_msg.header.frame_id = 'base_link'
        
        ackermann_msg.speed = speed
        ackermann_msg.steering_angle = steering_angle
        
        self.ackermann_pub.publish(ackermann_msg)
        
        # Log debug info
        self.get_logger().debug(
            f'Speed: {speed:.2f}, Steering: {steering_angle:.3f}, '
            f'Distance to goal: {dist_to_goal:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    controller = PurePursuitController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()