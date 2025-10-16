#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
import math


class AckermannToCmdVelNode(Node):
    def __init__(self):
        super().__init__('ackermann_to_cmdvel_node')
        
        # Declare parameters
        self.declare_parameter('wheelbase', 1.534)  # Distance between front and rear axles (meters)
        self.declare_parameter('max_steering_angle', 0.4)  # Maximum steering angle (radians)
        self.declare_parameter('input_topic', '/ackermann_cmd')
        self.declare_parameter('output_topic', '/cmd_vel')
        
        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # Create subscriber and publisher
        self.ackermann_sub = self.create_subscription(
            AckermannDrive,
            input_topic,
            self.ackermann_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            output_topic,
            10
        )
        
        self.get_logger().info(f'Ackermann to Cmd_Vel converter started')
        self.get_logger().info(f'Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'Max steering angle: {self.max_steering_angle} rad')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Output topic: {output_topic}')

    def ackermann_callback(self, msg):
        """
        Convert Ackermann drive command to Twist message
        
        Ackermann steering geometry:
        - Linear velocity is directly transferred
        - Angular velocity is calculated using: w = v * tan(steering_angle) / wheelbase
        """
        try:
            # Extract values from Ackermann message
            linear_velocity = msg.speed
            steering_angle = msg.steering_angle
            
            # Clamp steering angle to maximum allowed
            steering_angle = max(-self.max_steering_angle, 
                                min(self.max_steering_angle, steering_angle))
            
            # Calculate angular velocity using Ackermann steering geometry
            # w = v * tan(δ) / L
            # where w = angular velocity, v = linear velocity, δ = steering angle, L = wheelbase
            if abs(steering_angle) < 1e-6:  # Avoid division by zero for very small angles
                angular_velocity = 0.0
            else:
                angular_velocity = linear_velocity * math.tan(steering_angle) / self.wheelbase
            
            # Create and publish Twist message
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = angular_velocity
            
            self.cmd_vel_pub.publish(twist_msg)
            
            # Log conversion details (optional, can be commented out for performance)
            self.get_logger().debug(
                f'Converted: speed={linear_velocity:.3f} m/s, '
                f'steering={steering_angle:.3f} rad -> '
                f'linear.x={linear_velocity:.3f} m/s, '
                f'angular.z={angular_velocity:.3f} rad/s'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in ackermann_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AckermannToCmdVelNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()