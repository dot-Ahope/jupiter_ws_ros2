#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange

class LaserAvoid(Node):
    def __init__(self):
        super().__init__('laser_avoid')
        
        # Initialize parameters with descriptions and constraints
        self.declare_parameter(
            'switch',
            False,
            ParameterDescriptor(description='Enable/disable laser avoidance')
        )
        self.declare_parameter(
            'linear',
            0.3,
            ParameterDescriptor(
                description='Linear velocity',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=0.45, step=0.01)
                ]
            )
        )
        self.declare_parameter(
            'angular',
            1.0,
            ParameterDescriptor(
                description='Angular velocity',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'laser_angle',
            30,
            ParameterDescriptor(
                description='Laser scan angle range',
                integer_range=[
                    IntegerRange(from_value=10, to_value=180, step=1)
                ]
            )
        )
        self.declare_parameter(
            'response_dist',
            0.55,
            ParameterDescriptor(
                description='Response distance for obstacle avoidance',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=8.0, step=0.05)
                ]
            )
        )
        
        # Get parameter values
        self.switch = self.get_parameter('switch').value
        self.linear = self.get_parameter('linear').value
        self.angular = self.get_parameter('angular').value
        self.laser_angle = self.get_parameter('laser_angle').value
        self.response_dist = self.get_parameter('response_dist').value
        
        # Initialize variables
        self.moving = False
        self.running = False
        self.right_warning = 0
        self.left_warning = 0
        self.front_warning = 0
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
            
        # Create timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info("Laser avoidance node initialized")
        
    def laser_callback(self, scan_data):
        if self.running:
            return
            
        # Convert laser scan to numpy array
        ranges = np.array(scan_data.ranges)
        
        # Get indices of valid measurements (not inf or nan)
        valid_indices = np.isfinite(ranges)
        valid_ranges = ranges[valid_indices]
        
        if len(valid_ranges) == 0:
            return
            
        # Get angle increment and start angle
        angle_increment = scan_data.angle_increment
        angle_min = scan_data.angle_min
        
        # Calculate angles for all measurements
        angles = np.arange(len(ranges)) * angle_increment + angle_min
        
        # Define regions of interest
        front_indices = np.abs(angles) <= np.radians(self.laser_angle/2)
        left_indices = angles > np.radians(self.laser_angle/2)
        right_indices = angles < -np.radians(self.laser_angle/2)
        
        # Check for obstacles in each region
        self.front_warning = np.any(ranges[front_indices] < self.response_dist)
        self.left_warning = np.any(ranges[left_indices] < self.response_dist)
        self.right_warning = np.any(ranges[right_indices] < self.response_dist)
        
    def control_loop(self):
        if not self.switch:
            return
            
        self.running = True
        twist = Twist()
        
        # Implement avoidance logic
        if self.front_warning:
            if self.left_warning and self.right_warning:
                # Blocked on all sides, turn around
                twist.linear.x = -self.linear
                twist.angular.z = self.angular
            elif self.left_warning:
                # Turn right
                twist.linear.x = self.linear * 0.5
                twist.angular.z = -self.angular
            elif self.right_warning:
                # Turn left
                twist.linear.x = self.linear * 0.5
                twist.angular.z = self.angular
            else:
                # Random turn direction
                turn_direction = 1 if np.random.random() > 0.5 else -1
                twist.linear.x = 0.0
                twist.angular.z = self.angular * turn_direction
        else:
            # Move forward
            twist.linear.x = self.linear
            if self.left_warning:
                twist.angular.z = -self.angular * 0.5
            elif self.right_warning:
                twist.angular.z = self.angular * 0.5
            else:
                twist.angular.z = 0.0
                
        self.cmd_vel_pub.publish(twist)
        self.running = False
        
    def cancel(self):
        self.get_logger().info("Shutting down laser avoidance node")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LaserAvoid()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
