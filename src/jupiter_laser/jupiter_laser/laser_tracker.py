#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from rcl_interfaces.msg import SetParametersResult

class SinglePID:
    def __init__(self, p, i, d):
        self.kp = p
        self.ki = i
        self.kd = d
        self.error = 0
        self.last_error = 0
        self.integral = 0
        
    def compute(self, error):
        self.integral += error
        derivative = error - self.last_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

class LaserTracker(Node):
    def __init__(self):
        super().__init__('laser_tracker')
        
        # Initialize parameters with descriptions and constraints
        self.declare_parameter(
            'switch',
            False,
            ParameterDescriptor(description='Enable/disable laser tracking')
        )
        self.declare_parameter(
            'lin_Kp',
            3.0,
            ParameterDescriptor(
                description='Linear velocity P gain',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'lin_Ki',
            0.0,
            ParameterDescriptor(
                description='Linear velocity I gain',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'lin_Kd',
            0.5,
            ParameterDescriptor(
                description='Linear velocity D gain',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'ang_Kp',
            4.0,
            ParameterDescriptor(
                description='Angular velocity P gain',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'ang_Ki',
            0.0,
            ParameterDescriptor(
                description='Angular velocity I gain',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'ang_Kd',
            1.0,
            ParameterDescriptor(
                description='Angular velocity D gain',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'laser_angle',
            65,
            ParameterDescriptor(
                description='Laser scan angle range',
                integer_range=[
                    IntegerRange(from_value=10, to_value=180, step=1)
                ]
            )
        )
        self.declare_parameter(
            'response_dist',
            1.0,
            ParameterDescriptor(
                description='Response distance for tracking',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=8.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'priority_angle',
            30,
            ParameterDescriptor(
                description='Priority angle for tracking',
                integer_range=[
                    IntegerRange(from_value=10, to_value=50, step=1)
                ]
            )
        )
        
        # Get initial parameter values
        self.update_parameters()
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
            
        # Add parameter callback after parameters are declared
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Laser tracker node initialized')
        
    def update_parameters(self):
        """Update all parameter values"""
        self.switch = self.get_parameter('switch').value
        self.lin_Kp = self.get_parameter('lin_Kp').value
        self.lin_Ki = self.get_parameter('lin_Ki').value
        self.lin_Kd = self.get_parameter('lin_Kd').value
        self.ang_Kp = self.get_parameter('ang_Kp').value
        self.ang_Ki = self.get_parameter('ang_Ki').value
        self.ang_Kd = self.get_parameter('ang_Kd').value
        self.laser_angle = self.get_parameter('laser_angle').value
        self.response_dist = self.get_parameter('response_dist').value
        self.priority_angle = self.get_parameter('priority_angle').value
        
        # Update PID controllers
        self.lin_pid = SinglePID(self.lin_Kp, self.lin_Ki, self.lin_Kd)
        self.ang_pid = SinglePID(self.ang_Kp, self.ang_Ki, self.ang_Kd)
        
    def parameter_callback(self, params):
        """Parameter change callback"""
        for param in params:
            # Log the parameter change
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
            
        # Update all parameters
        self.update_parameters()
        return SetParametersResult(successful=True)
        
        # Initialize parameters
        self.declare_parameter('switch', False)
        self.declare_parameter('laser_angle', 65)
        self.declare_parameter('priority_angle', 35)
        self.declare_parameter('target_dist', 1.0)
        self.declare_parameter('lin_p', 3.0)
        self.declare_parameter('lin_i', 0.0)
        self.declare_parameter('lin_d', 0.5)
        self.declare_parameter('ang_p', 4.0)
        self.declare_parameter('ang_i', 0.0)
        self.declare_parameter('ang_d', 1.0)
        
        # Get parameter values
        self.switch = self.get_parameter('switch').value
        self.laser_angle = self.get_parameter('laser_angle').value
        self.priority_angle = self.get_parameter('priority_angle').value
        self.response_dist = self.get_parameter('target_dist').value
        
        # Initialize variables
        self.moving = False
        self.buzzer_state = False
        self.lin_pid = SinglePID(
            self.get_parameter('lin_p').value,
            self.get_parameter('lin_i').value,
            self.get_parameter('lin_d').value
        )
        self.ang_pid = SinglePID(
            self.get_parameter('ang_p').value,
            self.get_parameter('ang_i').value,
            self.get_parameter('ang_d').value
        )
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
            
        self.get_logger().info("Laser tracker node initialized")
        
    def laser_callback(self, scan_data):
        if not self.switch:
            return
            
        ranges = np.array(scan_data.ranges)
        back_state = True
        offset = 0.5
        front_min_dist_list = []
        front_id_list = []
        min_dist_list = []
        min_dist_id_list = []
        
        for i in np.argsort(ranges):
            if len(ranges) == 720:  # 360도를 720개의 포인트로 나눈 경우
                # Check back area
                if 270 < i < 450:
                    if ranges[i] < 0.5:
                        back_state = False
                # Check front priority area
                elif 0 < i < self.priority_angle * 2:
                    if ranges[i] < (self.response_dist + offset):
                        front_min_dist_list.append(ranges[i])
                        front_id_list.append(i / 2)
                elif (720 - self.priority_angle * 2) <= i:
                    if ranges[i] < (self.response_dist + offset):
                        front_min_dist_list.append(ranges[i])
                        front_id_list.append((i - 720) / 2)
                # Check side areas
                elif (self.priority_angle * 2) <= i <= (self.laser_angle * 2):
                    if ranges[i] < self.response_dist:
                        min_dist_list.append(ranges[i])
                        min_dist_id_list.append(i / 2)
                elif (720 - self.laser_angle * 2) <= i < (720 - self.priority_angle * 2):
                    if ranges[i] < self.response_dist:
                        min_dist_list.append(ranges[i])
                        min_dist_id_list.append((i - 720) / 2)
        
        # Control logic
        twist = Twist()
        
        if len(front_min_dist_list) > 0:
            # Object found in priority area
            min_dist = min(front_min_dist_list)
            target_angle = front_id_list[front_min_dist_list.index(min_dist)]
        elif len(min_dist_list) > 0:
            # Object found in side areas
            min_dist = min(min_dist_list)
            target_angle = min_dist_id_list[min_dist_list.index(min_dist)]
        else:
            # No object found
            twist.angular.z = 0.5  # Rotate to search
            self.cmd_vel_pub.publish(twist)
            return
            
        # Calculate control outputs
        distance_error = min_dist - self.response_dist
        angle_error = -target_angle
        
        linear_vel = self.lin_pid.compute(distance_error)
        angular_vel = self.ang_pid.compute(angle_error)
        
        # Apply velocity limits
        twist.linear.x = np.clip(linear_vel, -0.5, 0.5)
        twist.angular.z = np.clip(angular_vel, -1.5, 1.5)
        
        self.cmd_vel_pub.publish(twist)
        
    def cancel(self):
        self.get_logger().info("Shutting down laser tracker node")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LaserTracker()
    
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
