#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import asyncio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from jupiter_msgs.srv import Buzzer

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

class LaserWarning(Node):
    def __init__(self):
        super().__init__('laser_warning')
        
        # Initialize parameters
        self.declare_parameter(
            'switch',
            False,
            ParameterDescriptor(description='Enable/disable laser warning')
        )
        self.declare_parameter(
            'ang_Kp',
            3.0,
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
            3.0,
            ParameterDescriptor(
                description='Angular velocity D gain',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
                ]
            )
        )
        self.declare_parameter(
            'laser_angle',
            50,
            ParameterDescriptor(
                description='Laser scan angle range',
                integer_range=[
                    IntegerRange(from_value=10, to_value=180, step=1)
                ]
            )
        )
        self.declare_parameter(
            'response_dist',
            0.5,
            ParameterDescriptor(
                description='Response distance for warning',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=8.0, step=0.1)
                ]
            )
        )
        
        # Get initial parameter values
        self.switch = self.get_parameter('switch').value
        self.laser_angle = self.get_parameter('laser_angle').value
        self.response_dist = self.get_parameter('response_dist').value
        ang_Kp = self.get_parameter('ang_Kp').value
        ang_Ki = self.get_parameter('ang_Ki').value
        ang_Kd = self.get_parameter('ang_Kd').value
        
        # Initialize variables
        self.moving = False
        self.buzzer_state = False
        self.ang_pid = SinglePID(ang_Kp, ang_Ki, ang_Kd)
        
        # Create subscribers and publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
            
        # Create buzzer service client
        self.buzzer_client = self.create_client(Buzzer, 'buzzer')
        while not self.buzzer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Buzzer service not available, waiting...')
            
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Laser warning node initialized')
        
    def parameter_callback(self, params):
        """Parameter change callback"""
        for param in params:
            # Log the parameter change
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
            
            # Update parameters
            if param.name == 'switch':
                self.switch = param.value
            elif param.name == 'laser_angle':
                self.laser_angle = param.value
            elif param.name == 'response_dist':
                self.response_dist = param.value
            elif param.name == 'ang_Kp':
                self.ang_pid.kp = param.value
            elif param.name == 'ang_Ki':
                self.ang_pid.ki = param.value
            elif param.name == 'ang_Kd':
                self.ang_pid.kd = param.value
                
        return SetParametersResult(successful=True)
        
    async def laser_callback(self, scan_data):
        """Process laser scan data and publish warnings"""
        if not self.switch:
            return
            
        # Get laser angle range
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        ranges = np.array(scan_data.ranges)
        
        # Calculate indices for the warning angle range
        center_idx = len(ranges) // 2
        angle_range = int(self.laser_angle / 2 / (angle_increment * 180 / np.pi))
        start_idx = center_idx - angle_range
        end_idx = center_idx + angle_range
        
        # Get ranges within the warning angle
        warning_ranges = ranges[start_idx:end_idx]
        warning_ranges = warning_ranges[~np.isnan(warning_ranges)]  # Remove NaN values
        
        if len(warning_ranges) > 0:
            min_dist = np.min(warning_ranges)
            if min_dist < self.response_dist:
                # Object detected within warning distance
                min_idx = start_idx + np.argmin(warning_ranges)
                angle = (min_idx - center_idx) * angle_increment
                
                # Calculate angular velocity using PID
                angular_vel = -self.ang_pid.compute(angle)
                
                # Create and publish Twist message
                twist = Twist()
                twist.angular.z = angular_vel
                self.cmd_vel_pub.publish(twist)
                
                # Activate buzzer warning if not already active
                if not self.buzzer_state:
                    self.buzzer_control(1)  # Turn on buzzer
                    self.buzzer_state = True
                
                self.get_logger().warn(f'Object detected at {min_dist:.2f}m, angle: {angle:.2f}rad')
            else:
                # No objects within warning distance
                if self.moving:
                    twist = Twist()
                    self.cmd_vel_pub.publish(twist)
                    self.moving = False

    async def buzzer_control(self, value):
        """Control the buzzer
        
        Args:
            value (int): 0: off, 1: on, >=10: on for xx milliseconds
        """
        request = Buzzer.Request()
        request.buzzer = value
        
        try:
            future = self.buzzer_client.call_async(request)
            response = await future
            if response.result:
                self.get_logger().debug(f'Buzzer command {value} successful')
            else:
                self.get_logger().warn(f'Buzzer command {value} failed')
        except Exception as e:
            self.get_logger().error(f'Buzzer service call failed: {str(e)}')
            
    def cleanup(self):
        """Cleanup before node shutdown"""
        if self.buzzer_state:
            # Create a new event loop for the cleanup
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            # Turn off buzzer
            loop.run_until_complete(self.buzzer_control(0))
            loop.close()
        
def main(args=None):
    rclpy.init(args=args)
    node = LaserWarning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
