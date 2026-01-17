#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, ParameterType
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from math import copysign, sqrt, pow
from . import utilities

class CalibrateLinear(Node):
    def __init__(self):
        super().__init__('calibrate_linear')
        
        # Initialize variables
        self.start_test = False
        self.stop_robot = False
        self.test_distance = 0.0
        self.speed = 0.0
        self.tolerance = 0.0
        self.odom_linear_scale_correction = 1.0
        self.start_time = None
        self.position = Point()
        self.start_position = None
        
        # Declare parameters with constraints
        self.declare_parameter('test_distance', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=2.0,
                    step=0.1
                )],
                description='Test distance in meters'
            ))
            
        self.declare_parameter('speed', 0.15,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=0.3,
                    step=0.01
                )],
                description='Linear speed in meters per second'
            ))
            
        self.declare_parameter('tolerance', 0.01,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=0.1,
                    step=0.01
                )],
                description='Distance tolerance in meters'
            ))
            
        self.declare_parameter('odom_linear_scale_correction', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=3.0,
                    step=0.01
                )],
                description='Linear correction factor'
            ))
            
        self.declare_parameter('start_test', False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Start the calibration test'
            ))
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create timer for test monitoring
        self.create_timer(0.05, self.test_monitor)  # 20Hz
        
        self.get_logger().info('Linear Calibration Node has been initialized')
        
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'start_test' and param.value and not self.start_test:
                self.start_test = True
                self.test_distance = self.get_parameter('test_distance').value
                self.speed = self.get_parameter('speed').value
                self.tolerance = self.get_parameter('tolerance').value
                self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').value
                self.start_position = None
                self.start_calibration()
        return True
        
    def start_calibration(self):
        """Start the linear calibration test"""
        self.get_logger().info(
            f'Starting linear calibration test:\n' +
            f'Distance: {self.test_distance} m\n' +
            f'Speed: {self.speed} m/s\n' +
            f'Tolerance: {self.tolerance} m'
        )
        
    def odom_callback(self, msg):
        """Handle odometry updates"""
        if self.start_test:
            self.position = msg.pose.pose.position
            
            if self.start_position is None:
                self.start_position = self.position
            else:
                # Calculate distance traveled
                distance = sqrt(
                    pow(self.position.x - self.start_position.x, 2) +
                    pow(self.position.y - self.start_position.y, 2)
                )
                
                if distance >= self.test_distance - self.tolerance:
                    self.stop_robot = True
                    
    def test_monitor(self):
        """Monitor and control the test"""
        if self.start_test and not self.stop_robot:
            # Send movement command
            twist = Twist()
            twist.linear.x = copysign(self.speed, self.test_distance)
            self.cmd_vel_pub.publish(twist)
        elif self.stop_robot:
            # Stop robot
            self.cmd_vel_pub.publish(Twist())
            if self.start_test:
                self.start_test = False
                self.stop_robot = False
                # Calculate final distance
                final_distance = sqrt(
                    pow(self.position.x - self.start_position.x, 2) +
                    pow(self.position.y - self.start_position.y, 2)
                )
                self.get_logger().info(
                    f'Linear calibration completed.\n' +
                    f'Target distance: {self.test_distance} m\n' +
                    f'Actual distance: {final_distance:.3f} m\n' +
                    f'Error: {abs(self.test_distance - final_distance):.3f} m'
                )

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLinear()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure to stop the robot
        if hasattr(node, 'cmd_vel_pub'):
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
