#!/usr/bin/env python3
# coding:utf-8

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, ParameterType
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from jupiter_msgs.msg import JoyState
import numpy as np
from math import radians, copysign, sqrt, pow, pi
import tf2_ros
from tf2_ros import TransformListener, Buffer
import time
from . import utilities

class JupiterPatrol(Node):
    def __init__(self):
        super().__init__('jupiter_patrol')
        
        # Initialize variables
        self.moving = True
        self.Joy_active = False
        self.SetLoop = False
        self.Switch = False
        self.command_src = "finish"
        self.Command = "finish"
        self.reverse = 1
        self.warning = 1
        self.Length = 1.0
        self.Angle = 360
        self.ResponseDist = 0.7
        
        # Set up parameters with constraints
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Linear', 0.2, 
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 0.45, 0.01)]
                    )),
                ('Angular', 1.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 2.0, 0.1)]
                    )),
                ('Length', 1.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 3.0, 0.1)]
                    )),
                ('Angle', 360.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 360.0, 1.0)]
                    )),
                ('LineScaling', 0.9,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 2.0, 0.1)]
                    )),
                ('RotationScaling', 1.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 2.0, 0.1)]
                    )),
                ('LineTolerance', 0.1,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 3.0, 0.01)]
                    )),
                ('RotationTolerance', 0.3,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 5.0, 0.1)]
                    )),
                ('ResponseDist', 0.6,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        floating_point_range=[FloatingPointRange(0.0, 8.0, 0.1)]
                    )),
                ('LaserAngle', 30,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        integer_range=[IntegerRange(10, 180, 1)]
                    )),
                ('Command', 'Square'),
                ('SetLoop', False),
                ('Switch', False)
            ])
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        
        self.joy_sub = self.create_subscription(
            JoyState,
            'joy_state',
            self.joy_callback,
            10
        )
        
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create timer for main loop
        self.create_timer(0.05, self.update)  # 20Hz
        
        self.get_logger().info('Jupiter Patrol Node has been initialized')
        
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'Command':
                self.Command = param.value
            elif param.name == 'SetLoop':
                self.SetLoop = param.value
            elif param.name == 'Switch':
                self.Switch = param.value
            # Update other parameters as needed
        return True
        
    def laser_callback(self, msg):
        """Process laser scan data"""
        if len(msg.ranges) > 0:
            self.warning = 1
            laser_angle = self.get_parameter('LaserAngle').value
            response_dist = self.get_parameter('ResponseDist').value
            
            # Process laser data for obstacle detection
            for i in range(-laser_angle, laser_angle):
                if 0 <= msg.ranges[i] <= response_dist:
                    self.warning = 0
                    break
    
    def joy_callback(self, msg):
        """Process joystick state"""
        self.Joy_active = msg.state
        
    def update(self):
        """Main control loop"""
        if not self.Joy_active and self.Switch:
            try:
                # Get current robot pose from TF
                transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                
                # Execute movement based on current command
                if self.Command in ['LengthTest', 'AngleTest', 'Triangle', 'Square', 'Parallelogram', 'Circle']:
                    self.execute_command()
            except Exception as e:
                self.get_logger().warning(f'TF lookup failed: {str(e)}')
                
    def execute_command(self):
        """Execute the current movement command"""
        # Implement different movement patterns based on self.Command
        # Use self.get_parameter().value to get current parameter values
        twist = Twist()
        
        if self.warning:
            if self.Command == 'LengthTest':
                # Implement length test movement
                pass
            elif self.Command == 'AngleTest':
                # Implement angle test movement
                pass
            # Implement other patterns...
            
            self.cmd_vel_pub.publish(twist)
        else:
            # Stop if obstacle detected
            self.cmd_vel_pub.publish(Twist())
            
    def cancel(self):
        """Cleanup when node is shut down"""
        self.cmd_vel_pub.publish(Twist())
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JupiterPatrol()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
