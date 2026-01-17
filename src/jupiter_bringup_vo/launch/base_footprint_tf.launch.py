#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform from base_link to base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_broadcaster',
            arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),
    ])