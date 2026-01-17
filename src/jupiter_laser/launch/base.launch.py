from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
                # RPLidar node
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            ])
        ),

        # Jupiter driver node

        # Jupiter driver node
        Node(
            package='jupiter_bringup',
            executable='jupiter_driver',
            name='jupiter_node',
            output='screen',
            parameters=[{
                'imu': '/jupiter/imu',
                'vel': '/jupiter/get_vel'
            }]
        ),

        # Joystick control node
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('jupiter_ctrl'),
                'launch',
                'jupiter_joy.launch.py'
            ])
        )
    ])
