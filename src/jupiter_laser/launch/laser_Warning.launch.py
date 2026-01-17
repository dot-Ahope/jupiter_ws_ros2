from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Include base launch file
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('jupiter_laser'),
                'launch',
                'base.launch.py'
            ])
        ),

        # Laser warning node
        Node(
            package='jupiter_laser',
            executable='laser_warning',
            name='laser_warning',
            output='screen',
            parameters=[{
                'switch': False,
                'ang_Kp': 3.0,
                'ang_Ki': 0.0,
                'ang_Kd': 3.0,
                'laser_angle': 50,
                'response_dist': 0.5
            }]
        )
    ])
