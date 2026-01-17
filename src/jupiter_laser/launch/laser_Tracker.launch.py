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

        # Laser tracker node
        Node(
            package='jupiter_laser',
            executable='laser_tracker',
            name='laser_tracker',
            output='screen',
            parameters=[{
                'switch': False,
                'lin_Kp': 3.0,
                'lin_Ki': 0.0,
                'lin_Kd': 0.5,
                'ang_Kp': 4.0,
                'ang_Ki': 0.0,
                'ang_Kd': 1.0,
                'laser_angle': 65,
                'response_dist': 1.0,
                'priority_angle': 30
            }]
        )
    ])
