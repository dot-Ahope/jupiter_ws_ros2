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

        # Laser avoidance node
        Node(
            package='jupiter_laser',
            executable='laser_Avoidance',
            name='laser_Avoidance',
            output='screen',
            parameters=[{
                'switch': False,
                'linear': 0.3,
                'angular': 1.0,
                'laser_angle': 30,
                'response_dist': 0.55
            }]
        )
    ])
