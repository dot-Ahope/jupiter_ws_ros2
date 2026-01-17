from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_prefix = FindPackageShare('jupiter_bringup')
    
    params_path = PathJoinSubstitution(
        [pkg_prefix, 'param', 'jupiter_patrol_params.yaml']
    )
    
    return LaunchDescription([
        Node(
            package='jupiter_bringup',
            executable='jupiter_patrol',
            name='jupiter_patrol',
            output='screen',
            parameters=[params_path],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/scan', '/scan'),
                ('/joy_state', '/joy_state')
            ]
        )
    ])
