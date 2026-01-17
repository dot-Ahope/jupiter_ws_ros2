from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Device Service node
    device_srv_node = Node(
        package='jupiter_bringup',
        executable='device_srv',
        name='device_srv',
        output='screen'
    )
    
    # Driver node
    driver_node = Node(
        package='jupiter_bringup',
        executable='jupiter_driver',
        name='jupiter_node',
        output='screen',
        parameters=[{
            'imu': '/jupiter/imu',
            'vel': '/jupiter/get_vel',
            'kp': 1.0,
            'ki': 0.0,
            'kd': 4.0
        }]
    )
    
    # Include jupiter_joy launch
    jupiter_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jupiter_ctrl'),
                'launch',
                'jupiter_joy.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        device_srv_node,
        driver_node,
        jupiter_joy_launch
    ])
