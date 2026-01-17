from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jupiter_laser',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen',
            parameters=[{
                'port': '/dev/ydlidar',
                'frame_id': 'laser_frame',
                'baudrate': 128000,
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 5,
                'abnormal_check_count': 4,
                'resolution_fixed': True,
                'reversion': False,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'intensity': False,
                'support_motor_dtr': False,
                'max_angle': 180.0,
                'min_angle': -180.0,
                'max_range': 12.0,
                'min_range': 0.12,
                'angle_increment': 1.0
            }]
        )
    ])
