import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    
    # 파라미터 정의
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(sllidar_ros2_dir, 'maps', 'map.yaml'))
    params_file = os.path.join(sllidar_ros2_dir, 'config', 'navigation', 'map_server.yaml')
    
    # 맵 서버 노드
    map_server_node = Node(
        name="map_server",
        package='nav2_map_server',
        executable='map_server',
        parameters=[params_file],
        output='screen'
    )
    
    # 라이프사이클 매니저 노드
    lifecycle_manager = Node(
        name="lifecycle_manager_map",
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map yaml file to load'
        ),
        map_server_node,
        lifecycle_manager
    ])
