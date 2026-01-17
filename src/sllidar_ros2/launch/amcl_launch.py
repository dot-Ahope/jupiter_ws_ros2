import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    
    # AMCL 파라미터 설정
    params_file = os.path.join(sllidar_ros2_dir, 'config', 'navigation', 'amcl.yaml')
    
    # 맵 서버 포함
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'map_server_launch.py')
        )
    )
    
    # AMCL 노드
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )
    
    # 라이프사이클 매니저
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )
    
    return LaunchDescription([
        map_server_launch,
        amcl_node,
        lifecycle_manager
    ])
