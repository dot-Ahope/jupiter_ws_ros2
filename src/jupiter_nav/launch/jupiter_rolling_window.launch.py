import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    jupiter_nav_dir = get_package_share_directory('jupiter_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arguments
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start Rviz')

    # 1. Robot Bringup (Sensors: Velodyne, Realsense, IMU + EKF Odom)
    # Includes: velodyne_driver, velodyne_pointcloud, jupiter_driver, imu_calib, kiss_icp, robot_localization, realsense
    jupiter_indoor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jupiter_nav_dir, 'launch', 'jupiter_indoor.launch.py')
        )
    )
    
    # 2. Nav2 Navigation (No Map / Rolling Window Mode)
    # Config: nav2_no_map_params.yaml uses rolling_window: true, global_frame: odom
    nav2_params_file = os.path.join(jupiter_nav_dir, 'config', 'nav2_no_map_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'False',
            'autostart': 'True',
            'use_composition': 'True',
            'container_name': 'nav2_container'
        }.items()
    )

    # 3. RViz2
    # Using nav2_bringup default view if specific one is not available
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_use_rviz_cmd,
        jupiter_indoor_launch,
        nav2_launch,
        rviz_cmd
    ])
