#!/usr/bin/env python3
"""
Nav2 Navigation Launch for Transbot
SLAM Toolbox와 함께 사용 - 실시간 맵 생성하면서 자율 주행

사용법:
  터미널 1: ros2 launch jupiter_nav jupiter_full_system.launch.py
  터미널 2: ros2 launch jupiter_nav nav2_navigation.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 디렉토리 - jupiter_nav 패키지 사용 ⭐
    pkg_dir = get_package_share_directory('jupiter_nav')
    
    # 파라미터 파일 - jupiter_nav 패키지 config 사용 ⭐
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )
    
    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ]
    )
    
    # Lifecycle Manager for Navigation
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother'
            ]}
        ]
    )
    
    # Launch Description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add nodes
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(velocity_smoother_node)
    ld.add_action(lifecycle_manager_node)
    
    return ld
