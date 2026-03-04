"""
Nav2 Navigation Launch for 4-Sensor EKF Fusion System
======================================================

센서 융합 시스템(nav2_vslam_fused.launch.py)이 이미 실행 중인 상태에서
Nav2 네비게이션 스택만 별도로 실행합니다.

포함 노드:
  - goal_pose_restamper: Foxglove PC의 goal_pose를 Jetson 시간으로 재스탬핑
    (PC-Jetson 시계 차이 ~4s로 인한 TF extrapolation 방지)
  - Nav2 navigation stack (bt_navigator, controller, planner, behavior, costmaps)

사전 실행 필요:
  1. nav2_vslam_fused.launch.py  (센서 + EKF + SLAM Toolbox + LiDAR)
  2. VSLAM Docker (필요시)

실행:
  ros2 launch jupiter_nav_VO nav2_fused_navigation.launch.py

확인할 토픽:
  /odom (EKF 융합 출력)
  /map  (SLAM Toolbox 맵)
  /scan (RPLidar S2L)
  /cmd_vel (Nav2 → 로봇)
  /goal_pose_raw (Foxglove 원본 goal)
  /goal_pose (재스탬핑된 goal → Nav2)

확인할 TF:
  map → odom → base_footprint → base_link → laser
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 경로
    jupiter_nav_dir = get_package_share_directory('jupiter_nav_VO')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(
        jupiter_nav_dir,
        'config',
        'nav2_params_fused.yaml'
    )

    # ── Goal Pose Re-stamper ──────────────────────────────────────────
    # Foxglove(PC)가 /goal_pose에 발행 → restamper가 수신 후
    # Jetson 시간으로 재스탬핑 → /goal_pose로 재발행
    # 
    # 토픽 흐름:
    #   Foxglove → /goal_pose_raw → [restamper] → /goal_pose → Nav2
    goal_pose_restamper = Node(
        package='jupiter_nav_VO',
        executable='goal_pose_restamper',
        name='goal_pose_restamper',
        output='screen',
        remappings=[
            ('goal_pose_in', '/goal_pose_raw'),
            ('goal_pose_out', '/goal_pose'),
        ],
    )

    # Nav2 Navigation (No AMCL, No map_server)
    # SLAM Toolbox가 map→odom TF와 /map을 이미 제공하므로
    # navigation_launch.py만 사용 (localization_launch.py 불필요)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        # Log info
        LogInfo(msg='=== Nav2 Navigation (Fused Odometry Mode) ==='),
        LogInfo(msg='  EKF /odom + SLAM Toolbox /map + LiDAR /scan'),
        LogInfo(msg='  Goal pose re-stamper: /goal_pose_raw → /goal_pose'),
        LogInfo(msg='  Ensure nav2_vslam_fused.launch.py is running first!'),

        # Arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 parameters file (nav2_params_fused.yaml)'
        ),

        # Goal pose re-stamper (PC→Jetson 시간 변환)
        goal_pose_restamper,

        # Nav2 navigation stack
        nav2_launch,
    ])
