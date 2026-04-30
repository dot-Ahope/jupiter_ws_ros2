"""
Jupiter Safety Layer (Tier 2 + Tier 2.5 v2)
============================================

cmd_vel 흐름의 다층 안전 stack:

  토픽 흐름 (Tier 2.5 v2 — collision_monitor 가 nav2 stream 만 gate):
    [nav2] → /cmd_vel → collision_monitor → /cmd_vel_nav_filtered ─┐
                                ↑                                   │
                          /scan_merged                              │
                                                                    ├→ twist_mux → /cmd_vel_safe → driver
    [jupiter_joy_new] → /cmd_vel_joy (bypass) ─────────────────────┤
    [forced_recovery] → /cmd_vel_force (bypass) ────────────────────┘

  v1 (기존, deadlock 발생): collision_monitor 가 twist_mux 이후 → forced_recovery 후진까지 차단
  v2 (현재): collision_monitor 가 nav2 stream 만 gate → recovery + joy 는 직접 통과

  Tier 2 (jupiter_safety 자체):
    - twist_mux: 우선순위 multiplex (joy 100 > force 80 > nav 50)
    - stuck_detector: motor stall (cmd vs encoder) + IMU spike → /stuck_status
    - forced_recovery: stuck 시 /cmd_vel_force 후진 (자체 rear LiDAR safety check)
    - alert_publisher: /Buzzer 경보

  Tier 2.5 (nav2_collision_monitor):
    - LiDAR (/scan_merged) 기반 nav2 자동 주행에 emergency stop polygon
    - forced_recovery 의 후진 명령은 bypass — recovery 가 우선
    - joy 도 bypass — 운영자 override 가능

실행:
  ros2 launch jupiter_safety safety_layer.launch.py
또는 nav2_vslam_fused.launch.py 에서 IncludeLaunchDescription 으로 포함.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jupiter_safety')
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    # ============================================================
    # twist_mux — cmd_vel 우선순위 mux
    # ============================================================
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[
            # twist_mux default output topic 은 cmd_vel_out → 우리는 /cmd_vel_safe 로
            ('cmd_vel_out', '/cmd_vel_safe'),
        ],
    )

    # ============================================================
    # stuck_detector (Tier 2A + 2B)
    # ============================================================
    stuck_detector_node = Node(
        package='jupiter_safety',
        executable='stuck_detector',
        name='stuck_detector',
        output='screen',
    )

    # ============================================================
    # forced_recovery (Tier 2C)
    # ============================================================
    forced_recovery_node = Node(
        package='jupiter_safety',
        executable='forced_recovery',
        name='forced_recovery',
        output='screen',
    )

    # ============================================================
    # alert_publisher (Tier 2D)
    # ============================================================
    alert_publisher_node = Node(
        package='jupiter_safety',
        executable='alert_publisher',
        name='alert_publisher',
        output='screen',
    )

    # ============================================================
    # nav2_collision_monitor (Tier 2.5)
    # ============================================================
    # twist_mux 이후 단계에서 LiDAR 기반 emergency stop.
    # /cmd_vel_safe → collision_monitor → /cmd_vel_out → driver
    collision_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'collision_monitor.launch.py')
        )
    )

    return LaunchDescription([
        twist_mux_node,
        stuck_detector_node,
        forced_recovery_node,
        alert_publisher_node,
        collision_monitor_launch,
    ])
