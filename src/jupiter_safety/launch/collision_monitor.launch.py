"""
Nav2 Collision Monitor Launch — Tier 2.5
==========================================

cmd_vel 흐름의 마지막 안전 layer.
twist_mux 가 우선순위 multiplex 한 결과 (/cmd_vel_safe) 를 받아서,
LiDAR 기반 emergency polygon 검사 후 /cmd_vel_out 으로 forward.

Lifecycle: nav2_lifecycle_manager 가 자동 활성화.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jupiter_safety')
    cm_config = os.path.join(pkg_share, 'config', 'collision_monitor.yaml')

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[cm_config],
    )

    # collision_monitor 는 nav2 lifecycle node — 자동 활성화 필요
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor'],
            'bond_timeout': 4.0,
        }],
    )

    return LaunchDescription([
        collision_monitor_node,
        lifecycle_manager,
    ])
