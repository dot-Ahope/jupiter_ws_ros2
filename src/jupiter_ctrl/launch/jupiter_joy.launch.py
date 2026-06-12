# <launch>
#     <param name="use_sim_time" value="false"/>
#     <node name="joy_node" pkg="joy" type="joy_node" output="screen" respawn="false"/>
#     <node name="jupiter_joy" pkg="jupiter_ctrl" type="jupiter_joy.py" output="screen">
#         <param name="linear_speed_limit" type="double" value="0.45"/>
#         <param name="angular_speed_limit" type="double" value="2.0"/>
#     </node>
# </launch>

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부를 파라미터로 설정
    use_sim_time = False

    # joy_node 실행 - ROS1과 동일한 설정으로
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # jupiter_joy 노드 실행
    jupiter_joy_node = Node(
        package='jupiter_ctrl',
        executable='jupiter_joy',
        name='jupiter_joy',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            # [2026-05-12] 0.6/3.0 → 0.30/1.5. 측정: 0.6 m/s = nav2 max_vel_x(0.20) 의 3배 → 너무 빠름.
            #   1.5 rad/s 면 360°/4초 회전. nav2 의 wz_max(0.6) 보다 빠르지만 운영자 직접 조작 시 충분.
            {'linear_speed_limit': 0.30},
            {'angular_speed_limit': 1.5}
        ]
    )

    return LaunchDescription([
        joy_node,
        jupiter_joy_node
    ])