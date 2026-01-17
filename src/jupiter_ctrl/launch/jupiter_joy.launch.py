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
            {'linear_speed_limit': 0.6}, # [cite: 2] - 응답성 향상을 위해 0.45에서 0.6으로 증가
            {'angular_speed_limit': 3.0} # [cite: 2] - 회전 응답성 향상을 위해 2.0에서 3.0으로 증가
        ]
    )

    return LaunchDescription([
        joy_node,
        jupiter_joy_node
    ])