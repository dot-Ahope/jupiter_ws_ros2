# <launch>
#     <param name="use_sim_time" value="false"/>
#     <node name="joy_node" pkg="joy" type="joy_node" output="screen" respawn="false"/>
#     <node name="turtlebot_joy" pkg="jupiter_ctrl" type="turtlebot_joy.py" output="screen">
#         <param name="linear_speed_limit" type="double" value="2.0"/>
#         <param name="angular_speed_limit" type="double" value="2.0"/>
#     </node>
# </launch>

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부를 파라미터로 설정
    use_sim_time = False

    # joy_node 실행
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # turtlebot_joy 노드 실행
    turtlebot_joy_node = Node(
        package='jupiter_ctrl',
        executable='turtlebot_joy',
        name='turtlebot_joy',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'linear_speed_limit': 2.0}, # [cite: 1]
            {'angular_speed_limit': 2.0} # [cite: 1]
        ]
    )

    return LaunchDescription([
        joy_node,
        turtlebot_joy_node
    ])