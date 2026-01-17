# <launch>
#     <param name="use_sim_time" value="false"/>
#     <!--    Start the turtle node-->
#     <node name="turtlesim_node" pkg="turtlesim" type="turtlesim_node" output="screen" respawn="false"/>
#     <!--    Start the node for obtaining wireless handle information-->
#     <node name="joy_node" pkg="joy" type="joy_node" output="screen" respawn="false"/>
#     <!--    Start the wireless controller node-->
#     <node name="twist_joy" pkg="jupiter_ctrl" type="twist_joy.py" output="screen">
#         <param name="linear_speed_limit" type="double" value="2.0"/>
#         <param name="angular_speed_limit" type="double" value="2.0"/>
#     </node>
# </launch>

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부를 파라미터로 설정
    use_sim_time = False

    # turtlesim_node 실행
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # joy_node 실행
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # twist_joy 노드 실행
    twist_joy_node = Node(
        package='jupiter_ctrl',
        executable='twist_joy',
        name='twist_joy',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'linear_speed_limit': 2.0}, # [cite: 5]
            {'angular_speed_limit': 2.0} # [cite: 5]
        ]
    )

    return LaunchDescription([
        turtlesim_node,
        joy_node,
        twist_joy_node
    ])