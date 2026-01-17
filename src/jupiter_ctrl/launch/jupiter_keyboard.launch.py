from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # jupiter_keyboard 노드 실행
    jupiter_keyboard_node = Node(
        package='jupiter_ctrl',
        executable='jupiter_keyboard',
        name='jupiter_keyboard',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'linear_speed_limit': 0.45},
            {'angular_speed_limit': 2.0}
        ]
    )

    return LaunchDescription([
        jupiter_keyboard_node
    ])