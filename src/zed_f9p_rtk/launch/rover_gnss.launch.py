import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    zed_f9p_node = Node(
        package='zed_f9p_rtk',
        executable='zed_f9p_rtk_node',
        name='zed_f9p_rtk',
        output='screen',
        parameters=[{
            'port': '/dev/rtk_gps',
            'baudrate': 460800,
            'frame_id': 'gps_link',
            'ntrip_server': 'www.gnssdata.or.kr',
            'ntrip_port': 2101,
            'ntrip_user': 'geektrck@gmail.com',
            'ntrip_pass': 'gnss',
            'ntrip_mountpoint': 'SUWN-RTCM32'
        }]
    )

    return LaunchDescription([
        zed_f9p_node
    ])
