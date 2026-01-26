import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 패키지 경로
    jupiter_nav_dir = get_package_share_directory('jupiter_nav_VO')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 파라미터 파일 경로
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(
        jupiter_nav_dir,
        'config',
        'nav2_params_vslam.yaml'
    )
    
    # Nav2 실행 (Navigation Only, No Map Server, No AMCL)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        nav2_launch
    ])
