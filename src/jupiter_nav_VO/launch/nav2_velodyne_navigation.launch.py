import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_jupiter_nav_vo = get_package_share_directory('jupiter_nav_VO')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_velodyne_driver = get_package_share_directory('velodyne_driver')
    pkg_velodyne_pointcloud = get_package_share_directory('velodyne_pointcloud')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 1. Velodyne Driver (VLP16)
    velodyne_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_velodyne_driver, 'launch', 'velodyne_driver_node-VLP16-launch.py')
        ),
        launch_arguments={
            'device_ip': '192.168.1.201', # Default Velodyne IP
            'port': '2368',
            'frame_id': 'velodyne',
            'model': 'VLP16'
        }.items()
    )

    # 2. Velodyne Pointcloud Transform (Packet -> PointCloud2)
    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_velodyne_pointcloud, 'launch', 'velodyne_transform_node-VLP16-launch.py')
        ),
        launch_arguments={
            'calibration': os.path.join(pkg_velodyne_pointcloud, 'params', 'VLP16db.yaml'),
            'frame_id': 'velodyne',
            'min_range': '0.4',
            'max_range': '130.0'
        }.items()
    )

    # 3. Static TF for Velodyne (Simple approximation)
    # Adjust xyz/rpy to match your mounting. 
    # Example: 0.1m forward, 0.2m up
    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'velodyne']
    )

    # 4. Nav2 Bringup with Velodyne Params
    # We use our new 'nav2_params_velodyne.yaml' which has voxel_layer configured for /velodyne_points
    nav2_params_file = os.path.join(pkg_jupiter_nav_vo, 'config', 'nav2_params_velodyne.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        velodyne_driver_launch,
        velodyne_transform_launch,
        tf_base_to_velodyne,
        nav2_launch
    ])
