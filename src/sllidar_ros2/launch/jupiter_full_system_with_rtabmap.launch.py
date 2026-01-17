#!/usr/bin/env python3
"""
Jupiter Full System with RTAB-Map
기존 jupiter_full_system.launch.py에 카메라와 RTAB-Map 추가
- SLAM Toolbox: 2D LiDAR SLAM
- RTAB-Map: RGB-D Visual SLAM (동시 실행)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # 패키지 경로
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    jupiter_description_dir = get_package_share_directory('jupiter_description')
    jupiter_bringup_dir = get_package_share_directory('jupiter_bringup')
    
    # 라이다 시리얼 포트 파라미터
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/jupiter_lidar')
    
    # URDF 파일 경로
    urdf_file_path = os.path.join(
        jupiter_description_dir,
        'urdf', 
        'jupiter_simple.urdf'
    )
    
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    robot_description = ParameterValue(robot_description_raw, value_type=str)
    
    # Launch arguments
    use_robot_state_pub_arg = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='true',
        description='Use robot state publisher'
    )
    
    use_joint_state_pub_arg = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='true',
        description='Use joint state publisher'
    )
    
    declare_lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='Port for the LiDAR device'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz2 visualization'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_nav2_slam_arg = DeclareLaunchArgument(
        'use_nav2_slam',
        default_value='false',
        description='Use Nav2 SLAM for localization'
    )
    
    # RTAB-Map arguments
    rtabmapviz_arg = DeclareLaunchArgument(
        'rtabmapviz',
        default_value='true',
        description='Launch RTAB-Map visualization'
    )
    
    rtabmap_localization_arg = DeclareLaunchArgument(
        'rtabmap_localization',
        default_value='false',
        description='RTAB-Map localization mode'
    )
    
    rtabmap_database_arg = DeclareLaunchArgument(
        'rtabmap_database',
        default_value='~/.ros/rtabmap.db',
        description='RTAB-Map database path'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }],
        condition=IfCondition(LaunchConfiguration('use_robot_state_pub'))
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_pub'))
    )
    
    # RPLidar
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_frequency': 5.0,
            'range_min': 0.05,
            'range_max': 22.0,
            'scan_time': 0.2,
            'publish_intensity': False,
            'angle_min': -1.5,
            'angle_max': 1.5
        }],
        output='screen'
    )
    
    # Jupiter Driver
    driver_params_path = os.path.join(jupiter_bringup_dir, 'param', 'jupiter_driver_params.yaml')
    jupiter_driver_node = Node(
        package='jupiter_bringup',
        executable='jupiter_driver',
        name='jupiter_driver',
        output='screen',
        parameters=[driver_params_path],
        remappings=[
            ('/imu', '/jupiter/imu'),
            ('/vel', '/jupiter/get_vel')
        ]
    )
    
    # Jupiter Base
    jupiter_base_node = Node(
        package='jupiter_base',
        executable='base_node',
        name='base_node',
        output='screen',
        parameters=[{
            'linear_scale': 1.2,
            'angular_scale': 1.8819,
            'is_multi_robot': False
        }]
    )
    
    # IMU Calibration
    imu_calib_node = Node(
        package='imu_calib',
        executable='apply_calib_node',
        name='apply_calib',
        namespace='',
        parameters=[
            {'calib_file': '/home/jetson/jupiter_ws_ros2/imu_calib.yaml'},
            {'calibrate_gyros': True},
            {'gyro_calib_samples': 100}
        ],
        remappings=[
            ('/raw', '/jupiter/imu'),
            ('/corrected', '/imu/data_calibrated')
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Robot Localization EKF
    ekf_config_file = os.path.join(sllidar_ros2_dir, 'config', 'ekf_config.yaml')
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )
    
    # SLAM Toolbox
    slam_params_file = os.path.join(sllidar_ros2_dir, 'config', 'slam_params.yaml')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file],
        output='screen'
    )
    
    slam_toolbox_node_conditional = TimerAction(
        period=5.0,
        actions=[slam_toolbox_node],
        condition=UnlessCondition(LaunchConfiguration('use_nav2_slam'))
    )
    
    # ==================== 카메라 노드 추가 ====================
    
    # Astra Camera (Depth + IR)
    astra_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        name='astra_camera_node',
        output='screen',
        parameters=[{
            'camera_name': 'camera',
            'enable_depth': True,
            'enable_point_cloud': True,
            'depth_registration': True,
            'publish_tf': True,
            'tf_publish_rate': 10.0,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'enable_color': False,
            'enable_ir': True,
            'ir_width': 640,
            'ir_height': 480,
            'ir_fps': 30,
            'device_num': 0,
            'vendor_id': '0x2bc5',
            'product_id': '0x060f',
            'depth_qos': 'default',
            'point_cloud_qos': 'default',
        }]
    )
    
    # USB RGB Camera (device_srv)
    device_srv_node = Node(
        package='jupiter_bringup',
        executable='device_srv',
        name='device_srv',
        output='screen',
        parameters=[{
            'camera_device': 'usb',
            'image_width': 640,
            'image_height': 480,
            'camera_fps': 15
        }]
    )
    
    # ==================== RTAB-Map 추가 ====================
    
    # RTAB-Map parameters
    rtabmap_parameters = {
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'sync_queue_size': 30,
        'database_path': LaunchConfiguration('rtabmap_database'),
        'use_sim_time': False,
        
        # SLAM parameters
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        
        # Visual features
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1',
        'Vis/MaxDepth': '4.0',
        'Vis/FeatureType': '6',  # ORB
        
        # Registration
        'Reg/Strategy': '0',  # Visual
        'Reg/Force3DoF': 'true',
        
        # Odometry
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        
        # Grid
        'Grid/FromDepth': 'true',
        'Grid/CellSize': '0.05',
        'GridGlobal/MinSize': '20',
    }
    
    # RTAB-Map SLAM node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            ('rgb/image', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth/image', '/depth/image_raw'),
            ('depth/camera_info', '/depth/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        arguments=['--delete_db_on_start'],
        condition=UnlessCondition(LaunchConfiguration('rtabmap_localization'))
    )
    
    # RTAB-Map localization
    rtabmap_localization_params = rtabmap_parameters.copy()
    rtabmap_localization_params['Mem/IncrementalMemory'] = 'false'
    rtabmap_localization_params['Mem/InitWMWithAllNodes'] = 'true'
    
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_localization_params],
        remappings=[
            ('rgb/image', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth/image', '/depth/image_raw'),
            ('depth/camera_info', '/depth/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        condition=IfCondition(LaunchConfiguration('rtabmap_localization'))
    )
    
    # RTAB-Map visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'approx_sync': True,
            'sync_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth/image', '/depth/image_raw'),
            ('depth/camera_info', '/depth/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        condition=IfCondition(LaunchConfiguration('rtabmapviz'))
    )
    
    # RViz
    rviz_config_file = os.path.join(sllidar_ros2_dir, 'rviz', 'sllidar.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    rviz_condition = GroupAction(
        actions=[rviz_node],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        # Arguments
        use_robot_state_pub_arg,
        use_joint_state_pub_arg,
        use_nav2_slam_arg,
        use_rviz_arg,
        use_sim_time_arg,
        declare_lidar_port_arg,
        rtabmapviz_arg,
        rtabmap_localization_arg,
        rtabmap_database_arg,
        
        # Core nodes
        robot_state_publisher,
        joint_state_publisher,
        jupiter_driver_node,
        imu_calib_node,
        jupiter_base_node,
        robot_localization_node,
        sllidar_node,
        
        # Camera nodes
        astra_node,
        device_srv_node,
        
        # SLAM
        slam_toolbox_node_conditional,
        
        # RTAB-Map
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
        
        # Visualization
        rviz_condition,
    ])
