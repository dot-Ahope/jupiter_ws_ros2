#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # 패키지 경로
    jupiter_nav_dir = get_package_share_directory('jupiter_nav')
    velodyne_dir = get_package_share_directory('velodyne')
    jupiter_description_dir = get_package_share_directory('jupiter_description')
    jupiter_bringup_dir = get_package_share_directory('jupiter_bringup')
    
        # URDF 파일 경로 설정 (성능 최적화를 위한 단순화된 모델)
    urdf_file_path = os.path.join(
        get_package_share_directory('jupiter_description'),
        'urdf', 
        'jupiter_simple.urdf'  # 단순화된 URDF 사용
    )
    
    # 로봇 상태 발행 노드
    # URDF 파일을 로드하고 처리
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    robot_description = ParameterValue(robot_description_raw, value_type=str)
    
    # declare_launch_argument을 사용하여 중복 노드 실행 여부 제어
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
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0  # Hz - SLAM/네비게이션에 충분, CPU 부하 감소 (기본 10000Hz → 30Hz)
        }],
        condition=IfCondition(LaunchConfiguration('use_robot_state_pub'))
    )
    
    # 조인트 상태 발행 노드
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_pub'))
    )
    
    # Custom Velodyne Launch Setup to fix rotation and FOV issues
    # 1. Driver Node
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[{
            'device_ip': '192.168.1.201',
            'port': 2368,
            'model': 'VLP16',
            'frame_id': 'velodyne',
            'rpm': 600.0,
            'cut_angle': -1.0 # Disable cut_angle in driver
        }]
    )

    # 2. Transform Node (Pointcloud generation)
    # Fixes:
    # - rotated 180 degrees (view_direction: 3.14159)
    # - Full 360 degree view (view_width: 6.28318)
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        parameters=[{
            'calibration': os.path.join(get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml'),
            'model': 'VLP16',
            'min_range': 0.1, # Decreased to see closer objects for better matching
            'max_range': 130.0,
            'view_direction': 0.0, # Reset to 0 since URDF was reset
            'view_width': 6.2831853,      # Full 360 degrees
            'organize_cloud': False
        }]
    )

    # 3. Laserscan Node (2D scan for map/costmap)
    velodyne_laserscan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='both',
        parameters=[{
            'ring': -1,
            'resolution': 0.007
        }]
    )

    # Static frame transform from 'laser' (URDF) to 'velodyne' (Driver)
    # This ensures the lidar data is correctly placed in the robot's TF tree.
    tf_laser_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_velodyne',
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'velodyne'],
        output='screen'
    )
    
    # Jupiter 드라이버 노드 - IMU 원시 데이터 발행
    # IMU 센서에서 원시 데이터를 읽어 /jupiter/imu 토픽으로 발행
    driver_params_path = os.path.join(
        jupiter_bringup_dir, 
        'param', 
        'jupiter_driver_params.yaml'
    )
    
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
    
    # Jupiter 베이스 노드 - 오도메트리 계산 및 발행
    # /jupiter/get_vel 토픽을 구독하여 로봇의 속도를 적분하고 오도메트리 계산
    # /odom 토픽으로 오도메트리 데이터 발행 및 odom -> base_link TF 변환 발행
    # jupiter_base_node = Node(
    #     package='jupiter_base',
    #     executable='base_node',
    #     name='base_node',
    #     output='screen',
    #     parameters=[{
    #         'linear_scale': 1.0,
    #         # 🎯 Phase 5: angular_scale 재조정 (2025-11-10)
    #         # 90도 회전 테스트 결과 기반
    #         'angular_scale': 2.0251,
    #         'is_multi_robot': False
    #     }],
    #     remappings=[
    #         ('/tf', '/tf_ignored') # Disable TF from fake odom to avoid conflict with EKF/RF2O
    #     ]
    # )
    
    # IMU 보정 노드 - imu_calib 패키지 사용 ⭐
    # MPU6050의 자이로 바이어스와 센서 오차를 보정
    # 흐름: jupiter_driver → /jupiter/imu → apply_calib → /imu/data_calibrated → EKF
    #
    # imu_calib.yaml에 저장된 캘리브레이션 데이터:
    # - gyro bias (정지 상태 드리프트 제거)
    # - accelerometer bias
    # - scale factors
    imu_calib_node = Node(
        package='imu_calib',
        executable='apply_calib_node',
        name='apply_calib',
        namespace='',
        parameters=[
            {'calib_file': '/home/jetson/jupiter_ws_ros2/imu_calib.yaml'},
            {'calibrate_gyros': True},
            {'gyro_calib_samples': 500}  # 더 정확한 캘리브레이션을 위해 500 샘플
        ],
        remappings=[
            ('/raw', '/jupiter/imu'),
            ('/corrected', '/imu/data_calibrated')
        ],
        output='screen',
        emulate_tty=True
    )
    
    # KISS-ICP Node (LiDAR Odometry replacing RF2O)
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', '/velodyne_points'),
            ('odometry', '/kiss/odometry') 
        ],
        parameters=[{
            'base_frame': 'base_link',
            'lidar_odom_frame': 'odom', 
            'publish_odom_tf': False, # EKF handles TF
            'publish_debug_clouds': True,
            'invert_odom_tf': True, # Fix yaw inversion
            'deskew': False
        }]
    )

    # Robot Localization EKF 설정 - jupiter_nav 패키지 config 사용 ⭐
    ekf_config_file = os.path.join(jupiter_nav_dir, 'config', 'ekf_config.yaml')
    
    # 1. Local EKF: Odom -> Base_link (Existing)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )
    
    # =========================================================
    # GPS Integration Setup (New)
    # =========================================================
    
    # 2. GPS Launch Inclusion (Requires rtcm_ublox_bridge package)
    rtcm_ublox_bridge_dir = get_package_share_directory('rtcm_ublox_bridge')
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtcm_ublox_bridge_dir, 'launch', 'ngii_rtk_full.launch.py')
        )
    )
    
    # 3. Global EKF Config: Map -> Odom
    ekf_gps_config_file = os.path.join(jupiter_nav_dir, 'config', 'ekf_gps_config.yaml')
    
    # 4. Global EKF Node
    ekf_filter_gps_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_gps_config_file],
        remappings=[
            ('odometry/filtered', 'odometry/global')
        ]
    )
    
    # 5. NavSat Transform Node
    # Converts GPS (Lat/Lon) to Odom (X/Y)
    # Input: /imu/data_calibrated, /ublox_gps/fix, /odometry/filtered
    # Output: /odometry/gps, /gps/filtered
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_gps_config_file],
        remappings=[
            ('imu/data', '/imu/data_calibrated'),
            ('gps/fix', '/fix'), # Corrected to match the active topic
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # 6. Static TF for GPS
    # Transform from base_link to gps_link
    tf_base_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_gps',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link'],
        output='screen'
    )
    
    # use_sim_time 파라미터
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        use_robot_state_pub_arg,
        use_joint_state_pub_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        jupiter_driver_node,
        imu_calib_node,  # ⭐ imu_calib 패키지로 자이로 바이어스 보정
        #jupiter_base_node,
        kiss_icp_node, # Replaces RF2O
        robot_localization_node,
        gps_launch,          # GPS Driver + RTCM
        ekf_filter_gps_node, # Global EKF (Map->Odom)
        navsat_transform_node, # GPS->Odom conversion
        tf_base_gps,         # Base->GPS TF
        velodyne_driver_node,    # Added
        velodyne_transform_node, # Added
        velodyne_laserscan_node, # Added
        tf_laser_velodyne,
    ])
