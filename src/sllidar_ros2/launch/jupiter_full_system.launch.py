#!/usr/bin/env python3

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
    jupiter_nav_dir = get_package_share_directory('jupiter_nav')  # ⭐ 새 패키지
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    jupiter_description_dir = get_package_share_directory('jupiter_description')
    jupiter_bringup_dir = get_package_share_directory('jupiter_bringup')
    
    # 라이다 시리얼 포트 파라미터 추가
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    
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
    
    # 라이다 포트 설정 인수 추가
    declare_lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/jupiter_lidar',
        description='Port for the LiDAR device'
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
    
    # RPLidar A1 성능 최적화 설정
    # 라이다가 후방을 향함: URDF yaw=180°로 TF 정렬, 데이터는 그대로 사용
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,          # 데이터 그대로 사용 (TF가 좌표 변환 처리)
            'angle_compensate': True,
            'scan_frequency': 5.0,      # 스캔 주파수 (안정적인 성능)
            'range_min': 0.05,          # 최소 거리
            'range_max': 22.0,          # 최대 거리 (전체 범위)
            'scan_time': 0.2,           # 스캔 시간
            'publish_intensity': False,
            'angle_min': -1.5,
            'angle_max': 1.5
        }],
        output='screen'
    )
    
    # RViz2 실행 - launch argument로 제어
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Use RViz2 visualization'
    )
    
    rviz_config_file = os.path.join(sllidar_ros2_dir, 'rviz', 'sllidar.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
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
    jupiter_base_node = Node(
        package='jupiter_base',
        executable='base_node',
        name='base_node',
        output='screen',
        parameters=[{
            'linear_scale': 1.2,      # ROS1 캘리브레이션 값
            # 🎯 Phase 4: Hz 개선 (2025-11-03)
            # Hz 개선: 10Hz → 50Hz로 증가
            # angular_scale: 원래 값으로 복구 (SLAM 작동 확인 필요)
            # TODO: ekf_comparison_test 결과 재검토 필요
            'angular_scale': 1.8819,
            'is_multi_robot': False
        }]
    )
    
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
            {'gyro_calib_samples': 100}
        ],
        remappings=[
            ('/raw', '/jupiter/imu'),
            ('/corrected', '/imu/data_calibrated')
        ],
        output='screen',
        emulate_tty=True
    )
    
    # IMU 필터 노드 비활성화 - EKF가 raw IMU 직접 처리 (TF 충돌 방지)
    # 이유:
    # 1. robot_localization (EKF)가 센서 융합 (Odom + IMU)을 담당
    # 2. imu_filter_madgwick와 ekf_filter_node가 동시에 /tf 발행하여 충돌
    # 3. EKF가 raw IMU를 직접 처리하는 것이 더 강력 (드리프트 보정)
    # 
    # imu_filter_node = Node(
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',
    #     name='imu_filter_madgwick',
    #     parameters=[{
    #         'use_mag': False,
    #         'publish_tf': False,
    #         'world_frame': 'enu',
    #         'fixed_frame': 'base_link',
    #         'gain': 0.005,
    #         'zeta': 0.001,
    #         'remove_gravity_vector': True,
    #         'stateless': False,
    #         'constant_dt': 0.1
    #     }],
    #     remappings=[
    #         ('imu/data_raw', '/imu/data_calibrated'),
    #         ('imu/data', '/imu/data_filtered')
    #     ],
    #     output='screen'
    # )
    
    # TF 스태틱 변환은 URDF에서 정의된 joint들로 자동 발행됨 (robot_state_publisher가 담당)
    # 개별적인 static_transform_publisher는 URDF와 충돌하므로 제거
    # 대신 robot_state_publisher가 jupiter_simple.urdf의 모든 joint를 처리
    
    # SLAM Toolbox 파라미터 설정 - jupiter_nav 패키지 config 사용 ⭐
    slam_params_file = os.path.join(jupiter_nav_dir, 'config', 'slam_params.yaml')
    
    # Robot Localization EKF 설정 - jupiter_nav 패키지 config 사용 ⭐
    ekf_config_file = os.path.join(jupiter_nav_dir, 'config', 'ekf_config.yaml')
    
    # Robot Localization 노드 설정
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )
    
    # AMCL 사용 여부 설정
    use_nav2_slam_arg = DeclareLaunchArgument(
        'use_nav2_slam',
        default_value='false',
        description='Use Nav2 SLAM for localization'
    )
    
    # Nav2 SLAM 실행을 위한 인클루드 설정 (jupiter_localization 패키지 삭제됨 - 주석 처리)
    # nav2_slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('jupiter_localization'), 'launch', 'jupiter_full_system.launch.py')
    #     ),
    #     condition=IfCondition(LaunchConfiguration('use_nav2_slam'))
    # )
    
    # SLAM 노드는 AMCL이 사용되지 않을 때만 실행
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file],
        output='screen'
    )
    
    # 5초 지연 후 SLAM 시작 - TF 트리가 완전히 준비될 때까지 대기
    slam_toolbox_node_conditional = TimerAction(
        period=5.0,
        actions=[slam_toolbox_node],
        condition=UnlessCondition(LaunchConfiguration('use_nav2_slam'))
    )
    
    # 데이터 흐름 요약:
    # 1. jupiter_driver_node: IMU 원시 데이터와 속도 데이터 발행
    #    - IMU 원시 데이터를 /jupiter/imu 토픽으로 발행
    #    - 모터 속도 정보를 /jupiter/get_vel 토픽으로 발행
    # 2. jupiter_base_node: /jupiter/get_vel을 구독하여 오도메트리 계산
    #    - 속도 데이터를 적분하여 위치 추정
    #    - 오도메트리 데이터를 /odom 토픽으로 발행 및 odom->base_link TF 발행
    # 3. imu_calib_node: /jupiter/imu를 구독하여 imu_calib.yaml의 보정 파라미터 적용
    #    - 보정된 데이터를 /imu/data_calibrated 토픽으로 발행
    # 4. imu_filter_node: /imu/data_calibrated를 구독하여 Madgwick 필터 적용
    #    - 필터링된 데이터를 /imu/data 토픽으로 발행
    # 5. robot_localization_node(EKF): /imu/data와 /odom을 구독하여 위치 추정
    #    - 융합된 위치 정보를 /odometry/filtered 토픽으로 발행
    
    # use_sim_time 파라미터
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # RViz 조건부 실행
    rviz_condition = GroupAction(
        actions=[rviz_node],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_robot_state_pub_arg,
        use_joint_state_pub_arg,
        use_nav2_slam_arg,
        use_rviz_arg,
        use_sim_time_arg,
        declare_lidar_port_arg,
        robot_state_publisher,
        joint_state_publisher,
        jupiter_driver_node,
        imu_calib_node,  # ⭐ imu_calib 패키지로 자이로 바이어스 보정
        jupiter_base_node,
        # imu_filter_node,  # TF 충돌 방지 - EKF가 calibrated IMU 직접 처리
        robot_localization_node,
        sllidar_node,
        rviz_condition,
        slam_toolbox_node_conditional,
    ])
