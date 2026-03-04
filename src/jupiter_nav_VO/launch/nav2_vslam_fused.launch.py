#!/usr/bin/env python3
# =============================================================================
# VSLAM + RF2O + Wheel Odom + IMU + SLAM Toolbox 센서 융합 Launch 파일
# =============================================================================
#
# 아키텍처:
#   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
#   │ Isaac VSLAM   │  │ RF2O LiDAR   │  │ Wheel Odom   │  │ IMU (Calib)  │
#   │ /visual_slam/ │  │  Odometry    │  │ /odom_raw    │  │ /imu/data_   │
#   │  tracking/    │  │ /odom_rf2o   │  │ (base_node)  │  │  calibrated  │
#   │  odometry     │  │              │  │              │  │              │
#   └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘
#          │                 │                  │                  │
#          │  ┌──────────────┤          ┌───────▼───────┐         │
#          │  │              │          │ odom_cov_     │         │
#          │  │              │          │ adapter       │         │
#          │  │              │          │ /odom_adapted │         │
#          │  │              │          └───────┬───────┘         │
#          │  │              │                  │                  │
#          └──┼──────────────┼──────────────────┼──────────────────┘
#             │              ▼                  ▼
#             │  ┌─────────────────────────────────────┐
#             │  │  robot_localization (EKF)            │
#             │  │  odom0: wheel, odom1: VSLAM,        │
#             │  │  odom2: RF2O, imu0: IMU             │
#             │  │  TF: odom → base_footprint          │
#             │  └──────────────┬───────────────────────┘
#             │                 │
#   ┌─────────▼─────────┐      │
#   │  RPLidar S2L       │     │
#   │  /scan             │     │
#   └─────────┬──────────┘     │
#             │                │
#   ┌─────────▼────────────────▼───┐
#   │  SLAM Toolbox                │
#   │  TF: map → odom              │
#   │  /map                        │
#   └──────────────────────────────┘
#
# 사용법:
#   ros2 launch jupiter_nav_VO nav2_vslam_fused.launch.py
#
# 전제 조건:
#   - Isaac ROS Visual SLAM 노드는 별도로 실행 (Docker 컨테이너 또는 별도 launch)
#   - RealSense 카메라가 연결되어 있어야 함
#   - RPLidar S2L이 /dev/lidar_port에 연결되어 있어야 함
#
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # ============================================================
    # 패키지 경로
    # ============================================================
    pkg_jupiter_nav_vo = get_package_share_directory('jupiter_nav_VO')
    pkg_jupiter_nav = get_package_share_directory('jupiter_nav')
    pkg_jupiter_description = get_package_share_directory('jupiter_description')
    pkg_jupiter_bringup = get_package_share_directory('jupiter_bringup')
    
    # ============================================================
    # Launch Arguments
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Launch Foxglove Bridge for visualization'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/lidar_port',
        description='Port for the LiDAR device'
    )
    
    # ============================================================
    # 1. Robot Description (URDF)
    # ============================================================
    urdf_file = os.path.join(pkg_jupiter_description, 'urdf', 'jupiter_simple.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # ============================================================
    # 2. Static TF: base_link → camera_link
    # ============================================================
    # RealSense D455f 장착 위치 (사용자 실측값)
    # X=0.274m (전방), Y=0m (중앙), Z=0.055m (높이)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.274', '0', '0.055', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    # ============================================================
    # 3. Jupiter Driver (MCU 시리얼 통신)
    # ============================================================
    # 역할:
    #   - /cmd_vel 수신 → MCU 모터 제어
    #   - MCU IMU 데이터 → /jupiter/imu 발행 (100Hz)
    #   - MCU 엔코더 속도 → /jupiter/get_vel 발행 (50Hz)
    driver_params = os.path.join(pkg_jupiter_bringup, 'param', 'jupiter_driver_params.yaml')
    
    jupiter_driver = Node(
        package='jupiter_bringup',
        executable='jupiter_driver_compensated',
        name='jupiter_driver',
        output='screen',
        parameters=[driver_params],
        remappings=[
            ('/imu', '/jupiter/imu'),
            ('/vel', '/jupiter/get_vel')
        ]
    )
    
    # ============================================================
    # 4. IMU Calibration
    # ============================================================
    # /jupiter/imu (raw) → imu_calib → /imu/data_calibrated
    imu_calib = Node(
        package='imu_calib',
        executable='apply_calib_node',
        name='apply_calib',
        parameters=[
            {'calib_file': '/home/jetson/jupiter_ws_ros2/imu_calib.yaml'},
            {'calibrate_gyros': True},
            {'gyro_calib_samples': 500}
        ],
        remappings=[
            ('/raw', '/jupiter/imu'),
            ('/corrected', '/imu/data_calibrated')
        ],
        output='screen',
        emulate_tty=True
    )
    
    # ============================================================
    # 5. Base Node (Wheel Odometry Dead-Reckoning)
    # ============================================================
    # /jupiter/get_vel (Twist) → 적분 → /odom_raw (Odometry, 공분산=0)
    # use_imu_for_odom=true: angular velocity를 IMU에서 가져옴
    jupiter_base = Node(
        package='jupiter_base',
        executable='base_node',
        name='base_node',
        output='screen',
        parameters=[{
            'linear_scale': 1.0,
            'angular_scale': 2.0251,     # IMU 미사용 시 fallback (사용 안 됨)
            'use_imu_for_odom': True,    # ← IMU angular velocity 사용
            'is_multi_robot': False
        }]
    )
    
    # ============================================================
    # 6. Odometry Covariance Adapter (동적 공분산)
    # ============================================================
    # /odom_raw (공분산=0) → 속도 기반 동적 공분산 적용 → /odom_adapted
    #
    # 필요 이유:
    #   - base_node는 공분산을 설정하지 않음 (모두 0)
    #   - robot_localization은 공분산 0이면 측정값으로 사용하지 않음
    #   - MCU 엔코더는 정적 공분산이 설정되어 있지 않으므로
    #     속도/가속/회전에 따라 동적으로 불확실성을 추정해야 함
    odom_covariance_adapter = Node(
        package='jupiter_nav_VO',
        executable='odom_covariance_adapter',
        name='odom_covariance_adapter',
        output='screen',
        parameters=[{
            'input_odom_topic': '/odom_raw',
            'output_odom_topic': '/odom_adapted',
            # 정지 시 기본 공분산 (엔코더 노이즈 수준)
            'base_linear_cov': 0.005,        # (m/s)²
            'base_angular_cov': 0.01,        # (rad/s)²
            'base_pose_cov': 0.01,           # m²
            'base_orient_cov': 0.02,         # rad²
            # 속도 임계값
            'stationary_linear_threshold': 0.01,    # m/s
            'stationary_angular_threshold': 0.02,   # rad/s
            'high_speed_threshold': 0.3,            # m/s
            # 동적 스케일
            'speed_cov_scale': 0.1,
            'rotation_cov_scale': 0.3,
            'accel_cov_scale': 0.5,
            'stationary_cov_factor': 0.1,
        }]
    )
    
    # ============================================================
    # 7. VSLAM Covariance Adapter (최소 공분산 부여)
    # ============================================================
    # Isaac ROS cuVSLAM은 covariance를 0으로 발행하는 경우가 있음
    # (특히 vo_status: 0 추적 실패 시)
    # → robot_localization이 무한 신뢰하여 EKF 발산 원인
    # → 최소 공분산을 부여하여 안전하게 융합
    vslam_covariance_adapter = Node(
        package='jupiter_nav_VO',
        executable='vslam_covariance_adapter',
        name='vslam_covariance_adapter',
        output='screen',
        parameters=[{
            'input_topic': '/visual_slam/tracking/odometry',
            'output_topic': '/visual_slam/tracking/odometry_adapted',
            'min_position_cov': 0.05,        # m² - VSLAM 위치 최소 불확실성
            'min_orientation_cov': 0.02,      # rad² - VSLAM 방향 최소 불확실성
            'min_linear_vel_cov': 0.01,       # (m/s)² - VSLAM 선속도 최소 불확실성
            'min_angular_vel_cov': 0.005,     # (rad/s)² - VSLAM 각속도 최소 불확실성
            'clamp_existing': True,
            # vo_state 게이팅 (선택적 — VisualSlamStatus import 성공 시에만)
            'status_topic': '/visual_slam/status',
            'enable_vo_state_gating': True,
            'good_vo_state': 1,              # vo_state=1 → tracking OK
            # Anomaly detection (2026-02-27: 항상 활성 — Docker 메시지 의존성 없음)
            'max_yaw_delta': 0.15,           # rad (~8.6°) 스텝당 최대 yaw 변화
            'max_pos_delta': 0.5,            # m 스텝당 최대 위치 변화
            'good_count_threshold': 30,      # 연속 30회 정상 → tracking 복구 판정
            # 공통 복구 파라미터
            'recovery_delay': 1.0,           # tracking 복구 후 1초 대기
            'recovery_cov_multiplier': 5.0,  # 복구 직후 공분산 5배 증폭
            'recovery_cov_decay_time': 3.0,  # 3초에 걸쳐 1.0x로 수렴
        }]
    )

    # ============================================================
    # 8. RPLidar S2L (LiDAR 스캐너)
    # ============================================================
    # RPLidar S2L: 1000000 baudrate, Standard 스캔 모드
    # /scan 토픽 발행 → RF2O + SLAM Toolbox에서 사용
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 1000000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen'
    )
    
    # ============================================================
    # 9. RF2O Laser Odometry (LiDAR 기반 오도메트리)
    # ============================================================
    # /scan → RF2O → /odom_rf2o (10Hz)
    # EKF에서 odom2로 사용 (절대 위치: x, y, yaw)
    # publish_tf=False: EKF가 TF를 발행하므로 충돌 방지
    # respawn=True: RF2O가 간헐적으로 종료되는 문제 대응 (2026-03-04)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }]
    )
    
    # ============================================================
    # 10. RF2O Covariance Adapter (최소 공분산 부여)
    # ============================================================
    # RF2O는 pose/twist covariance를 모두 0으로 발행함
    # → EKF가 RF2O를 무한 신뢰하여 발산 위험
    # → vslam_covariance_adapter 재사용하여 최소 공분산 부여
    # /odom_rf2o → /odom_rf2o_adapted
    # 주의: anomaly detection은 RF2O에 부적합 (VSLAM 전용 설계)
    #       → max_yaw_delta/max_pos_delta를 큰 값으로 설정하여 사실상 비활성
    # respawn=True: RF2O 노드 재시작 시 adapter도 함께 유지 (2026-03-04)
    rf2o_covariance_adapter = Node(
        package='jupiter_nav_VO',
        executable='vslam_covariance_adapter',
        name='rf2o_covariance_adapter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'input_topic': '/odom_rf2o',
            'output_topic': '/odom_rf2o_adapted',
            'min_position_cov': 0.1,         # m² - RF2O 위치 불확실성 (VSLAM보다 높음)
            'min_orientation_cov': 0.05,      # rad² - RF2O 방향 불확실성
            'min_linear_vel_cov': 0.01,       # (m/s)² - RF2O 선속도 불확실성
            'min_angular_vel_cov': 0.01,      # (rad/s)² - RF2O 각속도 불확실성
            'clamp_existing': True,
            'enable_vo_state_gating': False,   # RF2O에는 vo_state 없음
            'max_yaw_delta': 99.0,            # anomaly detection 사실상 비활성
            'max_pos_delta': 99.0,            # RF2O는 자체 필터링으로 충분
        }]
    )

    # ============================================================
    # 11. SLAM Toolbox (맵 생성 + map→odom TF)
    # ============================================================
    # SLAM Toolbox가 /scan + odom→base_footprint TF를 사용하여:
    #   - /map 토픽 발행 (OccupancyGrid)
    #   - map → odom TF 발행
    # 5초 지연: IMU 캘리브 + EKF 초기화 후 시작
    slam_params_file = os.path.join(pkg_jupiter_nav, 'config', 'slam_params.yaml')
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file],
        output='screen'
    )
    
    delayed_slam_toolbox = TimerAction(
        period=5.0,
        actions=[slam_toolbox_node]
    )

    # ============================================================
    # 12. Robot Localization (EKF 센서 융합)
    # ============================================================
    # 입력:
    #   odom0: /odom_adapted       (Wheel Odom + 동적 공분산)
    #   odom1: /visual_slam/tracking/odometry_adapted (Isaac VSLAM)
    #   odom2: /odom_rf2o_adapted  (RF2O LiDAR Odometry + 최소 공분산)
    #   imu0:  /imu/data_calibrated (MCU IMU 보정)
    # 출력:
    #   /odometry/filtered (융합된 오도메트리)
    #   TF: odom → base_footprint
    ekf_config = os.path.join(pkg_jupiter_nav_vo, 'config', 'ekf_vslam_fusion.yaml')
    
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odom')
        ]
    )
    
    # ============================================================
    # 12. Foxglove Bridge (선택적 시각화)
    # ============================================================
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            # 시각화에 필요한 토픽만 허용 — 전체 허용('.*') 시 고속 토픽이
            # WebSocket을 포화시켜 시각화 지연 발생 (IMU 100Hz, VSLAM 90Hz 등)
            'topic_whitelist': [
                '/odom',              # EKF 융합 출력 (30Hz)
                '/map',               # SLAM Toolbox 맵
                '/scan',              # LiDAR (10Hz)
                '/tf',                # TF 트리
                '/tf_static',         # 정적 TF
                '/cmd_vel',           # 속도 명령
                '/diagnostics',       # EKF 진단
                '/robot_description', # URDF
                '/odom_adapted',      # Wheel odom (디버깅용)
                '/odom_rf2o_adapted', # RF2O odom (디버깅용)
                '/visual_slam/tracking/odometry_adapted',  # VSLAM odom (디버깅용)
                '/imu/data_calibrated',  # IMU calibrated (gyro scale 진단)
                '/jupiter/imu',          # IMU raw from driver (gyro scale 진단)
                # Nav2 관련
                '/local_costmap/costmap',
                '/global_costmap/costmap',
                '/plan',              # 글로벌 경로
                '/local_plan',        # 로컬 경로
            ],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            # ⭐ /goal_pose → /goal_pose_raw: Foxglove에서 goal_pose_raw로 발행하면
            #    goal_pose_restamper가 Jetson 시간으로 재스탬핑 후 /goal_pose로 재발행
            #    (PC-Jetson 시계 차이 ~4s로 인한 TF extrapolation 방지)
            #    Foxglove 3D Panel 설정에서 Publish topic을 /goal_pose_raw로 변경 필요!
            'client_topic_whitelist': ['/cmd_vel', '/goal_pose_raw', '/initialpose'],
            'num_threads': 2,
            'send_buffer_limit': 10000000,   # 10MB — 대용량 맵 메시지 허용
            'max_qos_depth': 5,              # QoS 큐 줄여서 지연 감소
        }],
        condition=IfCondition(LaunchConfiguration('use_foxglove'))
    )
    
    # ============================================================
    # Launch Description
    # ============================================================
    # 노드 시작 순서:
    #   1. URDF + TF (즉시)
    #   2. Driver + IMU Calib (즉시)
    #   3. Base Node + Covariance Adapters (즉시)
    #   4. LiDAR + RF2O (즉시)
    #   5. EKF (3초 지연 → IMU 캘리브 완료 대기)
    #   6. SLAM Toolbox (5초 지연 → EKF TF 준비 대기)
    #   7. Foxglove (즉시)
    
    # EKF는 IMU 캘리브레이션 완료 후 시작 (gyro_calib_samples=500 @ 100Hz = ~5초)
    delayed_ekf = TimerAction(
        period=3.0,
        actions=[robot_localization]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        use_foxglove_arg,
        use_rviz_arg,
        lidar_port_arg,
        
        # 1. Robot Description
        robot_state_publisher,
        joint_state_publisher,
        
        # 2. Static TF
        tf_base_to_camera,
        
        # 3. Hardware Interface
        jupiter_driver,
        imu_calib,
        
        # 4. Wheel Odometry
        jupiter_base,
        odom_covariance_adapter,
        
        # 5. VSLAM Covariance Adapter
        vslam_covariance_adapter,
        
        # 6. LiDAR + RF2O Odometry
        sllidar_node,
        rf2o_node,
        rf2o_covariance_adapter,
        
        # 7. Sensor Fusion (delayed)
        delayed_ekf,
        
        # 8. SLAM Toolbox (delayed)
        delayed_slam_toolbox,
        
        # 9. Visualization
        foxglove_bridge,
    ])
