from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os.path

def generate_launch_description():
    pkg_prefix = FindPackageShare('jupiter_bringup')
    desc_prefix = FindPackageShare('jupiter_description')
    
    # Launch arguments
    use_gui = LaunchConfiguration('use_gui')
    
    # Parameter files
    driver_params_path = PathJoinSubstitution(
        [pkg_prefix, 'param', 'jupiter_driver_params.yaml']
    )
    
    # Launch arguments declaration
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Whether to use joint_state_publisher_gui'
    )
    
    # Nodes
    device_srv_node = Node(
        package='jupiter_bringup',
        executable='device_srv',
        name='device_srv',
        output='screen',
        parameters=[{
            'camera_device': 'usb',  # USB 2.0 Camera 사용 (RGB)
            'image_width': 640,
            'image_height': 480,
            'camera_fps': 15
        }]
    )
    
    driver_node = Node(
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

    # Base node: publishes /odom_raw and broadcasts odom->base transforms
    base_node = Node(
        package='jupiter_base',
        executable='base_node',
        name='base_node',
        output='screen',
        parameters=[{
            'linear_scale': 1.2,      # ROS1 캘리브레이션 값
            # 방향별 angular scale (비대칭 보정)
            # - ekf_comparison_test 결과 기반 (2025-10-24)
            # - 반시계: IMU 89.94° / Raw 51.48° = 1.747
            # - 시계:   IMU 89.22° / Raw 62.82° = 1.420
            'angular_scale': 1.5618,      # 기본값 (하위 호환)
            'angular_scale_ccw': 1.747,   # 반시계 방향 보정
            'angular_scale_cw': 1.420,    # 시계 방향 보정
            'is_multi_robot': False
        }]
    )
    
    # Static transform publisher for IMU
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0.0', '0.0', '0.02', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )
    
    # IMU 교정 노드 - 가속도계 및 자이로스코프 교정 값 적용
    imu_calib_node = Node(
        package='imu_calib',
        executable='apply_calib_node',
        name='apply_calib',
        output='screen',
        parameters=[{
            'calib_file': PathJoinSubstitution([pkg_prefix, 'param', 'imu', 'imu_calib.yaml']),
            'calibrate_gyros': True,
            'gyro_calib_samples': 100
        }],
        remappings=[
            ('/raw', '/jupiter/imu'),
            ('/corrected', '/jupiter/imu_corrected')
        ]
    )

    # IMU Filter Madgwick 노드 비활성화 - EKF가 raw IMU 직접 처리 (TF 충돌 방지)
    # 이유:
    # 1. robot_localization (EKF)가 센서 융합 (Odom + IMU)을 담당
    # 2. imu_filter_madgwick와 ekf_filter_node가 동시에 /tf 발행하여 충돌
    # 3. EKF가 raw IMU를 직접 처리하는 것이 더 강력 (드리프트 보정)
    # 
    # imu_filter_node = Node(
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',
    #     name='imu_filter_madgwick',
    #     output='screen',
    #     parameters=[{
    #         'use_mag': False,
    #         'publish_tf': False,
    #         'use_magnetic_field_msg': False,
    #         'world_frame': 'enu',
    #         'orientation_stddev': 0.05,
    #         'gain': 0.01,
    #         'zeta': 0.01,
    #         'angular_scale': 1.08,
    #         'fixed_frame': 'base_link',
    #         'remove_gravity_vector': True,
    #         'do_bias_estimation': True,
    #         'do_adaptive_gain': True
    #     }],
    #     remappings=[
    #         ('/imu/data_raw', '/jupiter/imu_corrected'),
    #         ('/imu/data', '/imu/data')
    #     ]
    # )
    
    # Robot Localization - EKF (raw IMU 직접 사용)
    ekf_params_path = PathJoinSubstitution([pkg_prefix, 'param', 'ekf', 'robot_localization.yaml'])
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path],
        remappings=[
            ('/odometry/filtered', '/odom')
        ]
    )
    
    xacro_file = PathJoinSubstitution([desc_prefix, 'urdf', 'jupiter_simple.urdf'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gui)
    )
    
    # NOTE: 카메라는 RealSense D455F 사용. 별도 launch에서 실행:
    #   - isaac_ws_ros: VSLAM + nvblox (Docker 또는 native)
    #   - /scan은 RPLidar에서 직접 제공
    
    return LaunchDescription([
        declare_use_gui,
        device_srv_node,
        driver_node,
        base_node,
        static_tf_node,
        imu_calib_node,        # IMU 교정 노드
        # imu_filter_node,     # TF 충돌 방지 - EKF가 raw IMU 직접 처리
        ekf_node,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
    ])

