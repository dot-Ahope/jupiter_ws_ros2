#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # 패키지 경로
    # velodyne_dir = get_package_share_directory('velodyne')
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
            'publish_frequency': 30.0
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

    # Jupiter 드라이버 노드 - 하드웨어 제어 (IMU 사용 안함: vSLAM이 처리)
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
    
    # =========================================================
    # Isaac vSLAM Configuration (Localization + Mapping)
    # =========================================================

    # RealSense Node for vSLAM (Stereo IR + IMU)
    # realsense_camera_node = ComposableNode(
    #     name='camera',
    #     namespace='camera',
    #     package='realsense2_camera',
    #     plugin='realsense2_camera::RealSenseNodeFactory',
    #     parameters=[{
    #         'enable_infra1': True,
    #         'enable_infra2': True,
    #         'enable_color': False,
    #         'enable_depth': False,
    #         'depth_module.emitter_enabled': 0,
    #         'depth_module.profile': '640x360x90', # High FPS for VIO
    #         'enable_gyro': True,
    #         'enable_accel': True,
    #         'gyro_fps': 200,
    #         'accel_fps': 200,
    #         'unite_imu_method': 2 # Linear interpolation
    #     }],
    # )

    # Isaac ROS Visual SLAM Node
    # TF Tree: map -> odom -> base_link
    # visual_slam_node = ComposableNode(
    #     name='visual_slam_node',
    #     package='isaac_ros_visual_slam',
    #     plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
    #     parameters=[{
    #         'enable_image_denoising': False,
    #         'rectified_images': True,
    #         'enable_imu_fusion': True,
    #         'gyro_noise_density': 0.000244,
    #         'gyro_random_walk': 0.000019393,
    #         'accel_noise_density': 0.001862,
    #         'accel_random_walk': 0.003,
    #         'calibration_frequency': 200.0,
    #         'image_jitter_threshold_ms': 22.00,
    #         'base_frame': 'base_link',
    #         'odom_frame': 'odom',
    #         'map_frame': 'map',
    #         'imu_frame': 'camera_gyro_optical_frame', # RealSense IMU frame
    #         'enable_slam_visualization': True,
    #         'enable_landmarks_view': True,
    #         'enable_observations_view': True,
    #         'publish_odom_to_base_tf': True, # vSLAM handles odom -> base_link
    #         'publish_map_to_odom_tf': True,  # vSLAM handles map -> odom
    #         'camera_optical_frames': [
    #             'camera_infra1_optical_frame',
    #             'camera_infra2_optical_frame',
    #         ],
    #     }],
    #     remappings=[
    #         ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
    #         ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
    #         ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
    #         ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
    #         ('visual_slam/imu', '/camera/imu'),
    #     ],
    # )

    # visual_slam_launch_container = ComposableNodeContainer(
    #     name='visual_slam_launch_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         realsense_camera_node,
    #         visual_slam_node
    #     ],
    #     output='screen',
    # )

    # Static TF: Robot Base -> Camera Link
    # Fixes position of camera on the robot
    tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=['0.2', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'camera_link'],
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
        tf_base_camera,
        # visual_slam_launch_container
    ])
