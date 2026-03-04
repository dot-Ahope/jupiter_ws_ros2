import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_jupiter_nav_vo = get_package_share_directory('jupiter_nav_VO')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_jupiter_description = get_package_share_directory('jupiter_description')
    pkg_jupiter_bringup = get_package_share_directory('jupiter_bringup')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # map_yaml_file removed as we use VSLAM map (or no map server)
    
    # 0. Robot Description & Driver (Hardware Interface)
    # Load URDF (Includes camera_link definition)
    urdf_file = os.path.join(pkg_jupiter_description, 'urdf', 'jupiter_simple.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # TF bridge from base_link to camera_link (RealSense)
    # Connecting the robot base to the camera tree to fix "unconnected trees" error.
    # Isaac ROS Visual SLAM은 'camera_link'를 'camera_frame'으로 사용하고 있음
    # 따라서 base_link -> camera_link 변환이 필수적임
    # 업데이트(2026-02-04): 사용자 실측값 적용 (X=0.274, Z=0.055)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.274', '0', '0.055', '0', '0', '0', 'base_link', 'camera_link'] # camera_link를 child로
    )

    # Jupiter Driver (Low level serial comms)
    # [하드웨어 통신 노드 설명]
    # 이 노드는 로봇의 하단 MCU(마이크로 컨트롤러)와 시리얼 통신을 담당합니다.
    # 역할 1: 상위 제어기(Nav2 등)로부터 속도 명령을 받아 모터를 제어합니다.
    # 역할 2: MCU로부터 센서 데이터(IMU, 배터리 전압 등)를 수신하여 ROS 토픽으로 발행합니다.
    # 주의: base_node가 제거되었으므로, 이 노드가 수신하는 '/vel' 토픽에 Nav2의 '/cmd_vel'이 연결되는지 확인이 필요할 수 있습니다.
    #       현재 설정에서는 '/vel'이 '/jupiter/get_vel'로 리매핑되어 있습니다.
    driver_params = os.path.join(pkg_jupiter_bringup, 'param', 'jupiter_driver_params.yaml')
    
    # 기존 Driver Node - compensated driver로 교체(임시방편)
    # driver_node = Node(
    #     package='jupiter_bringup',
    #     executable='jupiter_driver',
    #     name='jupiter_driver',
    #     output='screen',
    #     parameters=[driver_params],
    #     remappings=[('/imu', '/jupiter/imu'), ('/vel', '/jupiter/get_vel')]
    # )

    # Compensated Driver Node (MCU exponential turn ratio 문제에 대한 보상처리 - turn_ratio가 제곱이고, 5000으로 나누고 그럼)
    driver_node = Node(
        package='jupiter_bringup',
        executable='jupiter_driver_compensated',
        name='jupiter_driver', # Keep node name same just in case
        output='screen',
        parameters=[driver_params],
        remappings=[('/imu', '/jupiter/imu'), ('/vel', '/jupiter/get_vel')]
    )

    # 2. Nav2 Bringup (Navigation only) - REMOVED for separate testing
    # We manually launch nav2 via 'nav2_vslam_navigation.launch.py' or similar.
    
    # NOTE: Since we removed the base_node (kinematics/odom) and EKF,
    # we assume VSLAM Node publishes the 'odom' -> 'base_link' TF.
    # User must ensure VSLAM is configured with base_frame='base_link'.
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rsp_node,
        tf_base_to_camera,
        driver_node
    ])
