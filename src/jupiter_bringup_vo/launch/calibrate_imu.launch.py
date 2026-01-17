from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directory
    pkg_prefix = FindPackageShare('jupiter_bringup')
    
    # Driver node
    driver_node = Node(
        package='jupiter_bringup',
        executable='jupiter_driver',
        name='jupiter_node',
        output='screen',
        parameters=[{
            'imu': '/jupiter/imu',
            'vel': '/jupiter/get_vel',
            'CameraDevice': 'astra'
        }]
    )
    
    # IMU Calibration node
    imu_calib_node = Node(
        package='imu_calib',
        executable='do_calib_node',
        name='do_calib',
        output='screen',
        parameters=[{
            'output_file': PathJoinSubstitution([
                pkg_prefix,
                'param',
                'imu',
                'imu_calib.yaml'
            ]),
            'measurements': 300,  # 각 방향당 측정 횟수 (기본값 500에서 300으로 줄임)
            'reference_acceleration': 9.80665  # 기준 중력 가속도 (m/s²)
        }],
        remappings=[
            ('/imu', '/jupiter/imu')  # IMU 토픽 리맵핑
        ]
    )
    
    return LaunchDescription([
        driver_node,
        imu_calib_node
    ])
