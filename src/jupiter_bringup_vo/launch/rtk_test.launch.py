from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch parameters
    device_param = LaunchConfiguration('device')
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description='GPS device port'
    )
    
    baudrate_param = LaunchConfiguration('baudrate')
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='38400',
        description='GPS baudrate'
    )
    
    ngii_host_param = LaunchConfiguration('ngii_host')
    ngii_host_arg = DeclareLaunchArgument(
        'ngii_host',
        default_value='ntrip.ngii.go.kr',  # 국토지리원 NTRIP 서버
        description='NGII CORS host address'
    )
    
    ngii_port_param = LaunchConfiguration('ngii_port')
    ngii_port_arg = DeclareLaunchArgument(
        'ngii_port',
        default_value='2101',
        description='NGII CORS port'
    )
    
    mountpoint_param = LaunchConfiguration('mountpoint')
    mountpoint_arg = DeclareLaunchArgument(
        'mountpoint',
        default_value='VRS-RTCM31',  # 또는 사용 가능한 NGII 마운트포인트
        description='NGII CORS mountpoint'
    )
    
    username_param = LaunchConfiguration('username')
    username_arg = DeclareLaunchArgument(
        'username',
        default_value='your_username',
        description='NGII CORS username'
    )
    
    password_param = LaunchConfiguration('password')
    password_arg = DeclareLaunchArgument(
        'password',
        default_value='your_password',
        description='NGII CORS password'
    )
    
    return LaunchDescription([
        device_arg,
        baudrate_arg,
        ngii_host_arg,
        ngii_port_arg,
        mountpoint_arg,
        username_arg,
        password_arg,
        
        # u-blox GPS 노드
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps',
            output='screen',
            parameters=[{
                'device': device_param,
                'frame_id': 'gps_link',
                'uart1.baudrate': baudrate_param,
                'publish.all': True,
                'nav_rate': 1,  # HPG 장치의 경우 1Hz 필요
                'rate': 1,
                'uart1.in': 1,   # 입력 활성화
                'uart1.out': 1,  # 출력 활성화
                'rtcm.ids': [1005, 1077, 1087, 1097, 1127, 1230], # RTCM 메시지 ID 설정
                'rtcm.rate': [0, 1, 1, 1, 1, 1],   # 각 RTCM 메시지 수신 주기
                'dynamic_model': 'portable'  # 다이나믹 모델 설정
            }]
        ),

        # NTRIP 클라이언트 (국립국토지리원 CORS)
        Node(
            package='ntrip_client',
            executable='ntrip_client_node',
            name='ntrip_client',
            output='screen',
            parameters=[{
                'host': ngii_host_param,
                'port': ngii_port_param,
                'mountpoint': mountpoint_param,
                'username': username_param,
                'password': password_param,
                'rtcm_message_package': 'rtcm_msgs',
                'use_nmea': True,  # NMEA GGA 메시지 자동 생성
                'nmea_from_fix': True  # NavSatFix 메시지에서 NMEA GGA 생성
            }]
        ),
        
        # RTCM-UBLOX 브릿지 노드
        Node(
            package='rtcm_ublox_bridge',
            executable='rtcm_ublox_bridge_node',
            name='rtcm_ublox_bridge',
            output='screen',
            parameters=[{
                'serial_port': device_param,
                'baudrate': baudrate_param,
                'rtcm_topic': '/rtcm'
            }]
        )
    ])
