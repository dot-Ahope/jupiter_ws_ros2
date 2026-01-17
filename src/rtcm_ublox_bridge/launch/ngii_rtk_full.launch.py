from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    # u-blox GPS 패키지의 설정 파일 경로 가져오기
    # ublox_config = os.path.join(
        #   get_package_share_directory('ublox_gps'),
        # 'config',
        # 'ublox_f9p.yaml' #사용하는 GPS 모델에 맞게 설정 파일 사용
    # )

    # Launch parameters
    device_param = LaunchConfiguration('device')
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/rtk_gps',
        description='GPS device port'
    )
    
    baudrate_param = LaunchConfiguration('baudrate')
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='38400',
        description='GPS baudrate'
    )
    
    #-- 국토지리원 NTRIP 서버 정보 --
    ngii_host_param = LaunchConfiguration('ngii_host')
    ngii_host_arg = DeclareLaunchArgument(
        'ngii_host',
        default_value='rts2.ngii.go.kr',  # 국토지리원 NTRIP 서버
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
        default_value='VRS-RTCM31',  # 국토지리원 마운트포인트, 최신 RTCM3.2 MSM4 프로토콜 권장
        description='NGII CORS mountpoint'
    )
    
    username_param = LaunchConfiguration('username')
    username_arg = DeclareLaunchArgument(
        'username',
        default_value='cjinwook94',  # 여기에 실제 국토지리원 계정 입력
        description='NGII CORS username'
    )
    
    password_param = LaunchConfiguration('password')
    password_arg = DeclareLaunchArgument(
        'password',
        default_value='ngii',  # 여기에 실제 국토지리원 비밀번호 입력
        description='NGII CORS password'
    )
    
    
    # Imports should be at top usually, but inside function works if indented correctly.
    # However, standard practice is top of file. 
    # Let's just fix indentation here.
    return LaunchDescription([
        device_arg,
        baudrate_arg,
        ngii_host_arg,
        ngii_port_arg,
        mountpoint_arg,
        username_arg,
        password_arg,
        
        # NTRIP 클라이언트 (Custom Script Direct Execution)
        ExecuteProcess(
            cmd=['python3', '/home/jetson/jupiter_ws_ros2/src/rtcm_ublox_bridge/rtcm_ublox_bridge/simple_ntrip.py',
                 '--ros-args', 
                 '-p', 'host:=rts2.ngii.go.kr',
                 '-p', 'port:=2101',
                 '-p', 'mountpoint:=VRS-RTCM31',
                 '-p', 'username:=cjinwook94',
                 '-p', 'password:=ngii'
                ],
            output='screen'
        ),


        
        # u-blox GPS 노드
        # /rtcm 토픽을 해당 노드가 직접 구독, 받은 데이터를 GPS 수신기로 보냄
        # ublox_gps 패키지에 RTK설정 포함, 브릿지 노드 불필요
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps',
            output='screen',
            parameters=[{
                'device': device_param,
                'frame_id': 'gps_link',

                # --- 통신 및 프로토콜 설정 ---
                'uart1.baudrate': baudrate_param,
                'uart1.in': 1,      # UBX 프로토콜 입력 활성화
                'uart1.out': 1,     # UBX 프로토콜 출력 활성화

                # ---측위 설정 ---
                'dynamic_model': 'portable',  # 휴대용 모델, 필요시 다른 모델로 변경
                'nav_rate': 1,  # HPG 장치의 경우 1Hz 필요(기본값), 업데이트 속도에 맞춰 늘릴 수 있음

                # --- RTK 설정 ---
                'rtcm_in.subscribe': True,  # /rtcm 토픽 구독 활성화
                'rtcm_in.topic': '/rtcm',

                'inf.uart1.rtcm_in': True,  # UART1을 통한 RTCM 입력 활성화
                'tmode3' : 0,  # 고정모드 (0: 고정모드, 1: 이동모드, 2: 정지모드)

                # --- NMEA 메시지 설정(NTRIP용) ---
                'nmea.gga' : True,  # GGA 메시지 활성화

                # --- 퍼블리시 설정 ---
                'publish.all': True,  # 모든 측위 데이터 퍼블리시
            }]
        )
    ])
