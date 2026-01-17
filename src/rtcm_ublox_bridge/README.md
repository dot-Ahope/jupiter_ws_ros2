# RTCM U-blox Bridge for NGII RTK

이 패키지는 국토지리정보원(NGII)의 NTRIP 서버(RTS2)로부터 RTCM 보정 신호를 수신하여, U-blox F9P GPS 모듈에 전달함으로써 정밀 측위(RTK)를 수행하기 위한 브릿지 패키지입니다.

## 주요 기능
- **NTRIP Client**: Python 기반의 경량 클라이언트(`simple_ntrip.py`)를 내장하여 외부 패키지 의존성 없이 동작합니다.
- **VRS 지원**: Virtual Reference Station 구동을 위해 주기적으로 NMEA GGA 메시지를 서버로 전송합니다.
- **ROS 2 통합**: `rtcm_msgs`를 통해 표준화된 방식으로 데이터를 중계합니다.

## 하드웨어 설정 (Device Setup)
RTK GPS 장치는 `/dev/rtk_gps`라는 고정된 심볼릭 링크를 사용하도록 설정되어 있습니다. (udev rule 적용됨)
- Baudrate: 38400

## 실행 방법 (Usage)

전체 시스템(NTRIP 클라이언트 + U-blox 드라이버)을 한번에 실행합니다.

```bash
ros2 launch rtcm_ublox_bridge ngii_rtk_full.launch.py
```

### 주요 파라미터 (Parameters)
런치 파일(`ngii_rtk_full.launch.py`) 내 기본값은 다음과 같이 설정되어 있습니다. 필요 시 변경하여 실행하십시오.

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `device` | `/dev/rtk_gps` | GPS 장치 경로 |
| `baudrate` | `38400` | 시리얼 통신 속도 |
| `ngii_host` | `rts2.ngii.go.kr` | NTRIP 서버 주소 |
| `ngii_port` | `2101` | NTRIP 포트 |
| `mountpoint` | `VRS-RTCM31` | RTCM 마운트 포인트 (보정 데이터 형식) |
| `username` | `cjinwook94` | NGII 사용자 ID |
| `password` | `ngii` | NGII 비밀번호 |

## 파일 구성
- **launch/ngii_rtk_full.launch.py**: 전체 시스템 실행 런치 파일.
- **rtcm_ublox_bridge/simple_ntrip.py**: 커스텀 NTRIP 클라이언트 노드 (Python).

## 트러블슈팅
- **연결 실패**: 네트워크 핑(`ping rts2.ngii.go.kr`) 및 포트(`nc -zv rts2.ngii.go.kr 2101`) 상태를 확인하세요.
- **데이터 없음**: 실내에서는 GPS 위성 신호가 잡히지 않아 Fix가 되지 않을 수 있으나, `/rtcm` 토픽은 네트워크만 연결되면 데이터가 들어와야 합니다 (`ros2 topic hz /rtcm` 으로 확인).

## TF (Transform) 구성 방법론

이 패키지를 통해 RTK GPS 위치(`/fix`)와 오도메트리 데이터가 확보되면, 로봇 좌표계(TF) 구성을 위해 다음 단계가 필요합니다.

### 1. `navsat_transform_node` 활용
`robot_localization` 패키지의 `navsat_transform_node`를 사용하여 WGS84 위경도 좌표(GPS)를 로봇의 오도메트리 좌표계(`odom` 또는 `map`)로 변환합니다.

#### 필요 데이터
- **IMU 데이터**: `/imu/data` (방향, 특히 Yaw 값 보정용)
- **GPS 데이터**: `/fix` (현재 U-blox 노드에서 발행 중)
- **Odometry 데이터**: `/odometry/filtered` (EKF 노드 출력)

#### 설정 예시 (`ekf_config` 또는 별도 launch 파일)
```yaml
navsat_transform:
  ros__parameters:
    frequency: 30.0
    magnetic_declination_radians: 0.0  # 지역 자북 편차 적용 필요
    yaw_offset: 1.5707963  # IMU 설치 방향에 따라 조정 (보통 0 또는 pi/2)
    zero_altitude: true
    broadcast_utm_transform: true  # 'utm' -> 'odom' (또는 'map') 변환 발행
    publish_filtered_gps: true
    use_odometry_yaw: false
    wait_for_datum: false
```

### 2. TF 트리 구조 제안
일반적인 실외 자율주행 로봇의 TF 트리는 다음과 같습니다.

```
map  (전역 좌표계)
 └── odom  (지역 좌표계 - odometry 누적)
      └── base_link  (로봇 중심)
           ├── imu_link  (IMU 센서 위치)
           ├── velodyne  (라이다 센서 위치)
           └── gps_link  (GPS 안테나 위치 - *중요*)
```

### 3. `gps_link` 설정 (URDF 또는 정적 TF)
GPS 안테나의 실제 부착 위치를 `base_link` 기준으로 정의해야 합니다. `tf2_ros`의 정적 변환 발행기(`static_transform_publisher`)를 런치 파일에 추가하십시오.

**예시 (런치 파일에 추가):**
```python
# base_link에서 gps_link로의 정적 변환 (x=0.1m, y=0.0m, z=0.2m 위치에 안테나가 있는 경우)
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='tf_base_gps',
    arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'gps_link']
)
```

이 설정을 완료하면 `navsat_transform_node`가 GPS 좌표를 받아 `map` -> `odom` 변환을 계산하고, 전체 로봇의 위치를 지도 상에 표시할 수 있게 됩니다.

