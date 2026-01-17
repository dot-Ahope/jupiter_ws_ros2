# LiDAR Timeout 및 SLAM 맵 생성 실패 해결

**날짜**: 2025년 11월 25일  
**시스템**: transbot_ws_ros2 (ROS2 Humble)  
**하드웨어**: RPLIDAR A1M8, Rosmaster X3, Jetson Nano/NX

---

## 1. 문제 상황

### 증상
- `ros2 launch transbot_nav transbot_full_system.launch.py` 실행 시 LiDAR 노드 타임아웃 발생
- 에러 메시지: `[ERROR] [sllidar_node-7]: Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!`
- SLAM Toolbox가 실행되지만 맵이 생성되지 않음
- `/scan` 토픽에 데이터가 발행되지 않음

### 부가 증상
- IMU 캘리브레이션 경고: "Gyro calibration quality poor (variance: 0.xxxx)"
- SLAM Toolbox 메시지 필터 큐 오버플로우 (맵 생성 후 정상)

---

## 2. 원인 분석

### 근본 원인
**Launch 파일의 install 디렉토리에 잘못된 시리얼 포트 설정이 캐싱됨**

### 상세 분석

#### 2.1 하드웨어 구성
```
/dev/ttyUSB0 (CP2102, VID:10c4 PID:ea60) → RPLIDAR A1M8
  ↓ 심볼릭 링크
/dev/lidar_port

/dev/ttyUSB1 (CH340, VID:1a86 PID:7523) → Rosmaster X3
  ↓ 심볼릭 링크
/dev/myserial
```

#### 2.2 설정 불일치 발견
1. **소스 파일** (`src/transbot_nav/launch/transbot_full_system.launch.py`):
   ```python
   lidar_port = LaunchConfiguration('lidar_port', default='/dev/lidar_port')  # 올바름
   ```

2. **Install 디렉토리** (`install/transbot_nav/share/transbot_nav/launch/`):
   ```python
   lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')  # 잘못됨!
   ```

3. **실제 전달된 파라미터** (`/tmp/launch_params_*`):
   ```yaml
   serial_port: /dev/ttyUSB1  # Rosmaster 포트로 연결 시도!
   ```

#### 2.3 문제 재현 과정
```
Launch 실행
  ↓
install/ 디렉토리의 오래된 설정 사용 (/dev/ttyUSB1)
  ↓
sllidar_node가 Rosmaster 포트로 LiDAR 프로토콜 통신 시도
  ↓
Rosmaster는 다른 프로토콜(0xFF/0xFC) 사용 → LiDAR 명령(0xA5) 무응답
  ↓
타임아웃 발생 → 노드 종료
  ↓
/scan 토픽 미발행 → SLAM 맵 생성 불가
```

---

## 3. 검증 과정

### 3.1 하드웨어 통신 검증

#### RPLIDAR A1M8 (✅ 정상)
```bash
# GET_INFO 명령 테스트
python3 -c "
import serial
ser = serial.Serial('/dev/lidar_port', 115200)
ser.write(bytes([0xA5, 0x50]))  # GET_INFO
response = ser.read(27)
print(f'Model: {response[7]} (0x{response[7]:02x})')
print(f'Firmware: {response[9]}.{response[8]}')
print(f'Hardware: {response[10]}')
"
```
**결과**: Model 24 (0x18), Firmware 1.29, Hardware 7

#### Rosmaster X3 (✅ 정상)
```python
# 50개 패킷 수신 테스트
# 프로토콜: 0xFF(HEAD) + 0xFC(DEVICE_ID) + ...
# 패킷 타입: 0x0A(Speed), 0x0C(Attitude), 0x0D(Encoder), 0x0E(IMU Raw)
```
**결과**: 100% 성공률, 모든 센서 데이터 정상

### 3.2 드라이버 단독 실행 테스트
```bash
# 올바른 포트로 직접 실행
ros2 run sllidar_ros2 sllidar_node \
  --ros-args \
  -p serial_port:=/dev/lidar_port \
  -p serial_baudrate:=115200
```
**결과**: ✅ 정상 작동
```
[INFO] SLLidar S/N: BAF4FA86C2E392D0A5E59FF72D205C60
[INFO] Firmware Ver: 1.29, Hardware Rev: 7
[INFO] current scan mode: Sensitivity, sample rate: 8 Khz, max_distance: 12.0 m
```

### 3.3 Launch 파일 파라미터 추적
```bash
# 실제 전달된 파라미터 확인
cat /tmp/launch_params_*
```
**발견**: `serial_port: /dev/ttyUSB1` (잘못된 값)

---

## 4. 해결 방법

### 4.1 실행한 명령
```bash
cd /home/jetson/transbot_ws_ros2
colcon build --packages-select transbot_nav
```

### 4.2 해결 원리
- `colcon build`는 `src/` 디렉토리의 소스 파일을 읽어 `install/` 디렉토리에 최종 파일 생성
- 이전에는 소스 파일만 수정하고 빌드하지 않아 install/ 디렉토리가 업데이트되지 않음
- 재빌드 후 올바른 `/dev/lidar_port` 설정이 install/ 디렉토리에 반영됨

### 4.3 빌드 결과
```
Starting >>> transbot_nav
Finished <<< transbot_nav [3.74s]

Summary: 1 package finished [4.29s]
```

### 4.4 검증
```bash
# Install 디렉토리 확인
grep "default=" install/transbot_nav/share/transbot_nav/launch/transbot_full_system.launch.py | grep lidar_port
```
**결과**: `default='/dev/lidar_port'` ✅

---

## 5. 최종 결과

### 5.1 시스템 정상 작동 확인

#### LiDAR 노드 시작
```
[sllidar_node-7] [INFO] [sllidar_node]: SLLidar S/N: BAF4FA86C2E392D0A5E59FF72D205C60
[sllidar_node-7] [INFO] [sllidar_node]: Firmware Ver: 1.29
[sllidar_node-7] [INFO] [sllidar_node]: Hardware Rev: 7
[sllidar_node-7] [INFO] [sllidar_node]: SLLidar health status : OK.
[sllidar_node-7] [INFO] [sllidar_node]: current scan mode: Sensitivity, sample rate: 8 Khz, max_distance: 12.0 m, scan frequency:5.0 Hz
```

#### /scan 토픽 발행
```bash
ros2 topic echo /scan --once
```
**결과**: ✅ 정상 데이터 수신
- 1800+ 포인트 (360도 전체 스캔)
- 거리 측정: 2.5~2.8m 범위
- Intensity 값: 47

#### SLAM 맵 생성
```bash
ros2 topic echo /map --once
```
**결과**: ✅ 맵 생성 성공
- 해상도: 0.05m (5cm)
- 맵 크기: 52x86 그리드
- 점유 데이터: 100 (장애물), -1 (미탐색)

### 5.2 실행 중인 노드
```
/apply_calib          # IMU 캘리브레이션
/base_node            # 베이스 제어
/ekf_filter_node      # 센서 융합
/joint_state_publisher
/robot_state_publisher
/slam_toolbox         # SLAM 맵핑
/sllidar_node         # LiDAR 드라이버 ✅
/transbot_driver      # 로봇 드라이버
```

---

## 6. 추가 발견 사항

### 6.1 IMU 캘리브레이션 경고 (비중요)
**경고**: "Gyro calibration quality poor (variance: 0.xxxx)"

**원인**: 분산 계산 알고리즘 오류
```cpp
// 현재 (잘못된 계산)
variance = |gx - gy| + |gz - gy| + |gz - gx|  // 서로 다른 축 비교

// 올바른 계산 (미적용)
variance = E[X²] - E[X]²  // 시간에 따른 분산
```

**영향**: 경고 메시지만 발생, 캘리브레이션은 정상 완료
**우선순위**: 낮음 (기능적 문제 없음)

### 6.2 중복 udev 규칙
**발견**: `/etc/udev/rules.d/`에 여러 LiDAR 규칙 존재
- `serial.rules`: lidar_port → ttyUSB0
- `rplidar.rules`: rplidar → ttyUSB0
- `ydlidar.rules`: ydlidar → ttyUSB0

**영향**: 심볼릭 링크 중복 생성, 현재는 문제 없음
**권장**: 사용하지 않는 규칙 제거 또는 백업

---

## 7. 교훈 및 권장사항

### 7.1 ROS2 빌드 시스템 이해
- `src/` 디렉토리 수정 후 **반드시** `colcon build` 실행
- Launch 파일은 `install/` 디렉토리에서 실행됨
- 소스 수정만으로는 변경사항이 반영되지 않음

### 7.2 디버깅 전략
1. **하드웨어 먼저 검증**: 직접 시리얼 통신으로 하드웨어 동작 확인
2. **드라이버 단독 실행**: Launch 없이 노드 직접 실행하여 파라미터 문제 분리
3. **파라미터 추적**: `/tmp/launch_params_*` 파일로 실제 전달된 값 확인
4. **소스 vs Install 비교**: `grep` 명령으로 두 디렉토리 차이 확인

### 7.3 예방 조치
```bash
# Launch 파일 수정 후 항상 실행
cd /home/jetson/transbot_ws_ros2
colcon build --packages-select <modified_package>
source install/setup.bash
```

---

## 8. 참조 파일

### 주요 수정 파일
- **Launch 파일** (간접 수정, 빌드로 해결):
  - `src/transbot_nav/launch/transbot_full_system.launch.py`
  - Line 24: `default='/dev/lidar_port'`

### 관련 설정 파일
- **udev 규칙**: `/etc/udev/rules.d/serial.rules`
- **LiDAR 드라이버**: `src/sllidar_ros2/src/sllidar_node.cpp`
- **Rosmaster 라이브러리**: `/usr/local/lib/python3.10/dist-packages/Rosmaster_Lib/Rosmaster_Lib.py`

### 하드웨어 정보
```
RPLIDAR A1M8:
  - Model ID: 0x18 (24)
  - Firmware: 1.29
  - Hardware Rev: 7
  - Serial: BAF4FA86C2E392D0A5E59FF72D205C60
  - USB: CP2102 (VID:10c4, PID:ea60)
  - Device: /dev/ttyUSB0 → /dev/lidar_port

Rosmaster X3:
  - IMU: ICM20948
  - USB: CH340 (VID:1a86, PID:7523)
  - Device: /dev/ttyUSB1 → /dev/myserial
  - Protocol: 0xFF/0xFC header
```

---

## 9. 결론

**문제**: Launch 파일의 install 디렉토리에 오래된 포트 설정이 남아있어 LiDAR가 잘못된 시리얼 포트로 연결 시도

**해결**: `colcon build --packages-select transbot_nav` 실행으로 올바른 설정 반영

**결과**: LiDAR 정상 작동, /scan 데이터 발행, SLAM 맵 생성 성공

**소요 시간**: 문제 발견부터 해결까지 약 2시간 (하드웨어 검증 + 파라미터 추적 + 빌드)

**핵심 교훈**: ROS2에서 Launch 파일 수정 후 반드시 빌드 필요, 소스 수정만으로는 불충분
