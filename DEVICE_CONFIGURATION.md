# Transbot ROS2 작업공간 장치 설정 가이드

## 시스템 정보
- **사용자**: jetson
- **홈 디렉토리**: /home/jetson
- **작업공간**: /home/jetson/transbot_ws_ros2

## 현재 시스템 장치 확인

### 시리얼 장치
```bash
ls -la /dev/tty* | grep -E "USB|THS|ACM"
```

현재 시스템에 확인된 장치:
- `/dev/ttyTHS1` - Jetson 내장 UART (Transbot 메인보드 연결용)
- `/dev/ttyTHS2` - Jetson 내장 UART (예비)
- `/dev/ttyUSB0` - USB 시리얼 장치 (GPS/GNSS 또는 LiDAR)
- `/dev/ttyUSB1` - USB 시리얼 장치 (GPS/GNSS 또는 LiDAR)

### 비디오 장치
```bash
ls -la /dev/video*
```

현재 시스템에 확인된 장치:
- `/dev/video0` - USB 2.0 카메라 또는 CSI 카메라
- `/dev/video1` - USB 2.0 카메라 또는 CSI 카메라

## 장치별 설정

### 1. Transbot 메인보드 (모터 제어)
- **포트**: `/dev/ttyTHS1` (Jetson UART)
- **설정 파일**: `src/transbot_bringup/transbot_bringup/transbot_driver.py`
- **보드레이트**: 115200
- **참고**: Jetson Nano/NX의 40핀 헤더 UART 포트 사용

### 2. GPS/GNSS (u-blox F9P)
- **포트**: `/dev/ttyUSB0` 또는 `/dev/ttyUSB1`
- **설정 파일들**:
  - `src/transbot_bringup/param/cors_config.yaml`
  - `src/transbot_bringup/param/ublox/minimal_config.yaml`
  - `src/transbot_bringup/param/ublox/f9p_rover_params.yaml`
  - `src/transbot_bringup/launch/rtk_test.launch.py`
- **보드레이트**: 115200 또는 460800
- **확인 방법**:
  ```bash
  ls -la /dev/ttyUSB* && lsusb | grep -i "u-blox\|GPS"
  ```

### 3. LiDAR (Slamtec RPLidar)
- **포트**: `/dev/ttyUSB0` 또는 `/dev/ttyUSB1`
- **설정 파일**: `src/sllidar_ros2/launch/*.launch.py`
- **보드레이트**: 115200 또는 256000 (모델에 따라 다름)
- **확인 방법**:
  ```bash
  ls -la /dev/ttyUSB* && lsusb | grep -i "slamtec\|rplidar\|cp210"
  ```

### 4. RGB 카메라 (USB 2.0 또는 Astra)
- **포트**: `/dev/video0` 또는 `/dev/video1`
- **설정 파일**:
  - `src/transbot_bringup/launch/bringup.launch.py` (camera_device 파라미터)
  - `src/transbot_bringup/transbot_bringup/device_srv.py`
- **지원 모드**:
  - `usb`: USB 2.0 카메라
  - `astra`: Orbbec Astra 카메라 (UVC 모드)
- **확인 방법**:
  ```bash
  v4l2-ctl --list-devices
  lsusb | grep -i "camera\|astra\|orbbec"
  ```

### 5. Astra 카메라 (Depth)
- **udev 규칙**: `99-astra-camera.rules` → `/etc/udev/rules.d/`
- **권한 설정 스크립트**: `setup_camera_permissions.sh`
- **확인 방법**:
  ```bash
  lsusb | grep -i "2bc5\|orbbec"
  ```

## 장치 포트 자동 인식 및 할당

다른 시스템에서 복사해온 작업공간이므로, 장치 포트 번호가 변경될 수 있습니다.

### USB 장치 포트 확인 방법

1. **모든 USB 시리얼 장치 확인**:
   ```bash
   ls -la /dev/ttyUSB*
   dmesg | tail -50 | grep ttyUSB
   ```

2. **장치별 제조사 확인**:
   ```bash
   # u-blox GPS 확인
   lsusb | grep -i "u-blox"
   
   # LiDAR 확인 (CP210x 칩 사용)
   lsusb | grep -i "cp210\|silicon\|slamtec"
   
   # Astra 카메라 확인
   lsusb | grep -i "2bc5\|orbbec\|astra"
   ```

3. **udev를 이용한 고정 포트 설정** (권장):
   `/etc/udev/rules.d/99-usb-serial.rules` 파일 생성:
   ```
   # u-blox F9P GPS
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps_ublox"
   
   # RPLidar
   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
   ```
   
   적용:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

## 설정 변경이 필요한 파일 목록

### 시리얼 포트 관련
1. `src/transbot_bringup/transbot_bringup/transbot_driver.py` - Transbot 메인보드 (ttyTHS1)
2. `src/transbot_bringup/param/cors_config.yaml` - GPS 포트
3. `src/transbot_bringup/param/ublox/*.yaml` - u-blox GPS 설정
4. `src/transbot_bringup/launch/rtk_test.launch.py` - GPS 포트
5. `src/sllidar_ros2/launch/*.launch.py` - LiDAR 포트
6. `src/rtcm_ublox_bridge/launch/*.launch.py` - RTCM 브릿지 포트

### 카메라 관련
1. `src/transbot_bringup/launch/bringup.launch.py` - 카메라 타입 및 장치 번호
2. `src/transbot_bringup/transbot_bringup/device_srv.py` - 카메라 초기화

## 시스템 권한 설정

### 시리얼 포트 권한
```bash
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인 필요
```

### 카메라 권한
```bash
sudo usermod -a -G video $USER
# Astra 카메라의 경우 추가 설정
sudo ./setup_camera_permissions.sh
```

## Transbot 라이브러리 관련

⚠️ **중요**: 현재 시스템에 Transbot 하드웨어 라이브러리가 설치되어 있지 않습니다.

### 필요한 작업
1. Transbot 라이브러리 설치 또는 복사
   - 이전 시스템: `/home/user/Transbot/transbot`
   - 현재 시스템: `/home/jetson/Transbot/transbot` (예상 위치)

2. 또는 `transbot_driver.py`에서 하드웨어 연동 부분 주석 처리 후 시뮬레이션 모드로 테스트

### 임시 해결책
`src/transbot_bringup/transbot_bringup/device_srv.py`에서:
- Transbot 라이브러리 import 주석 처리됨
- 하드웨어 없이 카메라만 사용 가능

## 빌드 및 테스트

### 1. 작업공간 빌드
```bash
cd ~/transbot_ws_ros2
colcon build --symlink-install
source install/setup.bash
```

### 2. 장치 확인 스크립트 실행
```bash
./check_all_sensors.sh
./check_camera_sensors.sh
```

### 3. 개별 노드 테스트
```bash
# 카메라만 테스트
ros2 launch transbot_bringup bringup.launch.py

# LiDAR 테스트
ros2 launch sllidar_ros2 sllidar_launch.py

# GPS 테스트
ros2 launch transbot_bringup rtk_test.launch.py
```

## 문제 해결

### 시리얼 포트를 찾을 수 없는 경우
1. 장치가 물리적으로 연결되어 있는지 확인
2. `dmesg | tail` 명령으로 연결 로그 확인
3. 권한 확인: `ls -la /dev/ttyUSB* /dev/ttyTHS*`

### 카메라를 열 수 없는 경우
1. 다른 프로세스가 카메라를 사용 중인지 확인
2. 권한 확인: `ls -la /dev/video*`
3. 카메라 연결 확인: `v4l2-ctl --list-devices`

### Transbot 라이브러리 관련 오류
1. 라이브러리 경로 확인: `/home/jetson/Transbot/transbot`
2. 임시로 하드웨어 연동 비활성화하고 시뮬레이션으로 테스트

## 참고 문서
- `CAMERA_SENSOR_STRUCTURE.md` - 카메라 센서 구조
- `CAMERA_TROUBLESHOOTING.md` - 카메라 문제 해결
- `ENCODER_DEBUG_GUIDE.md` - 인코더 디버깅
- `RTABMAP_COMPLETE_GUIDE.md` - RTAB-Map 설정

---
**작성일**: 2025-11-14
**시스템**: Jetson (Nano/NX)
**ROS2 버전**: Humble
