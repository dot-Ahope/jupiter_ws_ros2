# Transbot ROS2 작업공간 마이그레이션 완료 보고서

## 작업 개요
다른 시스템에서 복사해온 `transbot_ws_ros2` 작업공간의 디렉토리 경로 및 장치 드라이버 인터페이스를 현재 시스템에 맞게 수정 완료

**작업 일자**: 2025-11-14  
**대상 시스템**: Jetson (사용자: jetson)

---

## 수정된 주요 사항

### 1. 사용자 경로 변경
- **이전**: `/home/user/`
- **현재**: `/home/jetson/`

#### 수정된 파일:
1. `sensor_based_angular_calibration.py`
   - Launch 파일 경로 참조 변경
   
2. `scripts/log_imu_and_distance.py`
   - 파일 경로 주석 변경

### 2. Transbot 하드웨어 라이브러리 경로
**파일**: `src/transbot_bringup/transbot_bringup/device_srv.py`

#### 변경 사항:
```python
# 변경 전:
import sys
sys.path.append("/home/jetson/Transbot/transbot")

# 변경 후:
import sys
import os
# Transbot 라이브러리 경로 - 필요시 설치 또는 경로 수정 필요
# sys.path.append("/home/jetson/Transbot/transbot")
```

**참고**: Transbot 하드웨어 라이브러리가 현재 시스템에 설치되어 있지 않음. 필요시 별도 설치 필요.

### 3. 쉘 스크립트 경로 수정
모든 쉘 스크립트에서 하드코딩된 경로를 동적 경로로 변경:

#### 수정된 스크립트:
1. **setup_camera_permissions.sh**
   ```bash
   # 변경 전:
   sudo cp ~/transbot_ws_ros2/99-astra-camera.rules /etc/udev/rules.d/
   
   # 변경 후:
   SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
   sudo cp "$SCRIPT_DIR/99-astra-camera.rules" /etc/udev/rules.d/
   ```

2. **run_camera_test.sh**
   ```bash
   # 변경 전:
   cd ~/transbot_ws_ros2
   
   # 변경 후:
   SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
   cd "$SCRIPT_DIR"
   ```

3. **restart_camera_with_uvc.sh**
   - 동일한 방식으로 동적 경로 적용

4. **run_rtabmap_rgbd.sh**
   - 동일한 방식으로 동적 경로 적용

### 4. 기타 파일 경로 주석 수정
**파일**: `src/transbot_bringup/transbot_bringup/rtcm_ublox_bridge.py`
```python
# 변경 전: # filepath: ~/cors_config/rtcm_ublox_bridge.py
# 변경 후: # filepath: /home/jetson/transbot_ws_ros2/src/transbot_bringup/transbot_bringup/rtcm_ublox_bridge.py
```

---

## 시스템 장치 확인

### 현재 시스템에서 사용 가능한 장치

#### 시리얼 포트:
- `/dev/ttyTHS1` - Jetson 내장 UART (Transbot 메인보드용)
- `/dev/ttyTHS2` - Jetson 내장 UART (예비)
- `/dev/ttyUSB0` - USB 시리얼 장치 (GPS/LiDAR)
- `/dev/ttyUSB1` - USB 시리얼 장치 (GPS/LiDAR)

#### 비디오 장치:
- `/dev/video0` - USB/CSI 카메라
- `/dev/video1` - USB/CSI 카메라

---

## 장치별 설정 위치

### 1. Transbot 메인보드 (모터 제어)
- **포트**: `/dev/ttyTHS1`
- **파일**: `src/transbot_bringup/transbot_bringup/transbot_driver.py`
- **현재 상태**: 설정 완료 (변경 불필요)

### 2. GPS/GNSS (u-blox)
- **기본 포트**: `/dev/ttyUSB0`
- **설정 파일**:
  - `src/transbot_bringup/param/cors_config.yaml`
  - `src/transbot_bringup/param/ublox/minimal_config.yaml`
  - `src/transbot_bringup/param/ublox/f9p_rover_params.yaml`
  - `src/transbot_bringup/launch/rtk_test.launch.py`

### 3. LiDAR (RPLidar)
- **기본 포트**: `/dev/ttyUSB0` 또는 `/dev/ttyUSB1`
- **설정 파일**: `src/sllidar_ros2/launch/*.launch.py`

### 4. 카메라 (USB/Astra)
- **비디오 포트**: `/dev/video0`, `/dev/video1`
- **설정 파일**:
  - `src/transbot_bringup/launch/bringup.launch.py`
  - `src/transbot_bringup/transbot_bringup/device_srv.py`

---

## 빌드 확인

### 빌드 명령어:
```bash
cd /home/jetson/transbot_ws_ros2
colcon build --symlink-install
```

### 빌드 결과:
✅ **성공**
- `transbot_bringup` 패키지: 4.05초 만에 빌드 완료
- 오류 없음

---

## 생성된 문서

### DEVICE_CONFIGURATION.md
전체 장치 설정 및 문제 해결 가이드 문서를 생성했습니다.

**내용**:
- 시스템 정보
- 현재 장치 목록
- 장치별 설정 방법
- 포트 자동 인식 및 고정 설정 방법
- udev 규칙 설정 가이드
- 권한 설정
- 빌드 및 테스트 방법
- 문제 해결 팁

**위치**: `/home/jetson/transbot_ws_ros2/DEVICE_CONFIGURATION.md`

---

## 추가 작업이 필요한 사항

### 1. Transbot 하드웨어 라이브러리 설치 ⚠️
현재 시스템에 Transbot 라이브러리가 없습니다.

#### 해결 방법:
**옵션 A**: 라이브러리 설치
```bash
# 이전 시스템에서 복사
scp -r user@old-system:/home/user/Transbot /home/jetson/

# 또는 Git에서 클론 (리포지토리가 있는 경우)
# git clone <transbot-library-repo> /home/jetson/Transbot
```

**옵션 B**: 시뮬레이션 모드로 테스트
- `transbot_driver.py`에서 하드웨어 초기화 부분 주석 처리
- 카메라 및 센서만 사용하여 테스트

### 2. USB 장치 포트 확인
GPS와 LiDAR가 모두 USB 시리얼을 사용하므로, 실제 연결된 포트 확인 필요:

```bash
# 장치 확인
lsusb | grep -E "u-blox|CP210|Slamtec"

# 연결 로그 확인
dmesg | tail -20

# 포트 확인
ls -la /dev/ttyUSB*
```

### 3. udev 규칙으로 고정 포트 설정 (권장)
`/etc/udev/rules.d/99-usb-serial.rules` 생성:
```bash
# GPS
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps_ublox"

# LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
```

적용:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4. 카메라 권한 설정
```bash
# Astra 카메라 권한 설정
./setup_camera_permissions.sh

# 로그아웃 후 재로그인
```

---

## 테스트 방법

### 1. 전체 빌드
```bash
cd /home/jetson/transbot_ws_ros2
colcon build --symlink-install
source install/setup.bash
```

### 2. 장치 확인
```bash
# 모든 센서 확인
./check_all_sensors.sh

# 카메라만 확인
./check_camera_sensors.sh
```

### 3. 개별 노드 테스트
```bash
# 카메라 테스트
ros2 launch transbot_bringup bringup.launch.py

# LiDAR 테스트
ros2 launch sllidar_ros2 sllidar_launch.py

# GPS 테스트
ros2 launch transbot_bringup rtk_test.launch.py
```

---

## 변경된 파일 목록

### Python 파일:
1. `src/transbot_bringup/transbot_bringup/device_srv.py`
2. `src/transbot_bringup/transbot_bringup/rtcm_ublox_bridge.py`
3. `sensor_based_angular_calibration.py`
4. `scripts/log_imu_and_distance.py`

### Shell 스크립트:
1. `setup_camera_permissions.sh`
2. `run_camera_test.sh`
3. `restart_camera_with_uvc.sh`
4. `run_rtabmap_rgbd.sh`

### 생성된 문서:
1. `DEVICE_CONFIGURATION.md` - 장치 설정 가이드 (신규)
2. `MIGRATION_SUMMARY.md` - 이 파일 (신규)

---

## 요약

### ✅ 완료된 작업:
1. ✓ 사용자 경로 수정 (`/home/user` → `/home/jetson`)
2. ✓ Transbot 라이브러리 경로 처리 (주석 처리)
3. ✓ 쉘 스크립트 동적 경로 변환
4. ✓ 빌드 성공 확인
5. ✓ 장치 설정 문서 작성
6. ✓ 마이그레이션 요약 문서 작성

### ⚠️ 추가 작업 필요:
1. Transbot 하드웨어 라이브러리 설치
2. USB 장치 포트 확인 및 매핑
3. udev 규칙으로 고정 포트 설정
4. 카메라 권한 설정 및 테스트
5. 전체 시스템 통합 테스트

---

## 참고 문서
- `DEVICE_CONFIGURATION.md` - 전체 장치 설정 가이드
- `CAMERA_SENSOR_STRUCTURE.md` - 카메라 센서 구조
- `CAMERA_TROUBLESHOOTING.md` - 카메라 문제 해결
- `RTABMAP_COMPLETE_GUIDE.md` - RTAB-Map 설정

---

**마이그레이션 완료 상태**: ✅ 기본 설정 완료  
**다음 단계**: 하드웨어 연결 및 통합 테스트
