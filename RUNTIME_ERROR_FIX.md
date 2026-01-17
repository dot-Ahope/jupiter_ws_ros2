# 실행 오류 해결 가이드

**발생 시점**: 2025-11-14 13:23  
**명령어**: `ros2 launch transbot_nav transbot_full_system.launch.py use_rviz:=true`

---

## 발견된 문제 (3개)

### 1. ❌ 시리얼 포트 권한 오류 (중요)

**에러 메시지**:
```
[Errno 13] Permission denied: '/dev/ttyTHS1'
```

**원인**: 
- 사용자 `jetson`이 `dialout` 그룹에 속해있지 않음
- `/dev/ttyTHS1`의 권한: `crw-rw---- root:dialout`

**해결 방법**:

**영구 해결** (권장):
```bash
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인 또는
newgrp dialout
```

**임시 해결** (현재 세션만):
```bash
sudo chmod 666 /dev/ttyTHS1
```

### 2. ❌ IMU 캘리브레이션 파일 경로 오류

**에러 메시지**:
```
Failed to load calibration file: bad file: /home/user/transbot_ws_ros2/imu_calib.yaml
```

**원인**: 
- 하드코딩된 경로 `/home/user/` (이전 시스템)
- 현재 시스템: `/home/jetson/`

**해결됨**: ✅
- 4개 launch 파일에서 경로 수정 완료
- 재빌드 완료

**수정된 파일**:
1. `src/sllidar_ros2/launch/transbot_full_system.launch.py`
2. `src/sllidar_ros2/launch/transbot_full_system_with_rtabmap.launch.py`
3. `src/transbot_nav/launch/transbot_full_system.launch.py`
4. `src/transbot_nav/launch/transbot_hybrid_slam.launch.py`

### 3. ❌ LiDAR 포트 오류

**에러 메시지**:
```
[sllidar_node]: Error, unexpected error, code: 80008004
```

**원인**: 
- LiDAR가 연결되지 않았거나
- 잘못된 시리얼 포트 지정

**현재 상태**:
- LiDAR 하드웨어 감지됨: `Silicon Labs CP210x UART Bridge`
- `/dev/ttyUSB0` 존재 확인됨

**확인 필요**:
- LiDAR가 실제로 `/dev/ttyUSB0`에 연결되어 있는지
- 다른 프로세스가 포트를 사용 중인지

---

## 해결 순서

### Step 1: 권한 수정 (필수)

**옵션 A - 영구 해결** (권장):
```bash
# 1. dialout 그룹 추가
sudo usermod -a -G dialout $USER

# 2. 새 그룹 적용 (재로그인 대신)
newgrp dialout

# 3. 확인
groups
# 출력에 'dialout'이 포함되어야 함
```

**옵션 B - 임시 해결** (현재 세션만):
```bash
# 스크립트 실행
./fix_permissions.sh
```

### Step 2: 재빌드 (이미 완료됨)

```bash
cd /home/jetson/transbot_ws_ros2
colcon build --symlink-install --packages-select sllidar_ros2 transbot_nav
```

### Step 3: 실행

```bash
cd /home/jetson/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav transbot_full_system.launch.py use_rviz:=true
```

---

## 예상 결과

### 해결 후 정상 로그:

```
[transbot_driver] [INFO] Rosmaster hardware initialized successfully on /dev/ttyTHS1
[apply_calib_node] [INFO] Calibration loaded from: /home/jetson/transbot_ws_ros2/imu_calib.yaml
[sllidar_node] [INFO] SLLIDAR health status OK
[base_node] [INFO] Base node has been initialized
[ekf_node] [INFO] filter initialized successfully
```

---

## 문제별 우선순위

### 🔴 Critical (실행 불가)
1. **시리얼 포트 권한** - transbot_driver 실행 불가

### 🟡 Warning (기능 제한)
2. **IMU 캘리브레이션 경로** - 수정됨 ✅
3. **LiDAR 연결** - 하드웨어 확인 필요

---

## 빠른 해결 (복사 & 붙여넣기)

```bash
# 1. 권한 설정
sudo usermod -a -G dialout $USER
newgrp dialout

# 2. 환경 설정 및 실행
cd /home/jetson/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav transbot_full_system.launch.py use_rviz:=true
```

---

## 추가 디버깅

### LiDAR 문제 해결 (필요시)

```bash
# LiDAR 포트 확인
ls -la /dev/ttyUSB*

# LiDAR 연결 테스트
ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0

# 다른 포트 시도
ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB1
```

### Transbot 드라이버 단독 테스트

```bash
# 드라이버만 실행
ros2 run transbot_bringup transbot_driver

# IMU 데이터 확인
ros2 topic echo /transbot/imu

# 속도 명령 테스트
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
```

---

## 참고 파일

- **권한 수정 스크립트**: `./fix_permissions.sh`
- **IMU 캘리브레이션**: `/home/jetson/transbot_ws_ros2/imu_calib.yaml`
- **드라이버 파라미터**: `install/transbot_bringup/share/transbot_bringup/param/transbot_driver_params.yaml`

---

**상태**: 권한 문제만 해결하면 실행 가능 ✅
