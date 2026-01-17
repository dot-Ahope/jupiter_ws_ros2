# IMU 및 Odometry 캘리브레이션 종합 가이드

> **생성일:** 2025-10-31  
> **통합 문서:** IMU 캘리브레이션, 오도메트리 정확도, 센서 정렬

## 📅 작업 타임라인

**작업 기간:** 2025년 9월 30일 ~ 10월 21일

### 주요 작업 일정
- **2025-09-30**: 초기 하드웨어 파라미터 분석 및 센서 정확도 분석 시작
- **2025-10-16**: 회전 테스트 수행, 270도 오버슈트 문제 발견
- **2025-10-17**: 센서 정확도 파라미터 분석, 캘리브레이션 스크립트 개선
- **2025-10-21**: IMU/Odometry 통합 캘리브레이션 플로우 완성

### 참조된 원본 문서들
- `HARDWARE_PARAMETER_ANALYSIS.md` (2025-10-21)
- `ROTATION_TEST_RESULT_20251016_140235.md` (2025-10-16)
- `SENSOR_ACCURACY_PARAMETERS_ANALYSIS.md` (2025-10-17)
- `COMPLETE_CALIBRATION_FLOW_ANALYSIS.md` (2025-10-21)
- `CALIBRATION_SCRIPT_IMPROVEMENTS.md` (2025-10-17)
- `DATA_FLOW_ANALYSIS.md` (2025-10-17)
- `CALIBRATION_VISUAL_DIAGRAMS.md` (2025-10-17)

---

## 📋 목차
1. [개요](#개요)
2. [IMU 캘리브레이션](#imu-캘리브레이션)
3. [Odometry 정렬](#odometry-정렬)
4. [센서 파라미터 분석](#센서-파라미터-분석)
5. [테스트 및 검증](#테스트-및-검증)

---

## 개요

Transbot의 정확한 위치 추정을 위해서는:
- **IMU 센서**: MPU6050의 정확한 캘리브레이션
- **Wheel Odometry**: 모터 인코더 기반 위치 추정
- **센서 정렬**: LiDAR, IMU, Odometry 간의 좌표계 일치

### 핵심 원칙
1. **IMU 우선**: 회전 정보는 IMU가 가장 정확
2. **Odometry 보조**: 직선 이동은 휠 오도메트리 사용
3. **센서 퓨전**: EKF로 두 센서 통합

---

## IMU 캘리브레이션

### 1. 자동 캘리브레이션 실행

```bash
# 1단계: 캘리브레이션 실행
ros2 run imu_calib imu_calib_node

# 2단계: 생성된 파일 확인
cat imu_calib.yaml

# 3단계: 설정 적용
# transbot_full_system.launch.py가 자동으로 로드
```

### 2. 캘리브레이션 파라미터

**imu_calib.yaml 구조:**
```yaml
imu_calib_node:
  ros__parameters:
    gyro_bias_x: 0.0034   # 자이로 바이어스 (rad/s)
    gyro_bias_y: -0.0012
    gyro_bias_z: 0.0089
    
    accel_bias_x: 0.123   # 가속도계 바이어스 (m/s²)
    accel_bias_y: -0.045
    accel_bias_z: 0.267
    
    # 캘리브레이션 상태
    is_calibrated: true
    calibration_samples: 1000
```

### 3. 캘리브레이션 품질 확인

```bash
# 캘리브레이션된 IMU 데이터 확인
ros2 topic echo /imu/data

# 기대값:
# - 정지 상태에서 angular_velocity: ~0.001 rad/s 이하
# - linear_acceleration.z: ~9.81 m/s² (중력)
# - orientation: 안정적인 값 유지
```

### 4. 수동 캘리브레이션 (필요시)

```python
# calibration.py 스크립트 사용
cd /home/user/transbot_ws_ros2
python3 calibration.py

# 단계별 안내:
# 1. 로봇을 평평한 바닥에 정지
# 2. 1000개 샘플 수집 (10초)
# 3. 자동으로 imu_calib.yaml 생성
```

---

## Odometry 정렬

### 1. LiDAR-Odometry 좌표계 정렬

**문제:** LiDAR와 Odometry의 방향이 다름
- LiDAR: 전방 = X축 정방향
- Odometry: 후방 = X축 정방향 (초기 설정)

**해결:**
```cpp
// transbot_driver.cpp
// Odometry 방향 반전
odom_msg.twist.twist.linear.x = -velocity_x;  // X 반전
odom_msg.twist.twist.angular.z = -omega_z;    // Z 반전 (회전)
```

### 2. 정렬 검증

```bash
# 1. 로봇을 전방으로 이동
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -1

# 2. Odometry 확인
ros2 topic echo /odom
# twist.linear.x가 양수(+)여야 함

# 3. LiDAR 스캔 확인
ros2 topic echo /scan
# 전방 장애물이 0도 방향에 나타나야 함
```

### 3. TF 프레임 체인

```
odom (EKF 발행)
 └─ base_footprint (로봇 지면 투영)
     └─ base_link (로봇 중심)
         ├─ laser_frame (LiDAR)
         └─ imu_link (IMU)
```

---

## 센서 파라미터 분석

### 1. 하드웨어 파라미터

**transbot_params.yaml:**
```yaml
# 물리적 치수
wheel_diameter: 0.065      # 65mm 휠
wheelbase: 0.172           # 172mm 휠 간격
encoder_resolution: 44     # 44 ticks/회전

# 모터 제어
max_speed: 0.5             # 최대 속도 (m/s)
max_angular: 2.0           # 최대 각속도 (rad/s)
```

### 2. IMU 정확도 파라미터

**ekf_config.yaml에서 사용:**
```yaml
# IMU 각속도 covariance (매우 정확)
imu0_angular_velocity_covariance: 0.000025

# IMU 방향 covariance (정확)
imu0_orientation_covariance: 0.0001

# IMU 선가속도 covariance (부정확 - 사용 안 함)
imu0_linear_acceleration_covariance: 0.1
```

### 3. Odometry 정확도 파라미터

**ekf_config.yaml에서 사용:**
```yaml
# Odometry 위치 covariance
odom0_pose_covariance: 0.0225  # 15cm 정도 오차

# Odometry 속도 covariance
odom0_twist_covariance: 0.01   # 10cm/s 정도 오차
```

---

## 테스트 및 검증

### 1. 90도 회전 테스트

```bash
# 테스트 스크립트 실행
cd /home/user/transbot_ws_ros2
python3 test_90degree_rotation.py

# 기대 결과:
# - IMU 각도: 90도 ± 2도
# - EKF 각도: 90도 ± 3도
# - Odometry 각도: 90도 ± 5도
```

### 2. 직선 주행 테스트

```bash
# 1m 직선 주행
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -1
# 5초 후 중지
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" -1

# Odometry 확인
ros2 topic echo /odom | grep position
# x: ~1.0m ± 0.05m
```

### 3. 센서 데이터 로깅

```bash
# 센서 데이터 기록
ros2 bag record -a -o sensor_test

# 재생 및 분석
ros2 bag play sensor_test.db3
rqt_plot
```

---

## 문제 해결

### IMU 드리프트

**증상:** 정지 상태에서 orientation이 계속 변함

**해결:**
1. 재캘리브레이션: `ros2 run imu_calib imu_calib_node`
2. gyro_bias 확인: `cat imu_calib.yaml`
3. 온도 안정화: 로봇 전원 켠 후 1분 대기

### Odometry 누적 오차

**증상:** 장시간 주행 시 위치가 크게 벗어남

**해결:**
1. SLAM/Localization 사용 (필수)
2. 휠 미끄러짐 최소화 (타일 바닥 선호)
3. 속도 낮추기 (0.3 m/s 이하)

### 센서 좌표계 불일치

**증상:** TF tree에서 static_transform_publisher 경고

**해결:**
```bash
# TF tree 확인
ros2 run tf2_tools view_frames

# 예상 구조:
# odom → base_footprint → base_link → laser_frame
#                                   → imu_link
```

---

## 관련 문서

- `EKF_센서퓨전_문제해결.md` - EKF 설정 가이드
- `SLAM_최적화_가이드.md` - SLAM 사용 시 센서 활용
- `transbot_nav/README.md` - 통합 시스템 문서

---

## 참고 명령어

```bash
# IMU 캘리브레이션
ros2 run imu_calib imu_calib_node

# IMU 데이터 확인
ros2 topic echo /imu/data

# Odometry 확인
ros2 topic echo /odom

# TF 트리 확인
ros2 run tf2_tools view_frames

# 센서 주파수 확인
ros2 topic hz /imu/data   # ~100Hz
ros2 topic hz /odom       # ~50Hz
```

---

**문서 통합 완료:** 2025-10-31  
**원본 파일들:**
- IMU_CALIB_GUIDE.md
- IMU_OPTIMIZATION_GUIDE.md
- LIDAR_ODOMETRY_ALIGNMENT_GUIDE.md
- SENSOR_ACCURACY_PARAMETERS_ANALYSIS.md
- HARDWARE_PARAMETER_ANALYSIS.md
- CALIBRATION_*.md (다수)
