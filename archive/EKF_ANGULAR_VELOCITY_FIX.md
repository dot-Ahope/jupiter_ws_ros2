# EKF 각속도 융합 설정 수정

## 수정 날짜
2025-10-16

## 문제점
기존 EKF 설정에서 **휠 오도메트리의 각속도(yaw_dot)**를 사용하지 않고 **IMU 각속도만 사용**하여 회전 추적 시 문제 발생:

1. **IMU 드리프트 누적**: 장시간 회전 시 자이로 드리프트로 인한 오차 증가
2. **후면 라이다 제약**: 180° FOV로 회전 시 특징점 50% 손실
3. **결과**: 전진/후진은 정상, 회전 시 위치 추정 실패

## 기존 설정 (문제)

```yaml
odom0_config: [true, true, false,
              false, false, true,
              true, true, false,
              false, false, false,  # ← yaw_dot = false (휠 각속도 미사용)
              false, false, false]

imu0_config: [false, false, false,
             false, false, false,
             false, false, false,
             false, false, true,   # ← yaw_dot = true (IMU만 사용)
             false, false, false]
```

**문제**: 휠 오도메트리의 정확한 각속도 정보 버림 → IMU 드리프트만 누적

## 수정 내용

```yaml
odom0_config: [true, true, false,
              false, false, true,
              true, true, false,
              false, false, true,   # ← yaw_dot = true (휠 각속도 활성화!)
              false, false, false]

imu0_config: [false, false, false,
             false, false, false,
             false, false, false,
             false, false, true,   # ← yaw_dot = true (IMU도 계속 사용)
             false, false, false]
```

**개선**: 휠 오도메트리와 IMU 각속도를 **상호 보완적으로 융합**

## 작동 원리

### 센서 융합 전략

1. **휠 오도메트리 각속도** (`/odom_raw`)
   - **장점**: 
     - 기계적 계산으로 드리프트 없음
     - 장기 안정성 우수
   - **단점**: 
     - 휠 슬립에 민감
     - 업데이트 주기 낮음 (10Hz)

2. **IMU 각속도** (`/imu/data_filtered`)
   - **장점**:
     - 높은 샘플링 레이트 (100Hz)
     - 순간 변화 감지 우수
   - **단점**:
     - 자이로 드리프트 누적
     - 장기 정확도 하락

3. **EKF 융합 효과**
   - **단기**: IMU의 높은 정확도 활용
   - **장기**: 휠 오도메트리로 드리프트 보정
   - **상호 보완**: 각 센서의 약점을 다른 센서가 보완

## 데이터 흐름

```
Hardware Encoders → transbot_driver.py → /transbot/get_vel (Twist)
                                              ↓
                                      base_node (C++) → /odom_raw (Odometry)
                                                            ↓
                                                        [angular.z]
                                                            ↓
Hardware IMU → transbot_driver.py → /transbot/imu → imu_calib → imu_filter
                                                                      ↓
                                                              /imu/data_filtered
                                                                      ↓
                                                              [angular_velocity.z]
                                                                      ↓
                                                                  ┌───┴───┐
                                                                  │  EKF  │ ← 두 소스 융합!
                                                                  └───┬───┘
                                                                      ↓
                                                          /odometry/filtered (최적 추정)
                                                                      ↓
                                                                SLAM Toolbox
```

## ROS 표준 준수 확인

### REP-103 좌표계 규약
- **우수 좌표계** (right-handed)
- +X: 전방, +Y: 좌측, +Z: 상방
- **회전 방향**: 반시계방향(CCW) = 양수, 시계방향(CW) = 음수

### 현재 구현 검증 ✓

1. **cmd_vel → 모터 제어**
   ```python
   left_speed = base_speed - turn_speed   # angular_z > 0 → 좌회전
   right_speed = base_speed + turn_speed
   ```
   ✓ `angular.z > 0` → 반시계방향(좌회전)

2. **오도메트리 계산**
   ```cpp
   double delta_heading = angular_velocity_z_ * vel_dt_;
   heading_ += delta_heading;
   ```
   ✓ `angular_velocity_z > 0` → heading 증가 (반시계방향)

3. **IMU 각속도**
   ```python
   msg.angular_velocity.z = float(gz)
   ```
   ✓ 반시계방향 회전 → 양수

**결론**: 모든 각속도 데이터가 ROS REP-103 표준을 준수합니다.

## 예상 개선 효과

### Before (휠 각속도 미사용)
- ❌ 회전 중 IMU 드리프트 누적
- ❌ 후면 라이다 + 드리프트 → 위치 추정 실패
- ✓ 전진/후진만 정상

### After (휠 각속도 융합)
- ✅ IMU 드리프트를 휠 데이터로 보정
- ✅ 단기 정확도(IMU) + 장기 안정성(휠)
- ✅ 회전 추적 정확도 향상
- ✅ SLAM 맵 생성 안정화

## 테스트 방법

### 1. 시스템 실행
```bash
cd /home/user/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

### 2. 각속도 데이터 모니터링
```bash
# 터미널 1: 휠 오도메트리 각속도
ros2 topic echo /odom_raw --field twist.twist.angular.z

# 터미널 2: IMU 각속도
ros2 topic echo /imu/data_filtered --field angular_velocity.z

# 터미널 3: 융합 결과
ros2 topic echo /odometry/filtered --field twist.twist.angular.z
```

### 3. 회전 테스트
```bash
# 제자리 좌회전 (반시계방향)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"

# 제자리 우회전 (시계방향)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.5}}"
```

**확인 사항**:
1. `/odometry/filtered`의 `angular.z`가 두 센서 값의 중간 정도
2. RViz에서 회전 중 `odom` → `base_footprint` TF가 안정적
3. SLAM 맵이 회전 중에도 정상 생성

### 4. SLAM 성능 테스트
```bash
# RViz에서 확인
rviz2 -d /home/user/transbot_ws_ros2/src/sllidar_ros2/rviz/transbot_slam.rviz
```

**확인 사항**:
1. 전진/후진 + 회전 복합 동작 시 맵 품질
2. 제자리 360° 회전 후 원래 위치 복귀 정확도
3. Loop closure 성능

## 추가 튜닝 (필요시)

### 센서 신뢰도 조정

휠 각속도를 더 신뢰하려면:
```yaml
odom0_twist_rejection_threshold: 2.0   # 낮출수록 더 신뢰
process_noise_covariance[11]: 0.005    # yaw_dot 노이즈 감소
```

IMU 각속도를 더 신뢰하려면:
```yaml
imu0_twist_rejection_threshold: 0.3    # 낮출수록 더 신뢰
```

### 프로세스 노이즈 조정
회전 추적이 너무 느리면:
```yaml
process_noise_covariance[5]: 0.01     # yaw 노이즈 감소 (0.015→0.01)
process_noise_covariance[11]: 0.005   # yaw_dot 노이즈 감소 (0.01→0.005)
```

## 관련 파일
- `/home/user/transbot_ws_ros2/src/sllidar_ros2/config/ekf_config.yaml`
- `/home/user/transbot_ws_ros2/src/transbot_bringup/transbot_bringup/transbot_driver.py`
- `/home/user/transbot_ws_ros2/src/transbot_base/src/base.cpp`

## 참고 문서
- [ROS REP-103: Standard Units and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [robot_localization Documentation](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- `ROTATION_TRACKING_ANALYSIS.md`: 회전 추적 문제 상세 분석
- `REAR_LIDAR_SETUP.md`: 후면 라이다 설정 설명
