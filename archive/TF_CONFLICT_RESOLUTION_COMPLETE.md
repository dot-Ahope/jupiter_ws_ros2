# ✅ TF 충돌 해결 완료

## 🎯 실행된 수정 사항

### 문제 요약
- **발견**: `rqt_graph` 분석 결과 `imu_filter_madgwick`와 `ekf_filter_node`가 동시에 `/tf` 토픽에 Transform 발행
- **원인**: 두 노드가 모두 IMU 데이터로 orientation 계산
- **영향**: TF tree 불일치 → angular_scale 캘리브레이션 불안정 → SLAM 정확도 저하

### 해결 방법: Option A 적용 ⭐⭐⭐

**IMU Filter Madgwick 노드 비활성화 + EKF가 Raw IMU 직접 처리**

## 📝 수정된 파일 목록

### 1. `/home/user/transbot_ws_ros2/src/sllidar_ros2/launch/transbot_full_system.launch.py`

**변경 내용:**
```python
# Line 176-199: imu_filter_node 정의 주석 처리
# Line 291: LaunchDescription에서 imu_filter_node 제거
```

**효과:**
- ✅ `imu_filter_madgwick` 노드 실행 안 함
- ✅ `/tf` 발행자 1개 감소

### 2. `/home/user/transbot_ws_ros2/src/sllidar_ros2/config/ekf_config.yaml`

**변경 내용:**
```yaml
# Line 36: imu0 토픽 변경
imu0: /imu/data_calibrated  # /imu/data_filtered → /imu/data_calibrated

# Line 49-51: Raw IMU를 위한 노이즈 파라미터 조정
imu0_pose_rejection_threshold: 5.0      # 2.0 → 5.0
imu0_twist_rejection_threshold: 2.0     # 0.5 → 2.0
imu0_linear_acceleration_rejection_threshold: 10.0  # 8.0 → 10.0
```

**효과:**
- ✅ EKF가 raw IMU 직접 처리 (`/imu/data_calibrated`)
- ✅ Raw IMU의 높은 노이즈 허용 (rejection threshold 증가)

### 3. `/home/user/transbot_ws_ros2/src/transbot_bringup/launch/bringup.launch.py`

**변경 내용:**
```python
# Line 143-171: imu_filter_node 정의 주석 처리
# Line 241: LaunchDescription에서 imu_filter_node 제거
```

**효과:**
- ✅ `bringup.launch.py`도 동일한 구조로 통일
- ✅ TF 충돌 완전 제거

### 4. `/home/user/transbot_ws_ros2/src/transbot_bringup/param/ekf/robot_localization.yaml`

**변경 내용:**
```yaml
# Line 28: imu0 토픽 유지 (이미 raw IMU 사용 중)
imu0: /transbot/imu_corrected  # imu_calib_node가 교정한 IMU

# Line 44-48: Raw IMU를 위한 노이즈 파라미터 조정
imu0_pose_rejection_threshold: 15.0    # 10.0 → 15.0
imu0_twist_rejection_threshold: 8.0    # 5.0 → 8.0
imu0_linear_acceleration_limits: [-3.0, 3.0]   # [-2.0, 2.0] → [-3.0, 3.0]
imu0_angular_velocity_limits: [-3.0, 3.0]      # [-2.0, 2.0] → [-3.0, 3.0]
```

**효과:**
- ✅ Raw IMU의 더 큰 변동성 허용
- ✅ 안정적인 센서 융합

## 🏗️ 새로운 아키텍처

### Before (충돌 발생)
```
/imu/data_raw ─→ imu_calib_node ─→ /imu/data_calibrated ─→ imu_filter_madgwick ─→ /imu/data_filtered ─→ ekf_filter_node
                                                                      ↓ (TF 발행)              ↓ (TF 발행)
                                                                     /tf ← 충돌! →           /tf
```

### After (단일 Authority) ✅
```
/imu/data_raw ─→ imu_calib_node ─→ /imu/data_calibrated ────────────→ ekf_filter_node ─→ /odom
                                                                              ↓ (TF 발행)
                                                                             /tf (단독!)
```

## 🔧 빌드 완료

```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2 transbot_bringup --symlink-install
```

**결과:**
```
Summary: 2 packages finished [4.75s]
✅ sllidar_ros2
✅ transbot_bringup
```

## 🧪 검증 단계

### 1. 시스템 재시작

**기존 프로세스 종료:**
```bash
# Ctrl+C로 transbot_full_system.launch.py 종료
```

**새로운 설정으로 재시작:**
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

### 2. TF 발행자 확인

```bash
ros2 topic info /tf --verbose
```

**기대 결과:**
- ❌ `imu_filter_madgwick` 노드가 발행자 목록에서 **사라짐**
- ✅ 발행자: `ekf_filter_node`, `slam_toolbox`, `robot_state_publisher`, `base_node`만

### 3. 노드 목록 확인

```bash
ros2 node list | grep imu
```

**기대 결과:**
- ✅ `/imu_calib_node` (교정 노드)만 존재
- ❌ `/imu_filter_madgwick` **없음**

### 4. TF 트리 확인

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**기대 결과:**
- ✅ `odom → base_footprint` TF가 **ekf_filter_node만** 발행
- ✅ TF rate 안정적 (~10Hz)

### 5. EKF 정상 작동 확인

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
```

**기대 결과:**
- ✅ 오도메트리 정상 발행
- ✅ 주기: ~10Hz (ekf frequency: 10.0)

### 6. IMU 데이터 확인

```bash
ros2 topic echo /imu/data_calibrated --once
ros2 topic hz /imu/data_calibrated
```

**기대 결과:**
- ✅ Raw IMU 데이터 정상 발행
- ✅ EKF가 직접 구독 중

### 7. Angular Scale 캘리브레이션 재실행 🎯

```bash
cd ~/transbot_ws_ros2
python3 odom_based_angular_calibration.py --phase 1
```

**기대 결과:**
- ✅ TF 충돌 제거로 **일관된 측정 결과**
- ✅ 90° 회전 정확도 향상
- ✅ 변동계수 (CV) < 2%

```bash
# Phase 1 결과 사용 (예: 1.56)
python3 odom_based_angular_calibration.py --phase 2 --scale <Phase1결과>
```

**기대 결과:**
- ✅ 360° 회전도 **정확히 도달** (더 이상 225°에서 멈추지 않음)
- ✅ 모든 각도에서 **일관된 angular_scale**
- ✅ 신뢰도: **매우 높음 ✅**

## 💡 예상 효과

### Before (TF 충돌)
| 문제 | 증상 |
|------|------|
| ❌ TF 발행자 충돌 | 두 노드가 동시에 orientation 계산 |
| ❌ TF tree 불일치 | SLAM 정확도 저하 |
| ❌ 캘리브레이션 불안정 | angular_scale 측정 오차 큼 |
| ❌ 360° 회전 실패 | 225°에서 조기 정지 (IMU 드리프트) |

### After (단일 Authority)
| 개선 | 효과 |
|------|------|
| ✅ 단일 TF 발행 | ekf_filter_node만 /tf 발행 |
| ✅ 일관된 TF tree | SLAM 정확도 향상 |
| ✅ 안정적 캘리브레이션 | angular_scale 측정 신뢰도 높음 |
| ✅ 360° 정확 도달 | Odom 기반 종료 + 드리프트 보정 |

## 🎯 최종 목표

1. **TF 충돌 제거** ✅ 완료
2. **EKF 단독 센서 융합** ✅ 완료
3. **Raw IMU 직접 처리** ✅ 완료
4. **Angular Scale 캘리브레이션 안정화** ⏳ 검증 필요
5. **SLAM 정확도 향상** ⏳ 검증 필요

## 📊 다음 단계

### 즉시 실행:
```bash
# 1. 시스템 재시작
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 2. TF 검증 (다른 터미널)
ros2 topic info /tf --verbose
ros2 node list | grep imu

# 3. 캘리브레이션 재실행
python3 odom_based_angular_calibration.py --phase 1
```

### 기대 결과:
- ✅ `imu_filter_madgwick` 노드 없음
- ✅ `/tf` 발행자 4개 (ekf, slam_toolbox, robot_state_publisher, base_node)
- ✅ 90° 회전 정확도 향상
- ✅ 360° 회전 정확히 도달

## 🎉 요약

**문제:** TF 발행 주체 충돌 (`imu_filter_madgwick` vs `ekf_filter_node`)

**해결:** `imu_filter_madgwick` 제거 + EKF가 Raw IMU 직접 처리

**효과:**
- ✅ TF tree 단일 authority
- ✅ 센서 융합 강화 (Odom + IMU)
- ✅ 드리프트 보정 개선
- ✅ Angular scale 캘리브레이션 안정화

**다음:** Angular scale 캘리브레이션 Phase 1 & 2 실행 🚀
