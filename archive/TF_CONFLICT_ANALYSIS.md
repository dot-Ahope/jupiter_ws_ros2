# TF 발행 충돌 문제 분석

## 🔍 발견된 문제

`rqt_graph` 및 `ros2 topic info /tf --verbose` 분석 결과, **2개의 노드가 동시에 `/tf` 토픽에 Transform을 발행**하고 있습니다:

### 발행자 목록
```
1. ekf_filter_node       (robot_localization)
2. imu_filter_madgwick   (imu_filter_madgwick) ⚠️  
3. slam_toolbox
4. robot_state_publisher
5. base_node
```

## ⚠️  핵심 문제

### IMU Filter Madgwick vs EKF Filter Node 충돌

두 노드 모두 **IMU 데이터를 사용하여 로봇의 orientation (자세)를 계산**합니다:

#### imu_filter_madgwick
- **역할**: IMU raw 데이터 → 필터링된 orientation 계산
- **발행**: `/imu/data_filtered` (Madgwick 알고리즘으로 필터링된 IMU)
- **TF 발행**: 설정상 `publish_tf: False`이지만 실제로 발행 중!
- **문제점**: IMU만 사용, drift 발생 가능

#### ekf_filter_node
- **역할**: 여러 센서 융합 (Odom + IMU) → 최적 상태 추정
- **입력**: `/odom_raw` + `/imu/data_filtered`
- **발행**: `/odom` (융합된 오도메트리)
- **TF 발행**: `odom → base_footprint` Transform
- **장점**: 센서 융합으로 drift 최소화 ✅

## 🎯 권장 해결 방법

### Solution 1: IMU Filter Madgwick 노드 비활성화 (⭐⭐⭐ 권장)

**이유:**
1. **EKF가 더 강력**: 센서 융합 (Odom + IMU)
2. **IMU Filter는 중복**: EKF가 IMU 데이터를 직접 처리 가능
3. **TF 충돌 제거**: 단일 authority로 통일

**구현:**

#### 방법 A: 노드 자체를 launch하지 않음 (가장 깨끗)

```python
# transbot_full_system.launch.py 에서 제거
# imu_filter_node = Node(...)  # 주석 처리

# LaunchDescription 리스트에서도 제거
return LaunchDescription([
    lidar_node,
    # imu_filter_node,  # 제거
    ekf_node,
    ...
])
```

#### 방법 B: Raw IMU 직접 사용 (EKF 설정 변경)

`ekf_config.yaml` 수정:
```yaml
ekf_filter_node:
  ros__parameters:
    imu0: /imu/data_raw  # 필터링 전 IMU 직접 사용
    imu0_config: [false, false, false,
                 false, false, true,   # yaw만 사용
                 false, false, false,
                 false, false, true,   # vyaw만 사용
                 false, false, false]
```

### Solution 2: EKF의 TF 발행 비활성화 (❌ 비권장)

이 방법은 **센서 융합의 이점을 포기**하므로 권장하지 않습니다.

```yaml
ekf_filter_node:
  ros__parameters:
    publish_tf: false  # EKF TF 발행 비활성화
```

문제: IMU 단독 사용 시 **drift 발생** 가능성 높음.

## 📊 현재 구조 vs 권장 구조

### 현재 (충돌 발생)
```
/imu/data_raw ─→ imu_filter_madgwick ─→ /imu/data_filtered ─→ ekf_filter_node ─→ /odom
                           ↓ (TF 발행)                               ↓ (TF 발행)
                         /tf ← 충돌! →                              /tf
```

### 권장 (단일 Authority)
```
/imu/data_raw ────────────────────────────┐
                                          ↓
/odom_raw ─────────────────────────→ ekf_filter_node ─→ /odom
                                          ↓ (TF 발행)
                                         /tf (단독!)
```

또는:

```
/imu/data_raw ─→ imu_filter_madgwick ─→ /imu/data_filtered ─→ ekf_filter_node ─→ /odom
                   (TF 발행 안 함)                                  ↓ (TF 발행)
                                                                    /tf (단독!)
```

## 🔧 구체적 수정 사항

### 파일 1: `transbot_full_system.launch.py`

**Option A: IMU Filter 완전 제거 (추천)**

```python
# Line 175-190 주석 처리 또는 삭제
# imu_filter_node = Node(
#     package='imu_filter_madgwick',
#     ...
# )

# LaunchDescription에서도 제거
return LaunchDescription([
    lidar_node,
    # imu_filter_node,  # 제거!
    ekf_node,
    slam_toolbox_node,
    ...
])
```

**Option B: IMU Filter 유지하되 TF 발행 확실히 비활성화**

IMU Filter Madgwick의 `publish_tf: False` 설정이 무시되는 경우가 있습니다.
이 경우 소스 코드 레벨에서 확인이 필요합니다.

### 파일 2: `ekf_config.yaml` (Option A 선택 시)

Raw IMU 직접 사용:

```yaml
ekf_filter_node:
  ros__parameters:
    # IMU 입력 변경
    imu0: /imu/data_raw  # /imu/data_filtered → /imu/data_raw
    
    # IMU 설정은 동일
    imu0_config: [false, false, false,
                 false, false, true,
                 false, false, false,
                 false, false, true,
                 false, false, false]
    
    # 노이즈 파라미터 조정 (raw 데이터는 더 noisy)
    imu0_pose_rejection_threshold: 15.0  # 10.0 → 15.0
    imu0_twist_rejection_threshold: 8.0   # 5.0 → 8.0
```

### 파일 3: `bringup.launch.py`

동일한 수정 필요 (transbot_bringup 패키지도 동일한 구조):

```python
# Line 144-167 주석 처리
# imu_filter_node = Node(...)

return LaunchDescription([
    camera_node,
    base_node,
    # imu_filter_node,  # 제거!
    ekf_node,
    robot_state_publisher,
    ...
])
```

## 🧪 검증 방법

### 1. TF 발행자 확인
```bash
ros2 topic info /tf --verbose
```

**기대 결과:** `imu_filter_madgwick` 노드가 발행자 목록에서 사라짐

### 2. TF 트리 확인
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**기대 결과:** `odom → base_footprint` TF가 **ekf_filter_node만** 발행

### 3. EKF 정상 작동 확인
```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
```

**기대 결과:** 
- 오도메트리 정상 발행
- 주기: ~20Hz (ekf frequency: 20.0)

### 4. IMU 데이터 확인
```bash
# Option A (raw 직접 사용)
ros2 topic echo /imu/data_raw --once

# Option B (필터링된 것 사용)
ros2 topic echo /imu/data_filtered --once
```

### 5. Angular Scale 캘리브레이션 재실행
```bash
python3 odom_based_angular_calibration.py --phase 1
```

**기대 결과:** TF 충돌 제거로 일관된 측정 결과

## 💡 예상 효과

### Before (TF 충돌)
- ❌ 두 노드가 동시에 orientation 계산
- ❌ TF tree에 불일치 발생 가능
- ❌ angular_scale 캘리브레이션 불안정
- ❌ SLAM 정확도 저하

### After (단일 Authority)
- ✅ EKF가 단독으로 센서 융합
- ✅ 일관된 TF tree
- ✅ angular_scale 측정 안정화
- ✅ SLAM 정확도 향상

## 🚀 실행 계획

1. **백업**
   ```bash
   cd ~/transbot_ws_ros2/src
   cp -r sllidar_ros2 sllidar_ros2.backup
   cp -r transbot_bringup transbot_bringup.backup
   ```

2. **수정** (Option A 추천)
   - `transbot_full_system.launch.py`: imu_filter_node 제거
   - `bringup.launch.py`: imu_filter_node 제거
   - `ekf_config.yaml`: imu0 → /imu/data_raw (Option A)

3. **빌드**
   ```bash
   cd ~/transbot_ws_ros2
   colcon build --packages-select sllidar_ros2 transbot_bringup
   source install/setup.bash
   ```

4. **테스트**
   ```bash
   ros2 launch sllidar_ros2 transbot_full_system.launch.py
   
   # 다른 터미널에서
   ros2 topic info /tf --verbose  # imu_filter_madgwick 제거 확인
   ros2 run tf2_tools view_frames  # TF 트리 확인
   ```

5. **캘리브레이션 재실행**
   ```bash
   python3 odom_based_angular_calibration.py --phase 1
   python3 odom_based_angular_calibration.py --phase 2 --scale <Phase1결과>
   ```

## 📚 참고

### Robot Localization vs IMU Filter Madgwick

| 기능 | IMU Filter Madgwick | robot_localization (EKF) |
|------|---------------------|--------------------------|
| 센서 융합 | ❌ IMU 단독 | ✅ Odom + IMU + GPS |
| Drift 보정 | ❌ 없음 | ✅ 휠 오도메트리로 보정 |
| 알고리즘 | Madgwick (Gradient Descent) | Extended Kalman Filter |
| 사용 사례 | IMU orientation 필터링 | 전체 로봇 상태 추정 |
| TF 발행 | orientation만 | odom → base_link 전체 |

### 왜 EKF가 더 나은가?

1. **센서 융합**: 휠 오도메트리 + IMU → drift 상호 보정
2. **불확실성 추정**: 각 센서의 신뢰도 고려
3. **로버스트**: 하나의 센서 실패 시에도 작동
4. **SLAM 친화적**: `/odom` 토픽이 SLAM의 표준 입력

## 결론

**IMU Filter Madgwick 노드를 비활성화**하고 **EKF Filter Node가 단독으로 센서 융합 및 TF 발행**을 담당하도록 하는 것을 강력히 권장합니다. 이렇게 하면:

- TF 충돌 제거
- 센서 융합의 이점 활용
- angular_scale 캘리브레이션 안정화
- SLAM 정확도 향상

이 모든 것이 시스템 전체의 로컬라이제이션 성능을 크게 향상시킬 것입니다. ✅
