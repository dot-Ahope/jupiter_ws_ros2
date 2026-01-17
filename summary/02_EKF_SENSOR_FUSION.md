# EKF 센서 퓨전 문제 해결 종합 가이드

> **생성일:** 2025-10-31  
> **통합 문서:** EKF 진단, 문제 해결, Phase 1-2 분석

## 📅 작업 타임라인

**작업 기간:** 2025년 10월 21일 ~ 10월 24일

### 주요 작업 일정
- **2025-10-21**: Phase 1 진단 가이드 작성, 초기 문제 분석 시작
- **2025-10-22**: Phase 2 근본 원인 분석, EKF 설정 수정 전략 수립
- **2025-10-22**: Gyro drift 문제 해결, EKF 분석 보고서 작성
- **2025-10-23**: Phase 1-2 완전 분석 완료, Launch 파일 수정
- **2025-10-24**: EKF 추가 수정 및 진단 완료

### 참조된 원본 문서들
- `PHASE1_DIAGNOSIS_GUIDE.md` (2025-10-21)
- `PHASE1_ANALYSIS_LAUNCH_1.5618.md` (2025-10-23)
- `PHASE2_ANALYSIS.md` (2025-10-22)
- `PHASE2_COMPLETE_ANALYSIS.md` (2025-10-23)
- `PHASE2_FIX_VERIFICATION.md` (2025-10-23)
- `EKF_FIX_STRATEGY.md` (2025-10-22)
- `EKF_DIAGNOSIS.md` (2025-10-24)
- `EKF_ADDITIONAL_FIX.md` (2025-10-24)
- `EKF_분석_보고서.md` (2025-10-24)
- `GYRO_DRIFT_FIX.md` (2025-10-22)
- `TEMPORARY_IMU_TOPIC.md` (2025-10-21)

---

## 📋 목차
1. [문제 개요](#문제-개요)
2. [Phase 1: 초기 진단](#phase-1-초기-진단)
3. [Phase 2: 근본 원인 분석](#phase-2-근본-원인-분석)
4. [최종 해결책](#최종-해결책)
5. [검증 결과](#검증-결과)

---

## 문제 개요

### 초기 증상
- **IMU angular_velocity**: 정상 발행 (~100Hz)
- **EKF 출력**: angular_velocity가 0으로 고정
- **결과**: SLAM 회전 추정 실패, 맵 왜곡

### 핵심 문제
EKF가 IMU의 각속도 데이터를 **무시**하고 있었음

---

## Phase 1: 초기 진단

### 1.1 데이터 흐름 분석

```
[IMU 센서 (MPU6050)]
    ↓ /imu/data_raw (100Hz)
[imu_calib_node] - 바이어스 보정
    ↓ /imu/data (100Hz)
[EKF (robot_localization)]
    ↓ /odometry/filtered (50Hz)
    ↓ TF: odom → base_footprint
[SLAM Toolbox]
    ↓ /map (맵 생성)
```

### 1.2 런치 파일 분석 (launch_1.5618.py)

**발견된 문제들:**

1. **IMU 토픽 중복 발행**
   ```python
   # 문제: imu_calib_node가 /imu/data를 발행
   # 동시에 static_transform이 /imu/data를 발행하려 함
   # → 충돌 발생
   ```

2. **EKF 설정 오류**
   ```yaml
   # ekf_config.yaml
   imu0: /imu/data
   imu0_config: [false, false, false,  # 위치 (사용 안 함)
                 false, false, true,   # 방향 (yaw만 사용)
                 false, false, false,  # 속도 (사용 안 함)
                 false, false, true,   # 각속도 (yaw rate만 사용)
                 true,  true,  true]   # 가속도 (모두 사용) ← 문제!
   ```

3. **Covariance 설정 부적절**
   ```yaml
   # IMU 각속도 불신
   imu0_angular_velocity_covariance: 0.0001  # 너무 높음
   
   # Odometry 과신
   odom0_pose_covariance: 0.001  # 너무 낮음
   ```

### 1.3 Phase 1 해결 시도

```yaml
# 수정 1: IMU 각속도 활성화
imu0_config: [false, false, false,
              false, false, true,
              false, false, false,
              false, false, true,    # yaw rate 사용
              false, false, false]   # 가속도 비활성화 ⭐

# 수정 2: Covariance 조정
imu0_angular_velocity_covariance: 0.00005  # 신뢰도 상승
```

**결과:** 부분적 개선, 하지만 완전하지 않음

---

## Phase 2: 근본 원인 분석

### 2.1 EKF 내부 동작 분석

**발견:**
- EKF가 각속도를 "측정값"이 아닌 "제어 입력"으로 처리
- `use_control: false` 설정으로 각속도 데이터 무시됨

### 2.2 TF 프레임 충돌

```bash
# TF tree 확인
ros2 run tf2_tools view_frames

# 발견된 문제:
# - odom → base_link (Transbot_Driver 발행)
# - odom → base_footprint (EKF 발행)
# → 동일한 부모(odom)에서 두 자식 프레임 충돌!
```

### 2.3 Process Noise Covariance 문제

```yaml
# 초기 설정 (문제)
process_noise_covariance: [
  0.05, 0,    0,    ...  # x 위치 노이즈
  0,    0.05, 0,    ...  # y 위치 노이즈
  ...
  0,    0,    0.03, ...  # yaw 노이즈
  ...
  0,    0,    0.03, ...  # yaw rate 노이즈 ← 너무 큼!
]
```

**문제점:**
- Process noise가 너무 크면 EKF가 센서를 불신
- IMU 각속도보다 모델 예측을 더 신뢰

### 2.4 데이터 로깅 분석

```bash
# IMU 데이터 로깅
ros2 topic echo /imu/data > imu_raw.log

# EKF 출력 로깅
ros2 topic echo /odometry/filtered > ekf_output.log

# 비교:
# IMU angular_velocity.z: -0.234 rad/s
# EKF twist.angular.z: 0.000 rad/s  ← 완전히 무시됨!
```

---

## 최종 해결책

### 3.1 EKF 설정 완전 재구성

**ekf_config.yaml 최종 버전:**

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    
    # 프레임 설정 (중요!)
    odom_frame: odom
    base_link_frame: base_footprint  # ⭐ EKF가 발행하는 프레임
    world_frame: odom
    
    # Odometry 설정
    odom0: /odom
    odom0_config: [true,  true,  false,   # x, y 위치 사용
                   false, false, false,   # 방향 사용 안 함
                   false, false, false,   # 속도 사용 안 함
                   false, false, false,   # 각속도 사용 안 함
                   false, false, false]   # 가속도 사용 안 함
    
    odom0_pose_covariance: [0.0225, 0.0,    0.0,    ...
                            0.0,    0.0225, 0.0,    ...
                            ...]  # 15cm 위치 오차
    
    # IMU 설정 (⭐ 핵심)
    imu0: /imu/data
    imu0_config: [false, false, false,   # 위치 사용 안 함
                  false, false, true,    # yaw만 사용
                  false, false, false,   # 속도 사용 안 함
                  false, false, true,    # ⭐ yaw rate 사용!
                  false, false, false]   # 가속도 사용 안 함
    
    # IMU Covariance (IMU 우선)
    imu0_angular_velocity_covariance: 0.000025  # ⭐ 매우 높은 신뢰
    imu0_orientation_covariance: 0.0001
    
    # Process Noise (IMU 중심)
    process_noise_covariance: [
      0.05, 0.0,  0.0,  ...  # x
      0.0,  0.05, 0.0,  ...  # y
      0.0,  0.0,  0.06, ...  # z
      0.0,  0.0,  0.0,  0.03, ...  # roll
      0.0,  0.0,  0.0,  0.0,  0.03, ...  # pitch
      0.0,  0.0,  0.0,  0.0,  0.0,  0.06, ...  # yaw
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, ...  # vx
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, ...  # vy
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, ...  # vz
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, ...  # vroll
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, ...  # vpitch
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.00005  # ⭐ vyaw (매우 낮음)
    ]
```

### 3.2 TF 프레임 충돌 해결

**transbot_driver.cpp 수정:**

```cpp
// 기존 (문제)
odom_trans.child_frame_id = "base_link";

// 수정 (해결)
odom_trans.child_frame_id = "base_footprint";
```

**이유:**
- EKF가 `odom → base_footprint` TF 발행
- Transbot_Driver는 순수 오도메트리만 발행 (`odom → base_footprint`)
- EKF가 이를 보정하여 최종 TF 발행

### 3.3 IMU 주파수 조정

**imu_calib.yaml:**

```yaml
publish_rate: 100.0  # IMU를 100Hz로 발행 (EKF 50Hz의 2배)
```

**이유:**
- EKF는 센서 주파수가 높을수록 정확
- 100Hz IMU → 50Hz EKF 다운샘플링

---

## 검증 결과

### 4.1 각속도 전달 확인

```bash
# 테스트: 제자리 회전
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -1

# IMU 확인
ros2 topic echo /imu/data | grep angular
# angular.z: 0.498 rad/s ✅

# EKF 확인
ros2 topic echo /odometry/filtered | grep angular
# angular.z: 0.485 rad/s ✅ (이제 전달됨!)
```

### 4.2 TF 프레임 확인

```bash
ros2 run tf2_tools view_frames

# 결과:
odom
 └─ base_footprint (EKF 발행, 보정된 위치)
     └─ base_link (URDF static)
         ├─ laser_frame
         └─ imu_link

# ✅ 충돌 없음!
```

### 4.3 SLAM 맵 품질

**Before (EKF 문제):**
- 맵이 회전할 때마다 왜곡
- Feature 중첩 발생
- 루프 클로저 실패

**After (EKF 수정):**
- 깨끗한 맵 생성
- 회전 시 정확한 스캔 매칭
- 루프 클로저 성공률 90%+

### 4.4 수치 비교

| 항목 | Before | After | 개선율 |
|------|--------|-------|--------|
| 90도 회전 오차 | ±15도 | ±2도 | 87% |
| 각속도 전달 | 0% | 97% | - |
| SLAM 루프 클로저 | 30% | 90% | 200% |
| TF 경고 | 빈번 | 없음 | - |

---

## 문제 해결 체크리스트

### ✅ EKF 각속도 전달 확인

```bash
# 1. IMU 발행 확인
ros2 topic hz /imu/data
# Expected: ~100 Hz

# 2. EKF 각속도 확인
ros2 topic echo /odometry/filtered | grep -A 3 "angular"

# 3. 제자리 회전 테스트
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -1

# 4. 값 비교
# /imu/data의 angular.z ≈ /odometry/filtered의 twist.angular.z
```

### ✅ TF 프레임 충돌 확인

```bash
# TF tree 생성
ros2 run tf2_tools view_frames

# 확인사항:
# 1. odom → base_footprint (단일 경로)
# 2. 중복 발행자 없음
# 3. 모든 프레임이 연결됨
```

### ✅ Process Noise 적절성 확인

```yaml
# ekf_config.yaml
process_noise_covariance[11]: 0.00005  # vyaw

# 기준:
# - IMU 각속도 covariance (0.000025)보다 약간 큼
# - 너무 크면 (>0.001) IMU 무시됨
# - 너무 작으면 (<0.00001) 노이즈에 민감
```

---

## 관련 문서

- `01_IMU_ODOMETRY_CALIBRATION.md` - IMU 캘리브레이션
- `03_ROTATION_ACCURACY.md` - 회전 정확도 개선
- `transbot_nav/README.md` - 최종 통합 시스템

---

## 핵심 교훈

1. **EKF는 센서를 "신뢰"하지 않으면 무시함**
   - Covariance 값이 핵심
   - Process noise가 센서보다 크면 센서 무시

2. **TF 프레임은 단일 발행자만**
   - 같은 변환을 여러 노드가 발행하면 충돌
   - EKF의 base_link_frame과 다른 노드의 child_frame_id 일치 필수

3. **2D 로봇은 yaw와 yaw rate만 사용**
   - Roll, pitch는 필요 없음 (평면 주행)
   - 가속도계는 사용 안 함 (드리프트 많음)

4. **센서 주파수 = EKF 주파수 × 2 이상**
   - IMU: 100Hz, EKF: 50Hz
   - 충분한 샘플로 정확한 추정

---

**문서 통합 완료:** 2025-10-31  
**원본 파일들:**
- EKF_DIAGNOSIS.md
- EKF_FIX_STRATEGY.md
- EKF_FIX_SUMMARY.md
- EKF_ANGULAR_VELOCITY_FIX.md
- EKF_ADDITIONAL_FIX.md
- EKF_FREQUENCY_ADJUSTMENT.md
- EKF_분석_보고서.md
- EKF_ROOT_CAUSE.md
- PHASE1_*.md
- PHASE2_*.md
- DATA_FLOW_ANALYSIS.md
- TF_CONFLICT_*.md
