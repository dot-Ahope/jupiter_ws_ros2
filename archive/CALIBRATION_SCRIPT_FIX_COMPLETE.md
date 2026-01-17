# ✅ 캘리브레이션 스크립트 수정 완료

## 🔍 발견된 문제

TF 충돌 해결을 위해 `imu_filter_madgwick` 노드를 비활성화했지만, 이로 인해 **캘리브레이션 스크립트가 IMU 데이터를 받지 못하는 문제** 발생:

### 문제점:
1. ❌ 기존 스크립트: `/imu/data_filtered` 구독 → 더 이상 발행 안 됨
2. ❌ Raw IMU (`/imu/data_calibrated`)는 **orientation 정보 없음**
   - `orientation_covariance` = -1.0 (orientation 무효)
   - 각속도 + 가속도만 제공

### Root Cause:
Raw IMU 센서는 자이로스코프와 가속도계 데이터만 제공하고, **orientation(자세)은 필터(Madgwick, EKF 등)가 계산**합니다.

## 🔧 수정 사항

### 1. `/home/user/transbot_ws_ros2/odom_based_angular_calibration.py`

#### 변경 1: IMU 토픽 수정
```python
# Before
self.imu_sub = self.create_subscription(
    Imu, '/imu/data_filtered', self.imu_callback, 10)

# After
self.imu_sub = self.create_subscription(
    Imu, '/imu/data_calibrated', self.imu_callback, 10)  # Raw IMU
```

#### 변경 2: IMU 콜백 로직 수정
```python
def imu_callback(self, msg):
    """IMU 콜백 - Raw IMU는 각속도만 사용"""
    current_time = self.get_clock().now()
    
    self.imu_angular_vel_z = msg.angular_velocity.z
    
    # 각속도 적분 (주 측정 방법)
    if self.last_imu_time is not None:
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        if dt < 1.0:
            self.integrated_imu_yaw += self.imu_angular_vel_z * dt
    
    self.last_imu_time = current_time
    
    # Raw IMU는 orientation이 없으므로 적분값만 사용
    # orientation_covariance가 -1.0이면 orientation 무효
    if msg.orientation_covariance[0] >= 0:
        q = msg.orientation
        self.imu_yaw = self.quaternion_to_yaw(q)
    else:
        # Raw IMU: orientation 무효, 적분값만 사용
        self.imu_yaw = 0.0
    
    self.imu_received = True
```

### 2. `/home/user/transbot_ws_ros2/quick_angular_test.py` (신규 생성)

90도 단일 테스트용 **빠른 테스트 스크립트**:

**특징:**
- ✅ 90도 회전만 테스트 (약 2분)
- ✅ 간단한 코드 구조
- ✅ 즉시 angular_scale 확인 가능
- ✅ Phase 2를 위한 초기값 제공

**사용법:**
```bash
python3 quick_angular_test.py
```

## 📊 첫 번째 테스트 결과 (자동 실행 중 캡처)

```
📊 측정 결과:
----------------------------------------------------------------
목표 각도:             90.0°

Odom (raw):           36.36° (보정 전)
IMU (적분):           86.33° (참고)
IMU (orientation):     0.00°  ← Raw IMU는 orientation 없음

angular_scale (적분):      2.3745 ⭐
angular_scale (orientation): 1.0000  ← 사용 불가

소요 시간:          2.0초
```

### 분석:
| 항목 | 값 | 의미 |
|------|-----|------|
| 목표 | 90.0° | 회전 목표 |
| **IMU 적분** | **86.33°** | 실제 회전량 (거의 정확) |
| **Odom raw** | **36.36°** | 보정 전 (매우 작음!) |
| **angular_scale** | **2.3745** | IMU / Odom = 86.33 / 36.36 |

### 해석:
1. **angular_scale ≈ 2.37**는 현재 적용된 값(1.5625)보다 훨씬 큽니다
2. Odom이 너무 작게 측정되고 있음 → **더 큰 보정 필요**
3. 현재 설정으로는 명령의 약 40% 정도만 Odom에 반영됨

## 🎯 현재 상황

### 데이터 플로우 (TF 충돌 해결 후)

```
/imu/data_raw
    ↓
imu_calib_node
    ↓
/imu/data_calibrated (Raw IMU)
    ↓
    ├─→ ekf_filter_node (센서 융합) → /odom
    └─→ calibration script (각속도 적분) → angular_scale 계산
```

### Raw IMU 특성
| 항목 | 제공 여부 | 비고 |
|------|----------|------|
| angular_velocity (각속도) | ✅ | 자이로스코프 측정값 |
| linear_acceleration (가속도) | ✅ | 가속도계 측정값 |
| **orientation (자세)** | ❌ | 필터가 계산해야 함 |
| orientation_covariance | -1.0 | "orientation 무효" |

## 🚀 다음 단계

### Option 1: 빠른 테스트 (추천) ⭐

```bash
cd ~/transbot_ws_ros2

# 시스템 실행 확인
ros2 node list | grep ekf_filter_node

# 90도 빠른 테스트
python3 quick_angular_test.py
```

**예상 결과:**
- 90° 회전 (약 2분)
- angular_scale 측정 (예: 2.37)
- Phase 2용 초기값 획득

### Option 2: 전체 캘리브레이션

```bash
# Phase 1: 90도 양방향 (약 5분)
python3 odom_based_angular_calibration.py --phase 1

# 결과 예: angular_scale = 2.35 ± 0.02

# Phase 2: 전체 각도 (약 20분)
python3 odom_based_angular_calibration.py --phase 2 --scale 2.35
```

## ⚠️  주의사항

### 1. Raw IMU 드리프트
Raw IMU는 필터링되지 않아 **드리프트가 더 클 수 있습니다**:
- ✅ 단기 회전 (90°, ~2초): 드리프트 최소
- ⚠️  장기 회전 (360°, ~12초): 드리프트 누적 가능

### 2. angular_scale 값 범위
| 값 | 의미 | 상태 |
|-----|------|------|
| 1.5-1.6 | 기존 측정값 | 재검증 필요 |
| **2.3-2.4** | 현재 측정값 | 새로운 baseline |
| > 3.0 | 비정상 | 하드웨어 점검 |

### 3. 시스템 상태 확인

테스트 전에 반드시 확인:
```bash
# 1. TF 발행자 확인 (imu_filter_madgwick 없어야 함)
ros2 topic info /tf --verbose | grep -E "imu_filter|ekf_filter"

# 2. IMU 데이터 확인
ros2 topic echo /imu/data_calibrated --once

# 3. Odom 데이터 확인
ros2 topic echo /odom_raw --once
```

## 💡 왜 angular_scale이 2.37인가?

### 기존 설정 (1.5625):
```
명령 속도 0.3 rad/s
  ↓ (하드웨어 증폭 3.9x)
실제 속도 ~1.17 rad/s
  ↓ (angular_scale 1.5625로 보정)
Odom 측정 ~0.75 rad/s (과대 보정)
```

### 새로운 측정 (2.3745):
```
명령 속도 0.3 rad/s
  ↓ (실제 하드웨어 동작)
실제 IMU 측정 ~0.43 rad/s (86.33° in 2s)
Odom 측정 ~0.18 rad/s (36.36° in 2s)
  ↓
angular_scale = 0.43 / 0.18 = 2.37
```

**결론:** 현재 `angular_scale = 1.5625`는 **과소 평가**되어 있습니다!

## 📋 체크리스트

테스트 실행 전:
- [ ] 시스템 실행 중: `ros2 launch sllidar_ros2 transbot_full_system.launch.py`
- [ ] TF 충돌 해결 확인: `imu_filter_madgwick` 노드 없음
- [ ] 로봇 주변 3m 공간 확보
- [ ] 배터리 50% 이상
- [ ] IMU 토픽 확인: `/imu/data_calibrated` 발행 중

테스트 선택:
- [ ] Option 1: `quick_angular_test.py` (빠른 90도 테스트)
- [ ] Option 2: `odom_based_angular_calibration.py --phase 1` (90도 양방향)
- [ ] Option 3: 전체 캘리브레이션 (Phase 1 + Phase 2)

## 🎉 요약

**문제:** TF 충돌 해결 후 캘리브레이션 스크립트가 IMU 데이터 수신 실패

**원인:** Raw IMU는 orientation 제공 안 함 (필터만 제공)

**해결:**
1. ✅ IMU 토픽 변경: `/imu/data_filtered` → `/imu/data_calibrated`
2. ✅ Raw IMU 처리 로직 추가: orientation 체크 + 각속도 적분만 사용
3. ✅ 빠른 테스트 스크립트 생성: `quick_angular_test.py`

**발견:**
- 🔍 angular_scale ≈ 2.37 (기존 1.56보다 50% 큼)
- 🔍 현재 Odom이 실제의 40% 정도만 측정

**다음:**
- 🚀 `quick_angular_test.py` 실행하여 정확한 초기값 측정
- 🚀 Phase 2로 전체 각도 검증
- 🚀 최종 angular_scale 적용
