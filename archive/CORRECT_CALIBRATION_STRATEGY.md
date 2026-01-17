# 🎯 수정된 회전 보정 전략

## 핵심 개념 정리

### ❌ 잘못된 이해
```
angular_scale: Odom과 IMU를 일치시키는 파라미터
→ 두 센서를 서로 맞춘다
```

### ✅ 올바른 이해
```
angular_scale: Odom을 물리적 환경에 맞추는 파라미터
IMU calibration: IMU를 물리적 환경에 맞추는 파라미터

→ 각 센서를 독립적으로 물리적 환경(ground truth)에 맞춘다
→ 결과적으로 두 센서가 일치하게 됨
```

---

## 🎯 올바른 보정 전략

```
              물리적 환경 (90°)
                    ↓
        ┌───────────┴───────────┐
        ↓                       ↓
      Odom (58°)              IMU (145°)
        ↓                       ↓
   angular_scale          gyro_bias 제거
    (1.55 = 90/58)       (imu_calib)
        ↓                       ↓
      90° ✅                   90° ✅
        ↓                       ↓
        └───────────┬───────────┘
                    ↓
              EKF (센서 융합)
                    ↓
                  90° ✅
```

---

## 📋 작업 순서 (수정판)

### Priority 1: IMU 자이로 캘리브레이션 ⭐⭐⭐⭐⭐

**목적:** IMU를 물리적 환경에 맞춤

**방법:**
```bash
# 1. 시스템 시작
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 2. 캘리브레이션
ros2 run imu_calib do_calib /transbot/imu
```

**과정:**
```
Phase 1: 자이로 바이어스 측정
  - 로봇 완전히 정지
  - 100 샘플 수집
  - 결과: gyro_bias = [-0.005, -0.003, -0.008] rad/s

Phase 2: 가속도계 캘리브레이션
  - 6방향 자세
  - 각 자세 2-3초 유지
  - 결과: accel_bias, accel_scale

저장:
  mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml
```

**기대 효과:**
```
IMU 측정값:
  보정 전: 145° (실제 90° 회전 시)
  보정 후: 88-92° ✅

드리프트:
  보정 전: -0.008 rad/s (27.8°/min)
  보정 후: ~0.0001 rad/s (0.06°/min) ✅
```

---

### Priority 2: Phase 1 진단 실행 ⭐⭐⭐⭐⭐

**목적:** 두 센서를 독립적으로 물리적 환경과 비교

**방법:**
```bash
python3 phase1_odom_imu_diagnosis.py
```

**Test 1: 수동 90° 회전 (물리적 기준점)**
```
준비:
  1. 바닥에 0°, 90° 테이프 표시
  2. 로봇을 손으로 정확히 90° 회전
  3. 테이프 표시로 물리적 정확도 확인

측정:
  - 물리적 회전: 90.00° (ground truth)
  - Odom 측정:   XX.XX°
  - IMU 측정:    YY.YY°
  - EKF 출력:    ZZ.ZZ°

분석 1 - Odom 보정:
  angular_scale = 90.0 / Odom측정값
  예: Odom = 58° → scale = 1.55

분석 2 - IMU 검증:
  IMU 오차 = |IMU측정값 - 90.0|
  예: IMU = 145° → 오차 55° ❌ (재캘리브레이션 필요!)
  예: IMU = 91° → 오차 1° ✅ (캘리브레이션 성공!)

분석 3 - EKF 상태:
  if EKF ≈ 0°:
      "EKF가 IMU를 outlier로 무시"
      → Odom과 IMU 차이가 너무 큼
  elif EKF ≈ 90°:
      "EKF가 센서 융합 성공"
      → 두 센서가 물리적 환경과 일치
```

**Test 2: 모터 구동 90° 회전 (현재 설정 검증)**
```
과정:
  1. 현재 angular_scale로 Odom 90° 도달 시까지 회전
  2. 바닥 테이프와 비교하여 실제 회전각 확인

측정:
  - Odom 명령: 90°
  - 실제 회전: 바닥 표시로 확인
  - IMU 측정: ZZ°

분석:
  if 실제 < 90°:
      "angular_scale 증가 필요"
  elif 실제 > 90°:
      "angular_scale 감소 필요"
  else:
      "angular_scale 적절 ✅"
  
  if IMU ≈ 실제:
      "IMU 캘리브레이션 적절 ✅"
  else:
      "IMU 재캘리브레이션 필요"
```

---

### Priority 3: 보정 파라미터 적용

#### 3-1: angular_scale 업데이트
```python
# transbot_full_system.launch.py
'angular_scale': 1.XX,  # Phase 1 Test 1 결과 적용
```

#### 3-2: IMU 캘리브레이션 검증
```bash
# Phase 1 Test 1에서 IMU 오차가 크면
ros2 run imu_calib do_calib /transbot/imu  # 재실행
```

---

### Priority 4: 통합 검증

**4-1: 센서별 독립 검증**
```bash
# Odom 검증
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{angular: {z: 0.3}}'
# → 바닥 표시와 비교: 90° 정확한가?

# IMU 검증
ros2 topic echo /imu/data_calibrated --field angular_velocity.z
# → 정지 상태: ~0.0001 rad/s? (드리프트 없음)
```

**4-2: EKF 융합 검증**
```bash
ros2 topic echo /odometry/filtered
# → 90° 회전 시 EKF도 90° 출력?
# → 두 센서가 물리적으로 정확하면 EKF도 정확함
```

**4-3: SLAM 검증**
```bash
# 정사각형 경로 (1m × 1m, 90° × 4)
# 시작점으로 정확히 복귀하는가?
```

---

## 📊 진단 결과 해석 가이드

### Case 1: 두 센서 모두 정확 ✅
```
물리적: 90°
Odom:   90° (angular_scale 적용 후)
IMU:    90° (gyro_bias 제거 후)
EKF:    90° (융합 성공)

→ 완료! SLAM 테스트 진행
```

### Case 2: Odom만 부정확 ⚠️
```
물리적: 90°
Odom:   58° → angular_scale = 1.55 필요
IMU:    91° ✅
EKF:    ~90° (IMU에 의존)

→ angular_scale 업데이트 후 재테스트
```

### Case 3: IMU만 부정확 ⚠️
```
물리적: 90°
Odom:   90° ✅
IMU:    145° → gyro_bias 재측정 필요
EKF:    ~0° (IMU를 outlier로 거부)

→ imu_calib 재실행 후 재테스트
```

### Case 4: 두 센서 모두 부정확 ❌
```
물리적: 90°
Odom:   58°
IMU:    145°
EKF:    ~0° (혼란 상태)

→ Priority 1 & 2 재실행
```

---

## 🎓 기술적 배경

### 왜 각 센서를 독립적으로 보정하는가?

**잘못된 접근 (센서 간 매칭):**
```python
# ❌ 나쁜 방법
angular_scale = IMU측정값 / Odom측정값
# 문제: IMU가 부정확하면 Odom도 잘못 보정됨
```

**올바른 접근 (물리적 기준):**
```python
# ✅ 좋은 방법
angular_scale = 물리적회전 / Odom측정값
gyro_bias = IMU측정값 - 물리적회전 (적분 후)
# 장점: 각 센서가 독립적으로 정확해짐
```

### Ground Truth (물리적 기준)의 중요성

**바닥 테이프 표시 = Ground Truth**
```
센서 A → 센서 B 매칭: 둘 다 틀릴 수 있음
센서 → 물리적 환경 매칭: 정확도 보장
```

---

## ✅ 수정된 체크리스트

### 단계별 검증
- [ ] 1. IMU 캘리브레이션 실행
  - [ ] gyro_bias가 imu_calib.yaml에 저장됨
  - [ ] 정지 상태 드리프트 < 0.1°/min

- [ ] 2. Phase 1 진단 실행
  - [ ] Test 1: 수동 90° 회전
    - [ ] Odom 측정값 기록: _____°
    - [ ] IMU 측정값 기록: _____°
    - [ ] angular_scale 계산: _____
    - [ ] IMU 오차: _____° (< 5° 목표)
  
  - [ ] Test 2: 모터 구동 90° 회전
    - [ ] 실제 물리적 회전: _____°
    - [ ] angular_scale 검증: ✅/❌

- [ ] 3. 보정 파라미터 적용
  - [ ] angular_scale 업데이트
  - [ ] 필요시 IMU 재캘리브레이션
  - [ ] 시스템 재시작

- [ ] 4. 통합 검증
  - [ ] Odom: 90° 회전 → 실제 90° ✅
  - [ ] IMU: 정지 시 드리프트 없음 ✅
  - [ ] EKF: 90° 회전 시 90° 출력 ✅
  - [ ] SLAM: 회전 정확도 85%+ ✅

---

## 🔬 예상 결과

### 올바른 보정 후
```
센서별 독립 검증:
  Odom:
    물리적 90° → 측정 90° ✅
    오차: 0-2° (휠 슬립 범위 내)
  
  IMU:
    물리적 90° → 측정 88-92° ✅
    정지 드리프트: < 0.1°/min ✅

센서 융합:
  EKF:
    두 센서 모두 ~90° → 융합 성공
    outlier rejection 없음 ✅
  
  SLAM:
    회전 추적 정확도: 85%+
    누적 오차: 최소화
```

---

## 📝 핵심 원칙

**"각 센서를 물리적 환경(ground truth)에 맞춰라. 센서끼리 맞추지 마라."**

1. **Odom 보정:** angular_scale로 물리적 회전과 일치
2. **IMU 보정:** gyro_bias 제거로 물리적 회전과 일치
3. **결과:** 두 센서가 모두 정확하면 EKF와 SLAM도 정확

---

**다음 단계: Priority 1부터 시작!**
```bash
ros2 run imu_calib do_calib /transbot/imu
```
