# Phase 2 결과 분석 및 문제 진단

## 🚨 심각한 문제: IMU 과대 측정

### 측정 데이터 요약

| 목표 각도 | 물리적 회전 | Odom (raw) | IMU 적분 | IMU 오차 | 오차율 |
|-----------|-------------|------------|----------|----------|--------|
| 90° (CCW) | ~90° ✅ | 65.72° | 135.26° | +45.26° | **+50.3%** |
| 90° (CW)  | ~90° ✅ | 64.27° | 130.55° | +40.55° | **+45.1%** |
| 180° (CCW)| ~180° ✅ | 124.89° | 308.41° | +128.41° | **+71.3%** |
| 180° (CW) | ~180° ✅ | 122.39° | 281.21° | +101.21° | **+56.2%** |
| 270° (CCW)| ~270° ✅ | 178.03° | 432.91° | +162.91° | **+60.3%** |
| 270° (CW) | ~270° ✅ | 180.70° | 429.45° | +159.45° | **+59.1%** |
| 360° (CCW)| ~360° ✅ | 239.13° | 608.58° | +248.58° | **+69.1%** |
| 360° (CW) | ~360° ✅ | 240.77° | 585.74° | +225.74° | **+62.7%** |

---

## 📊 문제 분석

### 1. IMU 과대 측정 (심각!)

**IMU/물리적 비율:**
```
90°:  1.503x (135.3 / 90)
180°: 1.713x (308.4 / 180)
270°: 1.603x (432.9 / 270)
360°: 1.691x (608.6 / 360)

평균: 1.63x (63% 과대 측정!)
```

**결론:** IMU가 실제보다 약 **50~70% 크게** 측정하고 있습니다.

---

### 2. Odom 언더리포팅 (예상대로)

**Odom/물리적 비율:**
```
90°:  0.722x (65.0 / 90)  → 필요 scale: 1.385
180°: 0.694x (124.9 / 180) → 필요 scale: 1.441
270°: 0.659x (178.0 / 270) → 필요 scale: 1.517
360°: 0.664x (239.1 / 360) → 필요 scale: 1.506

평균 필요 angular_scale: 1.46 ⭐
```

**결론:** Odom은 일관되게 약 **30~35% 언더리포팅**

---

### 3. EKF가 IMU 무시 (센서 융합 실패)

**EKF Odom 출력:**
```
90°:  0.04~0.05° (거의 0°!)
180°: 0.04~0.05°
270°: 0.06~4.04°
360°: 0.08~0.09°
```

**원인:** Odom과 IMU의 차이가 너무 커서 EKF가 IMU를 outlier로 거부

**증거:**
```
Odom: 65° | IMU: 135° → 차이 2.08배
→ Mahalanobis distance >> rejection_threshold
→ EKF: "IMU가 비정상, 무시!"
```

---

## 🔍 원인 분석

### IMU 과대 측정 원인 후보

#### 1. 자이로 캘리브레이션 오류 ⭐⭐⭐
```yaml
# imu_calib.yaml 문제
gyro_bias:
  z: 잘못된 값 또는 없음

가능성:
  - 캘리브레이션 시 로봇이 움직임
  - 잘못된 부호 (+/-)
  - 스케일 팩터 오류
```

#### 2. 자이로 스케일 팩터 오류 ⭐⭐
```
MPU6050 설정:
  레인지: ±500°/s
  LSB: 65.5 LSB/(°/s)
  
문제 가능성:
  - 레지스터 설정 오류
  - 단위 변환 오류 (deg/s vs rad/s)
  - 펌웨어 스케일 팩터 버그
```

#### 3. 시간 간격(dt) 계산 오류 ⭐
```python
# imu_callback
dt = (current_time - self.last_imu_time).nanoseconds / 1e9
self.integrated_imu_yaw += self.imu_angular_vel_z * dt

문제 가능성:
  - dt가 실제보다 크게 계산됨
  - 타임스탬프 동기화 문제
```

#### 4. 적분 누적 오차 ⭐
```
회전량이 클수록 오차 증가:
  90°:  +50%
  180°: +64%
  270°: +60%
  360°: +66%

→ 누적 드리프트 + 스케일 오류 복합
```

---

## 🧪 진단 방법

### Test 1: 정지 상태 자이로 바이어스

```bash
# 로봇 완전히 정지
ros2 topic echo /transbot/imu --field angular_velocity.z

# 보정 후
ros2 topic echo /imu/data_calibrated --field angular_velocity.z

예상 (정상):
  /transbot/imu:        -0.008 rad/s (드리프트)
  /imu/data_calibrated: ~0.0001 rad/s (거의 0)

만약 비정상:
  /imu/data_calibrated: -0.015 rad/s (과대 보정!)
```

### Test 2: 회전 중 각속도 측정

```bash
# 0.3 rad/s 명령
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{angular: {z: 0.3}}'

# IMU 측정
ros2 topic echo /imu/data_calibrated --field angular_velocity.z

예상 (정상):
  0.28~0.32 rad/s (±7%)

실제 (비정상 예상):
  0.45~0.50 rad/s (+50~67%)
```

### Test 3: 10초 적분 테스트

```python
# 0.3 rad/s로 10초 회전
expected = 0.3 × 10 = 3.0 rad = 171.9°

if imu_integrated > 260°:
    print("IMU 과대 측정 확인!")
    print(f"실제 적분: {imu_integrated}°")
    print(f"과대 비율: {imu_integrated / 171.9}")
```

---

## ✅ 해결 방안

### Priority 1: IMU 재캘리브레이션 ⭐⭐⭐⭐⭐

**절차:**
```bash
# 1. 센서 온도 안정화
# 로봇을 5분 이상 켜두기

# 2. 완전히 정지된 평평한 바닥에 배치
# 진동 없는 환경

# 3. 캘리브레이션
ros2 run imu_calib do_calib /transbot/imu

# Phase 1: 정지 유지 (100 샘플, 10초)
# ⚠️ 절대 움직이지 말 것!

# Phase 2: 6방향 자세
# 천천히 회전, 각 자세 3초 유지

# 4. 저장 및 재시작
mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml
# 시스템 재시작
```

**검증:**
```bash
# 정지 상태 드리프트
ros2 topic echo /imu/data_calibrated --field angular_velocity.z
# 목표: ~0.0001 rad/s (거의 0)

# 90° 회전 테스트
python3 odom_based_angular_calibration.py --phase 1
# 목표: IMU 적분 88~92°
```

---

### Priority 2: angular_scale을 물리적 기준으로 계산 ⭐⭐⭐⭐

**현재 문제:**
```python
# 잘못된 계산
angular_scale = IMU적분 / Odom
                ↑ 과대 측정!
              = 135.3 / 65.0 = 2.08 ❌
```

**올바른 계산:**
```python
# 물리적 기준
angular_scale = 물리적_회전 / Odom

데이터 기반:
  90°:  90 / 65.0  = 1.385
  180°: 180 / 124.9 = 1.441
  270°: 270 / 178.0 = 1.517
  360°: 360 / 239.1 = 1.506

평균: 1.46 ⭐
표준편차: 0.055
변동계수: 3.8% (신뢰도 높음)

권장값: 1.46
```

**적용:**
```python
# transbot_full_system.launch.py
'angular_scale': 1.46,  # 현재: 1.5625

# 재빌드
colcon build --packages-select sllidar_ros2
```

---

### Priority 3: EKF rejection threshold 조정 (임시) ⭐⭐

**IMU 재캘리브레이션 전까지 임시 조치:**

```yaml
# ekf_config.yaml
imu0_twist_rejection_threshold: 5.0  # 2.0 → 5.0
# IMU가 과대 측정이므로 일단 더 큰 차이 허용

imu0_relative: false  # true → false
# 절대값 사용으로 누적 오차 감소
```

**주의:** 이것은 **임시 조치**입니다. 근본 해결은 IMU 재캘리브레이션!

---

## 📈 예상 개선 효과

### IMU 재캘리브레이션 후

**Before (현재):**
```
90° 회전 시:
  물리적: 90°
  Odom:   65° (1.385 scale 필요)
  IMU:    135° (50% 과대!)
  EKF:    0° (IMU 무시)
```

**After (재캘리브레이션 후):**
```
90° 회전 시:
  물리적: 90°
  Odom:   65° (1.46 scale 적용 → 95°)
  IMU:    88-92° (정확!)
  EKF:    90° (융합 성공!)
```

### angular_scale 적용 후

**Before:**
```
angular_scale: 1.5625 (Phase 1 잘못된 결과)
90° 명령 → 실제 102° 회전 (12% 과다)
```

**After:**
```
angular_scale: 1.46
90° 명령 → 실제 95° 회전 (5% 오차 허용 범위)
```

---

## 🎯 작업 순서

### 즉시 실행:

1. **IMU 재캘리브레이션** (30분)
   ```bash
   ros2 run imu_calib do_calib /transbot/imu
   mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml
   ```

2. **검증 테스트** (5분)
   ```bash
   # 정지 상태 드리프트
   ros2 topic echo /imu/data_calibrated --field angular_velocity.z
   
   # Phase 1 재실행
   python3 odom_based_angular_calibration.py --phase 1
   ```

3. **angular_scale 업데이트** (5분)
   ```python
   # launch 파일 수정
   'angular_scale': 1.46,
   
   # 재빌드
   colcon build --packages-select sllidar_ros2
   ```

4. **최종 검증** (20분)
   ```bash
   # Phase 2 재실행
   python3 odom_based_angular_calibration.py --phase 2 --scale 1.46
   
   # SLAM 회전 테스트
   # 90° × 4 정사각형 경로
   ```

---

## 📝 체크리스트

### IMU 캘리브레이션
- [ ] 센서 온도 안정화 (5분)
- [ ] 평평한 바닥, 진동 없음
- [ ] 완전히 정지 상태
- [ ] `ros2 run imu_calib do_calib /transbot/imu`
- [ ] Phase 1: 정지 유지 (움직이지 말 것!)
- [ ] Phase 2: 6방향 자세
- [ ] `mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml`
- [ ] 시스템 재시작

### 검증
- [ ] 정지 드리프트 < 0.001 rad/s
- [ ] Phase 1: IMU 적분 88~92° (90° 목표)
- [ ] angular_scale 계산: 1.35~1.55 범위
- [ ] EKF가 IMU 반영 (>10° 출력)

### angular_scale 적용
- [ ] launch 파일 수정: 1.46
- [ ] 재빌드
- [ ] Phase 2 재테스트
- [ ] 모든 각도에서 ±5° 오차

### SLAM 테스트
- [ ] 90° × 4 정사각형
- [ ] 360° 제자리 회전
- [ ] 시작점 복귀 오차 < 10cm, < 5°

---

## 🔬 기술적 배경

### IMU 적분 오차의 원인

1. **바이어스 드리프트**
   ```
   bias = -0.008 rad/s (캘리브레이션 전)
   90° 회전 (3초) 시:
     오차 = -0.008 × 3 = -0.024 rad = -1.4°
   ```

2. **스케일 팩터 오류**
   ```
   실제: 0.3 rad/s
   측정: 0.45 rad/s (50% 과대)
   90° 회전 시:
     오차 = (0.45 - 0.3) × 3 = 0.45 rad = 25.8°
   ```

3. **누적 오차**
   ```
   바이어스 + 스케일 오류 복합:
     90°:  -1.4° + 25.8° = +24.4°
     180°: -2.8° + 51.6° = +48.8°
     
   실제 측정:
     90°:  +45.3° (예측과 유사!)
     180°: +128.4° (더 큰 오차!)
   ```

### 왜 Phase 1에서는 문제가 덜 보였는가?

**Phase 1 (--phase 1):**
- IMU 기준으로 회전
- 단일 각도 (90°)
- 짧은 시간 (2초)
- 누적 오차 작음

**Phase 2 (--phase 2 --scale 1.516):**
- Odom 기준으로 회전
- 큰 각도 (360°)
- 긴 시간 (3.7초)
- 누적 오차 심각

---

## 💡 교훈

### 올바른 캘리브레이션 순서:

1. **IMU 먼저 캘리브레이션**
   - 정지 상태 바이어스 제거
   - 스케일 팩터 확인
   - 드리프트 < 0.1°/min

2. **Odom angular_scale 계산**
   - IMU가 정확한 상태에서
   - 물리적 기준과 비교
   - 여러 각도 테스트

3. **EKF 튜닝**
   - 두 센서가 정확한 상태에서
   - rejection threshold 조정
   - 융합 품질 검증

**절대 하지 말아야 할 것:**
- ❌ IMU가 부정확한 상태에서 angular_scale 계산
- ❌ 하나의 부정확한 센서를 기준으로 다른 센서 보정
- ❌ 센서 간 매칭 (둘 다 틀릴 수 있음)

**항상 해야 할 것:**
- ✅ 각 센서를 물리적 환경(ground truth)과 비교
- ✅ IMU 캘리브레이션 후 재검증
- ✅ 여러 각도/방향에서 일관성 확인

---

**다음 단계: IMU 재캘리브레이션 즉시 실행!**

```bash
ros2 run imu_calib do_calib /transbot/imu
```
