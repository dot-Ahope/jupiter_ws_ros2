# EKF 센서 융합 실패 분석 보고서

## 📋 요약

**문제**: EKF 융합 출력이 Odom과 100% 일치하며, IMU 데이터가 완전히 무시됨  
**테스트 날짜**: 2025-10-24  
**시스템**: Transbot with robot_localization EKF

---

## 1️⃣ EKF가 Odom과 일치하는 이유

### 테스트 결과 분석

```
┌──────────────────┬─────────┬─────────┬─────────┬──────────────┐
│ 테스트           │ IMU     │ Odom    │ EKF     │ EKF-Odom 차이│
├──────────────────┼─────────┼─────────┼─────────┼──────────────┤
│ Test 1 (IMU 반시계)│ 89.37°  │ 74.52°  │ 74.51°  │ -0.01°       │
│ Test 2 (IMU 시계)  │ 86.87°  │ 71.77°  │ 71.76°  │ -0.01°       │
│ Test 3 (Odom 반시계)│203.29°  │118.30°  │118.29°  │ -0.01°       │
│ Test 4 (Odom 시계) │220.57°  │129.01°  │129.01°  │  0.00°       │
└──────────────────┴─────────┴─────────┴─────────┴──────────────┘

결론: EKF 출력 = Odom 측정값 (소수점 둘째자리까지 일치)
      IMU 기여도 = 0%
```

### 근본 원인: Process Noise vs Sensor Covariance 불균형

#### 현재 EKF 설정 (ekf_config.yaml)

```yaml
imu0_angular_velocity_covariance: 0.0001  # IMU 각속도 분산 (0.01²)
odom0_twist_covariance[vyaw]: 0.01        # Odom 각속도 분산 (0.1²)
process_noise_covariance[11]: 0.001       # 각속도 프로세스 노이즈
```

#### Kalman Gain 계산

Kalman Filter는 다음 공식으로 센서 신뢰도를 계산:

```
K = P / (P + R)

where:
  P = Process Noise (시스템 모델 불확실성)
  R = Sensor Covariance (센서 측정 불확실성)
  K = Kalman Gain (0에 가까울수록 센서 신뢰)
```

**현재 상황**:

```
IMU 센서:
  K_IMU = 0.001 / (0.001 + 0.0001) = 0.001/0.0011 = 0.909 (91%)
  → Gain 높음 = 센서 무시, 예측 모델 우선

Odom 센서:
  K_Odom = 0.001 / (0.001 + 0.01) = 0.001/0.011 = 0.091 (9%)
  → Gain 낮음 = 센서 신뢰, 측정값 사용

결과: EKF는 Odom 측정값만 사용하고, IMU는 완전히 무시
```

#### 왜 Process Noise가 문제인가?

| Process Noise | IMU Covariance | 비율 | 결과 |
|--------------|----------------|------|------|
| 0.01 (이전)   | 0.0001         | 100:1 | IMU 완전 무시 ❌ |
| 0.001 (현재)  | 0.0001         | 10:1  | IMU 여전히 무시 ❌ |
| 0.0001 (권장) | 0.0001         | 1:1   | IMU와 Odom 균형 ✅ |
| 0.00005      | 0.0001         | 1:2   | IMU 우선 ✅ |

**원칙**: Process Noise는 가장 정확한 센서(IMU)의 covariance와 **같거나 작아야** 센서 융합이 작동

---

## 2️⃣ Odom Raw 값을 IMU에 맞추는 방법

### 현재 상황

```
90° 물리적 회전 시:
  ✅ IMU 적분: 89.37° (목표 90°, 오차 -0.63°) - 99.3% 정확
  ❌ Odom 측정: 74.52° (목표 90°, 오차 -15.48°) - 82.8% 정확
  ❌ Odom 원시값: 47.71° (angular_scale 1.5618 적용 전) - 53.0% 정확
  
문제: Odom이 물리적 회전의 53%만 측정 (47% 손실)
```

### 원인 분석: 휠 슬립 vs 엔코더 문제

#### `/transbot/get_vel` 데이터 흐름

```
하드웨어 (MCU) → /transbot/get_vel (Twist) → base_node → /odom_raw
                     ↑
                실제 명령 속도가 아닌 "측정 속도"
```

**`base.cpp` (line 47)**:
```cpp
angular_velocity_z_ = msg->angular.z * angular_scale_;
```

#### Odom 손실의 두 가지 가능성

1. **휠 슬립** (바닥과의 마찰 부족)
   - 모터는 회전하지만 로봇은 덜 회전
   - 증상: `/transbot/get_vel`이 명령값 반환, 실제 회전은 부족
   - 해결: 바닥 마찰 개선 불가능

2. **엔코더 언더리딩** (센서 문제) ← **가능성 높음**
   - 엔코더가 실제 휠 회전의 일부만 측정
   - 증상: `/transbot/get_vel`이 실제보다 작은 값 반환
   - 해결: 하드웨어 펌웨어 수정 또는 `angular_scale` 보정

### 해결 방법

#### ✅ **방법 1: Angular Scale 정밀 재보정**

현재 `angular_scale = 1.5618`은 **평균값**입니다. IMU 기준 재측정:

```python
# odom_based_angular_calibration.py 실행
# IMU를 기준(ground truth)으로 삼아 Odom 보정

실제 측정 예시 (Test 1):
  IMU 측정: 89.37°
  Odom 측정: 74.52°
  필요 scale: 89.37 / 74.52 = 1.199
  
  원시 Odom: 74.52 / 1.5618 = 47.71°
  새로운 scale: 89.37 / 47.71 = 1.873
```

**새로운 angular_scale 계산**:

```
Test 1 (반시계): 89.37 / 47.71 = 1.873
Test 2 (시계):   86.87 / 45.95 = 1.890
평균: (1.873 + 1.890) / 2 = 1.881
```

**적용**:
```yaml
# transbot_full_system.launch.py
'angular_scale': 1.881  # 기존 1.5618에서 변경
```

#### ⚠️ **방법 2: 하드웨어 펌웨어 확인** (고급)

`/transbot/get_vel`이 발행하는 값이:
- 실제 엔코더 측정값인지
- 명령값(cmd_vel)을 그대로 반환하는지

확인 방법:
```bash
# Terminal 1: 명령 전송
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.3}}"

# Terminal 2: 측정값 확인
ros2 topic echo /transbot/get_vel

# 비교:
# 만약 /transbot/get_vel = 0.3 (명령값과 동일) → 슬립 문제
# 만약 /transbot/get_vel < 0.3 (명령값보다 작음) → 엔코더 문제
```

#### ❌ **방법 3: EKF에만 의존** (비권장)

Odom 수정 없이 EKF가 IMU로 보정하도록 설정:
```yaml
# IMU를 강하게 신뢰
imu0_angular_velocity_covariance: 0.00001  # 매우 낮게
odom0_twist_covariance[vyaw]: 0.1          # 매우 높게
process_noise_covariance[11]: 0.00005      # 매우 낮게
```

**문제점**:
- Odom 원시값이 부정확하면 SLAM 초기화 실패 가능
- 센서 융합 불안정 (한쪽에만 의존)

---

## 3️⃣ 권장 수정 방안

### 단계 1: Process Noise 수정 (EKF 융합 활성화)

```yaml
# ekf_config.yaml
process_noise_covariance[11]: 0.00005  # 0.001 → 0.00005 (IMU의 50%)
```

**효과**:
```
새로운 Kalman Gain:
  K_IMU = 0.00005 / (0.00005 + 0.0001) = 0.333 (33%)
  K_Odom = 0.00005 / (0.00005 + 0.01) = 0.005 (0.5%)
  
결과: IMU 67% 신뢰, Odom 33% 신뢰 (센서 융합 작동!)
```

### 단계 2: Angular Scale 재보정

```python
# 1. IMU 기준 정밀 측정
python3 ~/transbot_ws_ros2/odom_based_angular_calibration.py

# 2. 4회 회전 테스트 (±90°, ±180°)
# 3. 평균 scale 계산
# 4. launch 파일에 적용
```

**예상 결과**:
```
수정 전: Odom 74.52° (IMU 89.37° 대비 83%)
수정 후: Odom ~88-90° (IMU 기준 98%+)
```

### 단계 3: 검증

```bash
# 시스템 재빌드
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2 transbot_bringup
source install/setup.bash

# 재시작
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 테스트
python3 ekf_comparison_test.py
```

**성공 기준**:
```
✅ EKF ≠ Odom (차이 5-10°)
✅ EKF가 IMU와 Odom 중간값
✅ SLAM 드리프트 < 1°/min
```

---

## 4️⃣ 기술적 배경

### Robot Localization EKF의 센서 융합 원리

```
EKF 상태 추정:
  x_k = F·x_{k-1} + w_k     (예측 단계, process noise w)
  y_k = H·x_k + v_k         (측정 단계, sensor noise v)
  
Kalman Gain:
  K = P·H^T / (H·P·H^T + R)
  
where:
  P = Process Noise Covariance
  R = Sensor Noise Covariance
  
센서 가중치:
  weight = (1 - K) = R / (P + R)
  
핵심: P ≤ R_best_sensor 일 때 최적 융합
```

### IMU vs Odom 특성 비교

| 센서 | 장점 | 단점 | 사용 목적 |
|-----|------|------|----------|
| **IMU** | • 드리프트 없음 (단기)<br>• 빠른 응답 (100Hz)<br>• 절대 각속도 | • 적분 오차 누적<br>• 자이로 바이어스 | 단기 각속도 정확도 |
| **Odom** | • 장기 안정성<br>• 위치 정보 제공 | • 휠 슬립<br>• 엔코더 오차 | 장기 위치 추정 |
| **EKF 융합** | • 두 센서 장점 결합<br>• 오차 상호 보정 | • 튜닝 복잡 | SLAM 기준 좌표계 |

---

## 5️⃣ 다음 단계

### 즉시 실행

1. **Process Noise 감소**
   ```bash
   # ekf_config.yaml line 84 수정
   # 0.001 → 0.00005
   ```

2. **Angular Scale 재측정**
   ```bash
   python3 odom_based_angular_calibration.py
   # 새로운 scale 값 기록
   ```

3. **Launch 파일 업데이트**
   ```python
   # transbot_full_system.launch.py
   'angular_scale': [새로운 값]
   ```

4. **재테스트**
   ```bash
   python3 ekf_comparison_test.py
   ```

### 장기 개선

1. **하드웨어 펌웨어 분석**
   - `/transbot/get_vel` 소스 코드 확인
   - 엔코더 PPR (Pulses Per Revolution) 검증
   - 휠 직경 실측

2. **Dynamic Reconfigure 활용**
   ```bash
   # 실시간 파라미터 조정
   ros2 param set /ekf_filter_node process_noise_covariance [...]
   ```

3. **로그 분석**
   ```bash
   ros2 topic echo /diagnostics | grep ekf
   ```

---

## 📝 결론

1. **EKF가 Odom과 일치하는 이유**:
   - Process Noise (0.001) > IMU Covariance (0.0001)
   - Kalman Filter가 IMU 무시하고 Odom만 사용
   - 해결: Process Noise를 0.00005로 감소

2. **Odom을 IMU에 맞추는 방법**:
   - 현재 Odom은 물리적 회전의 53%만 측정
   - Angular Scale을 1.5618 → 1.881로 증가
   - 또는 하드웨어 엔코더 설정 확인

**최종 목표**: EKF 융합으로 정확도 95%+ 달성, SLAM 드리프트 제거
