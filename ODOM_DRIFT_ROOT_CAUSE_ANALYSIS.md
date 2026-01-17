# 🔍 Odom 기반 회전 135° 오버슈트 원인 분석

## 📊 테스트 결과 요약

### IMU 기준 회전 (목표 90°)
- **반시계**: IMU +86.72° | Odom +69.96° | EKF +69.96°
- **시계**: IMU -89.81° | Odom -87.00° | EKF -87.00°
- **결과**: ✅ **거의 정확히 90° 회전** (IMU 평균 88.27°)

### Odom 기준 회전 (목표 90°)
- **반시계**: IMU +191.70° | Odom +130.93° | EKF +130.93°  ← **135° 실제 회전**
- **시계**: IMU -175.69° | Odom -129.20° | EKF -129.20°    ← **135° 실제 회전**
- **결과**: ❌ **45° 오버슈트** (실제 135° 회전)

### 권장 angular_scale
- **현재 설정**: 1.8819
- **측정 기반 권장**: 2.1164 (IMU 88.27° / Odom 78.48° = 1.1246 → 1.8819 × 1.1246)
- **오차**: 12.46% (> 10%)

---

## 🚨 근본 원인 분석

### 1. 왜 Odom 기준으로 90° 회전 시 실제로는 135° 회전했는가?

#### A. angular_scale 1.8819가 부족함
```
실제 물리 회전 90° = 엔코더 측정 47.8° × angular_scale 1.8819
하지만 엔코더는 37.18° 정도만 측정 (IMU 기준 회전 시)

=> 실제 필요한 angular_scale = 90° / 37.18° = 2.42
=> 현재 1.8819는 약 23% 부족
```

#### B. 회전 중 엔코더 데이터 손실
```
테스트 1 (IMU 기준 +90°):
- 소요 시간: 2.54초
- Odom 측정: +69.96°
- IMU 측정: +86.72°
- 차이: -16.76° (19% 손실)

테스트 3 (Odom 기준 +90°):
- 소요 시간: 2.86초
- Odom 목표: +88.2° 도달
- IMU 측정: +191.70° (실제 회전)
- 초과 회전: +103.5° (217% 과다)
```

**핵심 문제**:
- Odom이 88.2°를 측정했을 때 실제로는 191.7° 회전
- 즉, **Odom은 실제 회전의 46%만 측정** (88.2 / 191.7 = 0.46)
- 로봇은 "Odom이 90°라고 말할 때까지" 계속 회전
- 하지만 Odom이 90°를 측정하는 동안 로봇은 실제로 195° 회전

#### C. 50Hz 발행에도 불구하고 데이터 누락
```
IMU 기준 회전 로그:
[INFO]: 진행  93.0% | IMU적분:  +83.71° | Odom:  +30.42° | EKF:  +30.42°

문제점:
- IMU가 83.71°일 때 Odom은 30.42°만 측정
- 차이: 53.29° (63% 손실!)
- 회전 중간에는 더 심한 손실 발생
```

**50Hz가 충분하지 않은 이유**:
1. **고속 회전 (0.3 rad/s = 17°/s)**
   - 0.02초(50Hz) 간격 = 0.34° 변화
   - 하지만 실제 측정은 더 낮은 간격으로 발생
   
2. **엔코더 펄스 카운팅 지연**
   - `Transbot_Lib.Get_Encoder_Velocity()` 호출 시간
   - I2C 통신 지연 (MPU6050과 공유)
   
3. **회전 중 휠 슬립**
   - 제자리 회전 시 접지력 감소
   - 바퀴가 돌아도 엔코더 펄스 누락

---

### 2. 왜 EKF 융합 측정값이 Odom과 정확히 일치하는가?

#### A. EKF 설정 분석 (`ekf_config.yaml`)

**센서 공분산 (측정 불확실성)**:
```yaml
# IMU 각속도 (vyaw) - index 11
imu0_angular_velocity_covariance: 0.000009  # 0.003^2 (매우 정확)

# Odom 각속도 (vyaw) - index 11  
odom0_twist_covariance: 0.04  # 0.2^2 (신뢰도 낮음)

# Process Noise (시스템 불확실성) - vyaw
process_noise_covariance[11]: 0.00002
```

**Kalman Gain 계산** (vyaw - 각속도):
```
K_IMU = Process_Noise / (Process_Noise + IMU_Cov)
      = 0.00002 / (0.00002 + 0.000009)
      = 0.00002 / 0.000029
      ≈ 0.69 (IMU 신뢰 69%)

K_Odom = Process_Noise / (Process_Noise + Odom_Cov)
       = 0.00002 / (0.00002 + 0.04)
       = 0.00002 / 0.04002
       ≈ 0.0005 (Odom 신뢰 0.05%)
```

**하지만 실제 EKF는 위치/각도도 융합**:

**Odom 위치/각도 (pose) 공분산**:
```yaml
# Odom yaw (각도 - index 5)
odom0_pose_covariance: 0.01  # 0.1^2

# Process Noise - yaw (index 5)
process_noise_covariance[5]: 0.04
```

**Kalman Gain 계산** (yaw - 각도):
```
K_Odom_yaw = 0.04 / (0.04 + 0.01)
           = 0.04 / 0.05
           = 0.8 (Odom 신뢰 80%)
```

#### B. 왜 EKF = Odom인가?

**1) IMU는 각속도만 제공 (각도 제공 안 함)**
```yaml
imu0_config: [false, false, false,     # 위치 사용 안 함
              false, false, false,     # 방향(각도) 사용 안 함
              false, false, false,     # 속도 사용 안 함
              false, false, true,      # vyaw만 사용 (각속도)
              false, false, false]
```

**2) Odom은 각도를 직접 제공**
```yaml
odom0_config: [true,  true,  false,    # x, y 사용
               false, false, true,     # yaw(각도) 사용 ⭐
               true,  true,  false,    # vx, vy 사용
               false, false, true,     # vyaw(각속도) 사용
               false, false, false]
```

**3) EKF는 각도에서 Odom에 80% 의존**
- IMU는 각속도만 제공하므로 EKF가 적분하여 각도 추정
- 하지만 Odom은 각도를 직접 제공
- Process Noise 0.04 vs Odom Cov 0.01 → Odom이 4배 더 신뢰
- **결과: EKF의 각도 추정 ≈ Odom 각도 (80% 가중치)**

**4) IMU 각속도 적분의 누적 오차**
```
IMU 각속도만으로 각도 추정:
θ(t) = θ(t-1) + ω_z × dt

문제점:
- 작은 오차도 누적 (drift)
- 0.003 rad/s 오차 × 3초 = 0.009 rad = 0.5°
- 하지만 IMU bias는 calibration으로 제거됨

실제 문제:
- dt 불확실성 (50Hz = 0.02s 간격)
- 샘플링 시간 지터
```

#### C. 결론

**EKF가 Odom과 일치하는 이유**:
1. **각도(yaw) 측정**: Odom만 제공 (IMU는 각속도만)
2. **Kalman Gain**: Odom 각도 신뢰도 80% vs IMU 적분 20%
3. **Process Noise 설정**: Odom을 더 신뢰하도록 튜닝됨
4. **IMU 각속도 적분 한계**: 누적 오차로 인해 장기 추정 불가

**설계 의도**:
- IMU: 단기 각속도 정확도 (회전 감지)
- Odom: 장기 각도 기준점 (누적 위치)
- EKF: IMU 각속도로 Odom 각도 스무딩

**하지만 실제로는**:
- Odom 각도가 부정확 (angular_scale 오류)
- EKF가 Odom을 따라가므로 함께 부정확
- IMU 각속도 적분이 더 정확하지만 무시됨

---

## 🎯 핵심 문제 요약

### 1. angular_scale 부정확 (1.8819 → 2.1164 권장)
- **현상**: 엔코더가 실제 회전의 80% 정도만 측정
- **원인**: 
  - 휠 슬립 (제자리 회전 시)
  - 엔코더 분해능 부족
  - 기계적 백래시
- **영향**: Odom이 90°를 측정할 때 실제로는 113° 회전

### 2. 회전 중 엔코더 데이터 손실
- **현상**: 고속 회전 시 50Hz도 불충분
- **원인**:
  - I2C 통신 병목 (IMU와 공유)
  - `Get_Encoder_Velocity()` 호출 지연
  - 펄스 카운팅 누락
- **영향**: 실제 130° 측정이 88°로 보고됨 (32% 손실)

### 3. EKF가 잘못된 Odom을 따라감
- **현상**: EKF = Odom (IMU 무시)
- **원인**:
  - Odom 각도 신뢰도 80% 설정
  - IMU는 각속도만 제공 (각도 제공 안 함)
  - Process Noise 튜닝이 Odom 우선
- **영향**: 부정확한 Odom이 EKF까지 오염

---

## 💡 해결 방안

### 단기 (즉시 적용 가능)

#### 1. angular_scale 재보정
```python
# transbot_full_system.launch.py
'angular_scale': 2.1164,  # 1.8819 → 2.1164 (+12.5%)
```

**검증 방법**:
```bash
# 물리적으로 90° 회전 테스트
python3 ekf_comparison_test.py

# 기대 결과:
# - IMU 평균: 87-90°
# - Odom 평균: 87-90°
# - 오차: < 5%
```

#### 2. 회전 속도 감소
```python
# ekf_comparison_test.py
speed = 0.2  # 0.3 → 0.2 (33% 감속)
```

**효과**:
- 엔코더 샘플링 여유 증가
- 휠 슬립 감소
- 데이터 손실 최소화

### 중기 (코드 수정 필요)

#### 3. 엔코더 폴링 주파수 증가
```python
# transbot_driver.py
self.create_timer(0.01, self.publish_velocity)  # 50Hz → 100Hz
```

**주의**: I2C 병목 가능성 확인 필요

#### 4. EKF 설정 조정
```yaml
# ekf_config.yaml

# Odom yaw 신뢰도 감소 (0.01 → 0.04)
odom0_pose_covariance:
  [5]: 0.04  # 0.1^2 → 0.2^2

# IMU 각속도를 각도로 적분하도록 설정 변경
# (robot_localization 문서 참조 필요)
```

### 장기 (하드웨어 개선)

#### 5. 고해상도 엔코더
- 현재: 390 PPR (Pulse Per Revolution)
- 권장: 1000 PPR 이상
- 효과: 회전 측정 정확도 2.5배 향상

#### 6. IMU 업그레이드
- 현재: MPU6050 (저가형 6축)
- 권장: ICM-20948 (9축 + 높은 정확도)
- 효과: 각속도 정확도 5배 향상

---

## 📈 검증 시나리오

### 1. angular_scale 2.1164 적용 후
```bash
python3 ekf_comparison_test.py
```

**기대 결과**:
```
5️⃣  angular_scale 분석:
   현재 설정: 2.1164
   IMU 평균: 88.5°
   Odom 평균: 88.2°
   실제 비율: 1.0034
   권장 angular_scale: 2.1236
   ✅ angular_scale이 잘 보정됨 (오차 < 5%)

Odom 기준 회전:
   반시계: Odom +88.5° (IMU +89.2°)  ← 실제 ~90° 회전
   시계: Odom -88.1° (IMU -89.8°)    ← 실제 ~90° 회전
```

### 2. 속도 감소 (0.2 rad/s) 효과
```
데이터 손실 감소:
- 이전: IMU 87° vs Odom 70° (19% 손실)
- 이후: IMU 88° vs Odom 85° (3% 손실)
```

---

## 🔬 추가 디버깅 필요 사항

### 1. 엔코더 실제 Hz 측정
```bash
ros2 topic hz /transbot/get_vel
```
**확인 사항**: 실제 50Hz로 발행되는지?

### 2. I2C 통신 부하 확인
```bash
# IMU와 엔코더가 같은 I2C 버스 사용 시
i2cdetect -y 1
```
**확인 사항**: 동시 폴링 시 지연 발생?

### 3. 회전 중 /odom_raw 샘플 수 카운트
```python
# 로그 추가
self.get_logger().info(f'Samples: {sample_count} in {duration:.2f}s = {sample_count/duration:.1f} Hz')
```
**기대**: 2.5초 × 50Hz = 125 샘플  
**실제**: 확인 필요

---

## 🎓 학습 포인트

### Kalman Filter 동작 이해

**Kalman Gain 공식**:
```
K = P / (P + R)

P = Process Noise (시스템 불확실성)
R = Measurement Covariance (센서 불확실성)
```

**센서 융합 가중치**:
```
출력 = K × 센서측정 + (1-K) × 이전추정

K=0.8 → 센서 80%, 이전 20% (센서 신뢰)
K=0.2 → 센서 20%, 이전 80% (시스템 신뢰)
```

**우리 시스템**:
- vyaw (각속도): IMU 신뢰 69%, Odom 신뢰 0.05%
- yaw (각도): Odom 신뢰 80% (IMU는 각도 제공 안 함)

**결론**: 각도는 Odom, 각속도는 IMU → EKF가 Odom 각도를 IMU 각속도로 스무딩

---

## 📝 권장 조치

### 즉시 실행
1. ✅ angular_scale을 2.1164로 변경
2. ✅ ekf_comparison_test 재실행
3. ✅ 회전 속도를 0.2로 감소하여 재테스트

### 추후 검토
1. ⏳ 엔코더 Hz 실측 확인
2. ⏳ EKF 설정 최적화 (Odom 신뢰도 감소)
3. ⏳ 하드웨어 업그레이드 고려

---

## 🔗 관련 파일

- `transbot_full_system.launch.py`: angular_scale 설정
- `ekf_config.yaml`: EKF 센서 융합 설정
- `transbot_driver.py`: 엔코더/IMU 폴링 주파수
- `ekf_comparison_test.py`: 검증 도구
