# 270도 회전 문제 해결 가이드

## 🎯 문제 요약

**물리적으로 270° 회전했지만 센서는 90°만 측정**
→ **실제 회전이 명령의 3배로 실행됨**

## 🔍 근본 원인

### 1. Transbot에는 엔코더가 없음
```python
# Transbot_Lib.py Line 451
# Control PWM pulse of motor to control speed 
# (speed measurement without encoder)
```
→ 휠 오도메트리는 **명령 기반 추정**일 뿐, 실제 측정이 아님

### 2. 속도 스케일링 문제
```python
# transbot_driver.py Line 173
base_speed = linear_x * (100.0 / 0.2)  # 0.2 m/s → 100%
turn_speed = angular_z * (100.0 / 0.5)  # 0.5 rad/s → 100%
```

**현재 설정**:
- 명령: 0.2 rad/s
- 스케일: `0.2 * (100 / 0.5) = 40%` PWM
- **실제: 40% PWM으로 0.6 rad/s 회전** (3배!)

### 3. IMU도 증폭됨
```python
# transbot_driver.py Line 298
sensitivity_gain = 2.0
msg.angular_velocity.z = float(gz) * sensitivity_gain
```
- 실제 회전: 0.6 rad/s
- IMU 원본: ~1.8 rad/s (실제 × 3)
- 증폭 후: 3.6 rad/s (실제 × 6)

## 🛠️ 해결 방법

### 방법 1: 각속도 스케일 보정 (권장) ⭐

#### A. transbot_driver.py 수정
```python
# Line 173-176 수정

# Before
base_speed = linear_x * (100.0 / 0.2)  # Scale from m/s to percentage
turn_speed = angular_z * (100.0 / 0.5)  # Scale from rad/s to percentage

# After (1/3로 감소)
base_speed = linear_x * (100.0 / 0.6)   # 0.6 m/s → 100% (실제 속도에 맞춤)
turn_speed = angular_z * (100.0 / 1.5)  # 1.5 rad/s → 100% (3배 보정)
```

#### B. IMU sensitivity_gain 수정
```python
# Line 298 수정

# Before
sensitivity_gain = 2.0

# After
sensitivity_gain = 1.0  # 또는 0.5 (테스트 필요)
```

#### C. 빌드 및 재테스트
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_bringup
source install/setup.bash

# 기존 노드 종료
pkill -f transbot_driver

# 재시작 (bringup launch에서 자동 시작)

# 테스트
./run_rotation_test.sh
```

### 방법 2: 모터 캘리브레이션 (정확함)

#### A. 캘리브레이션 스크립트 실행
```bash
cd ~/transbot_ws_ros2
python3 calibration.py
```

#### B. 지시에 따라 측정
1. 360° 회전 시간 측정
2. 실제 각속도 계산
3. 스케일 팩터 자동 계산

#### C. 결과 적용
- calibration.py가 자동으로 파라미터 계산
- transbot_driver.py에 적용

## 📋 수정할 파일

### 1. transbot_driver.py
```bash
파일: ~/transbot_ws_ros2/src/transbot_bringup/transbot_bringup/transbot_driver.py
수정 라인:
- Line 173-174: 속도 스케일링 (3배 보정)
- Line 298: sensitivity_gain (2.0 → 1.0)
```

### 2. 빌드 대상
```bash
패키지: transbot_bringup
빌드 시간: ~10초
재시작 필요: transbot_driver 노드
```

## 🧪 검증 절차

### Step 1: 수정 적용
```bash
cd ~/transbot_ws_ros2/src/transbot_bringup/transbot_bringup
nano transbot_driver.py
# 위의 수정사항 적용
```

### Step 2: 빌드
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_bringup
source install/setup.bash
```

### Step 3: 드라이버 재시작
```bash
# 기존 프로세스 확인
ps aux | grep transbot_driver

# 종료
pkill -f transbot_driver

# bringup launch가 자동으로 재시작하거나
# 수동 재시작:
ros2 launch transbot_bringup transbot_bringup.launch.py
```

### Step 4: 회전 테스트
```bash
cd ~/transbot_ws_ros2
./run_rotation_test.sh
```

### Step 5: 결과 확인
```
목표: 90° 회전
────────────────────────────────────────
물리적:        90° ± 5° ✓
odom_raw:      90° ± 5° ✓
odom_filtered: 90° ± 5° ✓
IMU angular_z: 0.2 rad/s (명령과 일치) ✓
```

## 📊 예상 결과

### Before (현재)
```
명령: 0.2 rad/s
────────────────────────────────
PWM:        40%
물리 회전:  0.6 rad/s (3배!)
IMU 측정:   3.6 rad/s (6배!)
odom_raw:   0.2 rad/s (명령 기반)
────────────────────────────────
90° 목표 → 270° 실제 회전
```

### After (수정 후)
```
명령: 0.2 rad/s
────────────────────────────────
PWM:        13.3% (1/3로 감소)
물리 회전:  0.2 rad/s ✓
IMU 측정:   0.6 rad/s (3배, gain 1.0)
odom_raw:   0.2 rad/s (명령 기반)
────────────────────────────────
90° 목표 → 90° 실제 회전 ✓
```

## 🎯 중요 포인트

### 1. 엔코더가 없음
- 휠 오도메트리는 **명령 기반 추정**
- 실제 회전과 다를 수 있음
- 바닥 슬립에도 취약

### 2. PWM과 실제 속도 불일치
- PWM 40% ≠ 최대 속도의 40%
- 모터 특성에 따라 비선형
- 캘리브레이션 필수

### 3. IMU가 유일한 실측 센서
- 각속도는 IMU만 실제 측정
- EKF가 IMU를 신뢰해야 함
- sensitivity_gain 보정 중요

### 4. 스케일 팩터의 의미
```python
# Before
turn_speed = angular_z * (100.0 / 0.5)
# 해석: 0.5 rad/s 명령 → 100% PWM

# After
turn_speed = angular_z * (100.0 / 1.5)
# 해석: 1.5 rad/s 명령 → 100% PWM
#      0.5 rad/s 명령 → 33.3% PWM (3배 감소)
```

## ⚠️ 주의사항

### 1. 안전 테스트
- 수정 후 낮은 속도로 먼저 테스트
- 로봇 동작 범위 확보
- 이상 시 즉시 정지 준비

### 2. 미세 조정 필요
- 3배가 정확하지 않을 수 있음
- 여러 번 테스트 후 조정
- 바닥 재질에 따라 다를 수 있음

### 3. 선형 속도도 확인
- 각속도만 3배일 수도
- 직진 속도도 테스트 필요
- base_speed 스케일도 조정 필요할 수 있음

## 🔧 추가 개선 사항

### 1. 실제 속도 측정 추가
```python
# 시간과 각도 변화로 실제 각속도 계산
actual_angular_vel = delta_yaw / delta_time
# odom_raw에 반영
```

### 2. 동적 스케일 보정
```python
# IMU 피드백으로 실시간 보정
speed_correction = target_vel / actual_vel
adjusted_pwm = base_pwm * speed_correction
```

### 3. 슬립 감지
```python
# odom_raw와 IMU 적분값 비교
if abs(odom_yaw - imu_integrated_yaw) > threshold:
    self.get_logger().warn("Wheel slip detected!")
```

## 📝 다음 단계

1. ✅ transbot_driver.py 수정
2. ✅ 빌드 및 재시작
3. ✅ 회전 테스트 실행
4. ⏳ 결과 분석 및 미세 조정
5. ⏳ 직진 속도도 검증
6. ⏳ EKF 튜닝 (IMU 신뢰도 상향)
