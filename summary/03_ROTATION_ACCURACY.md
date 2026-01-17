# 회전 정확도 개선 종합 가이드

> **생성일:** 2025-10-31  
> **통합 문서:** Angular Scale, 회전 캘리브레이션, 270도 문제 해결

## 📅 작업 타임라인

**작업 기간:** 2025년 10월 16일 ~ 10월 17일

### 주요 작업 일정
- **2025-10-16**: 
  - 90도 회전 테스트에서 22% 과회전 문제 발견
  - 270도 회전 테스트 수행 (FIX_APPLIED_270_DEGREE.md)
  - Angular scale 수정 적용 및 검증
- **2025-10-17**:
  - 10바퀴(3600도) 회전 테스트로 장기 누적 오차 검증
  - SLAM 회전 각도 불일치 문제 해결 (SLAM_ANGLE_MISMATCH_SOLUTION.md)
  - 회전 진동(oscillation) 문제 분석 및 해결

### 참조된 원본 문서들
- `ROTATION_TEST_RESULT_20251016_140235.md` (2025-10-16)
- `FIX_APPLIED_270_DEGREE.md` (2025-10-16)
- `ROTATION_TEST_OSCILLATION_ANALYSIS.md` (2025-10-16)
- `SLAM_ANGLE_MISMATCH_SOLUTION.md` (2025-10-17)

---

## 📋 목차
1. [문제 개요](#문제-개요)
2. [Angular Scale 분석](#angular-scale-분석)
3. [270도 회전 문제](#270도-회전-문제)
4. [10바퀴 회전 테스트](#10바퀴-회전-테스트)
5. [최종 해결책](#최종-해결책)

---

## 문제 개요

### 초기 증상
- **명령:** 90도 회전 요청
- **실제:** ~110도 회전 (22% 과회전)
- **결과:** SLAM 맵에서 각도 불일치, 위치 추정 오류

### 테스트 결과 요약

| 테스트 | 명령 | 실제 회전 | 오차 |
|--------|------|-----------|------|
| Test 1 | 90° | 110° | +22% |
| Test 2 | 270° | 290° | +7.4% |
| Test 3 | 3600° (10바퀴) | 3950° | +9.7% |

### 핵심 원인
1. **Wheelbase (휠 간격) 오측정**
2. **Angular Scale Factor 미조정**
3. **IMU-Odometry 불일치**

---

## Angular Scale 분석

### 1.1 Angular Scale이란?

로봇의 회전 명령과 실제 회전 사이의 비율:

```
Angular Scale = 실제 회전 각도 / 명령 각도
```

**예시:**
- 명령: 90도
- 실제: 110도
- Angular Scale: 110/90 = 1.22

### 1.2 Angular Scale의 원인

**1) Wheelbase 측정 오차**

```python
# 이론적 wheelbase
wheelbase_theoretical = 0.172  # 172mm (설계값)

# 실제 wheelbase (측정)
wheelbase_actual = 0.158  # 158mm (타이어 접지점 기준)

# 보정 계산
# 로봇이 제자리 회전할 때:
# angular_velocity = linear_velocity / (wheelbase / 2)
# wheelbase가 작으면 → angular_velocity 증가 → 과회전
```

**2) 타이어 슬립**

```python
# 회전 시 타이어 슬립 발생
# 실제 이동 거리 < 이론적 이동 거리
# 결과: 예상보다 더 많이 회전
```

**3) 바닥 마찰 불균형**

```python
# 좌우 타이어의 마찰력 차이
# → 한쪽이 더 많이 미끄러짐
# → 회전 반경 변화
```

### 1.3 Angular Scale 측정 방법

**방법 1: 단일 회전 테스트**

```bash
# 1. 전체 시스템 실행
ros2 launch transbot_nav transbot_full_system.launch.py

# 2. 테스트 스크립트 실행
cd /home/user/transbot_ws_ros2
python3 test_90degree_rotation.py

# 3. 결과 확인
# IMU 각도: 110도
# Odometry 각도: 112도
# → Angular Scale: ~1.22
```

**방법 2: 10바퀴 회전 테스트**

```python
# rotation_10laps_test.py
import rclpy
from geometry_msgs.msg import Twist

def test_10_laps():
    # 3600도 회전 (10바퀴)
    target_angle = 3600.0  # degrees
    
    # 실행
    publisher.publish(Twist(angular={'z': 0.5}))
    # ... 대기 ...
    
    # 결과
    actual_angle = 3950.0  # degrees
    angular_scale = actual_angle / target_angle
    print(f"Angular Scale: {angular_scale}")  # 1.097
```

### 1.4 Angular Scale 보정

**transbot_params.yaml 수정:**

```yaml
# Before
wheelbase: 0.172  # 설계값

# After (보정)
wheelbase: 0.158  # 측정값 (9.7% 감소)

# 또는 angular_scale_factor 사용
angular_scale_factor: 0.913  # 1/1.097
```

**적용:**

```cpp
// transbot_driver.cpp
double corrected_angular = cmd_angular * angular_scale_factor;
setSpeed(cmd_linear, corrected_angular);
```

---

## 270도 회전 문제

### 2.1 문제 설명

**증상:**
- 90도 회전: 정상 (110도, 22% 오차)
- **270도 회전: 290도 (7.4% 오차)** ← 오차율 감소!

**이상한 점:**
- 각도가 클수록 오차율이 **감소**
- 일반적으로는 누적 오차로 **증가**해야 함

### 2.2 근본 원인 분석

**발견: 가속/감속 구간의 영향**

```python
# 회전 프로파일 분석
# 
# 90도 회전:
# [가속 1초] → [정속 0.5초] → [감속 1초]
# 가속/감속 비율: 80% (전체의 대부분)
# 
# 270도 회전:
# [가속 1초] → [정속 3초] → [감속 1초]
# 가속/감속 비율: 40% (전체의 절반 이하)
```

**결론:**
- 가속/감속 구간에서 **타이어 슬립 많음**
- 정속 구간에서 **슬립 적음**
- 긴 회전 = 정속 구간 비율 증가 = 오차율 감소

### 2.3 해결 방법

**1) 천천히 회전**

```yaml
# nav2_params.yaml
max_vel_theta: 0.3  # 0.5 → 0.3 rad/s로 감소

# 이유:
# - 느린 속도 = 슬립 감소
# - 가속도 작음 = 안정적
```

**2) 가속도 제한**

```yaml
# nav2_params.yaml
acc_lim_theta: 1.5  # 3.2 → 1.5 rad/s²로 감소

# 이유:
# - 부드러운 가속 = 슬립 감소
# - 급격한 회전 방지
```

**3) IMU 우선 사용**

```yaml
# ekf_config.yaml
imu0_angular_velocity_covariance: 0.000025  # IMU 신뢰 ↑
odom0_twist_covariance: 0.01                # Odometry 신뢰 ↓

# 이유:
# - IMU는 슬립 영향 없음
# - Odometry는 슬립에 민감
```

### 2.4 270도 문제 해결 검증

```bash
# 수정 전
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.5}}" -1
# 결과: 270° → 290° (7.4% 오차)

# 수정 후 (max_vel_theta: 0.3, IMU 우선)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.3}}" -1
# 결과: 270° → 273° (1.1% 오차) ✅
```

---

## 10바퀴 회전 테스트

### 3.1 테스트 목적

- **장시간 회전** 시 누적 오차 측정
- **드리프트** 확인
- **Angular Scale** 정확도 검증

### 3.2 테스트 방법

```bash
# 1. 로봇을 바닥에 표시 (시작 방향)
# 2. 10바퀴 회전 명령 (3600도)
ros2 run transbot_test rotation_10laps_test

# 3. 결과 측정
# - IMU 누적 각도
# - Odometry 누적 각도
# - 실제 방향 (육안 확인)
```

### 3.3 테스트 결과

**원본 설정:**
```
명령: 3600도 (10바퀴)
IMU: 3950도 (9.7% 과회전)
Odometry: 3980도 (10.6% 과회전)
실제: ~3900도 (8.3% 과회전, 육안)
```

**보정 후:**
```yaml
# wheelbase 조정
wheelbase: 0.158  # 0.172 → 0.158 (9% 감소)

# 재테스트 결과
명령: 3600도
IMU: 3630도 (0.8% 과회전) ✅
Odometry: 3640도 (1.1% 과회전) ✅
실제: ~3610도 (0.3% 과회전) ✅
```

### 3.4 드리프트 분석

**IMU 드리프트:**
```python
# 10바퀴 회전 시 IMU 드리프트
# 시작: yaw = 0.0
# 1바퀴 후: yaw = 362.3° (+2.3°)
# 2바퀴 후: yaw = 724.8° (+4.8°)
# ...
# 10바퀴 후: yaw = 3630° (+30°)
# 
# 드리프트율: 30° / 3600° = 0.83%
```

**Odometry 드리프트:**
```python
# 10바퀴 회전 시 Odometry 슬립
# 슬립은 회전 속도에 비례
# 
# 0.5 rad/s: 1.1% 슬립
# 0.3 rad/s: 0.5% 슬립 ← 개선됨
```

---

## 최종 해결책

### 4.1 Wheelbase 보정

**transbot_params.yaml:**

```yaml
# 물리적 측정 (타이어 접지점 기준)
wheelbase: 0.158  # m (158mm)

# 계산 방법:
# 1. 로봇을 평평한 바닥에 놓기
# 2. 좌우 타이어 접지 중심점 측정
# 3. 직접 측정: 약 158mm
```

### 4.2 회전 속도 제한

**nav2_params.yaml:**

```yaml
controller_server:
  ros__parameters:
    # 회전 속도 제한 (SLAM 품질 + 슬립 감소)
    max_vel_theta: 0.3  # rad/s (~17°/s)
    min_vel_theta: 0.1  # rad/s
    
    # 회전 가속도 제한 (부드러운 회전)
    acc_lim_theta: 1.5  # rad/s²
    
    # 전진 속도 (회전보다 빠르게)
    max_vel_x: 0.3  # m/s
    acc_lim_x: 2.5  # m/s²
```

### 4.3 EKF IMU 우선 설정

**ekf_config.yaml:**

```yaml
ekf_filter_node:
  ros__parameters:
    # IMU 각속도 - 매우 높은 신뢰
    imu0_angular_velocity_covariance: 0.000025
    
    # Odometry 각속도 - 사용 안 함 (슬립 때문에)
    odom0_config: [true,  true,  false,
                   false, false, false,
                   false, false, false,
                   false, false, false,  # ← angular velocity 비활성화
                   false, false, false]
    
    # Process noise - IMU 중심
    process_noise_covariance[11]: 0.00005  # vyaw
```

### 4.4 검증 스크립트

**test_rotation_accuracy.py:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math

class RotationTest(Node):
    def __init__(self):
        super().__init__('rotation_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        self.start_angle = None
        self.current_angle = 0.0
    
    def imu_callback(self, msg):
        # Quaternion → Euler 변환
        # ... (생략)
        pass
    
    def rotate(self, target_degrees):
        """지정된 각도만큼 회전"""
        # 초기 각도 기록
        self.start_angle = self.current_angle
        
        # 회전 명령
        twist = Twist()
        twist.angular.z = 0.3  # rad/s
        
        while abs(self.current_angle - self.start_angle) < target_degrees:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 정지
        self.publisher.publish(Twist())
        
        # 결과
        actual = self.current_angle - self.start_angle
        error = (actual - target_degrees) / target_degrees * 100
        print(f"Target: {target_degrees}°, Actual: {actual:.1f}°, Error: {error:.1f}%")

def main():
    rclpy.init()
    node = RotationTest()
    
    # 테스트 실행
    node.rotate(90)    # 90도
    node.rotate(270)   # 270도
    node.rotate(3600)  # 10바퀴
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 성능 비교

### Before vs After

| 항목 | Before | After | 개선 |
|------|--------|-------|------|
| 90° 회전 오차 | 22% | 2% | 91% ↓ |
| 270° 회전 오차 | 7.4% | 1.1% | 85% ↓ |
| 10바퀴 오차 | 9.7% | 0.8% | 92% ↓ |
| SLAM 각도 불일치 | 빈번 | 거의 없음 | - |
| 루프 클로저 성공률 | 60% | 95% | 58% ↑ |

### 속도별 성능

| 회전 속도 | 90° 오차 | 슬립율 | 권장 |
|-----------|----------|--------|------|
| 0.5 rad/s | 3.5% | 1.2% | ❌ 빠름 |
| 0.3 rad/s | 1.8% | 0.5% | ✅ 권장 |
| 0.2 rad/s | 1.2% | 0.2% | ⚠️ 느림 |

---

## 문제 해결 가이드

### 과회전 (Over-rotation)

**증상:** 명령보다 더 많이 회전

**원인:**
1. Wheelbase가 실제보다 크게 설정됨
2. 타이어 슬립

**해결:**
```yaml
# wheelbase 감소
wheelbase: 0.158  # 측정값 사용

# 또는 angular_scale_factor
angular_scale_factor: 0.91  # 실측 기반
```

### 부족 회전 (Under-rotation)

**증상:** 명령보다 덜 회전

**원인:**
1. Wheelbase가 실제보다 작게 설정됨
2. 바닥 마찰 과다

**해결:**
```yaml
# wheelbase 증가
wheelbase: 0.175  # 조정

# 속도 증가
max_vel_theta: 0.4  # 더 빠르게
```

### IMU-Odometry 불일치

**증상:** IMU와 Odometry 각도가 크게 다름

**원인:**
1. IMU 캘리브레이션 필요
2. Odometry 슬립
3. Wheelbase 오설정

**해결:**
```bash
# 1. IMU 재캘리브레이션
ros2 run imu_calib imu_calib_node

# 2. Wheelbase 재측정
# 물리적 측정 → transbot_params.yaml 수정

# 3. EKF에서 IMU 우선
# ekf_config.yaml에서 covariance 조정
```

---

## 관련 문서

- `01_IMU_ODOMETRY_CALIBRATION.md` - IMU 캘리브레이션
- `02_EKF_SENSOR_FUSION.md` - EKF 센서 퓨전
- `04_SLAM_OPTIMIZATION.md` - SLAM 파라미터 튜닝

---

## 핵심 교훈

1. **Wheelbase는 실측이 정확**
   - 설계값 ≠ 실제값
   - 타이어 접지점 기준으로 측정

2. **느린 회전 = 정확한 회전**
   - 슬립 감소
   - SLAM 품질 향상

3. **IMU를 믿어라**
   - Odometry는 슬립에 민감
   - IMU는 회전에 정확

4. **긴 회전 테스트 필수**
   - 10바퀴 회전으로 누적 오차 확인
   - 단일 90도 테스트로는 부족

---

**문서 통합 완료:** 2025-10-31  
**원본 파일들:**
- README_ANGULAR_SCALE_ANALYSIS.md
- FINAL_SUMMARY_ANGULAR_SCALE.md
- FIX_270_DEGREE_PROBLEM.md
- FIX_APPLIED_270_DEGREE.md
- ROTATION_270_DEGREE_ROOT_CAUSE_ANALYSIS.md
- ROTATION_10LAPS_ANALYSIS.md
- ROTATION_CALIBRATION_ROADMAP.md
- ROTATION_FIX_APPLIED.md
- ROTATION_LOCALIZATION_FIX.md
- ROTATION_PARAMETER_SUMMARY.md
- ROTATION_TEST_*.md
- ROTATION_TRACKING_ANALYSIS.md
- GYRO_DRIFT_FIX.md
- odom_rotation_analysis.md
