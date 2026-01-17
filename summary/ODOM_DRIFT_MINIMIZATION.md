# Odom 드리프트 최소화 가이드

> **작성일:** 2025-10-31  
> **목적:** map→odom→base_footprint TF 체인에서 odom 드리프트 최소화

---

## 🎯 적용된 개선 사항

### 1️⃣ EKF 설정 최적화

#### A. IMU 신뢰도 강화
```yaml
# ekf_config.yaml
imu0_angular_velocity_covariance: [0.000009, 0.0, 0.0,  # 0.003^2 (더 엄격)
                                   0.0, 0.000009, 0.0,
                                   0.0, 0.0, 0.000009]
```
**효과:** 회전 드리프트 최소화, IMU 각속도를 더 신뢰

#### B. Process Noise 감소
```yaml
# ekf_config.yaml
process_noise_covariance:
  - x, y 위치: 0.03 (0.05 → 0.03)
  - yaw 각도: 0.04 (0.06 → 0.04)
  - vyaw 각속도: 0.00002 (0.00005 → 0.00002) ⭐⭐⭐
  - vx, vy 속도: 0.02 (0.025 → 0.02)
```
**효과:** 드리프트 누적 억제, 센서 측정값에 더 의존

#### C. EKF 주파수 증가
```yaml
# ekf_config.yaml
frequency: 50.0  # 30 → 50Hz
```
**효과:** 더 빠른 업데이트로 드리프트 누적 시간 감소

---

### 2️⃣ SLAM 업데이트 빈도 증가

```yaml
# slam_params.yaml
transform_publish_period: 0.01    # 0.02 → 0.01 (100Hz)
map_update_interval: 0.5          # 1.0 → 0.5초
minimum_time_interval: 0.05       # 0.1 → 0.05초
```
**효과:** SLAM이 map→odom TF를 더 자주 보정

---

### 3️⃣ 스캔 매칭 민감도 증가

```yaml
# slam_params.yaml
minimum_travel_distance: 0.05     # 0.10 → 0.05m (5cm)
minimum_travel_heading: 0.03      # 0.05 → 0.03 rad (~1.7도)
scan_buffer_size: 25              # 20 → 25
```
**효과:** 더 자주, 더 정밀하게 위치 보정

---

## 📊 예상 효과

| 항목 | Before | After | 개선 |
|------|--------|-------|------|
| EKF 업데이트 | 30Hz | 50Hz | +67% |
| SLAM TF 발행 | 50Hz | 100Hz | +100% |
| 맵 업데이트 | 1.0초 | 0.5초 | +100% |
| 스캔 매칭 빈도 | 10cm/2.86° | 5cm/1.7° | +100% |
| Process Noise (vyaw) | 0.00005 | 0.00002 | -60% |

**종합 효과:**
- ✅ 드리프트 누적 속도 **~50% 감소** 예상
- ✅ SLAM 보정 빈도 **2배 증가**
- ✅ 회전 정확도 향상 (IMU 신뢰도 증가)

---

## 🔧 추가 확인 사항

### 1. Wheel Odometry 캘리브레이션

드리프트의 **근본 원인**은 휠 오도메트리 오차일 수 있습니다.

#### 확인 방법:
```bash
# 1. 직선 주행 테스트 (5m)
# odom이 5m 보고하는지 확인
ros2 topic echo /odom_raw | grep "position:"

# 2. 회전 테스트 (360도)
# odom이 정확히 2π rad 보고하는지 확인
ros2 topic echo /odom_raw | grep "orientation:"
```

#### 캘리브레이션 파일 확인:
```bash
# transbot_base 패키지의 파라미터 확인
find ~/transbot_ws_ros2/src -name "*param*" -o -name "*config*" | grep transbot_base
```

**주요 파라미터:**
- `wheel_radius`: 바퀴 반지름 (mm)
- `wheel_base`: 바퀴 간격 (mm)
- `angular_scale`: 회전 보정 계수

### 2. IMU 캘리브레이션 확인

```bash
# IMU 바이어스 확인
ros2 topic echo /imu/data_calibrated | grep "angular_velocity:"
```

**정상 범위 (정지 상태):**
- x, y, z < 0.01 rad/s

**바이어스가 크면:**
```bash
# IMU 재캘리브레이션
ros2 run transbot_base imu_calibration
```

### 3. TF 체인 모니터링

```bash
# 실시간 TF 확인
ros2 run tf2_tools view_frames

# odom 드리프트 시각화
rviz2
# Add → TF → 프레임 표시
# map, odom, base_footprint 모두 체크
```

**정상:**
- odom이 base_footprint를 잘 추적
- map이 odom을 부드럽게 보정

**비정상:**
- odom이 base_footprint에서 급격히 이탈
- map→odom 보정이 큰 점프 발생

---

## 🚀 적용 방법

### 1단계: 변경 사항 확인
```bash
cd ~/transbot_ws_ros2/src/transbot_nav/config
git diff ekf_config.yaml slam_params.yaml
```

### 2단계: 재빌드 (설정 파일은 symlink로 자동 반영)
```bash
cd ~/transbot_ws_ros2
# symlink-install 사용 시 재빌드 불필요
source install/setup.bash
```

### 3단계: 시스템 재시작
```bash
# 기존 프로세스 종료 (Ctrl+C)
# 새로 실행
ros2 launch transbot_nav transbot_full_system.launch.py
```

### 4단계: 드리프트 테스트

**테스트 A: 제자리 회전**
```bash
# 360도 회전 후 시작점 복귀
# odom 위치 변화 확인
ros2 topic echo /odom/pose/pose/position
```
**기대:** x, y < 5cm 변화

**테스트 B: 직선 왕복**
```bash
# 5m 전진 → 후진 5m
# odom 위치 변화 확인
```
**기대:** 최종 위치 < 10cm 오차

**테스트 C: 사각형 주행**
```bash
# 2m × 2m 사각형 주행 후 복귀
# odom 누적 오차 확인
```
**기대:** 최종 위치 < 15cm 오차

---

## 📈 성능 모니터링

### 1. EKF 성능 확인
```bash
# EKF 출력 확인
ros2 topic hz /odometry/filtered  # 50Hz 확인
ros2 topic echo /odometry/filtered | grep "covariance:"
```

**좋은 covariance:**
- x, y < 0.1
- yaw < 0.05

### 2. SLAM 업데이트 확인
```bash
# SLAM TF 발행 빈도
ros2 topic hz /tf  # 100Hz 확인

# 맵 업데이트 확인
ros2 topic echo /map --once
```

### 3. CPU 사용률 확인
```bash
top -p $(pgrep -f ekf_filter_node)
top -p $(pgrep -f slam_toolbox)
```

**정상 범위:**
- EKF: 5-10% CPU
- SLAM: 15-25% CPU

**높으면:** 주파수 다시 낮추기
```yaml
frequency: 40.0  # 50 → 40Hz
transform_publish_period: 0.02  # 0.01 → 0.02
```

---

## ⚠️ 주의사항

### 1. 과도한 최적화 방지

**증상:**
- CPU 100% 사용
- TF 지연 발생
- SLAM 노드 크래시

**해결:**
```yaml
# ekf_config.yaml
frequency: 40.0  # 50 대신

# slam_params.yaml
transform_publish_period: 0.02  # 0.01 대신
minimum_travel_distance: 0.08  # 0.05 대신
```

### 2. 센서 데이터 품질 우선

**EKF/SLAM 튜닝보다 중요:**
1. IMU 캘리브레이션
2. Wheel Odometry 정확도
3. LiDAR 스캔 품질

**확인:**
```bash
# 센서 데이터 주파수
ros2 topic hz /imu/data_calibrated  # ~100Hz
ros2 topic hz /odom_raw             # ~50Hz
ros2 topic hz /scan                 # ~10Hz
```

### 3. 점진적 튜닝

**권장 순서:**
1. EKF frequency: 30 → 40 → 50Hz (단계적)
2. Process noise 감소: 50% → 60% → 70%
3. SLAM 업데이트: 1초 → 0.7초 → 0.5초

**각 단계마다 테스트:**
- 드리프트 감소 확인
- CPU 사용률 확인
- 시스템 안정성 확인

---

## 🎓 드리프트 원인별 해결책

| 원인 | 증상 | 해결책 |
|------|------|--------|
| Wheel slip | 직선 주행 시 xy 드리프트 | 바닥 마찰 개선, 속도 제한 |
| Wheel 캘리브레이션 | 회전 시 각도 오차 | angular_scale 재측정 |
| IMU 바이어스 | 정지 시 회전 드리프트 | IMU 재캘리브레이션 |
| EKF 설정 | 느린 드리프트 누적 | Process noise 감소 ⭐ |
| SLAM 업데이트 느림 | 긴 주행 후 큰 오차 | 업데이트 빈도 증가 ⭐ |

---

## 📚 참고 문서

- [EKF 센서 퓨전 가이드](02_EKF_SENSOR_FUSION.md)
- [회전 정확도 개선](03_ROTATION_ACCURACY.md)
- [SLAM 최적화](04_SLAM_OPTIMIZATION.md)
- [robot_localization 문서](http://docs.ros.org/en/humble/p/robot_localization/)

---

**작성:** GitHub Copilot  
**날짜:** 2025-10-31  
**버전:** 1.0
