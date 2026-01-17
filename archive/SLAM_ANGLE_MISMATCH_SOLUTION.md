# 🎯 SLAM 각도 불일치 문제 해결 방안

**작성일**: 2025-10-17  
**문제**: SLAM 상 각도와 실제 로버 회전 각도 불일치

---

## 📊 근본 원인

### angular_scale=1.56의 의미

**90° 회전 테스트용 보정값**:
```
Odom_raw 측정 (144°) × 1.56 = 225° ≈ 실제 물리 회전
```

**문제**: 이 보정값이 **SLAM에도 적용**되어:
```
SLAM 입력: 보정된 오도메트리 (1.56배 증폭)
  → 지도 상 회전이 실제보다 과장됨
  → 로봇은 90° 회전했는데 SLAM은 140° 회전으로 인식
```

---

## 💡 해결 방안

### Option A: angular_scale 제거 (추천) ⭐

**base_node의 angular_scale을 1.0으로 되돌림:**

```python
# transbot_full_system.launch.py & bringup.launch.py
transbot_base_node = Node(
    parameters=[{
        'linear_scale': 1.2,
        'angular_scale': 1.0,  # 1.56 → 1.0 (보정 제거)
        'is_multi_robot': False
    }]
)
```

**이유**:
1. **SLAM은 상대적 회전만 필요**
   - 절대 각도가 아닌 **변화량**을 사용
   - 오도메트리가 일관성 있게만 측정하면 됨

2. **하드웨어 비선형성은 제어 레벨에서 보정**
   - 90° 목표 → 270° 실제 회전 문제는
   - **navigation 컨트롤러**에서 보정해야 함
   - 오도메트리는 "실제 회전량"을 정확히 측정하면 됨

3. **EKF가 IMU와 융합하여 정확도 향상**
   - `/odom_raw`의 오차를 IMU가 보정
   - angular_scale을 임의로 증폭하면 융합 왜곡

### Option B: Navigation 레벨에서 보정

**DWB Controller에 angular_scale 적용:**

```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # ... 기존 설정 ...
      angular_scale: 0.64  # 1/1.56 = 0.64 (역보정)
```

**효과**:
- SLAM은 정상 동작 (보정 없는 오도메트리 사용)
- Navigation 명령 시 각속도를 0.64배로 축소
- 90° 목표 → 144° 명령 → 90° 실제 회전

---

## 🧪 검증 방법

### 1단계: angular_scale = 1.0으로 복원

```bash
# 수정 필요한 파일:
# 1. /home/user/transbot_ws_ros2/src/transbot_bringup/launch/bringup.launch.py
# 2. /home/user/transbot_ws_ros2/src/sllidar_ros2/launch/transbot_full_system.launch.py

# 빌드
cd /home/user/transbot_ws_ros2
colcon build --packages-select transbot_bringup sllidar_ros2

# 시스템 재시작
sudo systemctl restart yahboomcar_bringup
```

### 2단계: SLAM 테스트

```bash
# RViz 실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py use_rviz:=true

# 수동으로 로봇 회전 (teleop 또는 리모컨)
# - 실제: 90° 회전
# - SLAM 맵: 90° 회전으로 표시되는지 확인
```

### 3단계: 회전 테스트

```bash
# 90° 회전 명령
./run_rotation_test.sh

# 예상 결과:
# - Odom_raw 측정: ~58° (angular_scale=1.0이므로 과소 측정)
# - 실제 회전: ~90° (하드웨어 비선형성)
# - SLAM: ~90° (EKF가 IMU 융합하여 보정)
```

---

## 🎓 핵심 교훈

### 1. 오도메트리 보정의 목적

**잘못된 접근**:
```
"90° 목표 → 270° 실제 회전" 문제를
오도메트리 레벨에서 angular_scale로 해결
```

**올바른 접근**:
```
오도메트리: "실제 회전량"을 정확히 측정
Navigation: "명령 각도"를 보정하여 원하는 회전 달성
```

### 2. SLAM vs Navigation의 역할

| 구성 요소 | 역할 | 보정 필요성 |
|-----------|------|-------------|
| **오도메트리** | 실제 이동량 측정 | 센서 캘리브레이션만 필요 |
| **SLAM** | 지도 생성 및 위치 추정 | 일관성만 중요 (절대값 불필요) |
| **Navigation** | 목표 지점으로 이동 | 제어 보정 필요 (PID, feedforward) |

### 3. 다층 보정의 분리

```
센서 레벨: 물리적 캘리브레이션 (IMU, 휠 인코더)
오도메트리 레벨: 측정 정확도 향상 (센서 융합)
제어 레벨: 명령 실행 보정 (하드웨어 비선형성)
```

---

## 📋 조치 사항

### 즉시 조치
- [ ] `angular_scale = 1.0`으로 복원 (두 launch 파일)
- [ ] 빌드 및 시스템 재시작
- [ ] SLAM 테스트로 각도 일치 확인

### 향후 조치
- [ ] Navigation 컨트롤러에 angular 보정 추가
- [ ] PID 튜닝으로 회전 정확도 향상
- [ ] Feedforward 모델로 하드웨어 비선형성 보정

### IMU 재캘리브레이션 (선택)
- [ ] IMU 캘리브레이션 날짜 확인
- [ ] 6개월 이상 지났다면 재캘리브레이션
- [ ] 새 `imu_calib.yaml` 적용

---

## 🚀 예상 결과

### angular_scale = 1.0 적용 후

**SLAM 동작**:
```
실제 회전: 90°
Odom_raw: ~58° (과소 측정이지만 일관성 있음)
EKF 융합: IMU와 결합하여 ~90° 추정
SLAM: 지도 상 ~90° 회전 ✅
```

**회전 테스트**:
```
90° 명령 → 270° 실제 회전 (여전히 문제)
  → Navigation 레벨에서 해결 필요
```

---

**결론**: `angular_scale=1.56`은 **테스트 전용 보정**이며,  
실제 SLAM 운영 시에는 **1.0으로 복원**해야 합니다!
