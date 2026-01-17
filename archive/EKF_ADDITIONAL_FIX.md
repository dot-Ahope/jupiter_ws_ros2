# 🔧 EKF 추가 수정 필요 사항

## 📊 테스트 결과 분석

### **개선 현황:**
```
이전: EKF 범위 92.04° (96.08° ~ 4.04°)
현재: EKF 범위 36.83° (50.54° ~ 13.71°)
개선율: 60% ✓

하지만 여전히 목표(< 10°) 미달
```

---

## 🔍 핵심 문제: Odom 비대칭

### **발견된 패턴:**
```
IMU 기준 회전 테스트:
  반시계: IMU 87.83° → Odom 41.54° (47% 전달)
  시계:   IMU 89.32° → Odom 72.96° (82% 전달)

차이: 31.42° (Odom이 방향에 따라 다르게 측정)
```

### **원인 추정:**
1. **모터 비대칭**: 좌우 모터 출력 불균형
2. **엔코더 비대칭**: 좌우 엔코더 해상도/노이즈 차이
3. **로봇 기구학**: 무게 중심 편향으로 인한 슬립 비대칭
4. **Launch scale의 한계**: 단일 scale로 양방향 보정 불가능

---

## 🎯 해결 방안

### **방안 1: 방향별 angular_scale 분리**

#### **현재 시스템:**
```python
# transbot_driver에서 단일 scale 사용
angular_scale = 1.5618  # 모든 방향에 동일 적용
```

#### **제안 시스템:**
```python
# 방향별 다른 scale 적용
angular_scale_ccw = 1.5618 * (90.0 / 41.54)  # 반시계: 3.38
angular_scale_cw = 1.5618 * (90.0 / 72.96)   # 시계: 1.93

# 회전 방향에 따라 선택
if angular_velocity > 0:  # 반시계
    scale = angular_scale_ccw
else:  # 시계
    scale = angular_scale_cw
```

**장점:**
- ✅ 물리적 비대칭 직접 보정
- ✅ EKF 입력 데이터 일관성 확보

**단점:**
- ❌ 코드 수정 필요 (transbot_driver)
- ❌ 작은 회전에서 방향 판단 어려움

---

### **방안 2: EKF 설정 더 완화** (임시 방편)

#### **현재 설정:**
```yaml
odom0_twist_rejection_threshold: 10.0
imu0_twist_rejection_threshold: 15.0
process_noise_covariance[11] (yaw_vel): 0.01
```

#### **추가 완화:**
```yaml
odom0_twist_rejection_threshold: 20.0   # 10.0 → 20.0
imu0_twist_rejection_threshold: 25.0    # 15.0 → 25.0
process_noise_covariance[11]: 0.02      # 0.01 → 0.02
```

**효과:**
- Odom의 31.42° 비대칭을 threshold 내로 수용
- EKF가 더 많은 데이터로 융합 시도

**Trade-off:**
- 이상치 수용 위험 증가
- 센서 품질이 낮아져도 거부하지 않음

---

### **방안 3: IMU 가중치 증가** (가장 안전)

#### **현재 전략:**
```
EKF가 Odom과 IMU를 균등하게 융합
→ Odom 비대칭이 EKF 결과에 반영됨
```

#### **새로운 전략:**
```yaml
# IMU를 더 신뢰 (Odom 노이즈 증가)
odom0_twist_covariance: [0.1]  # 큰 값 = 낮은 신뢰도
imu0_twist_covariance: [0.001] # 작은 값 = 높은 신뢰도
```

**근거:**
- IMU 비대칭: 1.49° (excellent)
- Odom 비대칭: 31.42° (poor)
- **IMU가 더 일관적 → 더 신뢰해야**

**설정 위치:**
```yaml
# ekf_config.yaml에 추가
odom0_twist_covariance: [0.1, 0.1, 0.1]  # x, y, yaw 속도
imu0_angular_velocity_covariance: [0.001, 0.001, 0.001]
```

---

## 📋 권장 조치 순서

### **1단계: IMU 가중치 증가** (즉시 시도)
```yaml
# ekf_config.yaml에 추가
odom0_twist_covariance: [0.1, 0.1, 0.1]
imu0_angular_velocity_covariance: [0.001, 0.001, 0.001]
```

**예상 효과:**
- EKF가 IMU에 90% 가중치
- Odom 비대칭 영향 최소화
- EKF 범위 36.83° → **< 10°** 예상

---

### **2단계: Threshold 추가 완화** (1단계 실패 시)
```yaml
odom0_twist_rejection_threshold: 20.0
imu0_twist_rejection_threshold: 25.0
process_noise_covariance[11]: 0.02
```

---

### **3단계: 방향별 Scale 구현** (근본 해결)
```python
# transbot_driver 수정
def apply_angular_scale(self, angular_vel):
    if angular_vel > 0:
        return angular_vel * self.scale_ccw
    else:
        return angular_vel * self.scale_cw
```

---

## 🎯 즉시 시도: 센서 공분산 추가

### **수정할 파일:**
`/home/user/transbot_ws_ros2/src/sllidar_ros2/config/ekf_config.yaml`

### **추가할 내용:**
```yaml
# Line 30 이후에 추가 (odom0_queue_size 다음)
odom0_twist_covariance: [0.1, 0.1, 0.1]  # vx, vy, vyaw - 낮은 신뢰도

# Line 51 이후에 추가 (imu0_queue_size 다음)  
imu0_angular_velocity_covariance: [0.001, 0.001, 0.001]  # 높은 신뢰도
```

**원리:**
- Covariance ↑ = 센서 신뢰도 ↓
- Odom 공분산 100배 증가 (0.001 → 0.1)
- EKF가 IMU를 주로 사용, Odom은 보조

---

## ⚠️ 주의사항

### **IMU 과신뢰 위험:**
- IMU는 90° 이상에서 과측정 (2.4배 at 360°)
- 현재 테스트는 90° 이내이므로 안전
- 큰 회전 시 IMU 드리프트 가능성

### **검증 방법:**
1. 설정 수정 후 `ekf_comparison_test.py` 재실행
2. EKF 범위 < 10° 확인
3. 실제 SLAM 동작 테스트 (회전 후 정지 시 드리프트 확인)

---

## 🚀 다음 단계

### **즉시 실행:**
```bash
# 1. 센서 공분산 추가 (아래 명령으로 수정)
nano ~/transbot_ws_ros2/src/sllidar_ros2/config/ekf_config.yaml

# 2. 빌드
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2
source install/setup.bash

# 3. 재부팅
sudo reboot

# 4. 테스트
python3 ekf_comparison_test.py
```

### **예상 결과:**
```
현재: EKF 50.54° vs 13.71° (36.83° 차이)
목표: EKF 85-90° vs 85-90° (< 5° 차이)
```
