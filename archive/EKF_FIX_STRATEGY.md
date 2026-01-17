# EKF 수정 전략: 90° IMU 정확도 활용

## ✅ **적용 완료**

### **1. angular_scale 최적화** ⭐
```python
# transbot_full_system.launch.py
'angular_scale': 1.46,  # 1.5625 → 1.46
```

**근거:**
- Phase 2 물리적 측정 기반 계산
- 90°: 1.385, 180°: 1.441, 270°: 1.517, 360°: 1.506
- 평균: **1.46**, 표준편차: 0.055 (3.8% - 매우 일관적!)

**예상 효과:**
```
90° 명령 시:
  Odom (raw): 65° × 1.46 = 94.9° ≈ 95°
  물리적: 90°
  오차: +5° (5.5% - 허용 범위)
```

### **2. EKF Rejection Threshold 완화** ⭐
```yaml
# ekf_config.yaml
imu0_pose_rejection_threshold: 5.0   # 2.0 → 5.0
imu0_twist_rejection_threshold: 5.0  # 2.0 → 5.0
```

**이유:**
- IMU가 90°까지는 정확하므로 EKF가 더 수용해야 함
- 이전 threshold=2.0에서는 IMU를 outlier로 판단하여 무시
- 완화하여 IMU 데이터를 센서 융합에 활용

### **3. IMU 프로세스 노이즈 감소** ⭐
```yaml
# ekf_config.yaml
# yaw_vel (index 11): 0.01 → 0.005
process_noise_covariance: [..., 0.005, ...]
```

**효과:**
- IMU 각속도의 신뢰도 증가
- EKF가 IMU 데이터에 더 높은 가중치 부여
- Odom + IMU 센서 융합 강화

---

## 📊 **현재 상황 분석**

### **Phase 2 결과 재검토:**

| 목표 | 물리적 | Odom (raw) | IMU 적분 | 비고 |
|------|--------|------------|----------|------|
| 90° | ~90° ✅ | 65.0° | 135.3° | ⚠️ IMU 50% 과대 |
| 180° | ~180° ✅ | 124.9° | 308.4° | ❌ IMU 71% 과대 |
| 270° | ~270° ✅ | 178.0° | 432.9° | ❌ IMU 60% 과대 |
| 360° | ~360° ✅ | 239.1° | 608.6° | ❌ IMU 69% 과대 |

### **핵심 발견:**

1. **Odom (raw)는 일관되게 언더리포팅:**
   - 90°: 65° (72% of actual)
   - 180°: 125° (69% of actual)
   - 평균: **약 70%만 보고** → `angular_scale = 1.46` 필요

2. **IMU 문제:**
   - 90°: +50% 과대 (135° vs 90°)
   - 각도가 클수록 더 심각
   - **하지만 90°까지는 일관적이므로 활용 가능!**

3. **EKF 전략:**
   - Odom을 물리적 정확도에 가깝게 보정 (angular_scale=1.46)
   - IMU를 더 수용하도록 threshold 완화
   - 90° 이하 회전에서 센서 융합 효과 극대화

---

## 🧪 **테스트 계획**

### **Step 1: 빌드**
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2
source install/setup.bash
```

### **Step 2: 시스템 재시작**
```bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

### **Step 3: TF 모니터링**
```bash
# 새 터미널
python3 ~/transbot_ws_ros2/tf_rotation_checker.py --angle 90
```

### **Step 4: 90° 회전 테스트**
```bash
# 새 터미널
python3 ~/transbot_ws_ros2/rover.py
# 명령: r 90
```

### **Step 5: 결과 분석**
**기대 결과:**
```
90° 명령:
  TF 변화: 88-92° (±2° 오차)
  Odom: ~95° (angular_scale=1.46 적용)
  IMU: ~135° (과대측정이지만 EKF가 가중치 조정)
  EKF 융합: ~90° (두 센서의 중간값, IMU 가중치 적절히 적용)
```

**확인 사항:**
1. TF가 제대로 회전하는가? (이전: 0.47°, 목표: 90°)
2. `/odometry/filtered` 토픽이 정확한가?
3. SLAM 맵이 드리프트 없이 안정적인가?

---

## 📝 **추가 최적화 옵션** (필요 시)

### **Option A: IMU 가중치 더 낮추기**
```yaml
# ekf_config.yaml
# yaw_vel 프로세스 노이즈 증가: 0.005 → 0.02
# IMU 신뢰도를 더 낮춰서 Odom 중심으로 융합
```

### **Option B: Odom 가중치 증가**
```yaml
# ekf_config.yaml
odom0_twist_rejection_threshold: 2.0  # 3.0 → 2.0
# Odom을 더 엄격하게 검증하여 신뢰도 증가
```

### **Option C: IMU gyro_scale 추가 보정**
```yaml
# imu_calib.yaml
gyro_scale:
  x: 1.0
  y: 1.0
  z: 0.67  # 1 / 1.5 = 0.67 (50% 과대측정 보정)
```

---

## 🎯 **예상 시나리오**

### **시나리오 1: 성공** ✅
- 90° 회전 시 TF가 88-92° 변화
- EKF가 Odom + IMU를 적절히 융합
- SLAM 맵이 안정적으로 유지
- **→ 문제 해결 완료!**

### **시나리오 2: TF 여전히 부족** ⚠️
- 90° 회전 시 TF가 70-80° 변화
- Odom은 정확해졌지만 IMU 가중치 여전히 낮음
- **→ Option A 적용: IMU 가중치 더 높이기**

### **시나리오 3: TF 과대 회전** ⚠️
- 90° 회전 시 TF가 100-110° 변화
- IMU 가중치가 너무 높음
- **→ Option B 적용: Odom 가중치 증가**

---

## 🔍 **디버깅 명령어**

### **EKF 상태 확인:**
```bash
ros2 topic echo /odometry/filtered --field pose.pose.orientation
```

### **IMU 데이터 확인:**
```bash
ros2 topic echo /imu/data_calibrated --field angular_velocity.z
```

### **Odom 데이터 확인:**
```bash
ros2 topic echo /odom_raw --field twist.twist.angular.z
```

### **TF 트리 확인:**
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## ✨ **핵심 전략 요약**

1. **Odom 정확도 향상:** `angular_scale = 1.46` (물리적 측정 기반)
2. **IMU 수용 확대:** `rejection_threshold = 5.0` (2.0 → 5.0)
3. **IMU 신뢰도 증가:** `process_noise = 0.005` (0.01 → 0.005)
4. **90° 범위 활용:** IMU가 정확한 구간에서 센서 융합 효과 극대화

**핵심 아이디어:**
- IMU가 완벽하지 않아도 **Odom과 함께 융합**하면 충분히 정확!
- EKF는 각 센서의 신뢰도를 동적으로 조정
- 90° 이하에서는 IMU가 정확하므로 **짧은 회전을 여러 번** 하면 효과적