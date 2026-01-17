# ⚡ EKF 문제 수정 완료

## 📋 수정 사항 요약

### **1. Rejection Threshold 증가 (즉시 효과)**
```yaml
# 수정 전
odom0_twist_rejection_threshold: 3.0
imu0_twist_rejection_threshold: 5.0

# 수정 후
odom0_twist_rejection_threshold: 10.0  # +7.0
imu0_twist_rejection_threshold: 15.0   # +10.0
```
**효과:** IMU-Odom 센서 차이 15-20°를 정상 범위로 수용

---

### **2. Relative 설정 통일 (방향 일관성)**
```yaml
# 수정 전
odom0_relative: false  # 절대 측정
imu0_relative: true    # 상대 측정

# 수정 후
odom0_relative: true   # 상대 측정으로 통일
imu0_relative: true    # 상대 측정
```
**효과:** 센서 기준점 통일, 방향 전환 시 일관성 확보

---

### **3. Process Noise 균형 조정 (센서 가중치)**
```yaml
# 수정 전
process_noise_covariance[5] (yaw): 0.015
process_noise_covariance[11] (yaw_vel): 0.005  # IMU 과신뢰

# 수정 후
process_noise_covariance[5] (yaw): 0.03        # +0.015 (2배)
process_noise_covariance[11] (yaw_vel): 0.01   # +0.005 (2배)
```
**효과:** IMU 편향 감소, Odom-IMU 균형 개선

---

### **4. 초기 공분산 안정화 (수렴 속도)**
```yaml
# 수정 전
initial_estimate_covariance[5] (yaw): 1000.0    # 극단적 불확실성
initial_estimate_covariance[11] (yaw_vel): 0.2

# 수정 후
initial_estimate_covariance[5] (yaw): 10.0      # -990.0 (100배 감소)
initial_estimate_covariance[11] (yaw_vel): 0.5  # +0.3 (2.5배 증가)
```
**효과:** 초기 수렴 안정화, 방향별 일관성 확보

---

## 🎯 예상 결과

### **수정 전:**
```
테스트 1 (IMU 반시계): EKF 96.08° ✓
테스트 2 (IMU 시계):   EKF  4.04° ✗ (24배 차이!)
테스트 3 (Odom 반시계): EKF 14.11° ✗
테스트 4 (Odom 시계):   EKF 39.95° ~

→ EKF 일관성: 92.04° (catastrophic failure)
```

### **수정 후 예상:**
```
테스트 1 (IMU 반시계): EKF ~85-90°
테스트 2 (IMU 시계):   EKF ~85-90°
테스트 3 (Odom 반시계): EKF ~85-95°
테스트 4 (Odom 시계):   EKF ~85-95°

→ EKF 일관성: <10° (excellent)
```

---

## 🔄 다음 단계

### **1. 빌드 & 재시작**
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2
source install/setup.bash

# 시스템 재시작
sudo reboot
```

### **2. EKF 비교 테스트 재실행**
```bash
cd ~/transbot_ws_ros2
python3 ekf_comparison_test.py
```

### **3. 결과 검증**
- ✅ **성공 기준:** EKF 일관성 < 10°
- ✅ **방향 대칭:** 반시계 vs 시계 차이 < 5°
- ✅ **센서 융합:** EKF가 IMU와 Odom 중간값 생성

### **4. 실패 시 추가 조치**
- Rejection threshold 더 증가 (10/15 → 20/20)
- Process noise 더 증가 (0.03/0.01 → 0.05/0.02)
- 또는 진단 로그 분석

---

## 📊 변경 파일

**파일:** `/home/user/transbot_ws_ros2/src/sllidar_ros2/config/ekf_config.yaml`

**변경 라인:**
- Line 29: `odom0_relative: false → true`
- Line 33: `odom0_twist_rejection_threshold: 3.0 → 10.0`
- Line 48: `imu0_twist_rejection_threshold: 5.0 → 15.0`
- Line 68 (index 5): `process_noise yaw: 0.015 → 0.03`
- Line 77 (index 11): `process_noise yaw_vel: 0.005 → 0.01`
- Line 96 (index 5): `initial yaw: 1000.0 → 10.0`
- Line 102 (index 11): `initial yaw_vel: 0.2 → 0.5`

**총 7개 파라미터 변경**

---

## 💡 핵심 원리

### **문제의 근본 원인:**
1. **Threshold 너무 낮음** → 정상 센서 차이를 거부
2. **Relative 불일치** → 방향별 기준점 다름
3. **IMU 과신뢰** → Process noise 너무 작음
4. **초기 불확실성 과다** → 수렴 불안정

### **해결 접근법:**
1. **더 관대한 허용** → Rejection threshold ↑
2. **기준점 통일** → Relative 설정 일치
3. **센서 균형** → Process noise ↑
4. **안정적 초기화** → Initial covariance ↓

### **Trade-off 관리:**
- 이상치 수용 위험 ↔ 정상 데이터 거부 방지
- 필터 반응성 ↔ 센서 균형
- 빠른 수렴 ↔ 잘못된 초기값 취약성

**결론:** 현재 설정은 "정상 센서 차이를 과도하게 거부"했으므로, 
더 관대한 설정이 전체적으로 이득.
