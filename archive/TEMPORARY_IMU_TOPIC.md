# ⚠️ 임시 조치: /transbot/imu 사용

## 📋 **상황**

### **문제:**
- `transbot_driver`가 아직 재시작되지 않아 `/imu/data_raw`를 발행하지 않음
- 현재는 `/transbot/imu`만 발행 중

### **확인:**
```bash
ros2 topic info /imu/data_raw
# Type: sensor_msgs/msg/Imu
# Publisher count: 0  ❌ (발행자 없음!)
# Subscription count: 1

ros2 topic echo /transbot/imu --once
# ✅ 데이터 출력됨
```

---

## ✅ **임시 조치**

### **1. EKF 설정 임시 변경:**
```yaml
# ekf_config.yaml
imu0: /transbot/imu  # ⚠️ 임시 (재시작 후 /imu/data_raw로 변경)
```

### **2. Phase 1 진단 스크립트 임시 변경:**
```python
# phase1_odom_imu_diagnosis.py
self.imu_sub = self.create_subscription(
    Imu, '/transbot/imu', ...)  # ⚠️ 임시
```

### **3. 재빌드 완료:**
```bash
colcon build --packages-select sllidar_ros2
# ✅ Finished in 0.43s
```

---

## 🚀 **현재 사용 가능**

### **바로 Phase 1 진단 실행 가능:**
```bash
cd ~/transbot_ws_ros2
python3 phase1_odom_imu_diagnosis.py
```

**데이터 흐름 (임시):**
```
transbot_driver → /transbot/imu → EKF
transbot_driver → /transbot/imu → phase1_odom_imu_diagnosis.py
```

---

## 🔄 **완전한 최적화 (나중에)**

### **시스템 전체 재시작 후:**

1. **launch 파일이 적용됨:**
   ```
   transbot_driver → /imu/data_raw → EKF
   ```

2. **설정 변경:**
   ```yaml
   # ekf_config.yaml
   imu0: /imu/data_raw  # ⭐ 최종 목표
   ```

3. **진단 스크립트 변경:**
   ```python
   # phase1_odom_imu_diagnosis.py
   self.imu_sub = self.create_subscription(
       Imu, '/imu/data_raw', ...)
   ```

4. **재빌드:**
   ```bash
   colcon build --packages-select sllidar_ros2
   source install/setup.bash
   ```

5. **전체 시스템 재시작:**
   ```bash
   ros2 launch sllidar_ros2 transbot_full_system.launch.py
   ```

---

## 📊 **현재 vs 최종**

### **현재 (임시):**
```
transbot_driver → /transbot/imu → EKF ✅ (작동함)
                   ↓
            phase1_diagnosis ✅
```

### **최종 (재시작 후):**
```
transbot_driver → /imu/data_raw → EKF ⭐ (최적화됨)
                   ↓
            phase1_diagnosis ⭐
```

**차이:**
- 토픽 이름만 다름
- 기능은 동일 (apply_calib은 이미 제거됨)
- 최적화는 시스템 재시작 후 자동 적용

---

## 🎯 **지금 할 수 있는 것**

### **✅ Phase 1 진단 바로 실행:**
```bash
python3 phase1_odom_imu_diagnosis.py
```

**예상 출력:**
```
⏳ 센서 적분 중... (Enter를 누르면 종료)
   IMU:   12.5° | Odom:    9.0°  ← 정상 작동! ⭐
   IMU:   34.8° | Odom:   25.1°
   IMU:   90.5° | Odom:   65.2°
```

---

## 💡 **요약**

- ⚠️ **현재**: `/transbot/imu` 사용 (임시)
- ✅ **작동**: Phase 1 진단 바로 사용 가능
- ⭐ **나중**: 시스템 재시작 시 `/imu/data_raw`로 자동 전환

**지금 바로 Phase 1 진단을 실행하세요!** 🚀
