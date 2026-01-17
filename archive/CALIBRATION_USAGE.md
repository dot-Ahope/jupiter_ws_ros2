# Odom Angular Scale Calibration 사용 가이드

## 🎯 **문제 해결**

### **문제:**
Launch 파일에 이미 `angular_scale`이 적용되어 있으면, calibration 스크립트가 받는 `/odom_raw` 데이터가 이미 보정된 값입니다.

**예시:**
```python
# launch 파일
'angular_scale': 1.46  # ⭐ 이미 적용됨!

# /odom_raw 토픽의 값
측정: 95° → 이미 1.46이 곱해진 값
원시: 65° → 실제 하드웨어 값 (95 / 1.46 = 65)
```

### **해결:**
`--launch-scale` 파라미터로 현재 launch 파일의 scale 값을 전달하면, 스크립트가 자동으로 원시 값을 복원합니다.

---

## 📋 **사용 방법**

### **Step 1: Launch 파일 확인**

현재 적용된 `angular_scale` 확인:
```bash
# 파일: src/sllidar_ros2/launch/transbot_full_system.launch.py
grep angular_scale src/sllidar_ros2/launch/transbot_full_system.launch.py
```

**예상 출력:**
```python
'angular_scale': 1.46,  # ⭐ 이 값을 기억
```

---

### **Step 2: Phase 1 실행**

**현재 launch_scale=1.46이 적용된 경우:**
```bash
python3 odom_based_angular_calibration.py --phase 1 --launch-scale 1.46
```

**다른 scale이 적용된 경우 (예: 1.5625):**
```bash
python3 odom_based_angular_calibration.py --phase 1 --launch-scale 1.5625
```

**Phase 1 출력 예시:**
```
⚙️  설정:
  Launch 파일 angular_scale: 1.46
  (Odom 원시값 = 측정값 / 1.46)

📊 측정 결과:
목표 각도:          90.0°

⚙️  Launch 파일 설정:
  현재 적용된 scale: 1.46

📍 Raw Odometry (base_node 출력):
  Odom (측정값):       95.0° (scale=1.46 적용 후)
  Odom (원시값):       65.1° (scale 적용 전) ⭐
  angular_scale:      1.383 (IMU 적분 기준, 원시 Odom 기준) ⭐
```

**결과:**
```
Phase 1 완료!
초기 angular_scale: 1.385 ± 0.003

다음 단계:
  python3 odom_based_angular_calibration.py --phase 2 --scale 1.385 --launch-scale 1.46
```

---

### **Step 3: Phase 2 실행**

Phase 1 결과를 사용하여:
```bash
python3 odom_based_angular_calibration.py --phase 2 --scale 1.385 --launch-scale 1.46
```

**Phase 2 출력 예시:**
```
📊 측정 결과 (90° 테스트):
목표 각도:          90.0°

⚙️  Launch 파일 설정:
  현재 적용된 scale: 1.46

📍 Raw Odometry (base_node 출력):
  Odom (측정값):       94.9° (scale=1.46 적용 후)
  Odom (원시값):       65.0° (scale 적용 전) ⭐
  angular_scale:      1.385 (IMU 적분 기준) ⭐

📍 IMU 직접 측정:
  IMU (적분):         90.0° (자이로 적분)
```

**최종 결과:**
```
⭐ 권장 angular_scale: 1.385
   (표준편차: ±0.005)
   (신뢰도: 매우 높음 ✅)
```

---

## 🔍 **보정 원리**

### **내부 계산:**

1. **측정값 (토픽에서 받은 값):**
   ```
   /odom_raw: 95° (이미 scale 적용됨)
   ```

2. **원시값 복원:**
   ```python
   odom_raw = 측정값 / launch_scale
   odom_raw = 95° / 1.46 = 65°
   ```

3. **angular_scale 계산:**
   ```python
   angular_scale = IMU / odom_raw
   angular_scale = 90° / 65° = 1.385
   ```

4. **검증:**
   ```python
   예측값 = odom_raw × angular_scale
   예측값 = 65° × 1.385 = 90°  ✅
   ```

---

## 📊 **예상 시나리오**

### **시나리오 1: Launch scale이 정확한 경우**

**설정:**
```python
# launch 파일
'angular_scale': 1.46

# 실제 물리적 정확도
Odom (raw): 65° → × 1.46 = 95°
물리적: 90°
오차: +5° (5.5%)
```

**Calibration 결과:**
```
원시 Odom: 65°
IMU: 90°
angular_scale: 1.385 (65 → 90)
```

**새 launch scale 적용:**
```python
'angular_scale': 1.385  # 1.46 → 1.385
```

**검증:**
```
Odom (raw): 65° × 1.385 = 90°  ✅
```

---

### **시나리오 2: Launch scale이 부정확한 경우**

**설정:**
```python
# launch 파일
'angular_scale': 1.8  # 과대 보정

# 실제
Odom (raw): 65° → × 1.8 = 117°
물리적: 90°
오차: +27° (30% 과대!)
```

**Calibration 결과:**
```
측정값: 117° (launch_scale=1.8 적용)
원시 복원: 117 / 1.8 = 65°
IMU: 90°
angular_scale: 1.385 (65 → 90) ✅
```

**새 launch scale 적용:**
```python
'angular_scale': 1.385  # 1.8 → 1.385 (수정)
```

---

## ⚠️  **주의사항**

### **1. 항상 launch_scale 확인**
```bash
# 확인 명령
grep angular_scale src/sllidar_ros2/launch/transbot_full_system.launch.py
```

### **2. Phase 1과 Phase 2에 동일한 launch_scale 사용**
```bash
# Phase 1
python3 odom_based_angular_calibration.py --phase 1 --launch-scale 1.46

# Phase 2 (동일한 1.46 사용)
python3 odom_based_angular_calibration.py --phase 2 --scale 1.385 --launch-scale 1.46
```

### **3. Launch scale 변경 후 재측정**
launch 파일의 `angular_scale`을 변경한 후:
1. 시스템 재시작
2. Phase 1부터 다시 실행
3. 새로운 launch_scale 값 사용

---

## 🧪 **테스트 시나리오**

### **Test 1: 기본 설정 (launch_scale=1.46)**
```bash
# Phase 1
python3 odom_based_angular_calibration.py --phase 1 --launch-scale 1.46

# 결과 예시: initial_scale = 1.385

# Phase 2
python3 odom_based_angular_calibration.py --phase 2 --scale 1.385 --launch-scale 1.46

# 결과 예시: recommended_scale = 1.385
```

### **Test 2: 다른 설정 (launch_scale=1.5625)**
```bash
# Phase 1
python3 odom_based_angular_calibration.py --phase 1 --launch-scale 1.5625

# 결과 예시: initial_scale = 1.48

# Phase 2
python3 odom_based_angular_calibration.py --phase 2 --scale 1.48 --launch-scale 1.5625

# 결과 예시: recommended_scale = 1.48
```

---

## 📝 **요약**

### **핵심 포인트:**

1. **문제:**
   - `/odom_raw` 토픽은 이미 launch의 `angular_scale`이 적용된 값

2. **해결:**
   - `--launch-scale` 파라미터로 현재 scale 전달
   - 스크립트가 자동으로 원시 값 복원
   - 정확한 `angular_scale` 계산

3. **사용:**
   ```bash
   # Phase 1
   python3 odom_based_angular_calibration.py \
     --phase 1 \
     --launch-scale 1.46
   
   # Phase 2
   python3 odom_based_angular_calibration.py \
     --phase 2 \
     --scale <Phase1_result> \
     --launch-scale 1.46
   ```

4. **검증:**
   - "Odom (원시값)" 확인
   - "angular_scale (원시 Odom 기준)" 확인
   - 측정값 = 원시값 × launch_scale

---

## 🚀 **다음 단계**

Calibration 완료 후:

1. **Launch 파일 수정:**
   ```python
   'angular_scale': <recommended_scale>,
   ```

2. **빌드:**
   ```bash
   colcon build --packages-select sllidar_ros2
   source install/setup.bash
   ```

3. **테스트:**
   ```bash
   # 90° 회전 테스트
   python3 rover.py
   # 명령: r 90
   
   # TF 확인
   python3 tf_rotation_checker.py --angle 90
   ```

4. **검증:**
   - TF 회전이 90° ± 2° 범위 내인지 확인
   - SLAM 맵이 안정적인지 확인
