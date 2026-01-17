# 🔍 Phase 1 진단 가이드: Odom vs IMU 불일치 해결

## 🎯 **문제 상황**

### **현재 증상:**
```
명령: 90° 회전
결과:
  - Odom (raw):    90°  (angular_scale 적용 후)
  - IMU (적분):   145°  (1.61x 차이!)
  - EKF Odom:       0°   (IMU를 outlier로 무시!)
```

### **문제 분석:**
1. **Odom과 IMU 차이가 너무 큼** (1.61배)
2. **EKF가 이 차이를 데이터 충돌로 판단**
3. **EKF가 IMU를 비정상 값(outlier)으로 거부**
4. **결과: IMU 데이터가 전혀 반영되지 않음**

---

## 🔧 **진단 전략**

### **핵심 질문 3가지:**

#### **1. 실제 로봇이 정확히 90° 회전할 때:**
- Odom은 얼마를 측정하는가?
- IMU는 얼마를 측정하는가?
- EKF는 얼마를 출력하는가?

#### **2. Odom과 IMU가 일치하려면:**
- angular_scale이 얼마여야 하는가?

#### **3. 왜 센서들이 물리적 회전과 차이가 나는가:**
- Odom 언더리포팅: 휠 슬립? 펌웨어?
- IMU 오버리포팅: 캘리브레이션? 드리프트?

---

## 📋 **진단 스크립트: `phase1_odom_imu_diagnosis.py`**

### **기능:**

#### **Test 1: 수동 90° 회전 (물리적 정확도 확인)**
```python
목적: 실제 90° 회전 시 센서들이 어떻게 측정하는가?

방법:
  1. 바닥에 테이프로 0°, 90° 방향 표시
  2. 로봇을 손으로 천천히 정확히 90° 회전
  3. 각 센서의 측정값 기록

측정:
  - Odom (raw): angular_scale 적용 전 값
  - EKF Odom: 센서 융합 후 값
  - IMU (적분): 자이로 적분

분석:
  - 실제 90° / Odom 측정값 = 필요한 angular_scale
  - IMU / Odom = 현재 불일치 비율
```

#### **Test 2: 모터 구동 90° 회전 (현재 설정 확인)**
```python
목적: 현재 angular_scale로 90° 명령 시 실제 얼마나 회전하는가?

방법:
  1. Odom이 90° 도달할 때까지 모터 구동
  2. 바닥 표시와 비교하여 실제 회전각 확인
  3. 센서 측정값과 물리적 회전 비교

측정:
  - Odom: 90° (명령값)
  - EKF: ? (융합 결과)
  - IMU: ? (실제 측정)
  - 물리적 회전: 바닥 표시로 확인

분석:
  - 실제 < 90°: angular_scale 증가 필요
  - 실제 > 90°: angular_scale 감소 필요
  - EKF ≈ 0°: IMU outlier 처리 중
```

---

## 🚀 **사용 방법**

### **1. 준비:**
```bash
# 바닥 표시
1. 테이프로 시작 방향 (0°) 표시
2. 왼쪽 90° 방향에도 테이프 표시
3. 로봇을 0° 선에 정렬

# 시스템 실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

### **2. 진단 실행:**
```bash
cd ~/transbot_ws_ros2
python3 phase1_odom_imu_diagnosis.py
```

### **3. 진행 순서:**

#### **Step 1: 수동 회전 테스트**
```
프롬프트: "준비 완료 후 Enter를 누르세요..."
→ Enter

프롬프트: "로봇을 천천히 손으로 90° 회전시키세요..."
→ 로봇을 90° 표시까지 정확히 회전
→ Enter

출력:
  📊 수동 회전 측정 결과
  실제 물리적 회전:     90.00°
  
  센서 측정값:
    Odom (raw):         ?.??°
    EKF Odom:           ?.??°
    IMU (적분):         ?.??°
  
  필요한 보정:
    angular_scale = ?.????
```

#### **Step 2: 모터 구동 테스트**
```
프롬프트: "준비 완료 후 Enter를 누르세요..."
→ Enter

진행:
  진행  50% | Odom:  45.0° | EKF:   0.2° | IMU:  72.0°
  진행 100% | Odom:  90.0° | EKF:   0.4° | IMU: 145.0°
  
출력:
  📊 모터 구동 측정 결과
  명령:                  90.0° (Odom 기준)
  
  센서 측정값:
    Odom (raw):          90.00° ⭐
    EKF Odom:             0.40°
    IMU (적분):         145.00°
  
  IMU/Odom 비율:         1.61x
  
  ⚠️ EKF가 ~0°이면: IMU를 outlier로 무시 중
  
  💡 실제 물리적 회전을 바닥 표시로 확인!
     - 정확히 90°? → angular_scale OK
     - 90°보다 적음? → angular_scale 증가 필요
     - 90°보다 많음? → angular_scale 감소 필요
```

#### **Step 3: 종합 분석**
```
🎯 종합 분석 및 권장사항

1️⃣ IMU 정확도: 98.0%
   ✅ IMU가 정확합니다!

2️⃣ Odom 언더리포팅: 1.61x
   → angular_scale = 1.6100 필요
   
   ⚠️ Odom이 실제보다 61.0% 적게 측정
      원인: 휠 슬립, 펌웨어 under-counting 등

3️⃣ EKF 상태:
   ❌ EKF가 IMU를 무시하고 있습니다! (0.4°)
      원인: Odom과 IMU 차이가 너무 큼
      해결: angular_scale을 1.6100로 조정

4️⃣ 최종 권장 angular_scale:
   
   방법 A (IMU 기준): 1.6100
     - Odom과 IMU를 일치시킴
     - EKF가 두 센서를 모두 사용
     - IMU가 정확하다고 가정
   
   방법 B (물리적 기준): 1.5800
     - Odom이 정확히 90° 측정하도록
     - 물리적 회전이 정확하다고 가정
     - IMU는 보조 역할
   
   ⭐ 권장: 두 값의 평균 = 1.5950
```

---

## 📊 **예상 시나리오별 대응**

### **시나리오 1: IMU가 정확, Odom 부족**
```
수동 테스트 결과:
  실제:    90.0°
  Odom:    56.0° (언더리포팅!)
  IMU:     90.5° (정확!)
  EKF:      0.2° (IMU 무시)

분석:
  - IMU가 물리적 회전과 일치
  - Odom이 37% 부족
  - 펌웨어 under-counting 의심

해결:
  angular_scale = 90.0 / 56.0 = 1.607
  
적용:
  transbot_full_system.launch.py:
    'angular_scale': 1.607
```

---

### **시나리오 2: Odom 정확, IMU 과다**
```
수동 테스트 결과:
  실제:    90.0°
  Odom:    88.0° (거의 정확!)
  IMU:    145.0° (과다!)
  EKF:      0.3° (IMU 무시)

분석:
  - Odom이 물리적 회전과 일치
  - IMU가 61% 과다
  - IMU 캘리브레이션 필요

해결:
  1. IMU 재캘리브레이션:
     imu_calib.yaml의 gyro_scale 조정
     
  2. EKF에서 IMU 가중치 낮춤:
     ekf_config.yaml:
       imu0_twist_rejection_threshold: 0.5 → 5.0
```

---

### **시나리오 3: 둘 다 부정확**
```
수동 테스트 결과:
  실제:    90.0°
  Odom:    60.0° (부족!)
  IMU:    105.0° (과다!)
  EKF:      0.2° (IMU 무시)

분석:
  - Odom: 33% 부족
  - IMU: 17% 과다
  - 둘 다 보정 필요

해결:
  1. angular_scale = 90.0 / 60.0 = 1.500
  2. IMU gyro_scale 조정 (105.0 → 90.0)
  3. 재테스트
```

---

### **시나리오 4: EKF가 잘 융합 중**
```
수동 테스트 결과:
  실제:    90.0°
  Odom:    88.0°
  IMU:     91.0°
  EKF:     89.5° (융합 중!)

분석:
  - 두 센서 거의 일치
  - EKF가 정상 작동
  - 아주 작은 조정만 필요

해결:
  angular_scale = 90.0 / 88.0 = 1.023
  (현재값이 1.560이면 → 1.596)
```

---

## 🔍 **EKF Outlier 판단 메커니즘**

### **EKF 설정 (ekf_config.yaml):**
```yaml
# Odom 설정
odom0_twist_rejection_threshold: 3.0

# IMU 설정
imu0_twist_rejection_threshold: 2.0
```

### **Outlier 판단 기준:**
```python
# 1. 센서 값 읽기
odom_angular_vel = 0.3 rad/s  (현재 측정)
imu_angular_vel = 0.48 rad/s  (현재 측정)

# 2. 예측값과 비교
predicted_vel = 0.3 rad/s  (EKF 예측)

# 3. 잔차(residual) 계산
odom_residual = |0.3 - 0.3| = 0.0
imu_residual = |0.48 - 0.3| = 0.18

# 4. 표준화된 잔차
odom_mahalanobis = 0.0 / σ_odom = 0.0
imu_mahalanobis = 0.18 / σ_imu = 2.5  (예시)

# 5. Threshold 비교
if imu_mahalanobis > 2.0:
    # IMU를 outlier로 판단
    # 이번 업데이트에서 IMU 무시
```

### **현재 문제:**
```
Odom과 IMU 차이가 1.61배
→ IMU residual이 항상 threshold 초과
→ EKF가 계속 IMU를 outlier로 처리
→ IMU 데이터가 전혀 반영되지 않음
```

### **해결:**
```
Option 1 (권장): angular_scale 조정
  - Odom과 IMU를 일치시킴
  - EKF가 자연스럽게 두 센서 융합
  
Option 2: Threshold 완화
  ekf_config.yaml:
    imu0_twist_rejection_threshold: 2.0 → 5.0
  - 더 큰 차이도 수용
  - 근본 원인 미해결
```

---

## 💡 **핵심 인사이트**

### **1. Odom 언더리포팅 원인:**
```
가능성 A: 휠 슬립
  - 회전 시 바퀴가 미끄러짐
  - 실제보다 적은 회전 측정
  
가능성 B: 펌웨어 under-counting
  - STM32 엔코더 카운트 누락
  - 각속도 계산 시 스케일링 오류
  
가능성 C: 운동학 파라미터 오류
  - wheelbase 값이 실제보다 큼
  - 회전 반경 과대 평가
```

### **2. IMU 드리프트 vs 정확도:**
```
IMU 적분은 시간에 따라 드리프트:
  - 단기(< 10초): 정확
  - 장기(> 1분): 누적 오차
  
Phase 1 진단은 단기 측정:
  - IMU 드리프트 영향 최소
  - IMU 측정값을 참조값으로 사용 가능
```

### **3. angular_scale의 의미:**
```
angular_scale = 실제 회전 / Odom 측정
             = IMU 측정 / Odom 측정  (IMU가 정확하다면)

현재 1.5625:
  - Odom 144° → 실제 225°
  - 1.5625배 보정
  
진단 후 1.61:
  - 더 큰 보정 필요
  - Odom이 더 많이 under-report
```

---

## 🎯 **다음 단계**

### **1. Phase 1 진단 실행:**
```bash
python3 phase1_odom_imu_diagnosis.py
```

### **2. 권장 angular_scale 획득:**
```
출력:
  ⭐ 권장: 1.5950
```

### **3. Launch 파일 수정:**
```python
# transbot_full_system.launch.py
'angular_scale': 1.5950,
```

### **4. 재빌드:**
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2
source install/setup.bash
```

### **5. 검증:**
```bash
# 시스템 재실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 다시 진단
python3 phase1_odom_imu_diagnosis.py

# 확인:
  - EKF Odom이 0° 근처가 아님
  - Odom과 IMU 차이 < 10%
  - EKF가 두 센서를 융합 중
```

### **6. Phase 2 캘리브레이션:**
```bash
python3 odom_based_angular_calibration.py --phase 1
# 초기 scale 획득

python3 odom_based_angular_calibration.py --phase 2 --scale 1.5950
# 전체 각도 테스트
```

---

## ✅ **성공 기준**

### **Phase 1 진단 후:**
```
✅ IMU 정확도: 95-105%
✅ Odom 언더리포팅 비율 확인
✅ 권장 angular_scale 획득
✅ EKF outlier 원인 파악
```

### **angular_scale 적용 후:**
```
✅ EKF Odom > 80° (90° 목표)
✅ Odom과 IMU 차이 < 10%
✅ EKF가 두 센서 모두 사용
✅ 물리적 90° 회전 확인
```

### **Phase 2 캘리브레이션 후:**
```
✅ 모든 각도에서 일관성
✅ 표준편차 < 5%
✅ 최종 angular_scale 확정
```

---

## 🚨 **문제 해결**

### **문제 1: 수동 회전 시 센서 변화 없음**
```
원인: 센서 토픽이 발행되지 않음
해결:
  1. ros2 topic list 확인
  2. ros2 topic echo /odom_raw
  3. 시스템 재실행
```

### **문제 2: EKF가 여전히 ~0°**
```
원인: angular_scale 변경 안 됨
해결:
  1. launch 파일 수정 확인
  2. colcon build 재실행
  3. source install/setup.bash
  4. 터미널 재시작
```

### **문제 3: IMU 적분이 계속 증가**
```
원인: IMU 드리프트
해결:
  - 정상입니다 (단기 측정에서는 무시 가능)
  - 10초 내 측정 완료
```

---

## 📝 **요약**

이 진단 도구는 **Odom과 IMU 불일치 문제를 근본적으로 해결**합니다:

1. ✅ **물리적 정확도 확인** (수동 회전)
2. ✅ **센서 측정값 비교** (Odom, IMU, EKF)
3. ✅ **필요한 angular_scale 계산** (자동)
4. ✅ **EKF outlier 원인 분석**
5. ✅ **권장 조치 제공**

**목표: EKF가 Odom과 IMU를 모두 활용하여 정확한 위치 추정!** 🎯
