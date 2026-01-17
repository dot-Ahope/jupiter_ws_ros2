# 🎯 회전 정합 작업 로드맵

## 📊 현재 상황 분석

### 1. 작업 배경 및 목표
**목표:** IMU 데이터, 오도메트리 데이터, 실제 물리적 환경에서의 회전을 정확하게 일치시키기

**핵심 문제:**
```
실제 90° 회전 시:
  - 물리적 회전:  90.00°  (바닥 표시 기준)
  - Odom (raw):   ~58°    (언더리포팅, angular_scale 적용 전)
  - IMU (적분):   ~145°   (과대 측정 또는 바이어스)
  - EKF Odom:     ~0°     (IMU를 outlier로 무시!)

문제 분석:
  1. Odom과 IMU 차이 1.61배 → EKF가 IMU를 거부
  2. angular_scale 보정 필요 (Odom 언더리포팅 보정)
  3. IMU 자이로 바이어스 제거 필요 (드리프트 제거)
```

---

## ✅ 완료된 작업

### Phase 0: 문제 식별 ✅
- [x] SLAM 회전 정확도 문제 발견 (선형 98%, 회전 30%)
- [x] EKF가 IMU를 outlier로 처리하는 현상 확인
- [x] Odom/IMU 비율 1.61x 차이 측정

### SLAM 최적화 ✅
- [x] 차등 엄격도 전략 구현:
  - 실시간 스캔 매칭: 느슨하게 (link_match_minimum_response_fine: 0.10)
  - 루프 클로저: 엄격하게 (loop_match_minimum_response_fine: 0.50)
- [x] 회전 추적 개선 파라미터 적용

### IMU 데이터 흐름 최적화 ✅
- [x] 불필요한 중간 노드(apply_calib) 제거 시도
- [x] 데이터 흐름 단순화: transbot_driver → imu_calib → EKF
- [x] 최종: imu_calib 패키지 사용 결정 (검증된 도구)

### IMU 자이로 바이어스 제거 ✅
- [x] 문제 발견: 정지 상태에서 -0.008 rad/s 드리프트 (시계방향)
  - 누적: 27.8°/min 드리프트!
- [x] imu_calib 패키지로 전환 (자이로 + 가속도계 캘리브레이션)
- [x] imu_calib.yaml 생성 완료 (가속도계 바이어스/스케일만 포함)
- [x] ⚠️ **자이로 바이어스 데이터 누락** - 재캘리브레이션 필요!

### Phase 1 진단 도구 개발 ✅
- [x] `phase1_odom_imu_diagnosis.py` 스크립트 작성
  - Test 1: 수동 90° 회전 (물리적 정확도 측정)
  - Test 2: 모터 구동 90° 회전 (현재 설정 검증)
  - 실시간 센서 적분 (non-blocking input)
  - angular_scale 자동 계산

### 시스템 최적화 ✅
- [x] robot_state_publisher 주파수: 10000Hz → 30Hz (99.7% CPU 절감)
- [x] EKF 파라미터 튜닝
- [x] 문서화: 6개 가이드 문서 작성

---

## 🔴 현재 문제점

### 1. imu_calib.yaml에 자이로 바이어스 데이터 없음
```yaml
# 현재 imu_calib.yaml
accel_scale: [...]   # ✅ 있음
accel_bias: [...]    # ✅ 있음
gyro_bias: [...]     # ❌ 없음! → 드리프트 계속 발생
```

**원인:** imu_calib 캘리브레이션 시 자이로 보정 단계를 완료하지 않음

**영향:**
- 정지 상태에서도 -0.008 rad/s 드리프트
- 27.8°/min 누적 오차
- SLAM 지도 품질 저하

### 2. angular_scale 최적값 미확정
```python
# transbot_full_system.launch.py
'angular_scale': 1.5625  # 임시값 (물리적 225° / 측정 144° = 1.5625)

# 실제 필요값은 Phase 1 진단 실행 후 결정해야 함
```

**현재 상태:** Phase 1 진단 도구는 준비되었으나 **아직 실행하지 않음**

### 3. EKF-IMU 통합 미검증
- EKF가 여전히 IMU를 outlier로 처리하는지 확인 필요
- `/odometry/filtered`가 IMU 데이터를 제대로 반영하는지 검증 필요

---

## 🚀 다음 작업 (우선순위 순)

### ⭐ Priority 1: IMU 자이로 바이어스 캘리브레이션

**작업:** imu_calib으로 자이로 바이어스 측정 및 저장

**명령:**
```bash
# 1. 시스템 시작
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 2. 새 터미널에서 캘리브레이션
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 run imu_calib do_calib /transbot/imu
```

**과정:**
1. **Phase 1: 자이로 바이어스 측정**
   - 로봇을 완전히 정지
   - 100 샘플 수집 (약 10초)
   - 자이로 X, Y, Z 바이어스 계산

2. **Phase 2: 가속도계 캘리브레이션**
   - 6방향 자세 캘리브레이션
   - 각 자세에서 2-3초 유지

3. **저장:**
   ```bash
   mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml
   ```

**예상 결과:**
```yaml
gyro_bias:
  x: -0.005119
  y: -0.003254
  z: -0.008100  # ← 드리프트 원인!
accel_bias: [...]
accel_scale: [...]
```

**검증:**
```bash
# 시스템 재시작 후
ros2 topic echo /imu/data_calibrated --field angular_velocity.z
# 출력: ~0.000 (이전: -0.008) ✅
```

**소요 시간:** 5분  
**중요도:** ⭐⭐⭐⭐⭐ (정지 상태 드리프트 제거)

---

### ⭐ Priority 2: Phase 1 진단 실행

**작업:** 물리적 90° 회전과 센서 측정값 비교

**명령:**
```bash
# 1. 바닥에 테이프로 0°, 90° 방향 표시
# 2. 로봇을 0° 위치에 정렬

# 3. 진단 실행
cd ~/transbot_ws_ros2
python3 phase1_odom_imu_diagnosis.py
```

**Test 1: 수동 90° 회전**
- 로봇을 손으로 천천히 90° 회전
- 센서 측정값 기록:
  ```
  Odom (raw):      XX.XX°
  IMU (적분):      YY.YY°
  EKF Odom:        ZZ.ZZ°
  ```
- 자동 계산:
  ```
  필요한 angular_scale = 90.0 / Odom측정값
  IMU 일치 scale = IMU측정값 / Odom측정값
  ```

**Test 2: 모터 구동 90° 회전**
- 현재 angular_scale로 Odom 90° 도달 시 실제 회전각 확인
- 바닥 표시와 비교

**예상 결과:**
```
권장 angular_scale:
  방법 A (IMU 기준):    1.38
  방법 B (물리적 기준): 1.55
  평균:                  1.47  ← 이 값 사용!
```

**소요 시간:** 10분  
**중요도:** ⭐⭐⭐⭐⭐ (angular_scale 최적화)

---

### ⭐ Priority 3: angular_scale 업데이트

**작업:** Phase 1 진단 결과를 launch 파일에 적용

**수정 파일:**
1. `transbot_full_system.launch.py` (Line 147)
2. `bringup.launch.py` (transbot_bringup 패키지)

**명령:**
```python
# transbot_full_system.launch.py
'angular_scale': 1.47  # Phase 1 진단 결과 적용
```

**재빌드:**
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2 transbot_bringup
source install/setup.bash
```

**소요 시간:** 2분  
**중요도:** ⭐⭐⭐⭐ (Odom 정확도 개선)

---

### ⭐ Priority 4: EKF-IMU 통합 검증

**작업:** EKF가 IMU를 제대로 융합하는지 확인

**테스트:**
```bash
# 1. 90° 회전 명령
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{angular: {z: 0.3}}'

# 2. 센서 모니터링
ros2 topic echo /odom_raw --field pose.pose.orientation
ros2 topic echo /imu/data_calibrated --field angular_velocity.z
ros2 topic echo /odometry/filtered --field pose.pose.orientation
```

**성공 기준:**
```
Odom 90° 회전 시:
  - /odom_raw:            90° ✅
  - /imu (적분):          88-92° ✅ (자이로 바이어스 제거됨)
  - /odometry/filtered:   89-91° ✅ (IMU를 융합함, outlier 아님!)
```

**실패 징후:**
```
  - /odometry/filtered: 0-5° ❌ (여전히 IMU 무시)
  → EKF rejection threshold 조정 필요
```

**소요 시간:** 5분  
**중요도:** ⭐⭐⭐⭐ (센서 융합 검증)

---

### Priority 5: SLAM 회전 정확도 테스트

**작업:** 실제 SLAM 환경에서 회전 추적 정확도 측정

**테스트 시나리오:**

#### Test A: 제자리 360° 회전
```bash
목표: 회전 추적 정확도 측정
방법: 제자리에서 360° 회전 후 시작 위치로 복귀
측정: 루프 클로저 오차
```

#### Test B: 정사각형 경로 (1m × 1m)
```bash
목표: 누적 회전 오차 측정
방법: 1m 직진 → 90° 좌회전 × 4
측정: 최종 위치/방향 오차
```

#### Test C: 원형 경로
```bash
목표: 연속 회전 추적
방법: 반경 1m 원 주행
측정: 경로 일관성
```

**성공 기준:**
```
회전 추적 정확도:
  - 이전: 30% (98m 중 30m만 정확)
  - 목표: 85% 이상
  - 허용 오차: ±5° per 90° turn
```

**소요 시간:** 20분  
**중요도:** ⭐⭐⭐ (최종 검증)

---

## 📈 작업 플로우차트

```
┌─────────────────────────────────────────────────────────────┐
│ 현재 상태: IMU 드리프트 + angular_scale 미최적화            │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Priority 1: IMU 자이로 바이어스 캘리브레이션 (5분)         │
│   - ros2 run imu_calib do_calib /transbot/imu               │
│   - gyro_bias 데이터를 imu_calib.yaml에 추가                │
│   결과: 정지 상태 드리프트 27.8°/min → 0.06°/min ✅        │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Priority 2: Phase 1 진단 실행 (10분)                        │
│   - python3 phase1_odom_imu_diagnosis.py                    │
│   - Test 1: 수동 90° 회전 → angular_scale 계산              │
│   - Test 2: 모터 90° 회전 → 현재 설정 검증                  │
│   결과: 최적 angular_scale = 1.47 (예시) ✅                │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Priority 3: angular_scale 업데이트 (2분)                    │
│   - transbot_full_system.launch.py 수정                     │
│   - colcon build & 재시작                                    │
│   결과: Odom 정확도 향상 ✅                                 │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Priority 4: EKF-IMU 통합 검증 (5분)                         │
│   - 90° 회전 테스트                                          │
│   - /odometry/filtered 모니터링                             │
│   결과: EKF가 IMU를 융합함 (outlier 아님) ✅               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ Priority 5: SLAM 회전 정확도 테스트 (20분)                  │
│   - 360° 회전, 정사각형, 원형 경로                          │
│   - 회전 추적 30% → 85% 향상 확인 ✅                        │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 완료: 모든 센서가 물리적 회전과 일치 ✅                     │
│   - IMU: 드리프트 제거                                       │
│   - Odom: angular_scale 최적화                               │
│   - EKF: 센서 융합 정상                                      │
│   - SLAM: 회전 정확도 85%+                                   │
└─────────────────────────────────────────────────────────────┘
```

**총 소요 시간:** 약 42분

---

## 🎯 작업별 상세 가이드

### 1. IMU 자이로 바이어스 캘리브레이션

**준비:**
- 로봇을 평평한 바닥에 배치
- 주변 진동 최소화
- 배터리 50% 이상

**실행:**
```bash
# Terminal 1: 시스템 시작
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# Terminal 2: 캘리브레이션
ros2 run imu_calib do_calib /transbot/imu

# Phase 1 진행:
[INFO] 로봇을 정지시키고 움직이지 마세요
[INFO] 100 샘플 수집 중...
[INFO] 자이로 바이어스 측정 완료

# Phase 2 진행 (6방향 자세):
[INFO] 다음 자세로 회전하세요: 평평하게
[INFO] Enter를 누르세요... [Enter]
[INFO] 다음 자세로 회전하세요: 뒤집어서
[INFO] Enter를 누르세요... [Enter]
# ... (6방향 완료)

# 완료
[INFO] 캘리브레이션 완료! imu.yaml 저장됨
```

**검증:**
```bash
# imu_calib.yaml 확인
cat ~/transbot_ws_ros2/imu_calib.yaml

# gyro_bias 섹션이 있는지 확인
# 예:
# gyro_bias:
#   x: -0.005119
#   y: -0.003254
#   z: -0.008100  ← 이 값이 드리프트 원인!
```

**문제 해결:**
- "Could not find topic" 에러:
  ```bash
  ros2 topic list | grep imu
  # /transbot/imu 확인
  ```
- 캘리브레이션 중 로봇 움직임:
  - 재시작 후 다시 시도
  - 안정적인 표면 선택

---

### 2. Phase 1 진단 실행

**준비:**
```bash
# 1. 바닥 표시
- 테이프로 시작 방향 (0°) 표시
- 왼쪽 90° 방향에도 테이프 표시
- 로봇을 0° 선에 정렬

# 2. 주변 공간 확보
- 반경 1.5m 장애물 없음
```

**실행:**
```bash
cd ~/transbot_ws_ros2
python3 phase1_odom_imu_diagnosis.py

# Test 1: 수동 회전
[INFO] 준비 완료 후 Enter... [Enter]
[INFO] 초기값 기록 완료
[INFO] 로봇을 손으로 90° 회전시키세요...
# → 천천히 손으로 회전 (5초)
[INFO] 완료 후 Enter... [Enter]
[INFO] 결과:
  Odom (raw):      58.02°
  IMU (적분):      90.52°
  필요한 angular_scale = 1.5517

# Test 2: 모터 구동
[INFO] 준비 완료 후 Enter... [Enter]
[INFO] 로봇이 자동으로 회전합니다!
# → 바닥 표시와 비교하여 실제 회전각 관찰
[INFO] 결과:
  Odom: 90.00° (명령값)
  실제 물리적 회전: ~58° (바닥 표시 기준)
  → angular_scale이 1.55로 증가 필요
```

**결과 해석:**
```
권장 angular_scale:
  방법 A (IMU 기준):    1.56  (90.52 / 58.02)
  방법 B (물리적 기준): 1.55  (90.00 / 58.00)
  평균:                  1.555

→ transbot_full_system.launch.py에 1.555 적용!
```

---

### 3. angular_scale 업데이트

**파일 수정:**
```python
# transbot_full_system.launch.py (Line 147)
'angular_scale': 1.555,  # Phase 1 진단 결과 (이전: 1.5625)
```

**재빌드:**
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

---

### 4. EKF-IMU 통합 검증

**간단 테스트:**
```bash
# Terminal 1: 토픽 모니터링
ros2 topic echo /odometry/filtered --field pose.pose.orientation

# Terminal 2: 90° 회전 명령
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.3}}'

# 5초 후 정지
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

**성공 확인:**
```
/odometry/filtered orientation:
  z: 0.707  # sin(90°/2) ≈ 0.707
  w: 0.707  # cos(90°/2) ≈ 0.707
→ 약 90° 회전 ✅ (IMU가 반영됨!)

실패 시:
  z: 0.001
  w: 0.999
→ 약 0° ❌ (여전히 IMU 무시)
```

---

### 5. SLAM 회전 정확도 테스트

**Test A: 360° 제자리 회전**
```bash
# RViz에서 관찰
ros2 launch sllidar_ros2 transbot_full_system.launch.py use_rviz:=true

# 로봇 360° 회전 (수동 조작)
# RViz에서 로봇 궤적 확인
# 시작 위치와 종료 위치 오차 측정
```

**성공 기준:**
- 위치 오차: < 10cm
- 방향 오차: < 5°

---

## 📊 예상 개선 효과

### Before (현재 상태)
```
센서 상태:
  - IMU 드리프트: -0.008 rad/s (27.8°/min)
  - Odom 언더리포팅: 64% (실제 90° → 측정 58°)
  - EKF: IMU outlier 처리 (융합 실패)
  - SLAM 회전 정확도: 30%

문제:
  - 정지 상태에서 지도 회전
  - 회전 시 추적 실패
  - 누적 오차 심각
```

### After (작업 완료 후)
```
센서 상태:
  - IMU 드리프트: ~0.0001 rad/s (0.06°/min) ✅
  - Odom 정확도: 98% (실제 90° → 측정 88-92°) ✅
  - EKF: IMU+Odom 융합 성공 ✅
  - SLAM 회전 정확도: 85%+ ✅

개선:
  - 정지 상태 안정
  - 회전 추적 정확
  - 장시간 운행 가능
```

### 개선율
```
IMU 드리프트:     99.6% 감소 (27.8°/min → 0.06°/min)
Odom 정확도:      53% 향상 (64% → 98%)
SLAM 회전:        183% 향상 (30% → 85%)
```

---

## 🎓 기술 배경

### 왜 angular_scale이 필요한가?

**Odom 언더리포팅 원인:**
1. **휠 슬립:** 가속/감속 시 휠이 미끄러짐
2. **펌웨어 카운팅:** 엔코더 해상도 제한
3. **타이어 마모:** 실제 직경 < 설정 직경
4. **바닥 조건:** 카펫, 타일 등 마찰 차이

**해결책:**
- angular_scale: 측정값 × scale = 실제값
- 경험적 캘리브레이션: Phase 1 진단으로 최적값 찾기

### 왜 IMU에 바이어스가 있는가?

**MPU6050 자이로 특성:**
- 제조 편차: ±20°/s offset
- 온도 드리프트: ±0.03°/s/°C
- 전원 불안정: ±5°/s variation

**해결책:**
- 정적 바이어스: imu_calib으로 측정 및 제거
- 온도 보상: 주기적 재캘리브레이션
- 센서 융합: EKF로 Odom과 교차 검증

### 왜 EKF가 IMU를 거부하는가?

**Mahalanobis Distance:**
```python
# EKF는 센서 간 차이를 통계적으로 평가
distance = (sensor1 - sensor2)^2 / covariance

if distance > threshold:
    reject_as_outlier()  # 비정상 데이터로 판단
```

**현재 문제:**
```
Odom: 58°, IMU: 145° → 차이 2.5배
→ Mahalanobis distance >> threshold
→ EKF가 IMU를 outlier로 거부
```

**해결책:**
- angular_scale로 Odom 보정 → 58° → 90°
- IMU 바이어스 제거 → 145° → 90°
- 센서 일치 → EKF가 융합 시작

---

## 📚 관련 문서

- `PHASE1_DIAGNOSIS_GUIDE.md` - Phase 1 진단 상세 가이드
- `IMU_CALIB_GUIDE.md` - imu_calib 사용법
- `GYRO_DRIFT_FIX.md` - 자이로 드리프트 해결
- `SLAM_DUAL_STRATEGY.md` - SLAM 파라미터 최적화
- `IMU_OPTIMIZATION_GUIDE.md` - IMU 데이터 흐름

---

## ✅ 체크리스트

### 즉시 실행 가능한 작업
- [ ] 1. IMU 자이로 바이어스 캘리브레이션 (5분)
- [ ] 2. imu_calib.yaml에 gyro_bias 확인
- [ ] 3. 시스템 재시작 & 드리프트 제거 검증
- [ ] 4. Phase 1 진단 실행 (10분)
- [ ] 5. 최적 angular_scale 계산
- [ ] 6. launch 파일 업데이트
- [ ] 7. EKF-IMU 통합 검증
- [ ] 8. SLAM 회전 정확도 테스트

### 각 단계별 성공 기준
- [ ] IMU: angular_velocity.z ≈ 0 (정지 상태)
- [ ] Odom: 90° 명령 → 88-92° 측정
- [ ] EKF: /odometry/filtered ≈ 90° (IMU 반영)
- [ ] SLAM: 회전 추적 85%+ 정확도

---

**다음 단계: Priority 1부터 순서대로 진행하세요!**

작업 시작 시각: ___________
완료 예상 시각: ___________ (약 42분 후)

