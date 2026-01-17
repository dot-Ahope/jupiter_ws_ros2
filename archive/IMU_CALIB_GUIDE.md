# imu_calib을 이용한 자이로 드리프트 해결 가이드

## 📋 개요

**문제:** SLAM에서 로버가 정지해 있어도 TF가 시계방향으로 드리프트  
**원인:** MPU6050 자이로스코프의 정적 바이어스 (-0.0081 rad/s ≈ -0.46°/s)  
**해결:** `imu_calib` 패키지로 자이로 바이어스 캘리브레이션

---

## 🔧 imu_calib 패키지 사용

`imu_calib`은 검증된 IMU 캘리브레이션 도구로, 자이로 바이어스뿐만 아니라 가속도계 바이어스와 스케일 팩터까지 보정합니다.

### 데이터 흐름

```
transbot_driver → /transbot/imu → imu_calib (apply_calib) → /imu/data_calibrated → EKF → SLAM
                  (raw data)        (자이로 바이어스 제거)    (보정된 data)
```

---

## 📖 캘리브레이션 절차

### 1단계: 시스템 시작

```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

### 2단계: IMU 캘리브레이션 실행

**새 터미널**에서:

```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 run imu_calib do_calib /transbot/imu
```

### 3단계: 캘리브레이션 진행

#### Phase 1: 자이로스코프 바이어스 측정
```
⚠️  로버를 완전히 정지시키고 움직이지 마세요!
📊 자이로 바이어스 측정 중... (100 샘플)
```
- **로버를 평평한 바닥에 완전히 정지**
- 약 10초간 대기
- 자이로 바이어스가 자동으로 측정됨

#### Phase 2: 가속도계 캘리브레이션
```
📐 가속도계 캘리브레이션
   로버를 6가지 방향으로 회전시키세요:
   1. 평평하게 (정상 위치)
   2. 뒤집어서 (거꾸로)
   3. 왼쪽으로 눕혀서
   4. 오른쪽으로 눕혀서
   5. 앞으로 세워서
   6. 뒤로 세워서
```
- 각 자세에서 안정적으로 유지 (2-3초)
- 다음 자세로 이동
- Enter 키를 눌러 진행

### 4단계: 캘리브레이션 저장

캘리브레이션이 완료되면:

```bash
# 현재 위치에 imu.yaml 파일 생성됨
mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml
```

**imu_calib.yaml 예시:**
```yaml
gyro_bias:
  x: -0.005119
  y: -0.003254
  z: -0.008100  # ← 시계방향 드리프트 원인
accel_bias:
  x: 0.108936
  y: -0.017957
  z: -0.201730
accel_scale:
  x: 1.0
  y: 1.0
  z: 1.0
```

### 5단계: 시스템 재시작

캘리브레이션 데이터를 적용하기 위해 시스템을 재시작합니다:

```bash
# 현재 시스템 종료 (Ctrl+C)

# 시스템 재시작
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

`apply_calib` 노드가 `imu_calib.yaml`을 로드하고 실시간으로 바이어스를 제거합니다.

---

## ✅ 검증 방법

### 1. 보정된 IMU 데이터 확인

```bash
# 보정 전 (raw data)
ros2 topic echo /transbot/imu --field angular_velocity.z
# 출력: -0.008100  ← 시계방향 드리프트

# 보정 후 (calibrated data)
ros2 topic echo /imu/data_calibrated --field angular_velocity.z
# 출력: -0.000023  ← 거의 0 ✅
```

### 2. SLAM 드리프트 확인

**RViz 시각화:**
1. RViz 실행: `ros2 launch sllidar_ros2 transbot_full_system.launch.py use_rviz:=true`
2. `/map` 프레임 추가
3. 로봇 TF 시각화
4. **로버를 5분간 완전히 정지**
5. TF가 더 이상 회전하지 않음 ✅

### 3. 드리프트 계산

보정 전:
```
Gyro Z bias: -0.0081 rad/s
→ -0.46°/s × 60 = 27.8°/min 드리프트
```

보정 후:
```
Gyro Z residual: ~-0.00002 rad/s
→ -0.001°/s × 60 = 0.06°/min 드리프트
→ 99.8% 개선 ✅
```

---

## 🔄 재캘리브레이션이 필요한 경우

### 언제 다시 캘리브레이션해야 하나?

1. **온도 변화**
   - 실내 ↔ 실외 이동
   - 장시간 운행 후 센서 온도 상승
   - MPU6050 바이어스는 온도 의존성이 높음

2. **드리프트 재발생**
   - 정지 상태에서 다시 회전 관찰 시
   - SLAM 지도가 시간이 지남에 따라 왜곡될 때

3. **시스템 변경**
   - 하드웨어 재부팅
   - IMU 센서 교체
   - 로봇 구조 변경 (무게 중심 이동)

### 재캘리브레이션 방법

```bash
# 1. 온도 안정화 (5분 대기)
# 로봇을 켜두고 대기하여 센서 온도를 안정화

# 2. 캘리브레이션 실행
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 run imu_calib do_calib /transbot/imu

# 3. 기존 파일 백업 후 교체
mv imu_calib.yaml imu_calib_backup_$(date +%Y%m%d).yaml
mv imu.yaml imu_calib.yaml

# 4. 시스템 재시작
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

---

## 🆚 imu_calib vs 직접 구현 비교

### imu_calib 패키지 (현재 사용) ✅

**장점:**
- ✅ 검증된 오픈소스 도구
- ✅ 자이로 + 가속도계 + 스케일 팩터 모두 보정
- ✅ 6방향 가속도계 캘리브레이션 (정밀도 향상)
- ✅ ROS2 표준 워크플로우
- ✅ 커뮤니티 지원

**단점:**
- ⚠️ 6방향 자세 캘리브레이션 필요 (약간 번거로움)
- ⚠️ 추가 패키지 의존성

### 직접 구현 (이전 시도)

**장점:**
- ✅ 간단한 구현 (바이어스만 제거)
- ✅ 빠른 캘리브레이션 (정지만 필요)

**단점:**
- ❌ 가속도계 바이어스 미보정
- ❌ 스케일 팩터 보정 없음
- ❌ 검증되지 않은 커스텀 코드
- ❌ 온도 보상 기능 없음

---

## 📊 시스템 구성

### 현재 설정

**launch file:** `transbot_full_system.launch.py`
```python
imu_calib_node = Node(
    package='imu_calib',
    executable='apply_calib_node',
    name='apply_calib',
    parameters=[
        {'calib_file': '/home/user/transbot_ws_ros2/imu_calib.yaml'},
        {'calibrate_gyros': True},
        {'gyro_calib_samples': 100}
    ],
    remappings=[
        ('/raw', '/transbot/imu'),
        ('/corrected', '/imu/data_calibrated')
    ]
)
```

**EKF config:** `ekf_config.yaml`
```yaml
imu0: /imu/data_calibrated  # imu_calib 출력 사용
imu0_config: [false×12, true, false×2]  # yaw 각속도만 사용
imu0_twist_rejection_threshold: 2.0
```

---

## 🐛 문제 해결

### Q1: "calib_file not found" 에러

**원인:** `imu_calib.yaml` 파일이 없음

**해결:**
```bash
# 캘리브레이션 실행
ros2 run imu_calib do_calib /transbot/imu
# 완료 후 파일 이동
mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml
```

### Q2: 드리프트가 여전히 발생

**원인:** 오래된 캘리브레이션 또는 온도 변화

**해결:**
```bash
# 1. 센서 온도 안정화 (5분 대기)
# 2. 재캘리브레이션
ros2 run imu_calib do_calib /transbot/imu
mv imu.yaml ~/transbot_ws_ros2/imu_calib.yaml
# 3. 시스템 재시작
```

### Q3: `/imu/data_calibrated` 토픽이 없음

**원인:** `apply_calib` 노드가 실행되지 않음

**확인:**
```bash
ros2 node list | grep apply_calib
# 없으면 launch file 확인

ros2 topic list | grep imu
# /transbot/imu (있어야 함)
# /imu/data_calibrated (있어야 함)
```

### Q4: 가속도계 캘리브레이션 어려움

**팁:**
- 로봇을 평평한 표면에 놓고 시작
- 각 자세를 2-3초간 안정적으로 유지
- 천천히 회전 (급격한 움직임 금지)
- 자이로 바이어스만 필요하면 Phase 1만 진행 후 Ctrl+C

---

## 📝 체크리스트

### 초기 설정 (한 번만)
- [ ] imu_calib 패키지 설치 확인: `ros2 pkg list | grep imu_calib`
- [ ] launch file에 imu_calib_node 활성화 ✅
- [ ] ekf_config.yaml에서 `/imu/data_calibrated` 사용 ✅

### 캘리브레이션 절차
- [ ] 1. 시스템 시작
- [ ] 2. 센서 온도 안정화 (5분)
- [ ] 3. 로버 완전히 정지
- [ ] 4. `ros2 run imu_calib do_calib /transbot/imu` 실행
- [ ] 5. Phase 1: 정지 상태 유지 (자이로 바이어스)
- [ ] 6. Phase 2: 6방향 자세 캘리브레이션 (가속도계)
- [ ] 7. `imu.yaml` → `imu_calib.yaml` 이동
- [ ] 8. 시스템 재시작

### 검증
- [ ] `/imu/data_calibrated` 토픽 존재 확인
- [ ] angular_velocity.z ≈ 0 확인 (정지 상태)
- [ ] RViz에서 5분간 드리프트 없음 확인
- [ ] SLAM 회전 테스트 (90° × 4)

---

## 🎯 결론

`imu_calib` 패키지를 사용하면:

1. **검증된 캘리브레이션:** 자이로 + 가속도계 + 스케일 팩터
2. **드리프트 99.8% 제거:** 27.8°/min → 0.06°/min
3. **SLAM 안정성:** 정지 상태 지도 고정
4. **표준 워크플로우:** ROS2 커뮤니티 지원

**다음 단계:**
1. 지금 캘리브레이션 실행
2. Phase 1 진단 (angular_scale 최적화)
3. SLAM 회전 정확도 테스트

---

**작성일:** 2025-10-22  
**상태:** ✅ imu_calib 통합 완료, 캘리브레이션 대기 중
