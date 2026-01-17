# 90도 회전 테스트 가이드

## 목적
로버가 90도 회전할 때 오도메트리 **데이터**와 **물리적 동작**이 일치하는지 확인

## 테스트 내용

### 확인 사항
1. **회전 방향 일치**: 명령한 방향(좌/우)과 실제 회전 방향이 같은가?
2. **회전 각도 정확도**: 90도 명령 시 실제로 90도 회전하는가?
3. **센서 데이터 일관성**: 휠 오도메트리와 EKF 융합 결과가 합리적인가?
4. **ROS 표준 준수**: 양수 각속도 = 반시계방향(좌회전)인가?

## 사용 방법

### 1단계: 시스템 실행

**터미널 1 - 메인 시스템 실행:**
```bash
cd /home/user/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

시스템이 완전히 초기화될 때까지 대기 (약 10초)

### 2단계: 테스트 실행

**터미널 2 - 90도 회전 테스트:**
```bash
cd /home/user/transbot_ws_ros2
source install/setup.bash
python3 test_90degree_rotation.py
```

### 3단계: 결과 확인

테스트 스크립트가 자동으로:
1. 초기 각도 기록
2. 90도 반시계방향 회전 명령 (좌회전)
3. 실시간 각도 변화 모니터링
4. 최종 결과 분석 및 평가

## 출력 결과 해석

### 정상 결과 예시

```
[최종 결과 분석]
======================================================================

[목표]
  회전 각도:   90.00°
  회전 방향: 반시계방향 (좌회전)
  ROS 규약:  양수 = 반시계방향(CCW, 좌회전), 음수 = 시계방향(CW, 우회전)

──────────────────────────────────────────────────────────────────────

[휠 오도메트리] /odom_raw
  초기 Yaw:      0.00°
  최종 Yaw:     92.30°
  실제 회전량:  92.30°
  오차:          2.30° (2.6%)
  회전 방향:   반시계방향 (좌회전)
  방향 일치:   ✓ 일치

──────────────────────────────────────────────────────────────────────

[융합 오도메트리] /odometry/filtered (EKF)
  초기 Yaw:      0.00°
  최종 Yaw:     89.80°
  실제 회전량:  89.80°
  오차:         -0.20° (0.2%)
  회전 방향:   반시계방향 (좌회전)
  방향 일치:   ✓ 일치

──────────────────────────────────────────────────────────────────────

[종합 평가]

✓ 회전 방향: 데이터와 물리적 동작이 일치합니다.
✓ 휠 오도메트리 정확도: 우수 (오차 2.30°)
✓ 융합 오도메트리 정확도: 우수 (오차 0.20°)
```

### 문제 발견 시

#### 문제 1: 회전 방향 불일치
```
✗ 회전 방향: 데이터와 물리적 동작이 불일치합니다! (심각)
```

**원인**:
- 모터 배선 반대로 연결
- 좌/우 모터 번호 반대
- cmd_vel_callback 로직 오류

**해결**:
```python
# transbot_driver.py 확인
left_speed = int(base_speed - turn_speed)   # 이게 맞는지?
right_speed = int(base_speed + turn_speed)  # 이게 맞는지?

# 또는 모터 번호 확인
self.bot.set_motor(1, left_speed)   # 1번이 왼쪽?
self.bot.set_motor(2, right_speed)  # 2번이 오른쪽?
```

#### 문제 2: 오차가 큰 경우 (15도 이상)
```
✗ 휠 오도메트리 정확도: 불량 (오차 25.30°)
```

**원인**:
- 휠 직경 캘리브레이션 필요
- 바닥 슬립
- 배터리 전압 부족

**해결**:
1. 캘리브레이션 실행:
   ```bash
   ros2 run transbot_bringup calibrate_angular
   ```

2. 바닥 상태 확인:
   - 미끄러운 바닥 피하기
   - 카펫이나 고무 매트 사용

3. 배터리 확인:
   ```bash
   ros2 topic echo /transbot/battery_voltage
   ```
   → 11.5V 이하면 충전

## 추가 테스트

### 시계방향(우회전) 테스트

스크립트 수정:
```python
# test_90degree_rotation.py 마지막 부분
tester.run_rotation_test(target_angle_deg=-90.0, angular_speed=-0.3)  # 우회전
```

### 360도 회전 테스트

```python
tester.run_rotation_test(target_angle_deg=360.0, angular_speed=0.5)  # 한 바퀴
```

### 반복 테스트

```python
for i in range(4):  # 4번 반복 (총 360도)
    tester.run_rotation_test(target_angle_deg=90.0, angular_speed=0.3)
    time.sleep(2.0)  # 안정화
```

## 데이터 분석

### CSV 파일 생성

테스트 후 자동으로 생성됨:
```
/home/user/transbot_ws_ros2/rotation_test_20251016_143025.csv
```

### CSV 파일 구조
```csv
time,source,yaw_rad,yaw_deg,angular_z_rad_s
0.000,odom_raw,0.000000,0.00,0.000000
0.100,odom_raw,0.030000,1.72,0.300000
0.100,odom_filtered,0.029500,1.69,0.295000
0.100,imu,,,0.305000
...
```

### Python으로 그래프 그리기

```python
import pandas as pd
import matplotlib.pyplot as plt

# 데이터 로드
df = pd.read_csv('rotation_test_20251016_143025.csv')

# 시간에 따른 yaw 각도 변화
plt.figure(figsize=(12, 6))

odom_raw = df[df['source'] == 'odom_raw']
odom_filtered = df[df['source'] == 'odom_filtered']

plt.plot(odom_raw['time'], odom_raw['yaw_deg'], label='Wheel Odom', linewidth=2)
plt.plot(odom_filtered['time'], odom_filtered['yaw_deg'], label='EKF Filtered', linewidth=2)
plt.axhline(y=90, color='r', linestyle='--', label='Target (90°)')

plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (degrees)')
plt.title('90° Rotation Test - Yaw Angle vs Time')
plt.legend()
plt.grid(True)
plt.savefig('rotation_test_yaw.png', dpi=300)
plt.show()

# 각속도 비교
plt.figure(figsize=(12, 6))

imu = df[df['source'] == 'imu']
plt.plot(odom_raw['time'], odom_raw['angular_z_rad_s'], label='Wheel Angular Vel', alpha=0.7)
plt.plot(odom_filtered['time'], odom_filtered['angular_z_rad_s'], label='EKF Angular Vel', alpha=0.7)
plt.plot(imu['time'], imu['angular_z_rad_s'], label='IMU Angular Vel', alpha=0.5)

plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('90° Rotation Test - Angular Velocity vs Time')
plt.legend()
plt.grid(True)
plt.savefig('rotation_test_angular_vel.png', dpi=300)
plt.show()
```

## 예상 결과

### 정상 동작
- ✓ 회전 방향 일치 (명령과 실제가 같음)
- ✓ 오차 10도 이하
- ✓ 휠 오도메트리와 EKF 결과가 비슷함
- ✓ IMU 각속도가 명령값(0.3 rad/s)과 유사

### EKF 융합 효과
- 휠 오도메트리: 약간 오버슈트 가능 (92~95도)
- EKF 융합: 더 정확 (88~92도)
- IMU가 휠 오도메트리의 오차를 보정

## 문제 해결

### 로버가 움직이지 않음
```bash
# 모터 테스트
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

### 토픽이 수신되지 않음
```bash
# 토픽 확인
ros2 topic list
ros2 topic hz /odom_raw
ros2 topic hz /odometry/filtered
ros2 topic hz /imu/data_filtered
```

### 오도메트리 값이 이상함
```bash
# 오도메트리 리셋 (노드 재시작)
# Ctrl+C로 중단 후 다시 실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

## 성공 기준

| 항목 | 우수 | 양호 | 불량 |
|------|------|------|------|
| 회전 방향 일치 | ✓ 일치 | ✓ 일치 | ✗ 불일치 |
| 오차 (휠) | < 5° | < 15° | ≥ 15° |
| 오차 (EKF) | < 3° | < 10° | ≥ 10° |
| 재현성 | 3회 모두 일관 | 2회 일관 | 불일관 |

## 다음 단계

### 테스트 통과 시
1. ✓ 기본 회전 기능 정상
2. SLAM 성능 테스트 진행
3. 복잡한 경로 주행 테스트

### 테스트 실패 시
1. 문제 원인 분석 (위 문제 해결 참고)
2. 캘리브레이션 수행
3. 하드웨어 점검 (모터, 배선, 배터리)
4. 재테스트

## 관련 파일
- `test_90degree_rotation.py`: 이 테스트 스크립트
- `ekf_config.yaml`: EKF 설정
- `transbot_driver.py`: 모터 제어 로직
- `base.cpp`: 오도메트리 계산
- `EKF_ANGULAR_VELOCITY_FIX.md`: 각속도 융합 설정 문서
