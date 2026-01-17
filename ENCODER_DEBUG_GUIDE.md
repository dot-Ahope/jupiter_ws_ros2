# 인코더 데이터 손실 해결 가이드

## 🔴 문제 요약

**증상:**
- **반시계 회전 (CCW)**: IMU 85.58°, Odom 8.41° ← **90% 데이터 손실**
- **시계 회전 (CW)**: IMU 88.85°, Odom 88.84° ← **정상**

**영향:**
- EKF가 잘못된 Odom 데이터를 신뢰하여 융합 결과 부정확
- 누적 오도메트리 오차로 인해 후속 테스트 영향

---

## 🔍 1단계: 즉시 진단 (5분)

### A. 토픽 발행 확인

```bash
# Terminal 1: 시스템 실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# Terminal 2: Odom 토픽 주기 확인
ros2 topic hz /odom_raw
# 예상: 50 Hz (정상)
# 실제: 5-10 Hz (문제)

# Terminal 3: 실시간 모니터링
ros2 topic echo /odom_raw --field pose.pose.orientation
```

### B. 반시계 회전 시 데이터 관찰

```bash
# Terminal 4: 수동 회전 명령
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'

# 동시에 Terminal 3에서 yaw 값 변화 관찰
# 정상: 지속적으로 값 증가
# 문제: 값이 멈추거나 느리게 증가
```

### C. 진단 스크립트 실행

```bash
cd ~/transbot_ws_ros2/src/transbot_nav/scripts

# 기존 진단 스크립트가 있다면 사용
python3 diagnose_encoder.py

# 또는 새로 생성한 스크립트
python3 encoder_quality_test.py
```

---

## 🔧 2단계: 하드웨어 점검 (10분)

### A. 인코더 연결 확인

```bash
# 1. 시스템 종료
sudo killall -9 python3

# 2. 인코더 연결 육안 확인
# - 왼쪽/오른쪽 모터 인코더 케이블
# - 4핀 커넥터 (VCC, GND, A, B)
# - 느슨한 연결 확인

# 3. 저항 측정 (멀티미터)
# VCC-GND: 5V 확인
# A/B 신호: 펄스 발생 시 0-5V 변화
```

### B. 케이블 품질 확인

```bash
# 케이블 흔들어보기 (동작 중)
ros2 topic hz /odom_raw

# 케이블을 약간 움직이면서:
# - Hz가 변동되면: 접촉 불량
# - 안정적이면: 케이블 정상
```

### C. 모터 드라이버 확인

```bash
# transbot_driver 노드 로그
ros2 node info /transbot_driver

# 에러 메시지 확인
ros2 topic echo /rosout | grep -i "error\|warn"
```

---

## 💻 3단계: 소프트웨어 점검 (15분)

### A. transbot_base 노드 로그

```bash
# 노드 재시작 후 로그 캡처
ros2 run transbot_base base_node --ros-args --log-level debug 2>&1 | tee base_node.log

# 다른 터미널에서 회전 테스트
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'

# 로그 분석
grep -i "encoder\|odom\|velocity" base_node.log
```

### B. 토픽 메시지 손실 확인

```bash
# 메시지 수 카운트
ros2 topic hz /wheel_odom
# 또는
ros2 topic hz /transbot/get_vel

# 대역폭 확인
ros2 topic bw /odom_raw
```

### C. CPU 사용률 확인

```bash
# Jetson Nano는 CPU 제한적
htop

# 만약 CPU 100%:
# - 다른 프로세스 종료
# - 카메라 노드 일시 중지
# - SLAM 주기 낮추기
```

---

## 🛠️ 4단계: 구체적 해결책

### 해결책 1: 인코더 신호 안정화 (하드웨어)

```bash
# 1. 풀업 저항 추가 (선택사항)
# A, B 라인에 4.7kΩ 저항을 VCC에 연결

# 2. 케이블 교체
# 더 짧고 품질 좋은 케이블 사용 (< 30cm)

# 3. 접지 강화
# GND 라인 확실히 연결
```

### 해결책 2: 소프트웨어 필터링

transbot_base 노드에 필터 추가:

```python
# base_node 수정 (예시)
class TransbotBase:
    def __init__(self):
        self.last_encoder_time = time.time()
        self.min_encoder_interval = 0.01  # 100Hz 최대
        
    def encoder_callback(self, msg):
        now = time.time()
        if now - self.last_encoder_time < self.min_encoder_interval:
            return  # 너무 빠른 메시지 무시
        
        # 데이터 유효성 검사
        if abs(msg.angular_velocity) > 10.0:  # rad/s
            self.get_logger().warn('비정상적인 각속도 무시')
            return
        
        self.last_encoder_time = now
        self.process_encoder(msg)
```

### 해결책 3: angular_scale 방향별 보정

```python
# transbot_full_system.launch.py
parameters=[{
    'linear_scale': 1.2,
    'angular_scale': 1.8819,      # 기본값
    'angular_scale_ccw': 10.18,   # 반시계 보정 (IMU/Odom 비율)
    'angular_scale_cw': 1.0002,   # 시계 보정
    'use_directional_scale': True
}]
```

```python
# base_node.cpp 수정 (예시)
double angular_scale = angular_scale_;
if (use_directional_scale_) {
    if (cmd_vel.angular.z > 0) {
        angular_scale = angular_scale_ccw_;
    } else if (cmd_vel.angular.z < 0) {
        angular_scale = angular_scale_cw_;
    }
}
```

### 해결책 4: 인코더 펄스 카운트 확인

```bash
# 원시 인코더 값 확인
ros2 topic echo /transbot/get_vel

# 또는 Serial 직접 읽기
sudo minicom -D /dev/ttyUSB0 -b 115200
```

### 해결책 5: 펌웨어 업데이트

```bash
# Transbot STM32 펌웨어 확인
cd ~/Transbot/Samples
ls -la

# 펌웨어 재업로드 (필요시)
# Arduino IDE 또는 STM32CubeProgrammer 사용
```

---

## 🧪 5단계: 검증 테스트

### A. 단방향 테스트

```bash
cd ~/transbot_ws_ros2/src/transbot_nav/scripts

# 반시계만 테스트
python3 -c "
import rclpy
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node('test')
pub = node.create_publisher(Twist, 'cmd_vel', 10)
twist = Twist()
twist.angular.z = 0.3
for _ in range(100):
    pub.publish(twist)
    rclpy.spin_once(node, timeout_sec=0.05)
"

# Odom 확인
ros2 topic echo /odom_raw --field pose.pose.orientation.z
```

### B. 전체 재테스트

```bash
# 수정 후 전체 테스트
python3 ekf_comparison_test.py

# 기대 결과:
# 테스트 1 (IMU CCW): Odom 80-90° (정상)
# 테스트 2 (IMU CW):  Odom 80-90° (정상)
```

---

## 📊 6단계: 근본 원인별 해결 매트릭스

| 증상 | 가능한 원인 | 해결책 | 우선순위 |
|------|------------|--------|---------|
| CCW만 데이터 손실 | 왼쪽 인코더 불량 | 케이블/커넥터 재연결 | ⭐⭐⭐ |
| 간헐적 손실 | 접촉 불량 | 케이블 교체/고정 | ⭐⭐⭐ |
| 양방향 손실 | CPU 과부하 | 불필요한 노드 종료 | ⭐⭐ |
| 속도 의존 손실 | 펄스 처리 한계 | 펌웨어 타이머 조정 | ⭐⭐ |
| 시작 시만 손실 | 초기화 타이밍 | 노드 시작 순서 조정 | ⭐ |
| 불규칙 간격 | ROS2 메시지 드롭 | QoS 설정 조정 | ⭐ |

---

## 🎯 즉시 실행할 명령어

```bash
# 1. 현재 상태 스냅샷
ros2 topic hz /odom_raw &
ros2 topic hz /imu/data_calibrated &
ros2 topic hz /transbot/get_vel &

# 2. 로그 수집
ros2 bag record -a -o encoder_debug

# 3. 회전 테스트
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'
# 5초 후
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'

# 4. 분석
ros2 bag info encoder_debug/
ros2 bag play encoder_debug/ --topics /odom_raw
```

---

## 💡 임시 회피 방법

문제 해결 전까지 사용 가능한 방법:

### 방법 1: IMU 우선 융합

```yaml
# ekf_config.yaml 수정
odom0: /odom_raw
odom0_config: [false, false, false,  # x, y, z
               false, false, false,  # roll, pitch, yaw
               false, false, false,  # vx, vy, vz
               false, false, false,  # vroll, vpitch, vyaw
               false, false, false]  # ax, ay, az

imu0: /imu/data_calibrated
imu0_config: [false, false, false,
              false, false, true,   # yaw 사용
              false, false, false,
              false, false, true,   # vyaw 사용
              false, false, false]
```

### 방법 2: 시계방향만 사용

네비게이션에서 반시계 회전 대신 시계방향 큰 회전 사용:
- 90° CCW → 270° CW로 대체

### 방법 3: SLAM 의존

Odom 대신 SLAM loop closure에 의존:

```yaml
# slam_params.yaml
mode: mapping
use_scan_matching: true
use_loop_closure: true
minimum_travel_distance: 0.01  # 매우 짧게
```

---

## 📝 체크리스트

- [ ] 1단계: 토픽 Hz 확인 (`ros2 topic hz /odom_raw`)
- [ ] 2단계: 케이블 육안 점검
- [ ] 3단계: 진단 스크립트 실행
- [ ] 4단계: 로그 수집 (`ros2 bag record`)
- [ ] 5단계: 하드웨어 테스트 (케이블 교체)
- [ ] 6단계: 소프트웨어 필터 추가
- [ ] 7단계: 재테스트 (`ekf_comparison_test.py`)
- [ ] 8단계: 결과 문서화

---

## 🚨 긴급 연락처

**Yahboom 기술 지원:**
- 이메일: service@yahboom.net
- 포럼: http://www.yahboom.net/forum

**Transbot GitHub:**
- https://github.com/yahboom-robotics

**ROS2 Discourse:**
- https://discourse.ros.org/

---

**작성일:** 2025-10-31  
**버전:** 1.0  
**상태:** 진단 중
