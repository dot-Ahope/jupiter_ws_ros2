# 🔄 SLAM 회전 추적 문제 진단 가이드

## 🚨 **문제 상황**
- ✅ **선형 이동 (x, y)**: 정확히 추적됨
- ❌ **회전 (yaw)**: 부정확하게 추적됨

---

## 🔍 **진단 체크리스트**

### **1단계: 오도메트리 각속도 확인**

#### ✅ **A. /odom_raw 토픽 확인**

```bash
# 터미널 1: SLAM 시스템 실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 터미널 2: 오도메트리 모니터링
ros2 topic echo /odom_raw
```

**제자리 회전 테스트:**
```bash
# 터미널 3: 반시계 회전 명령
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"

# 10초 후 정지
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**확인 사항:**
```yaml
/odom_raw 출력에서:
  pose.pose.orientation: 
    # quaternion을 yaw로 변환했을 때
    # 실제 회전량과 일치하는가?
  
  twist.twist.angular.z: 
    # 각속도 값이 명령값(0.5)과 유사한가?
    # angular_scale = 1.5625 적용 후의 값
```

**문제 가능성:**
- ❌ yaw 변화량이 실제보다 작음 → `angular_scale` 부족
- ❌ yaw 변화량이 실제보다 큼 → `angular_scale` 과다

---

#### ✅ **B. angular_scale 검증**

**현재 설정:**
```python
# transbot_full_system.launch.py Line 147
'angular_scale': 1.5625
```

**캘리브레이션 실행:**
```bash
cd ~/transbot_ws_ros2

# Phase 1: 초기 측정
python3 odom_based_angular_calibration.py --phase 1

# Phase 2: 정밀 측정 (Phase 1 결과 사용)
python3 odom_based_angular_calibration.py --phase 2 --scale <Phase1_결과>
```

**측정 지표:**
```
목표:     360°
Odom:     XXX° (angular_scale 적용 후)
IMU적분:  YYY° (실제 회전량)

angular_scale = YYY / XXX (이상적으로 1.0에 가까워야 함)
```

---

### **2단계: IMU 각속도 확인**

#### ✅ **C. /imu/data_calibrated 확인**

```bash
# IMU 각속도 모니터링
ros2 topic echo /imu/data_calibrated | grep angular_velocity -A 3
```

**제자리 회전 중 확인:**
```yaml
angular_velocity:
  x: 0.0      # roll rate (거의 0이어야 함)
  y: 0.0      # pitch rate (거의 0이어야 함)
  z: 0.XXX    # yaw rate (회전 중 비영값)
```

**문제 가능성:**
- ❌ `z` 값이 0에 가까움 → IMU 자이로 오류 또는 교정 문제
- ❌ `z` 값이 비정상적으로 큼 → `gyro_ratio` 오류

**IMU 교정 확인:**
```bash
cat ~/transbot_ws_ros2/imu_calib.yaml
```

```yaml
# 자이로 바이어스 확인
gyros:
  x: XXX  # 0에 가까워야 함
  y: XXX
  z: XXX  # yaw 바이어스
```

**재교정 필요 시:**
```bash
# 로봇을 완전 정지 상태에서
ros2 run imu_calib do_calib --ros-args -p calibrate_gyros:=true
```

---

### **3단계: EKF 센서 융합 확인**

#### ✅ **D. EKF 설정 검토**

**파일 위치:**
```
~/transbot_ws_ros2/src/sllidar_ros2/config/ekf_config.yaml
```

**핵심 파라미터 확인:**

```yaml
# 1. IMU 각속도 설정
imu0: /imu/data_calibrated
imu0_config: 
  # ... 
  - [false, false, true]   # yaw 각속도 활성화 ✅
  
imu0_differential: true     # 변화량 사용 ✅
imu0_relative: true

# 2. 오도메트리 각속도 설정
odom0: /odom_raw
odom0_config:
  # ...
  - [false, false, true]   # yaw 각속도 활성화 ✅
  
odom0_differential: true    # 변화량 사용 ✅
```

**문제 체크:**
- ❌ `imu0_config` 12번째 원소가 `false` → IMU 각속도 미사용
- ❌ `odom0_config` 12번째 원소가 `false` → Odom 각속도 미사용

**EKF 출력 확인:**
```bash
ros2 topic echo /odometry/filtered
```

**제자리 회전 시:**
```yaml
twist.twist.angular.z: XXX  # 회전 속도가 명령값과 유사한가?
pose.pose.orientation: ...  # yaw가 실제 회전과 일치하는가?
```

---

### **4단계: SLAM 스캔 매칭 확인**

#### ✅ **E. SLAM Toolbox 파라미터**

**파일 위치:**
```
~/transbot_ws_ros2/src/sllidar_ros2/config/slam_params.yaml
```

**회전 추적 관련 파라미터:**

```yaml
# 1. 최소 이동 임계값
minimum_travel_distance: 0.05  # 선형 이동 (5cm)
minimum_travel_heading: 0.05   # 회전 각도 (약 2.9°) ⭐

# 2. 스캔 매칭 강도
link_match_minimum_response_fine: 0.15    # 매칭 응답 임계값
link_scan_maximum_distance: 2.0           # 매칭 거리

# 3. 각도 관련
angle_variance_penalty: 1.0               # 각도 분산 페널티 ⭐
minimum_angle_penalty: 0.95               # 각도 페널티 ⭐
coarse_search_angle_offset: 0.349         # 각도 검색 범위 (±20°)
fine_search_angle_offset: 0.00349         # 정밀 각도 검색 (±0.2°)
```

**문제 가능성:**

**A. `minimum_travel_heading` 너무 큼**
```yaml
# 현재: 0.05 rad = 2.9°
# 만약 로봇이 2.9° 미만 회전하면 SLAM이 무시함

# 해결: 값을 줄임
minimum_travel_heading: 0.02  # 1.15°
```

**B. 각도 페널티 너무 높음**
```yaml
# angle_variance_penalty가 높으면 회전 변화를 덜 신뢰
angle_variance_penalty: 0.8   # 1.0 → 0.8로 감소

# minimum_angle_penalty가 높으면 각도 변화 억제
minimum_angle_penalty: 0.8    # 0.95 → 0.8로 감소
```

**C. 스캔 매칭 응답 임계값 너무 높음**
```yaml
# 회전 시 스캔 매칭이 어려우면 응답값이 낮아짐
# 임계값이 높으면 회전을 거부할 수 있음

link_match_minimum_response_fine: 0.1    # 0.15 → 0.1
loop_match_minimum_response_fine: 0.4    # 0.5 → 0.4
```

---

#### ✅ **F. SLAM 진단 메시지 확인**

```bash
# SLAM 실행 중 터미널 출력 확인
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 회전 중 다음 메시지 주의:
# - "Scan matching failed" → 스캔 매칭 실패
# - "Response too low" → 응답값 부족
# - "Angle penalty too high" → 각도 페널티 과다
```

**디버그 로깅 활성화:**
```yaml
# slam_params.yaml
debug_logging: true  # false → true
```

---

### **5단계: LiDAR 스캔 품질 확인**

#### ✅ **G. 스캔 데이터 품질**

```bash
# 스캔 토픽 확인
ros2 topic echo /scan | head -50
```

**확인 사항:**
```yaml
ranges: [...]
  # 너무 많은 'inf' 또는 'nan' 값 → 스캔 품질 나쁨
  # 회전 시 주변 환경 특징이 충분한가?
  
angle_min: -1.57     # -90°
angle_max: 1.57      # +90°
angle_increment: XXX # 각도 해상도
```

**문제 가능성:**
- ❌ 넓고 텅 빈 공간 → 특징점 부족 → 회전 추적 어려움
- ❌ 스캔 범위 좁음 → 회전 시 매칭 실패

**개선 방법:**
```yaml
# transbot_full_system.launch.py의 sllidar_node
parameters:
  angle_min: -3.14159  # -180° (전체 범위)
  angle_max: 3.14159   # +180°
  range_max: 12.0      # 최대 거리 확대
```

---

### **6단계: TF 트리 확인**

#### ✅ **H. TF 변환 체크**

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# 생성된 frames.pdf 확인
evince frames.pdf
```

**확인 사항:**
```
map -> odom -> base_footprint

1. map -> odom (SLAM이 발행)
   - 회전 시 업데이트되는가?
   
2. odom -> base_footprint (EKF가 발행)
   - 회전 중 각도 변화가 보이는가?
```

**TF 실시간 모니터링:**
```bash
# odom -> base_footprint 변환 확인
ros2 run tf2_ros tf2_echo odom base_footprint

# 회전 중 출력:
- Translation: [x, y, z] (거의 변화 없음)
- Rotation: quaternion (z, w 값 변화) ⭐
```

---

## 🛠️ **문제별 해결 방법**

### **문제 1: Odom 각속도 부정확**

**증상:**
- `/odom_raw`의 yaw 변화가 실제와 다름

**해결:**
```bash
# angular_scale 재캘리브레이션
python3 odom_based_angular_calibration.py --phase 2
```

**적용:**
```python
# transbot_full_system.launch.py
'angular_scale': <새로운_값>,  # 1.5625 → X.XXXX
```

---

### **문제 2: IMU 각속도 이상**

**증상:**
- `/imu/data_calibrated`의 angular_velocity.z가 0 또는 비정상

**해결:**
```bash
# IMU 재교정
ros2 run imu_calib do_calib \
  --ros-args -p calibrate_gyros:=true -p gyro_calib_samples:=200

# 생성된 imu_calib.yaml 확인
cat /tmp/imu_calib.yaml

# 복사
cp /tmp/imu_calib.yaml ~/transbot_ws_ros2/imu_calib.yaml
```

---

### **문제 3: EKF가 각속도 무시**

**증상:**
- `/odometry/filtered`가 회전을 제대로 반영 안 함

**해결:**
```yaml
# ekf_config.yaml 수정

# IMU 각속도 rejection threshold 증가
imu0_twist_rejection_threshold: 3.0  # 2.0 → 3.0

# 또는 프로세스 노이즈 조정 (yaw rate)
process_noise_covariance:
  # 12번째 요소 (yaw rate) 감소
  [..., 0.005, ...]  # 0.01 → 0.005
```

---

### **문제 4: SLAM 회전 임계값 문제**

**증상:**
- 작은 회전을 SLAM이 무시

**해결:**
```yaml
# slam_params.yaml 수정

minimum_travel_heading: 0.02           # 0.05 → 0.02 (1.15°)
angle_variance_penalty: 0.8            # 1.0 → 0.8
minimum_angle_penalty: 0.8             # 0.95 → 0.8
link_match_minimum_response_fine: 0.1  # 0.15 → 0.1
```

**재빌드:**
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2
source install/setup.bash
```

---

### **문제 5: 환경 특징 부족**

**증상:**
- 넓은 공간에서 회전 추적 실패

**해결:**
1. **좁은 공간에서 테스트**
   - 벽, 장애물이 많은 환경
   - 코너, 문틀 등 특징적인 구조

2. **LiDAR 범위 확대**
   ```yaml
   # sllidar_node 파라미터
   angle_min: -3.14159  # 전체 360°
   angle_max: 3.14159
   range_max: 12.0
   ```

3. **스캔 주파수 증가**
   ```yaml
   scan_frequency: 7.0  # 5.0 → 7.0 (더 자주 스캔)
   throttle_scans: 1    # 모든 스캔 사용
   ```

---

## 🧪 **체계적 테스트 방법**

### **테스트 1: 제자리 회전 (Pure Rotation)**

```bash
# 1. RViz 실행
rviz2

# 2. 제자리 360° 회전
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.3}}"

# 3. RViz에서 확인:
# - /odom_raw (빨강): 오도메트리 궤적
# - /odometry/filtered (파랑): EKF 궤적
# - map (초록): SLAM 보정된 위치

# 4. 10초 후 정지
Ctrl+C
```

**기대 결과:**
- 로봇이 제자리에서 회전
- RViz에서 로봇 방향(화살표)이 따라 회전

**문제 발견:**
- 방향 화살표가 회전 안 함 → Odom 또는 EKF 문제
- 방향은 회전하지만 위치가 이동 → SLAM 문제

---

### **테스트 2: 정사각형 경로 (Square Path)**

```bash
# 정사각형 그리기 (1m x 1m)
# 직진 → 90° 회전 반복

# 스크립트 작성
cat > square_test.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareTest(Node):
    def __init__(self):
        super().__init__('square_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move_forward(self, distance=1.0, speed=0.2):
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed
        
        start = time.time()
        while (time.time() - start) < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        
        self.stop()
        
    def turn_90(self, angular_speed=0.5):
        twist = Twist()
        twist.angular.z = angular_speed
        
        # 90° = π/2 rad
        # duration = (π/2) / angular_speed
        duration = 1.57 / angular_speed
        
        start = time.time()
        while (time.time() - start) < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        
        self.stop()
        
    def stop(self):
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
    
    def run_square(self):
        print("Starting square test...")
        for i in range(4):
            print(f"Side {i+1}: Moving forward")
            self.move_forward()
            time.sleep(1)
            
            print(f"Corner {i+1}: Turning 90°")
            self.turn_90()
            time.sleep(1)
        
        print("Square completed!")

def main():
    rclpy.init()
    node = SquareTest()
    node.run_square()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x square_test.py
python3 square_test.py
```

**기대 결과:**
- RViz에서 정사각형 경로 그려짐
- 시작점과 끝점이 일치

**문제 발견:**
- 경로가 사각형이 아님 → 회전 각도 부정확
- 끝점이 시작점과 다름 → 누적 오차

---

## 📊 **진단 플로우차트**

```
회전 추적 문제
    │
    ├─ 1. /odom_raw yaw 확인
    │   ├─ ❌ 부정확 → angular_scale 재캘리브레이션
    │   └─ ✅ 정확 → 다음 단계
    │
    ├─ 2. /imu/data_calibrated 확인
    │   ├─ ❌ angular_velocity.z 이상 → IMU 재교정
    │   └─ ✅ 정상 → 다음 단계
    │
    ├─ 3. /odometry/filtered 확인
    │   ├─ ❌ yaw 부정확 → EKF 설정 조정
    │   └─ ✅ 정확 → 다음 단계
    │
    ├─ 4. SLAM map->odom TF 확인
    │   ├─ ❌ 회전 반영 안 됨 → SLAM 파라미터 조정
    │   └─ ✅ 반영됨 → 환경 문제
    │
    └─ 5. 환경 및 LiDAR 확인
        ├─ 특징점 부족 → 좁은 공간 테스트
        └─ 스캔 품질 나쁨 → LiDAR 설정 조정
```

---

## 🎯 **우선 확인 순서**

### **즉시 확인 (5분):**
1. ✅ `/odom_raw` 토픽 모니터링하며 제자리 회전
2. ✅ `/imu/data_calibrated` angular_velocity.z 확인
3. ✅ RViz에서 로봇 방향 화살표 관찰

### **파라미터 검토 (10분):**
4. ✅ `angular_scale` 값 확인 (1.5625)
5. ✅ `ekf_config.yaml` IMU/Odom 각속도 활성화 확인
6. ✅ `slam_params.yaml` `minimum_travel_heading` 확인

### **캘리브레이션 (30분):**
7. ✅ `python3 odom_based_angular_calibration.py --phase 2`
8. ✅ 새로운 `angular_scale` 적용 및 테스트

### **SLAM 튜닝 (필요 시):**
9. ✅ `angle_variance_penalty` 감소
10. ✅ `minimum_travel_heading` 감소
11. ✅ 스캔 매칭 임계값 조정

---

## 📝 **체크리스트**

실행 후 체크:
```
□ /odom_raw yaw가 실제 회전과 일치하는가?
□ /imu/data_calibrated angular_velocity.z가 비영인가?
□ /odometry/filtered가 회전을 반영하는가?
□ RViz에서 로봇 방향이 실제와 일치하는가?
□ map -> odom TF가 회전 시 업데이트되는가?
□ 제자리 회전 후 시작 위치로 돌아오는가?
□ 정사각형 경로가 닫히는가?
```

---

## 🚀 **빠른 해결책 (가장 가능성 높은 문제)**

```bash
# 1. angular_scale 재캘리브레이션 (가장 중요!)
cd ~/transbot_ws_ros2
python3 odom_based_angular_calibration.py --phase 2 --scale 1.56

# 2. 결과 적용
# transbot_full_system.launch.py 수정
# 'angular_scale': <새_값>

# 3. SLAM 파라미터 조정
# slam_params.yaml 수정
minimum_travel_heading: 0.02  # 0.05 → 0.02

# 4. 재빌드 및 테스트
colcon build --packages-select sllidar_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```
