# Odometry와 IMU 각도 정렬 가이드

> **작성일:** 2025-10-31  
> **목적:** /odom_raw와 /imu/data의 각도(yaw)를 일치시키고 실제 물리적 환경과 정렬

---

## 🎯 목표

1. **센서 간 일치**: odom_raw의 yaw와 IMU의 yaw가 같은 방향을 가리킴
2. **물리적 정렬**: 로봇이 북쪽을 향할 때 두 센서 모두 yaw = 0
3. **EKF 융합 최적화**: 센서 정렬로 EKF가 올바른 융합 수행

---

## 📋 목차
1. [문제 진단](#문제-진단)
2. [각도 오차 측정](#각도-오차-측정)
3. [IMU 정렬 방법](#imu-정렬-방법)
4. [Odometry 정렬 방법](#odometry-정렬-방법)
5. [물리적 환경 정렬](#물리적-환경-정렬)
6. [검증 및 테스트](#검증-및-테스트)

---

## 1. 문제 진단

### 1.1 현재 상태 확인

#### Step 1: EKF 비교 테스트 실행

**기존 검증된 스크립트 사용:**
```bash
# 시스템 실행 (터미널 1)
ros2 launch transbot_nav transbot_full_system.launch.py

# EKF 비교 테스트 실행 (터미널 2)
cd ~/transbot_ws_ros2/src/transbot_nav/scripts
python3 ekf_comparison_test.py
```

이 스크립트는 다음을 자동으로 수행합니다:
- IMU 기준 90도 회전 (반시계/시계)
- Odom 기준 90도 회전 (반시계/시계)
- 각 센서의 측정값 비교
- EKF 융합 결과 분석
- Angular scale 자동 계산

#### Step 2: 수동 각도 확인 (필요 시)
**수동 토픽 확인:**
```bash
# 터미널 1: odom_raw 각도 확인
ros2 topic echo /odom_raw | grep -A 3 "orientation:"

# 터미널 2: IMU 각도 확인  
ros2 topic echo /imu/data_calibrated | grep -A 3 "orientation:"

# 터미널 3: EKF filtered odometry 확인
ros2 topic echo /odometry/filtered | grep -A 3 "orientation:"
```

---

## 2. 각도 오차 측정

### 2.1 정적 오프셋 측정

로봇을 **정지** 상태에서:

```bash
# 5초간 데이터 수집
python3 compare_sensor_angles.py
# 출력 예시:
# Odom yaw:   15.32°
# IMU yaw:    12.78°
# 차이:       2.54°
```

**분석:**
- **일정한 차이** (예: 항상 ~2.5°) → **정적 오프셋** 존재
- **변동하는 차이** (예: -5° ~ +10°) → **동적 스케일** 문제

### 2.2 동적 오차 측정 (회전 테스트)

```bash
# 360도 회전 테스트
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
# 5초 후 정지
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.0}}"
```

**데이터 기록:**
```bash
# 회전 중 각도 변화 기록
python3 compare_sensor_angles.py > angle_test_rotation.log
```

**분석:**
| 시간 | Odom Yaw | IMU Yaw | 차이 |
|------|----------|---------|------|
| 0초  | 0.0°     | 0.0°    | 0.0° |
| 1초  | 30.5°    | 28.7°   | 1.8° |
| 2초  | 61.2°    | 57.3°   | 3.9° |
| 3초  | 92.0°    | 85.9°   | 6.1° |

**결론:**
- 차이가 **증가**하면 → Odom 스케일 문제 (angular_scale 재조정 필요)
- 차이가 **일정**하면 → 단순 오프셋 (yaw_offset 추가)

---

## 3. IMU 정렬 방법

### 3.1 IMU 프레임 방향 확인

**IMU 장착 방향:**
```
로봇 전방(+X) = IMU의 어느 축?
로봇 좌측(+Y) = IMU의 어느 축?
로봇 상단(+Z) = IMU의 어느 축?
```

**확인 방법:**
```bash
# IMU 데이터 확인 (로봇 정지)
ros2 topic echo /imu/data | grep -A 3 "angular_velocity:"

# 로봇을 손으로 들어서:
# 1. 앞뒤로 기울이기 (pitch) → 어느 축 변화?
# 2. 좌우로 기울이기 (roll) → 어느 축 변화?
# 3. 좌우로 회전 (yaw) → 어느 축 변화? ⭐
```

### 3.2 IMU Yaw 오프셋 보정

**방법 A: imu_calib_node 수정** (권장)

IMU 캘리브레이션 노드에 yaw 오프셋 추가:

```python
# transbot_base/transbot_base/imu_calib.py 수정
class ImuCalib(Node):
    def __init__(self):
        super().__init__('imu_calib_node')
        
        # 기존 파라미터
        self.declare_parameter('gyro_bias_x', 0.0)
        self.declare_parameter('gyro_bias_y', 0.0)
        self.declare_parameter('gyro_bias_z', 0.0)
        
        # ⭐ 새로운 파라미터: Yaw 오프셋
        self.declare_parameter('yaw_offset', 0.0)  # 라디안 단위
        
        self.yaw_offset = self.get_parameter('yaw_offset').value
        
        # ... (기존 코드)
    
    def imu_callback(self, msg):
        # 기존 바이어스 보정
        corrected = Imu()
        # ... (angular_velocity 보정)
        
        # ⭐ Yaw 오프셋 적용 (Quaternion 회전)
        if abs(self.yaw_offset) > 0.001:
            from tf_transformations import quaternion_from_euler, quaternion_multiply
            
            # 원본 quaternion
            q_orig = [msg.orientation.x, msg.orientation.y, 
                      msg.orientation.z, msg.orientation.w]
            
            # Yaw 오프셋 quaternion
            q_offset = quaternion_from_euler(0, 0, self.yaw_offset)
            
            # 곱셈으로 회전 적용
            q_corrected = quaternion_multiply(q_orig, q_offset)
            
            corrected.orientation.x = q_corrected[0]
            corrected.orientation.y = q_corrected[1]
            corrected.orientation.z = q_corrected[2]
            corrected.orientation.w = q_corrected[3]
        
        self.pub.publish(corrected)
```

**Launch 파일에 파라미터 추가:**

```python
# transbot_full_system.launch.py
imu_calib_node = Node(
    package='transbot_base',
    executable='imu_calib_node',
    name='imu_calib_node',
    parameters=[{
        'gyro_bias_x': 0.0,
        'gyro_bias_y': 0.0,
        'gyro_bias_z': 0.0,
        'yaw_offset': 0.044,  # ⭐ 2.5도 = 0.044 rad (측정값에 따라 조정)
    }]
)
```

**방법 B: Static Transform 사용**

IMU 프레임과 base_link 사이에 회전 추가:

```xml
<!-- transbot_description/urdf/transbot_simple.urdf -->
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0.044"/>  <!-- ⭐ yaw=2.5도 -->
</joint>
```

---

## 4. Odometry 정렬 방법

### 4.1 Angular Scale 보정

**문제:** Odom이 360도 회전을 390도로 보고

**원인:**
- Wheel base 측정 오차
- 바퀴 미끄러짐
- 인코더 해상도

**해결:**

```python
# transbot_base 파라미터 확인
find ~/transbot_ws_ros2/src -name "*.yaml" | xargs grep -l "wheel_base\|angular_scale"
```

**파라미터 조정:**
```yaml
# transbot_base/config/params.yaml (예시)
transbot_driver:
  ros__parameters:
    wheel_radius: 0.034  # 34mm
    wheel_base: 0.170    # 170mm
    angular_scale: 1.0   # ⭐ 조정 필요
```

**계산 방법:**
```python
# 실제 측정
actual_rotation = 360.0  # 로봇을 손으로 정확히 360도 회전

# Odom이 보고한 값
ros2 topic echo /odom_raw | grep "orientation:"
# quaternion → euler 변환
reported_rotation = 390.0  # 예시

# Angular scale 보정 계수
new_angular_scale = old_angular_scale * (actual_rotation / reported_rotation)
new_angular_scale = 1.0 * (360.0 / 390.0) = 0.923
```

### 4.2 초기 Yaw 오프셋 설정

**방법 A: Odom 노드 수정**

```python
# transbot_base/transbot_base/driver.py
class TransbotDriver(Node):
    def __init__(self):
        super().__init__('Transbot_Driver')
        
        # ⭐ Yaw 초기 오프셋
        self.declare_parameter('initial_yaw_offset', 0.0)
        self.yaw_offset = self.get_parameter('initial_yaw_offset').value
        
        self.yaw = 0.0
        # ...
    
    def publish_odom(self):
        # 기존 계산
        # ... 
        
        # ⭐ Yaw에 오프셋 적용
        adjusted_yaw = self.yaw + self.yaw_offset
        
        # Quaternion 생성
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, adjusted_yaw)
        
        # Odometry 메시지 발행
        # ...
```

**방법 B: TF Tree에서 보정**

```python
# transbot_full_system.launch.py에 static transform 추가
from launch_ros.actions import Node

static_tf_odom_correction = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odom_correction',
    arguments=['0', '0', '0', '0.044', '0', '0', 'odom_corrected', 'odom']
    # yaw=2.5도 = 0.044 rad
)
```

---

## 5. 물리적 환경 정렬

### 5.1 기준 방향 설정

**방법 A: 벽을 기준으로**

```bash
# 1. 로봇을 벽과 평행하게 배치
# 2. 시스템 시작
ros2 launch transbot_nav transbot_full_system.launch.py

# 3. 초기 각도 확인
ros2 topic echo /odometry/filtered | grep -A 3 "orientation:"
# yaw가 0에 가까워야 함

# 4. 필요시 오프셋 조정
```

**방법 B: 자기 나침반 사용**

```python
#!/usr/bin/env python3
"""
자기 나침반으로 로봇 방향 확인
"""
import subprocess
import re

def get_compass_heading():
    # 스마트폰 나침반 앱 사용 또는
    # HMC5883L 같은 나침반 센서 연결
    pass

def get_robot_yaw():
    result = subprocess.run(
        ['ros2', 'topic', 'echo', '/odometry/filtered', '--once'],
        capture_output=True, text=True
    )
    # quaternion 파싱
    # ...
    return yaw_degrees

# 비교
compass = get_compass_heading()
robot = get_robot_yaw()
print(f"나침반: {compass}°")
print(f"로봇:   {robot}°")
print(f"오차:   {compass - robot}°")
```

### 5.2 SLAM 맵과 정렬

**목표:** SLAM 맵의 Y축이 북쪽을 가리키도록

```yaml
# slam_params.yaml
slam_toolbox:
  ros__parameters:
    map_start_pose: [0.0, 0.0, 1.5708]  # ⭐ yaw=90도 (예: 동쪽 시작)
```

**또는 런타임에 설정:**
```bash
ros2 service call /slam_toolbox/reset_pose \
  geometry_msgs/srv/Pose \
  "{pose: {position: {x: 0, y: 0, z: 0}, \
           orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}}}"
# quaternion (0,0,0.7071,0.7071) = yaw 90도
```

---

## 6. 검증 및 테스트

### 6.1 정적 테스트 (정지 상태)

```bash
# 테스트 1: 초기 정렬 확인
python3 compare_sensor_angles.py
# 기대: Odom과 IMU yaw 차이 < 1도
```

### 6.2 동적 테스트 (회전)

```python
#!/usr/bin/env python3
"""
360도 회전 테스트
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import time
import math

class RotationTest(Node):
    def __init__(self):
        super().__init__('rotation_test')
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_cb, 10)
        
        self.odom_yaw = 0.0
        self.imu_yaw = 0.0
        self.initial_odom = None
        self.initial_imu = None
    
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_yaw = yaw
    
    def imu_cb(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = yaw
    
    def run_test(self):
        # 초기값 저장
        time.sleep(1.0)
        self.initial_odom = self.odom_yaw
        self.initial_imu = self.imu_yaw
        
        self.get_logger().info('360도 회전 시작...')
        
        # 회전 시작
        twist = Twist()
        twist.angular.z = 0.5  # 0.5 rad/s
        
        # 6.28초 = 360도
        start = time.time()
        while time.time() - start < 6.28:
            self.pub.publish(twist)
            time.sleep(0.1)
        
        # 정지
        twist.angular.z = 0.0
        self.pub.publish(twist)
        
        time.sleep(1.0)
        
        # 결과 계산
        odom_change = math.degrees(self.odom_yaw - self.initial_odom)
        imu_change = math.degrees(self.imu_yaw - self.initial_imu)
        
        self.get_logger().info(f'\n결과:')
        self.get_logger().info(f'  Odom 변화: {odom_change:.2f}°')
        self.get_logger().info(f'  IMU 변화:  {imu_change:.2f}°')
        self.get_logger().info(f'  차이:      {abs(odom_change - imu_change):.2f}°')
        
        if abs(odom_change - imu_change) < 5.0:
            self.get_logger().info('✅ 정렬 성공! (오차 < 5도)')
        else:
            self.get_logger().warn('⚠️  정렬 실패. 추가 조정 필요.')

def main():
    rclpy.init()
    node = RotationTest()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**실행:**
```bash
cd ~/transbot_ws_ros2/src/transbot_nav/scripts
python3 rotation_test.py
```

### 6.3 장시간 주행 테스트

```bash
# 1. 5분간 무작위 주행
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 2. 주기적으로 각도 차이 로깅
python3 compare_sensor_angles.py > long_term_test.log

# 3. 분석
grep "차이:" long_term_test.log | awk '{sum+=$2; n++} END {print "평균 오차:", sum/n, "도"}'
```

---

## 7. 종합 체크리스트

### ✅ Phase 1: 진단
- [ ] `compare_sensor_angles.py` 실행
- [ ] 정적 오프셋 측정 (정지 상태)
- [ ] 동적 오차 측정 (360도 회전)
- [ ] 오차 패턴 분석 (일정 vs 누적)

### ✅ Phase 2: IMU 보정
- [ ] IMU 프레임 방향 확인
- [ ] Yaw 오프셋 계산
- [ ] `imu_calib_node`에 오프셋 적용
- [ ] 또는 URDF에 회전 추가

### ✅ Phase 3: Odom 보정
- [ ] Angular scale 측정 (360도 테스트)
- [ ] Wheel base 재확인
- [ ] 파라미터 파일 업데이트
- [ ] 초기 yaw 오프셋 설정 (필요시)

### ✅ Phase 4: 검증
- [ ] 정적 테스트 (차이 < 1도)
- [ ] 360도 회전 테스트 (차이 < 5도)
- [ ] 장시간 주행 테스트 (평균 오차 < 3도)

### ✅ Phase 5: 물리적 정렬
- [ ] 기준 방향 설정 (벽/나침반)
- [ ] SLAM 초기 pose 설정
- [ ] 전체 시스템 통합 테스트

---

## 8. 실전 예제

### 예제 1: IMU가 2.5도 앞서는 경우

**측정:**
```
Odom yaw:  45.0°
IMU yaw:   47.5°
차이:      -2.5°  (IMU가 2.5도 더 큼)
```

**보정:**
```python
# imu_calib_node 파라미터
yaw_offset: -0.044  # -2.5도 = -0.044 rad
```

### 예제 2: Odom이 360도를 380도로 보고

**측정:**
```
실제 회전: 360도
Odom 보고: 380도
비율: 380/360 = 1.056
```

**보정:**
```yaml
# transbot_base 파라미터
angular_scale: 0.947  # 1.0 / 1.056 = 0.947
```

### 예제 3: 두 센서 모두 북쪽에서 15도 벗어남

**측정:**
```
나침반: 0° (북쪽)
Odom:   15°
IMU:    15°
```

**보정 (방법 A - 초기 pose):**
```bash
ros2 service call /slam_toolbox/reset_pose \
  geometry_msgs/srv/Pose \
  "{pose: {orientation: {z: -0.1305, w: 0.9914}}}"
# -15도 = quaternion(0, 0, -0.1305, 0.9914)
```

**보정 (방법 B - map_start_pose):**
```yaml
# slam_params.yaml
map_start_pose: [0.0, 0.0, -0.2618]  # -15도 = -0.2618 rad
```

---

## 9. 문제 해결

### 문제 1: 각도 차이가 시간에 따라 증가

**원인:** IMU drift 또는 Odom angular scale 오류

**해결:**
1. IMU 캘리브레이션 재수행
2. Angular scale 재측정
3. EKF process noise 감소

### 문제 2: 회전 방향이 반대

**원인:** IMU Z축 또는 Odom angular velocity 부호 반전

**해결:**
```python
# imu_calib_node
corrected.angular_velocity.z *= -1  # 부호 반전

# 또는 driver
self.angular_velocity_z *= -1
```

### 문제 3: 각도가 갑자기 점프

**원인:** Quaternion singularity 또는 -π~π 경계

**해결:**
- Continuous angle tracking 구현
- Unwrap 함수 사용

---

## 📚 참고 자료

### 내부 문서
- [IMU & Odometry 캘리브레이션](01_IMU_ODOMETRY_CALIBRATION.md)
- [회전 정확도 개선](03_ROTATION_ACCURACY.md)
- [EKF 센서 퓨전](02_EKF_SENSOR_FUSION.md)

### 외부 자료
- [tf2 Documentation](https://docs.ros.org/en/humble/p/tf2/)
- [Quaternion Math](https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/)
- [IMU Calibration Guide](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)

---

**작성:** GitHub Copilot  
**날짜:** 2025-10-31  
**버전:** 1.0
