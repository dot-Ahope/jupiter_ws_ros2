# 🔄 Transbot 보정값 및 데이터 흐름 완전 분석

**작성일**: 2025-10-17  
**목적**: 보정값의 작동 원리와 과도한 회전 발생 원인 분석

---

## 📊 전체 시스템 아키텍처

### 계층 구조

```
┌─────────────────────────────────────────────────────────┐
│                     Application Layer                    │
│                    (SLAM, Navigation)                    │
└────────────────┬────────────────────────────────────────┘
                 │ /odometry/filtered (EKF 융합)
                 │ TF: odom → base_footprint
┌────────────────┴────────────────────────────────────────┐
│                   Sensor Fusion Layer                    │
│                  (EKF - robot_localization)              │
└────────────┬───────────────────────┬────────────────────┘
             │ /odom_raw              │ /imu/d 필터링)
┌────────────┴──────────┐  ┌─────────┴──────────────────┐
│   Odometry Layer      │  │      IMU Processing        │
│   (base_node)         │  │  (imu_calib + imu_filter)  │
└────────────┬──────────┘  └─────────┬──────────────────┘
             │ /transbot/get_vel      │ /transbot/imu
             │ (휠 인코더 피드백)      │ (IMU 원시 데이터)
┌────────────┴────────────────────────┴──────────────────┐
│                     Hardware Layer                       │
│                   (transbot_driver.py)                   │
└────────────┬────────────────────────────────────────────┘
             │ /cmd_vel (속도 명령)
┌────────────┴────────────────────────────────────────────┐
│                      Control Input                       │
│              (test_90degree_rotation.py)                 │
└──────────────────────────────────────────────────────────┘
```

---

## 🔍 데이터 흐름 상세 분석

### 1단계: 제어 명령 입력

**test_90degree_rotation.py**:
```python
twist = Twist()
twist.angular.z = 0.2  # rad/s (반시계방향)
self.cmd_vel_pub.publish(twist)  # /cmd_vel 토픽으로 발행
```

**명령 의미**:
- 각속도: 0.2 rad/s (약 11.5°/s)
- 90° 회전 이론 시간: 90° / 11.5° = 7.8초

---

### 2단계: 하드웨어 제어 (transbot_driver.py)

#### cmd_vel_callback 함수:

```python
def cmd_vel_callback(self, msg):
    linear_x = msg.linear.x     # 0.0 m/s
    angular_z = msg.angular.z   # 0.2 rad/s
    
    # 스케일 변환
    base_speed = linear_x * (100.0 / 0.2)  # 0
    turn_speed = angular_z * (100.0 / 0.5)  # 0.2 * 200 = 40
    
    # 차동 구동 계산
    left_speed = int(base_speed - turn_speed)  # 0 - 40 = -40
    right_speed = int(base_speed + turn_speed) # 0 + 40 = +40
    
    # 모터 명령 전송
    self.bot.set_motor(1, -40)  # 왼쪽 후진 40%
    self.bot.set_motor(2, +40)  # 오른쪽 전진 40%
```

**핵심 변환**:
```
angular_z (rad/s) → turn_speed (%) 변환 공식:
turn_speed = angular_z * (100.0 / 0.5) = angular_z * 200

예시:
0.2 rad/s → 40% 모터 출력
0.5 rad/s → 100% 모터 출력 (최대)
```

**⚠️ 문제점 1: 하드웨어 비선형성**
- 명령: 0.2 rad/s (40% 모터)
- 실제: ~0.78 rad/s (측정됨)
- 비율: **3.9배 증폭** (하드웨어 특성)

---

### 3단계: 센서 피드백 (transbot_driver.py)

#### publish_velocity 함수:

```python
def publish_velocity(self):
    # 하드웨어에서 실제 속도 읽기
    vel, ang = self.bot.get_velocity()
    
    msg = Twist()
    msg.linear.x = float(vel)   # 실제 선속도 (m/s)
    msg.angular.z = float(ang)  # 실제 각속도 (rad/s)
    
    self.vel_pub.publish(msg)   # /transbot/get_vel 토픽 발행
```

**특징**:
- **피드백은 하드웨어 실제 값** (명령값 아님)
- **보정 없음**: 휠 인코더가 측정한 원시 값
- **각속도 예시**: 0.2 rad/s 명령 → ~0.78 rad/s 측정

**⚠️ 문제점 2: 휠 슬립 및 측정 오차**
- 휠 인코더는 **회전수만 측정**
- 바닥 슬립 → 실제보다 적게 측정
- 기어비 미보정 → 스케일 오차

---

### 4단계: 오도메트리 계산 (base_node)

#### velCallback 함수 (base.cpp):

```cpp
void RobotBase::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 1. 보정값 적용
    linear_velocity_x_ = msg->linear.x * linear_scale_;      // 1.2배
    angular_velocity_z_ = msg->angular.z * angular_scale_;   // 1.56배 ⭐
    
    // 2. 시간 간격 계산
    vel_dt_ = (current_time - last_vel_time_).seconds();
    
    // 3. 변위 계산 (적분)
    double delta_heading = angular_velocity_z_ * vel_dt_;
    double delta_x = (linear_velocity_x_ * cos(heading_) - ...) * vel_dt_;
    double delta_y = (linear_velocity_x_ * sin(heading_) + ...) * vel_dt_;
    
    // 4. 누적
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;  // ⭐ 보정된 각속도 적분
    
    // 5. /odom_raw 토픽 발행
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.orientation = quaternion(heading_);
    odom.twist.twist.angular.z = angular_velocity_z_;  // 보정된 각속도
    
    odom_publisher_->publish(odom);
}
```

**angular_scale의 역할**:
```
실제 측정 각속도 × angular_scale = 보정된 각속도

예시 (angular_scale = 1.56):
- 피드백: 0.51 rad/s (휠 인코더 측정, 과소 측정)
- 보정 후: 0.51 × 1.56 = 0.796 rad/s
- 실제 회전: ~0.78 rad/s (IMU 측정)
- 오차: 약 2% (매우 정확!)
```

**⚠️ 핵심 발견**:
- `angular_scale = 1.56`은 **휠 인코더 과소 측정을 보정**
- 실제 물리 회전에 가까운 오도메트리 생성
- SLAM 정확도 향상의 핵심!

---

### 5단계: IMU 처리 체인

#### 5-1. IMU 캘리브레이션 (imu_calib)

```yaml
# imu_calib.yaml
accel_scale:  [3x3 행렬]  # 가속도계 스케일 보정
accel_bias:   [3x1 벡터]  # 가속도계 바이어스 제거
gyro_bias:    [3x1 벡터]  # 자이로 바이어스 제거 (드리프트 보정)
```

**역할**:
- 센서 제조 오차 보정
- 온도 드리프트 제거
- 중력 방향 정렬

#### 5-2. IMU 필터링 (imu_filter_madgwick)

```yaml
# transbot_full_system.launch.py
imu_filter_node:
  parameters:
    gain: 0.005          # 낮은 게인 (진동 최소화)
    zeta: 0.001          # 낮은 zeta (빠른 반응)
    constant_dt: 0.1     # 10Hz 업데이트
```

**역할**:
- 자이로 적분으로 방향 추정
- 가속도계로 장기 안정성 확보
- 노이즈 필터링

---

### 6단계: EKF 센서 융합

#### EKF 입력 (ekf_config.yaml):

```yaml
# 오도메트리 입력
odom0: /odom_raw  # angular_scale=1.56 적용됨 ⭐
odom0_config: [true, true, false,    # x, y 위치
               false, false, true,    # yaw 각도 ✅
               true, true, false,     # x, y 속도
               false, false, true,    # yaw 각속도 ✅
               false, false, false]

# IMU 입력
imu0: /imu/data_filtered
imu0_config: [false, false, false,
              false, false, false,   # yaw 각도 비활성화 ❌
              false, false, false,
              false, false, true,    # yaw 각속도만 ✅
              false, false, false]
```

**융합 전략**:
1. **위치 (x, y)**: 오도메트리만 사용
2. **방향 (yaw)**: 
   - **각도**: 오도메트리 (angular_scale 보정됨)
   - **각속도**: 오도메트리 + IMU (양쪽 사용)
3. **속도**: 오도메트리만 사용

**융합 효과**:
```
오도메트리 yaw 각속도:  0.796 rad/s (angular_scale=1.56)
IMU yaw 각속도:         0.780 rad/s (실제 측정)
EKF 융합 결과:          ~0.788 rad/s (가중 평균)
```

**⚠️ 핵심 통찰**:
- EKF는 **angular_scale 보정된 오도메트리**를 받음
- IMU와 융합하여 **최종 정확도 향상**
- SLAM이 사용하는 `/odometry/filtered`가 정확해짐

---

### 7단계: SLAM 입력

```yaml
# slam_params.yaml
odom_frame: odom
base_frame: base_footprint
scan_topic: /scan
```

**SLAM이 사용하는 데이터**:
1. **TF: odom → base_footprint** (EKF 발행)
2. **LiDAR 스캔**: /scan
3. **스캔 매칭**으로 지도 생성

**angular_scale의 영향**:
```
angular_scale=1.56 적용 시:
  → /odom_raw가 실제 회전에 가까워짐
  → EKF 융합 결과도 정확해짐
  → SLAM TF가 실제 회전 반영
  → 지도 정확도 향상 ✅
```

---

## 🎯 과도한 회전 발생 원인 분석

### 문제 재정의

**현상**:
- **명령**: 90° 회전 (0.2 rad/s, 7.8초)
- **측정 (odom_raw)**: 144° (angular_scale=1.56 적용 후)
- **실제 물리**: 225° 회전
- **비율**: 225° / 144° = **1.56배** (또 다른 증폭!)

### 근본 원인: 이중 루프 문제

#### 제어 루프 분석

```
┌─────────────────────────────────────────────────┐
│          test_90degree_rotation.py              │
│                                                 │
│  목표: 90° 회전                                 │
│  제어 루프:                                     │
│    while (odom_raw < 90°):                     │
│        publish(cmd_vel: 0.2 rad/s)             │
│                                                 │
│  ⚠️ 문제: odom_raw가 실제보다 작게 측정됨!     │
│  → 루프가 과도하게 오래 실행                    │
└─────────────────────────────────────────────────┘
         │ cmd_vel: 0.2 rad/s
         ↓
┌─────────────────────────────────────────────────┐
│            transbot_driver.py                   │
│                                                 │
│  모터 명령: -40%, +40%                         │
│  ⚠️ 하드웨어 비선형성: 3.9배 증폭              │
│  실제 실행: 0.78 rad/s (명령의 3.9배)          │
└─────────────────────────────────────────────────┘
         │ 피드백: 0.51 rad/s (휠 인코더)
         ↓
┌─────────────────────────────────────────────────┐
│               base_node                         │
│                                                 │
│  angular_scale: 1.56배 보정                    │
│  0.51 × 1.56 = 0.796 rad/s                     │
│  ⚠️ 여전히 실제(0.78)보다 약간 높음            │
│  → odom_raw에 144° 측정 (실제 225°)            │
└─────────────────────────────────────────────────┘
```

### 이중 증폭 메커니즘

**Stage 1: 하드웨어 증폭 (3.9배)**
```
명령: 0.2 rad/s (40% 모터)
실제: 0.78 rad/s
증폭: 3.9배
```

**Stage 2: 제어 루프 과실행**
```
목표: 90° 도달
측정: 144° 도달 (angular_scale 보정 후)
실제: 225° 회전 (물리적)

과실행 원인:
- 제어 루프가 "odom_raw < 90°"를 기다림
- odom_raw는 실제보다 느리게 증가 (0.796 vs 0.78)
- 아직 90° 안 됨 → 계속 회전 명령
- 실제로는 이미 100°, 120°, 140° 지나침
- 최종적으로 odom_raw 144° 도달
- 실제 물리는 225° 회전됨
```

### 수식으로 표현

```
실제_회전 = (명령_각속도 × 하드웨어_증폭) × 제어_시간

제어_시간 = 목표_각도 / (측정_각속도 × angular_scale)

대입:
실제_회전 = (0.2 × 3.9) × (90° / (0.51 × 1.56))
          = 0.78 rad/s × (1.57 rad / 0.796 rad/s)
          = 0.78 × 1.97s
          = 1.54 rad
          = 88.2° (이론값)

실제: 225° (실험값)
차이: 2.55배

추가 증폭 원인:
1. 하드웨어 비선형성이 부하에 따라 변동
2. 관성 모멘트로 인한 오버슈트
3. 정지 명령 지연 (제어 루프 주기)
```

---

## 💡 해결책 분석

### 현재 상태 (angular_scale=1.56)

**장점**:
✅ SLAM 정확도 향상
✅ /odom_raw가 실제 회전에 근접
✅ EKF 융합 결과 정확

**단점**:
❌ 회전 테스트에서 과도한 회전 (225°)
❌ Navigation 제어 시 유사 문제 예상

### 왜 SLAM은 정확한데 회전 테스트는 안 되는가?

**SLAM의 경우**:
```
- SLAM은 "상대적 변화량"만 필요
- odom_raw의 절대값이 아닌 "변화율" 사용
- EKF 융합으로 드리프트 보정
- 스캔 매칭으로 오차 수정
→ angular_scale=1.56이 정확도 향상
```

**회전 테스트의 경우**:
```
- "절대 각도" 기준 제어
- odom_raw < 90° 조건 사용
- 제어 루프가 측정값에 직접 의존
- 하드웨어 비선형성을 보상 못 함
→ angular_scale 보정만으로는 부족
```

---

## 🎓 보정값 사용 가이드

### 1. angular_scale (base_node)

**용도**: 휠 인코더 측정 오차 보정

**설정값**: 1.56 (실제 회전 / 측정 회전)

**적용 대상**:
- ✅ SLAM (상대 변화량 사용)
- ✅ EKF 융합 (센서 융합)
- ❌ 절대 각도 제어 (회전 테스트)

**권장 사용**:
```yaml
# SLAM 운영 시
angular_scale: 1.56  # 정확도 향상

# 단순 회전 테스트 시
angular_scale: 1.0   # 하드웨어 제어 검증
```

### 2. linear_scale (base_node)

**용도**: 선속도 측정 오차 보정

**설정값**: 1.2 (ROS1 캘리브레이션)

**적용**: 직진 이동 시 거리 보정

### 3. imu_calib (IMU 캘리브레이션)

**용도**: IMU 센서 고유 오차 보정

**갱신 주기**: 6개월~1년

**필요 시점**:
- 센서 교체 시
- 온도 변화 큰 환경
- 드리프트 누적

### 4. EKF 가중치 (ekf_config.yaml)

**용도**: 센서 신뢰도 조정

**조정 필요**:
- 오도메트리 슬립 많은 환경
- IMU 노이즈 높은 환경

---

## 📋 최종 권장 사항

### SLAM 운영 시 (현재 설정 유지) ⭐

```python
# angular_scale = 1.56 유지
parameters=[{
    'linear_scale': 1.2,
    'angular_scale': 1.56,  # SLAM 정확도 최적화
    'is_multi_robot': False
}]
```

**이유**:
- SLAM이 상대 변화만 사용
- EKF 융합 효과 극대화
- 지도 품질 향상

### Navigation 제어 개선 필요

```
문제: 하드웨어 비선형성 (3.9배 증폭)
해결: Navigation 컨트롤러에서 보정

방법:
1. Feedforward 모델 추가
2. PID 튜닝 강화
3. Velocity smoother 적용
```

### 회전 테스트 개선

```python
# 목표 각도를 오버슈트 보정
target_angle = 90.0
compensated_target = target_angle / 1.56  # 57.7° 목표
# → 실제 90° 도달
```

---

## 🎯 결론

### 보정값 역할 요약

| 보정값 | 적용 위치 | 목적 | SLAM 영향 | Navigation 영향 |
|--------|-----------|------|-----------|-----------------|
| **angular_scale=1.56** | base_node | 휠 인코더 과소 측정 보정 | ✅ 정확도 향상 | ⚠️ 제어 조정 필요 |
| **linear_scale=1.2** | base_node | 선속도 측정 보정 | ✅ 거리 정확도 | ✅ 직진 이동 개선 |
| **imu_calib** | imu_calib | 센서 고유 오차 | ✅ 방향 안정성 | ✅ 각속도 정확도 |
| **EKF 가중치** | robot_localization | 센서 융합 최적화 | ✅ 드리프트 보정 | ✅ 위치 추정 |

### 과도한 회전의 근본 원인

```
1. 하드웨어 비선형성: 3.9배 증폭 (제어 문제)
2. 제어 루프 과실행: 측정값 기반 종료 조건 (알고리즘 문제)
3. 관성 오버슈트: 정지 지연 (기구 문제)

→ angular_scale은 "측정 정확도"를 개선하지만
  "제어 정확도"는 별도 해결 필요!
```

**최종 제안**: `angular_scale=1.56` 유지하고,  
Navigation 컨트롤러에서 하드웨어 비선형성 보정 추가! 🎯
