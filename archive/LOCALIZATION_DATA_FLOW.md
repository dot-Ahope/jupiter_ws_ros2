# 🗺️ Transbot 위치 추정 데이터 흐름 분석

## 📊 **전체 아키텍처**

```
┌─────────────────┐         ┌──────────────────┐
│  하드웨어 센서   │         │   ROS2 노드      │
└─────────────────┘         └──────────────────┘

   STM32 펌웨어                transbot_driver
   (모터 엔코더)      ────►    (속도 발행)
        │                           │
        │                           ▼
        │                    /transbot/get_vel
        │                           │
        ▼                           ▼
   MPU6050 IMU        ────►    base_node
   (자이로/가속도)              (오도메트리 계산)
                                    │
                                    ▼
                             /odom_raw (Odometry)
                          odom -> base_footprint (TF)
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
            /imu/data_calibrated          EKF Filter Node
            (IMU 보정값)                   (센서 융합)
                    │                               │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                          /odometry/filtered
                       map -> odom (TF) ❌ 발행 안 함
                       odom -> base_footprint (TF) ✅
                                    │
                                    ▼
                              SLAM Toolbox
                           (지도 생성 + 보정)
                                    │
                                    ▼
                          map -> odom (TF) ✅
                       (SLAM이 odom 프레임 위치 보정)
```

---

## 🎯 **위치 추정에 사용되는 데이터 토픽**

### **1단계: 원시 센서 데이터**

#### **A. 모터 엔코더 데이터**
```yaml
토픽: /transbot/get_vel
타입: geometry_msgs/Twist
발행자: transbot_driver
주파수: ~25Hz (40ms 주기)
내용:
  - linear.x: 선속도 (m/s)
  - angular.z: 각속도 (rad/s)
```

**데이터 출처:**
- STM32 펌웨어가 좌/우 모터 엔코더 카운트를 읽음
- 펌웨어 내부에서 속도 계산 (wheelbase, encoder PPR, wheel diameter 사용)
- 시리얼 통신으로 Transbot_Lib.py에 전송
- transbot_driver가 ROS2 토픽으로 발행

#### **B. IMU 원시 데이터**
```yaml
토픽: /transbot/imu
타입: sensor_msgs/Imu
발행자: transbot_driver
주파수: ~25Hz (40ms 주기)
내용:
  - angular_velocity (자이로스코프)
    - x, y, z (rad/s)
  - linear_acceleration (가속도계)
    - x, y, z (m/s²)
  - orientation: 무효 (covariance[0] = -1)
```

**데이터 출처:**
- MPU6050/MPU9250 IMU 센서
- 자이로: ±500°/s 범위, 65.5 LSB/(°/s)
- 가속도: ±2g 범위, 16384 LSB/g

---

### **2단계: 교정된 IMU 데이터**

```yaml
토픽: /imu/data_calibrated
타입: sensor_msgs/Imu
발행자: apply_calib (imu_calib 패키지)
주파수: ~25Hz
내용:
  - angular_velocity (교정 후 자이로)
  - linear_acceleration (교정 후 가속도)
  - orientation: 여전히 무효
```

**교정 내용:**
- `imu_calib.yaml` 바이어스 보정 적용
- 자이로스코프 드리프트 제거
- 가속도계 오프셋 보정

---

### **3단계: 오도메트리 (Raw)**

```yaml
토픽: /odom_raw
타입: nav_msgs/Odometry
발행자: base_node (transbot_base 패키지)
주파수: ~25Hz
파라미터:
  - linear_scale: 1.2
  - angular_scale: 1.5625  ⭐ 캘리브레이션 값
```

**계산 방식:**
```cpp
// base.cpp에서
angular_velocity_z_ = msg->angular.z * angular_scale_;  // 1.5625 곱함
linear_velocity_x_ = msg->linear.x * linear_scale_;     // 1.2 곱함

// 적분
heading_ += angular_velocity_z_ * dt;
x_ += linear_velocity_x_ * cos(heading_) * dt;
y_ += linear_velocity_x_ * sin(heading_) * dt;
```

**발행 내용:**
- `pose.pose.position`: (x, y, 0)
- `pose.pose.orientation`: heading을 quaternion으로 변환
- `twist.twist.linear.x`: 선속도
- `twist.twist.angular.z`: 각속도

**TF 발행:**
```
odom -> base_footprint (base_node가 발행)
```

---

### **4단계: EKF 융합 오도메트리**

```yaml
토픽: /odometry/filtered
타입: nav_msgs/Odometry
발행자: ekf_filter_node (robot_localization)
주파수: 10Hz (frequency: 10.0)
```

#### **EKF 입력 소스:**

**A. 오도메트리 입력 (odom0)**
```yaml
odom0: /odom_raw
odom0_config:
  - [true, true, false]      # x, y 위치 ✅
  - [false, false, true]     # yaw 각도 ✅
  - [true, true, false]      # x, y 선속도 ✅
  - [false, false, true]     # yaw 각속도 ✅
  - [false, false, false]    # 가속도 ❌

odom0_differential: true     # 변화량 사용 (휠 슬립 보상)
odom0_queue_size: 25
```

**B. IMU 입력 (imu0)**
```yaml
imu0: /imu/data_calibrated
imu0_config:
  - [false, false, false]    # 위치 ❌
  - [false, false, false]    # 방향 ❌ (오도메트리가 담당)
  - [false, false, false]    # 선속도 ❌
  - [false, false, true]     # yaw 각속도 ✅ (IMU 전담)
  - [false, false, false]    # 선가속도 ❌

imu0_differential: true      # 변화량 사용
imu0_relative: true          # 상대 측정값
imu0_queue_size: 20
```

#### **EKF 센서 융합 전략:**

| 상태 변수 | 오도메트리 | IMU | 최종 추정 |
|----------|-----------|-----|----------|
| **x, y 위치** | ✅ 주 입력 | ❌ | Odom 기반 |
| **yaw 각도** | ✅ 보조 | ❌ | Odom 기반 |
| **vx, vy 선속도** | ✅ 주 입력 | ❌ | Odom 기반 |
| **yaw 각속도** | ✅ 보조 | ✅ **주 입력** | **IMU 우선** |

**핵심 포인트:**
1. **IMU는 각속도(yaw rate)만 제공** → 회전 변화 정밀 추적
2. **오도메트리가 위치와 방향의 주 소스**
3. **EKF가 두 소스를 융합**하여 드리프트 보정

#### **EKF 출력:**

**A. 토픽 발행:**
```yaml
/odometry/filtered (nav_msgs/Odometry)
  - pose: EKF가 추정한 odom 프레임 기준 위치
  - twist: EKF가 추정한 속도
```

**B. TF 발행:**
```
odom -> base_footprint  ✅ (EKF가 발행)
```

**⚠️ 중요:** EKF는 `map -> odom` TF를 발행하지 **않음**
- `world_frame: odom` 설정으로 SLAM이 map 프레임 관리

---

### **5단계: SLAM Toolbox (지도 생성 + 위치 보정)**

```yaml
scan_topic: /scan (LiDAR 스캔)
odom_frame: odom
map_frame: map
base_frame: base_footprint

mode: mapping  # 매핑 모드
```

#### **SLAM 입력:**

**A. 레이저 스캔**
```yaml
토픽: /scan
타입: sensor_msgs/LaserScan
발행자: sllidar_node
주파수: 5Hz (scan_frequency: 5.0)
범위: 0.05m ~ 22.0m
```

**B. 오도메트리 (간접적)**
- SLAM은 `/odometry/filtered`를 **직접 구독하지 않음**
- 대신 **TF 트리**를 통해 `odom -> base_footprint` 변환 사용
- EKF가 발행한 TF를 읽어서 로봇 이동량 추정

#### **SLAM 프로세스:**

1. **스캔 매칭 (Scan Matching)**
   - 현재 LiDAR 스캔과 이전 스캔 비교
   - 로봇의 실제 이동량 계산
   - 오도메트리 오차 감지

2. **루프 클로저 (Loop Closure)**
   - 이전에 방문한 장소 재인식
   - 누적된 오도메트리 드리프트 보정
   - 지도 일관성 향상

3. **포즈 그래프 최적화**
   - 모든 로봇 위치와 스캔을 그래프로 표현
   - 전역 최적화로 지도 정확도 향상

#### **SLAM 출력:**

**A. TF 발행:**
```
map -> odom  ✅ (SLAM이 발행)
```

**의미:**
- SLAM이 오도메트리 프레임(`odom`)의 위치를 지도 프레임(`map`) 기준으로 보정
- 오도메트리 드리프트를 실시간으로 수정

**B. 지도 발행:**
```yaml
/map (nav_msgs/OccupancyGrid)
  - 2D 점유 격자 지도
  - resolution: 0.05m (5cm)
```

---

## 🔗 **최종 TF 트리**

```
map  ←─────────  (SLAM Toolbox)
  │
  └→ odom  ←────  (EKF Filter Node)
       │
       └→ base_footprint  ←──  (robot_state_publisher)
            │
            ├→ base_link
            ├→ laser  (LiDAR)
            └→ imu_link
```

---

## 📌 **위치 추정 요약**

### **로봇이 자기 위치를 판단하는 데 사용하는 토픽:**

#### **직접 사용:**
1. ✅ **`/odom_raw`** (nav_msgs/Odometry)
   - base_node가 모터 엔코더 기반으로 계산
   - angular_scale = 1.5625 적용

2. ✅ **`/imu/data_calibrated`** (sensor_msgs/Imu)
   - IMU 자이로 데이터 (각속도)
   - imu_calib으로 보정됨

3. ✅ **`/scan`** (sensor_msgs/LaserScan)
   - LiDAR 스캔 데이터
   - SLAM이 위치 보정에 사용

#### **간접 사용 (TF를 통해):**
4. ⚙️ **TF: `odom -> base_footprint`**
   - EKF가 발행
   - SLAM이 읽어서 로봇 이동량 추정

---

## 🎯 **각 노드의 역할**

| 노드 | 입력 | 출력 | 역할 |
|-----|------|------|------|
| **transbot_driver** | 시리얼 (STM32) | `/transbot/get_vel`, `/transbot/imu` | 센서 데이터 발행 |
| **base_node** | `/transbot/get_vel` | `/odom_raw`, TF | 휠 오도메트리 계산 |
| **imu_calib** | `/transbot/imu` | `/imu/data_calibrated` | IMU 보정 |
| **ekf_filter_node** | `/odom_raw`, `/imu/data_calibrated` | `/odometry/filtered`, TF | 센서 융합 |
| **slam_toolbox** | `/scan`, TF | `/map`, TF | 지도 생성 + 위치 보정 |

---

## 🔍 **핵심 인사이트**

### **1. 위치 추정의 3단계:**
```
모터 엔코더 (base_node)
    ↓ angular_scale = 1.5625
오도메트리 (/odom_raw)
    ↓ EKF 융합 (+ IMU)
필터링된 오도메트리 (/odometry/filtered)
    ↓ SLAM 보정 (+ LiDAR)
지도 기준 위치 (map -> odom TF)
```

### **2. angular_scale의 영향:**
- **적용 지점:** base_node (1단계)
- **영향 범위:** /odom_raw → EKF → SLAM (전체 시스템)
- **값:** 1.5625 (펌웨어 각속도 과소 측정 보정)

### **3. EKF의 역할:**
- 오도메트리 위치/방향 (주)
- IMU 각속도 (보조)
- 융합하여 단기 드리프트 감소

### **4. SLAM의 역할:**
- LiDAR 스캔 매칭으로 오도메트리 오차 감지
- 장기 누적 드리프트 보정
- 일관된 지도 생성

---

## ⚠️ **현재 시스템의 한계**

1. **절대 방향 측정 불가:**
   - 나침반/자력계 없음
   - 초기 yaw 불확실성 매우 큼 (1000.0)
   - SLAM이 점진적으로 보정

2. **IMU 역할 제한:**
   - 각속도만 제공 (orientation 무효)
   - 방향 추정은 오도메트리 주도

3. **펌웨어 하드웨어 파라미터:**
   - wheelbase, encoder PPR, wheel diameter
   - 펌웨어에 내장되어 접근 불가
   - angular_scale로 보정 중

---

## 📝 **캘리브레이션 체크리스트**

현재 상태:
- ✅ **linear_scale**: 1.2 (ROS1에서 측정)
- ✅ **angular_scale**: 1.5625 (Phase 2 캘리브레이션 예정)
- ⚠️ **bringup.launch.py**: 1.560 (통일 필요)

권장:
1. Phase 2 캘리브레이션 완료 후 새 값 적용
2. transbot_full_system.launch.py와 bringup.launch.py 동시 업데이트
3. EKF 파라미터 미세 조정 (필요 시)

---

## 🎓 **결론**

**로봇이 자기 위치를 판단할 때 사용하는 핵심 토픽:**

1. **`/odom_raw`** - 휠 오도메트리 (angular_scale 적용)
2. **`/imu/data_calibrated`** - IMU 각속도 (회전 정밀 추적)
3. **`/scan`** - LiDAR 스캔 (위치 보정)

**데이터 흐름:**
```
엔코더 + IMU → /odom_raw → EKF → /odometry/filtered
                                      ↓
                           LiDAR → SLAM → map 프레임
```

**TF 트리가 최종 위치:**
```
map -> odom -> base_footprint
```
- `map` 기준 = 절대 위치 (SLAM 보정됨)
- `odom` 기준 = 상대 위치 (드리프트 있음)
- `base_footprint` = 로봇 중심
