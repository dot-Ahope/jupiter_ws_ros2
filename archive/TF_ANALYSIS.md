# TF 노드 역할 분석: ekf_filter_node, slam_toolbox, robot_state_publisher

## 📋 **TF 발행 책임 요약**

### **1. robot_state_publisher**

**역할:** URDF 기반 정적 및 동적 조인트 변환 발행

**발행 TF:**
```
base_footprint → base_link         (static, fixed joint)
base_link → left_wheel_link        (dynamic, continuous joint)
base_link → right_wheel_link       (dynamic, continuous joint)
base_link → laser                  (static, fixed joint, yaw=180°)
base_link → imu_link               (static, fixed joint)
base_link → camera_link            (static, fixed joint)
```

**설정:**
- `publish_frequency: 30.0 Hz`
- URDF 파일: `transbot_simple.urdf`

**충돌 가능성:** ❌ **없음** (로봇 내부 조인트만 담당)

---

### **2. ekf_filter_node (robot_localization)**

**역할:** Odom + IMU 센서 융합하여 로봇 위치 추정

**발행 TF:**
```
odom → base_footprint              (dynamic, sensor fusion)
```

**설정 (ekf_config.yaml):**
```yaml
publish_tf: true                    # ⭐ TF 발행 활성화
odom_frame: odom
base_link_frame: base_footprint     # ⭐ 주의: EKF가 base_footprint 사용
world_frame: odom
map_frame: map                      # (EKF는 map 프레임 발행 안 함)
```

**입력:**
- `/odom_raw`: base_node의 휠 오도메트리 (x, y, yaw 위치 + 속도)
- `/imu/data_calibrated`: imu_calib 보정된 IMU (yaw 각속도)

**출력:**
- `/odometry/filtered`: 융합된 오도메트리 토픽
- **TF: `odom → base_footprint`** (센서 융합 결과)

**충돌 가능성:** ⚠️ **주의 필요** (base_node의 TF 발행과 충돌 가능)

---

### **3. slam_toolbox**

**역할:** LiDAR 스캔 매칭으로 맵 생성 및 루프 클로저

**발행 TF:**
```
map → odom                          (dynamic, scan matching)
```

**설정 (slam_params.yaml):**
```yaml
odom_frame: odom
map_frame: map
base_frame: base_footprint          # ⭐ SLAM이 base_footprint 참조
scan_topic: /scan
transform_publish_period: 0.02      # 50Hz TF 발행
```

**동작:**
- LiDAR 스캔을 `base_footprint`에서 읽음
- Scan matching으로 `map → odom` 변환 계산
- 루프 클로저로 맵 보정

**충돌 가능성:** ❌ **없음** (map → odom만 담당, 다른 프레임과 독립)

---

### **4. base_node (transbot_base)** - 현재 주석 처리됨

**역할:** 휠 오도메트리 계산

**발행 TF (코드 주석 확인):**
```cpp
// TF 발행을 robot_localization에 맡기기 위해 주석 처리
// tf_broadcaster_->sendTransform(transformStamped);
```

**현재 상태:** TF 발행 **비활성화** ✅

**발행 토픽:**
- `/odom_raw`: 휠 오도메트리 메시지 (TF 없이 토픽만)

**충돌 가능성:** ✅ **해결됨** (TF 발행 안 함)

---

## 🔍 **TF 트리 구조**

### **정상 동작 시 TF 트리:**

```
map (SLAM)
 └─ odom (SLAM)
     └─ base_footprint (EKF)
         └─ base_link (robot_state_publisher)
             ├─ left_wheel_link (robot_state_publisher)
             ├─ right_wheel_link (robot_state_publisher)
             ├─ laser (robot_state_publisher, yaw=180°)
             ├─ imu_link (robot_state_publisher)
             └─ camera_link (robot_state_publisher)
```

### **노드별 책임 분담:**

| 노드 | 발행 TF | 역할 | 주파수 |
|------|---------|------|--------|
| **slam_toolbox** | `map → odom` | 글로벌 위치 보정 | 50 Hz |
| **ekf_filter_node** | `odom → base_footprint` | 센서 융합 (Odom+IMU) | 10 Hz |
| **robot_state_publisher** | `base_footprint → base_link` + 조인트 | URDF 조인트 | 30 Hz |
| **base_node** | ❌ (주석 처리) | 토픽만 발행 | - |

---

## ⚠️ **잠재적 충돌 시나리오**

### **Scenario 1: base_node TF 활성화 시**

만약 `base_node`의 TF 발행 주석을 해제하면:

```
충돌:
  ekf_filter_node:  odom → base_footprint (센서 융합)
  base_node:        odom → base_footprint (휠만)
  
→ ROS2 에러: "Multiple publishers on /tf for odom → base_footprint"
→ TF lookup 실패 또는 불안정한 변환
```

**해결책:** 둘 중 하나만 TF 발행
- **현재 설정 (권장):** EKF만 발행 ✅
- **대안:** base_node만 발행, EKF는 `publish_tf: false`

---

### **Scenario 2: Frame 이름 불일치**

**현재 설정:**
```yaml
# ekf_config.yaml
base_link_frame: base_footprint  ✅

# slam_params.yaml
base_frame: base_footprint       ✅

# URDF
<link name="base_footprint"/>    ✅
<link name="base_link"/>         ✅
```

**결과:** ✅ **일치함** (모든 노드가 `base_footprint` 사용)

---

## ✅ **협업 구조 분석**

### **역할 분담 (현재 설정):**

#### **1. SLAM → EKF → Robot State (계층 협업)**

```
SLAM Toolbox:
  역할: 글로벌 맵 좌표계 관리
  TF:   map → odom
  
  ↓ (SLAM이 odom 프레임을 맵에 정렬)

EKF Filter:
  역할: 로컬 로봇 위치 추정 (센서 융합)
  TF:   odom → base_footprint
  입력: /odom_raw (휠) + /imu/data_calibrated (IMU)
  
  ↓ (EKF가 base_footprint를 odom에 정렬)

Robot State Publisher:
  역할: 로봇 내부 조인트 변환
  TF:   base_footprint → base_link, laser, wheels, etc.
```

#### **2. 데이터 흐름:**

```
물리 센서:
  transbot_driver → /transbot/imu (원시 IMU)
  transbot_driver → /transbot/get_vel (휠 속도)
  
  ↓

보정 레이어:
  imu_calib → /imu/data_calibrated (gyro_bias 제거)
  base_node → /odom_raw (휠 적분, TF 없음)
  
  ↓

센서 융합:
  ekf_filter_node → odom → base_footprint TF
                  → /odometry/filtered 토픽
  
  ↓

SLAM:
  slam_toolbox → map → odom TF
               → /map 토픽
```

---

## 🎯 **결론**

### **충돌 여부:**
❌ **충돌 없음** - 각 노드가 명확히 분리된 TF를 발행

### **협업 구조:**
✅ **계층적 협업** - 3단계 레이어 구조:

1. **SLAM Layer** (`map → odom`): 글로벌 위치 보정
2. **Sensor Fusion Layer** (`odom → base_footprint`): 로컬 로봇 위치 추정
3. **Robot Structure Layer** (`base_footprint → base_link, ...`): 조인트 변환

### **현재 설정 평가:**
✅ **올바름** - 각 노드가 역할 분담:
- SLAM: 맵 좌표계 관리
- EKF: 센서 융합 (Odom + IMU)
- Robot State Publisher: 로봇 구조

---

## 🧪 **검증 방법**

### **1. TF 트리 확인**

```bash
# TF 트리 시각화
ros2 run tf2_tools view_frames

# 생성된 PDF 확인
evince frames_*.pdf
```

**예상 결과:**
```
map
 └─ odom (slam_toolbox)
     └─ base_footprint (ekf_filter_node)
         └─ base_link (robot_state_publisher)
             ├─ left_wheel_link
             ├─ right_wheel_link
             ├─ laser (yaw=180°)
             ├─ imu_link
             └─ camera_link
```

### **2. 각 TF 발행자 확인**

```bash
# TF 발행 노드 확인
ros2 run tf2_ros tf2_monitor

# 특정 변환 발행자 확인
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

### **3. 중복 발행 체크**

```bash
# /tf 토픽에서 같은 parent→child를 여러 노드가 발행하는지 확인
ros2 topic echo /tf --once | grep -A 10 "frame_id: odom"
```

**정상 출력 (odom → base_footprint가 1개만 나타남):**
```yaml
transforms:
- header:
    frame_id: odom
  child_frame_id: base_footprint
  # (단일 항목만 존재)
```

---

## 🔧 **문제 해결 가이드**

### **Issue 1: EKF TF가 회전을 반영하지 않음**

**증상:**
```bash
ros2 run tf2_ros tf2_echo odom base_footprint
# Rotation: in RPY (degree) [0.000, -0.000, -176.521]
# (거의 변하지 않음)
```

**원인:** IMU 과대 측정으로 EKF가 IMU를 outlier 거부

**해결:**
1. IMU 캘리브레이션 확인:
   ```bash
   ros2 topic echo /imu/data_calibrated --field angular_velocity.z
   # 기대값: ~0.0001 rad/s (정지 시)
   ```

2. Phase 1 재실행 (IMU 검증):
   ```bash
   python3 odom_based_angular_calibration.py --phase 1
   # 예상: IMU 적분 88~92° (현재 135°에서 개선)
   ```

3. EKF rejection threshold 조정 (임시):
   ```yaml
   # ekf_config.yaml
   imu0_twist_rejection_threshold: 5.0  # 2.0 → 5.0
   ```

---

### **Issue 2: TF 충돌 감지**

**증상:**
```
[WARN] TF_REPEATED_DATA ignoring data with redundant timestamp
[ERROR] Lookup would require extrapolation into the past
```

**원인:** 두 노드가 같은 TF 발행

**해결:**
```cpp
// base.cpp에서 확인
// tf_broadcaster_->sendTransform(transformStamped);  // 주석 처리 필수!
```

```yaml
# ekf_config.yaml
publish_tf: true  # EKF만 발행

# 또는 base_node만 발행하려면
publish_tf: false  # EKF 비활성화
```

---

## 📊 **성능 고려사항**

### **TF 발행 주파수:**

| 노드 | 주파수 | 적정성 |
|------|--------|--------|
| slam_toolbox | 50 Hz | ✅ 높음 (실시간 매칭) |
| ekf_filter_node | 10 Hz | ⚠️ 낮음 (권장 30+ Hz) |
| robot_state_publisher | 30 Hz | ✅ 적정 |

**권장 수정:**
```yaml
# ekf_config.yaml
frequency: 30.0  # 10.0 → 30.0 (반응성 향상)
```

---

## 🎓 **핵심 개념**

### **TF 계층 구조:**

1. **글로벌 프레임** (`map`): 절대 좌표계, SLAM이 관리
2. **로컬 프레임** (`odom`): 로봇 시작점 기준, 드리프트 있음
3. **로봇 프레임** (`base_footprint`): 센서 융합 결과
4. **센서 프레임** (`laser`, `imu_link`, ...): 로봇에 고정

### **센서 융합 이점:**

**Without EKF (base_node 단독):**
- 휠 슬립 시 오차 누적
- 회전 시 큰 오차 (angular_scale 보정 필요)

**With EKF (Odom + IMU 융합):**
- IMU가 회전 변화 정밀 추적
- 휠 슬립 보상
- 더 부드러운 궤적

---

## 📝 **체크리스트**

시스템 시작 시 확인:

- [ ] TF 트리 정상 (`ros2 run tf2_tools view_frames`)
- [ ] 모든 변환 단일 발행자 (`ros2 run tf2_ros tf2_monitor`)
- [ ] IMU 드리프트 < 0.001 rad/s (`ros2 topic echo /imu/data_calibrated`)
- [ ] EKF가 IMU 반영 (`/odometry/filtered` 회전값 확인)
- [ ] SLAM이 맵 업데이트 (`ros2 topic echo /map --once`)

---

**결론: 현재 시스템은 TF 충돌 없이 협업 구조로 올바르게 설정되어 있습니다. ✅**

**문제는 TF 구조가 아닌 IMU 캘리브레이션입니다!** ⚠️
