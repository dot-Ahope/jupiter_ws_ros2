# 🔗 Nav2 TF 프레임 분리 문제 해결 가이드

## ❌ **문제 상황**

**증상:** Nav2 이동 중 `odom`이 `base_link`와 `map`에서 분리됨
- RViz에서 로봇이 두 곳에 표시됨
- Costmap이 로봇 위치를 추적하지 못함
- 경로 계획 실패 또는 이상 동작

---

## 🔍 **원인 분석**

### **TF 트리 구조 (정상)**

```
map (SLAM Toolbox 발행)
  ↓
odom (EKF 발행: odom → base_footprint)
  ↓
base_footprint (EKF 기준 프레임)
  ↓
base_link (robot_state_publisher 발행)
  ↓
laser, imu_link, camera_link 등
```

### **문제의 근본 원인: 프레임 불일치**

#### **1. robot_base_frame 불일치**

**잘못된 설정 (이전):**
```yaml
# EKF 설정
base_link_frame: base_footprint    # ← EKF는 base_footprint 사용

# Nav2 설정
robot_base_frame: base_link        # ← Nav2는 base_link 사용 ❌
```

**문제:**
```
EKF: odom → base_footprint 변환 발행
Nav2: odom → base_link 변환 요청
  ↓
TF lookup 실패 또는 시간 지연
  ↓
로봇 위치 분리 ❌
```

#### **2. global_frame 불일치 (local_costmap)**

**정상 TF 체인:**
```
odom (EKF 발행)
  ↓
base_footprint
```

**잘못된 참조:**
```yaml
local_costmap:
  global_frame: odom             # ✅ 맞음
  robot_base_frame: base_link    # ❌ EKF와 불일치
```

**결과:** local_costmap이 `odom → base_link` 변환을 찾지 못함

---

## ✅ **해결 방법**

### **핵심 원칙: 모든 Nav2 컴포넌트를 EKF 프레임에 맞춤**

EKF가 사용하는 프레임:
- `odom_frame: odom`
- `base_link_frame: base_footprint`

→ Nav2도 동일하게 설정 ✅

---

## 🔧 **적용된 수정 사항**

### **1. bt_navigator**

```yaml
# 변경 전
robot_base_frame: base_link        ❌

# 변경 후
robot_base_frame: base_footprint   ✅
```

**이유:** BT Navigator가 경로 계획 시 올바른 로봇 위치 참조

---

### **2. local_costmap**

```yaml
# 변경 전
global_frame: odom                 ✅ (맞음)
robot_base_frame: base_link        ❌

# 변경 후
global_frame: odom                 ✅
robot_base_frame: base_footprint   ✅
```

**이유:** Local costmap이 `odom → base_footprint` 변환 사용

---

### **3. global_costmap**

```yaml
# 변경 전
global_frame: map                  ✅ (맞음)
robot_base_frame: base_link        ❌

# 변경 후
global_frame: map                  ✅
robot_base_frame: base_footprint   ✅
```

**이유:** Global costmap이 `map → base_footprint` 변환 사용

---

### **4. behavior_server**

```yaml
# 변경 전
global_frame: odom                 ✅ (맞음)
robot_base_frame: base_link        ❌

# 변경 후
global_frame: odom                 ✅
robot_base_frame: base_footprint   ✅
```

**이유:** 복구 행동(Spin, BackUp)이 올바른 로봇 프레임 참조

---

## 📊 **TF 트리 구조 (수정 후)**

### **전체 TF 체인:**

```
map (SLAM Toolbox)
  ↓ [SLAM이 발행]
odom (EKF)
  ↓ [EKF가 발행]
base_footprint (EKF 기준)
  ↓ [robot_state_publisher 발행]
base_link (URDF 정의)
  ↓ [robot_state_publisher 발행]
├─ laser (LiDAR)
├─ imu_link (IMU)
├─ camera_link (Astra)
└─ 기타 센서들
```

### **Nav2 컴포넌트별 프레임 참조:**

| 컴포넌트 | global_frame | robot_base_frame | 변환 체인 |
|---------|-------------|-----------------|---------|
| **bt_navigator** | map | base_footprint | map → odom → base_footprint ✅ |
| **local_costmap** | odom | base_footprint | odom → base_footprint ✅ |
| **global_costmap** | map | base_footprint | map → odom → base_footprint ✅ |
| **behavior_server** | odom | base_footprint | odom → base_footprint ✅ |
| **controller_server** | - | - | costmap 프레임 상속 ✅ |

---

## 🎯 **핵심 개념**

### **base_link vs base_footprint**

| 프레임 | 위치 | 용도 | 발행자 |
|-------|------|------|-------|
| **base_footprint** | 바닥면 (z=0) | 로봇의 지면 투영 | EKF (odom 변환) |
| **base_link** | 로봇 중심 (z>0) | 센서/링크 부모 | robot_state_publisher |

**관계:**
```
base_footprint (z=0)
  ↓ [고정 변환: z축만 이동]
base_link (z=0.1~0.2)
```

**왜 EKF는 base_footprint를 사용하나?**
- 2D 네비게이션은 지면 기준 (z=0)
- 오도메트리 계산이 바닥면에서 이루어짐
- SLAM과 일관성 유지

---

## 🔍 **문제 진단 방법**

### **1. TF 트리 시각화**

```bash
# 시스템 실행 중
ros2 run tf2_tools view_frames

# frames.pdf 생성 → 확인
evince frames.pdf
```

**확인 사항:**
- [ ] map → odom 연결 (SLAM Toolbox)
- [ ] odom → base_footprint 연결 (EKF)
- [ ] base_footprint → base_link 연결 (robot_state_publisher)
- [ ] 모든 프레임이 하나의 트리로 연결

---

### **2. TF 변환 실시간 확인**

```bash
# odom → base_footprint 변환 (EKF)
ros2 run tf2_ros tf2_echo odom base_footprint

# map → base_footprint 변환 (전체 체인)
ros2 run tf2_ros tf2_echo map base_footprint

# base_footprint → base_link 변환 (static)
ros2 run tf2_ros tf2_echo base_footprint base_link
```

**정상 출력:**
```
At time 1234.567
- Translation: [x, y, z]
- Rotation: [x, y, z, w]
```

**오류 출력:**
```
Lookup would require extrapolation into the future.
```
→ 프레임 불일치 또는 발행 지연 ❌

---

### **3. Nav2 로그 확인**

```bash
ros2 launch sllidar_ros2 nav2_navigation.launch.py
```

**정상 로그:**
```
[controller_server]: Costmap update succeeded
[planner_server]: Planning path from (x,y) to (x,y)
```

**오류 로그:**
```
[controller_server]: Could not transform from base_link to odom
[planner_server]: Timed out waiting for transform
```
→ 프레임 불일치 확인 ❌

---

## 🚀 **적용 방법**

### **1. 수정 완료 ✅**

모든 Nav2 파라미터가 `base_footprint`로 통일되었습니다:
- `bt_navigator.robot_base_frame: base_footprint`
- `local_costmap.robot_base_frame: base_footprint`
- `global_costmap.robot_base_frame: base_footprint`
- `behavior_server.robot_base_frame: base_footprint`

### **2. 빌드 완료 ✅**

```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2 --symlink-install
```

### **3. 테스트**

```bash
# 터미널 1: 전체 시스템
ros2 launch sllidar_ros2 transbot_full_system.launch.py use_rviz:=true

# 터미널 2: Nav2
ros2 launch sllidar_ros2 nav2_navigation.launch.py

# 터미널 3: TF 확인
ros2 run tf2_ros tf2_echo map base_footprint
```

**확인 사항:**
- RViz에서 로봇이 한 곳에만 표시 ✅
- Costmap이 로봇을 정확히 추적 ✅
- TF 변환이 실시간으로 업데이트 ✅

---

## 📈 **추가 최적화 (선택사항)**

### **1. Transform Tolerance 조정**

빠른 이동 시 TF lookup 실패가 발생하면:

```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    transform_tolerance: 0.2        # 기본값
    → 0.5                           # 여유 증가

behavior_server:
  ros__parameters:
    transform_tolerance: 0.1
    → 0.3                           # 여유 증가
```

**주의:** 너무 높이면 오래된 변환 사용 가능

---

### **2. EKF 주파수 증가**

TF 업데이트가 느리면:

```yaml
# ekf_config.yaml
frequency: 30.0 Hz                  # 기본값
→ 50.0 Hz                           # 더 빠른 업데이트
```

**효과:** TF 변환이 더 자주 발행 (지연 감소)

**주의:** CPU 부하 증가 (Jetson에서 주의)

---

### **3. SLAM Toolbox TF 발행 주기**

map → odom 변환이 느리면:

```yaml
# slam_params.yaml
transform_publish_period: 0.02      # 50 Hz (기본값)
→ 0.01                              # 100 Hz (더 빠름)
```

**주의:** CPU 부하 증가

---

## 💡 **베스트 프랙티스**

### **1. 프레임 명명 일관성 유지**

모든 설정 파일에서 동일한 프레임 이름 사용:
- ✅ `base_footprint` (EKF 기준)
- ✅ `odom` (EKF 발행)
- ✅ `map` (SLAM 발행)

### **2. EKF를 TF 트리의 중심으로**

EKF가 `odom → base_footprint` 변환의 **유일한 발행자**:
- ❌ 다른 노드가 동일 변환 발행 금지
- ❌ static_transform_publisher로 덮어쓰기 금지

### **3. robot_state_publisher는 URDF만**

robot_state_publisher는 URDF 정의 joint만 발행:
- ✅ `base_footprint → base_link` (URDF에 정의)
- ✅ `base_link → laser` (URDF에 정의)
- ❌ `odom → base_footprint` (EKF가 담당)

---

## 🔍 **트러블슈팅**

### **Case 1: "Could not transform" 오류**

```
[ERROR] [controller_server]: Could not transform from base_link to odom
```

**원인:** Nav2가 `base_link`를 참조하는데 EKF는 `base_footprint` 발행

**해결:**
```yaml
robot_base_frame: base_link → base_footprint
```

---

### **Case 2: RViz에서 로봇이 두 곳에 표시**

**원인:** 
- Nav2가 `base_link` 사용
- SLAM이 `base_footprint` 기준
- 두 프레임이 다른 위치

**해결:** Nav2를 `base_footprint`로 통일

---

### **Case 3: Costmap이 로봇을 추적 안 함**

**원인:** local_costmap의 `robot_base_frame` 불일치

**해결:**
```yaml
local_costmap:
  robot_base_frame: base_footprint
```

---

### **Case 4: "Extrapolation into the future" 경고**

**원인:** TF 변환 타이밍 문제

**해결:**
1. `transform_tolerance` 증가 (0.2 → 0.5)
2. EKF `frequency` 증가 (30 → 50 Hz)
3. 시스템 클럭 동기화 확인

---

## 📝 **체크리스트**

설정 파일에서 다음 확인:

### **EKF (ekf_config.yaml)**
- [ ] `base_link_frame: base_footprint`
- [ ] `odom_frame: odom`
- [ ] `publish_tf: true`

### **Nav2 (nav2_params.yaml)**
- [ ] `bt_navigator.robot_base_frame: base_footprint`
- [ ] `local_costmap.robot_base_frame: base_footprint`
- [ ] `global_costmap.robot_base_frame: base_footprint`
- [ ] `behavior_server.robot_base_frame: base_footprint`
- [ ] `local_costmap.global_frame: odom`
- [ ] `global_costmap.global_frame: map`

### **SLAM (slam_params.yaml)**
- [ ] `base_frame: base_footprint`
- [ ] `odom_frame: odom`
- [ ] `map_frame: map`

---

## 📊 **요약**

| 문제 | 원인 | 해결 |
|------|------|------|
| odom 분리 | `robot_base_frame` 불일치 | 모두 `base_footprint`로 통일 ✅ |
| TF lookup 실패 | 프레임 이름 오류 | EKF 프레임 이름과 일치 ✅ |
| Costmap 미추적 | local_costmap 프레임 불일치 | `base_footprint` 설정 ✅ |
| 경로 계획 실패 | BT Navigator 프레임 불일치 | `base_footprint` 설정 ✅ |

**핵심 원칙:**
- 🎯 **EKF가 TF 트리의 중심**
- 🎯 **모든 Nav2 컴포넌트는 EKF 프레임 따름**
- 🎯 **base_footprint 통일 사용**

**✨ 이제 TF 프레임이 완벽하게 연결되어 분리 문제가 해결되었습니다!**
