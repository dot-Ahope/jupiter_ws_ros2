# Nav2 네비게이션 설정 문제 해결 가이드

## 📋 목차
1. [개요](#개요)
2. [문제 1: BT Navigator 플러그인 누락](#문제-1-bt-navigator-플러그인-누락)
3. [문제 2: Behavior Tree XML 파일 비어있음](#문제-2-behavior-tree-xml-파일-비어있음)
4. [문제 3: XML 파일 경로 오류](#문제-3-xml-파일-경로-오류)
5. [최종 설정](#최종-설정)
6. [사용 방법](#사용-방법)

---

## 개요

**목표:** SLAM Toolbox와 함께 Nav2를 사용하여 실시간 맵 생성하면서 자율 주행

**환경:**
- ROS2 Humble
- Transbot (차동 구동 로봇)
- RPLidar A1
- SLAM Toolbox (맵 생성 + 위치 추정)
- Nav2 (경로 계획 + 장애물 회피)

---

## 문제 1: BT Navigator 플러그인 누락

### 🔴 에러 메시지
```
[bt_navigator-4] [ERROR] [1761700603.072174288] [bt_navigator]: Exception when loading BT: Error at line 12: -> Node not recognized: RemovePassedGoals
[bt_navigator-4] [ERROR] [1761700603.072265971] [bt_navigator]: Error loading XML file: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
[lifecycle_manager-6] [ERROR] [1761700603.073268336] [lifecycle_manager_navigation]: Failed to change state for node: bt_navigator
[lifecycle_manager-6] [ERROR] [1761700603.073407156] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

### 🔍 원인
`nav2_params.yaml`의 `bt_navigator` 섹션에서 `plugin_lib_names`에 필요한 플러그인이 누락됨:
- `nav2_navigate_through_poses_action_bt_node` ❌
- `nav2_remove_passed_goals_action_bt_node` ❌

### ✅ 해결 방법

**파일:** `/home/user/transbot_ws_ros2/src/sllidar_ros2/config/nav2_params.yaml`

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_navigate_through_poses_action_bt_node      # ✅ 추가됨
    - nav2_remove_passed_goals_action_bt_node         # ✅ 추가됨
    - nav2_controller_cancel_bt_node
```

**추가:** `bt_navigator_navigate_to_pose_rclcpp_node`, `bt_navigator_navigate_through_poses_rclcpp_node` 섹션 추가

### 📚 플러그인 설명

Nav2 Behavior Tree는 다양한 플러그인을 조합하여 복잡한 네비게이션 로직을 구현합니다. 각 플러그인의 역할:

#### **Action 노드 (실행)**
| 플러그인 | 역할 |
|---------|------|
| `nav2_compute_path_to_pose_action_bt_node` | 현재 위치에서 목표까지 경로 계산 요청 |
| `nav2_compute_path_through_poses_action_bt_node` | 여러 waypoint를 거쳐가는 경로 계산 |
| `nav2_follow_path_action_bt_node` | 계산된 경로를 따라 로봇 이동 |
| `nav2_spin_action_bt_node` | 제자리 회전 (360도 스캔 등) |
| `nav2_wait_action_bt_node` | 지정된 시간 대기 |
| `nav2_back_up_action_bt_node` | 후진 이동 (막혔을 때) |
| `nav2_truncate_path_action_bt_node` | 경로 일부 자르기 (재계획 시) |
| `nav2_navigate_to_pose_action_bt_node` | 단일 목표로 네비게이션 (전체 프로세스) |
| `nav2_navigate_through_poses_action_bt_node` | 여러 목표를 순차 방문 |
| `nav2_remove_passed_goals_action_bt_node` | ⭐ 지나간 waypoint 제거 (필수!) |

#### **Service 노드 (서비스 호출)**
| 플러그인 | 역할 |
|---------|------|
| `nav2_clear_costmap_service_bt_node` | Costmap 초기화 (오래된 장애물 제거) |

#### **Condition 노드 (조건 판단)**
| 플러그인 | 역할 |
|---------|------|
| `nav2_is_stuck_condition_bt_node` | 로봇이 막혔는지 확인 |
| `nav2_goal_reached_condition_bt_node` | 목표 도착 확인 |
| `nav2_goal_updated_condition_bt_node` | 새로운 목표 수신 확인 |
| `nav2_is_path_valid_condition_bt_node` | 현재 경로가 유효한지 확인 |
| `nav2_transform_available_condition_bt_node` | TF 변환 가능 여부 확인 |
| `nav2_time_expired_condition_bt_node` | 타임아웃 확인 |
| `nav2_distance_traveled_condition_bt_node` | 이동 거리 확인 |

#### **Control 노드 (실행 흐름 제어)**
| 플러그인 | 역할 |
|---------|------|
| `nav2_rate_controller_bt_node` | 실행 주기 제어 (Hz) |
| `nav2_distance_controller_bt_node` | 거리 기반 재실행 |
| `nav2_speed_controller_bt_node` | 속도 기반 재실행 |
| `nav2_pipeline_sequence_bt_node` | 순차 실행 (파이프라인) |
| `nav2_round_robin_node_bt_node` | 라운드 로빈 스케줄링 |
| `nav2_recovery_node_bt_node` | 복구 행동 실행 |
| `nav2_goal_updater_node_bt_node` | 목표 업데이트 처리 |
| `nav2_controller_cancel_bt_node` | Controller 취소 |

#### **Decorator 노드 (보조)**
| 플러그인 | 역할 |
|---------|------|
| `nav2_single_trigger_bt_node` | 한 번만 실행 (재실행 방지) |

### 💡 주요 플러그인 상세 설명

#### 1. `nav2_remove_passed_goals_action_bt_node` ⭐
- **중요도:** 필수
- **역할:** 여러 waypoint를 방문할 때, 이미 지나간 지점을 목록에서 제거
- **없으면:** "Node not recognized: RemovePassedGoals" 에러 발생
- **사용 케이스:** 순찰, 여러 지점 방문

#### 2. `nav2_navigate_to_pose_action_bt_node` 
- **역할:** 단일 목표로 이동하는 전체 프로세스 (경로 계획 + 추종)
- **내부 동작:**
  1. Planner에 경로 요청
  2. Controller에 경로 추종 요청
  3. 목표 도달 확인
  4. 실패 시 복구 행동
- **사용:** RViz2 "2D Goal Pose"로 목표 설정 시

#### 3. `nav2_compute_path_to_pose_action_bt_node`
- **역할:** Planner Server에 경로 계산 요청
- **입력:** 시작점, 목표점
- **출력:** 경로 (Path 메시지)
- **실패 케이스:** 경로 없음, 목표가 장애물 안, TF 오류

#### 4. `nav2_follow_path_action_bt_node`
- **역할:** Controller Server에 경로 추종 요청
- **입력:** 계획된 경로
- **출력:** 속도 명령 (`/cmd_vel`)
- **실패 케이스:** 경로 이탈, 장애물 충돌, 진행 없음

#### 5. `nav2_clear_costmap_service_bt_node`
- **역할:** 오래된 장애물 정보 삭제 (복구 행동)
- **사용 시점:** 로봇이 막혔을 때, 센서 오류 시
- **효과:** 잘못된 장애물 제거 후 재시도

---

## 문제 2: Behavior Tree XML 파일 비어있음

### 🔴 에러 메시지
```
[bt_navigator-4] [INFO] [1761700900.189203027] [bt_navigator]: Begin navigating from current location (-0.07, -0.10) to (-0.09, -0.08)
[bt_navigator-4] [ERROR] [1761700900.189505659] [BehaviorTreeEngine]: Behavior tree threw exception: Empty Tree. Exiting with failure.
[bt_navigator-4] [WARN] [1761700900.189543772] [bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
[bt_navigator-4] [ERROR] [1761700900.189723553] [bt_navigator]: Goal failed
```

### 🔍 원인
RViz2에서 "2D Goal Pose"를 설정했지만, BT Navigator에 실행할 Behavior Tree XML 파일이 지정되지 않음:

```yaml
# ❌ 잘못된 설정
default_nav_to_pose_bt_xml: ""
default_nav_through_poses_bt_xml: ""
```

### ⚠️ 시도한 해결 방법 (실패)

**package:// URI 사용 (작동하지 않음):**
```yaml
# ❌ 패키지 URI - ROS2에서 인식하지 못함
default_nav_to_pose_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
```

**에러:**
```
[bt_navigator-4] [ERROR] [1761701188.640342817] [bt_navigator]: Couldn't open input XML file: package://nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
```

### ✅ 해결 방법 (절대 경로 사용)

사용 가능한 XML 파일 확인:
```bash
ls /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/
```

출력:
```
navigate_to_pose_w_replanning_and_recovery.xml
navigate_through_poses_w_replanning_and_recovery.xml
navigate_to_pose_w_replanning_goal_patience_and_recovery.xml
...
```

**최종 설정 (절대 경로):**
```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"
```

### 📊 주요 BT XML 파일 설명

| XML 파일 | 용도 | 특징 |
|---------|------|------|
| `navigate_to_pose_w_replanning_and_recovery.xml` | **단일 목표 네비게이션** | 기본 추천, 경로 재계획 + 복구 포함 |
| `navigate_through_poses_w_replanning_and_recovery.xml` | **여러 waypoint 방문** | 순찰, 배달 경로 등 |
| `navigate_to_pose_w_replanning_goal_patience_and_recovery.xml` | 목표 인내심 포함 | 목표 근처에서 재시도 |
| `navigate_w_replanning_only_if_path_becomes_invalid.xml` | 경로 무효화 시만 재계획 | CPU 부하 최소화 |
| `navigate_w_replanning_time.xml` | 시간 기반 재계획 | 정기적 경로 업데이트 (동적 환경) |
| `navigate_w_replanning_distance.xml` | 거리 기반 재계획 | 일정 거리마다 경로 업데이트 |

### 🔄 Behavior Tree 동작 원리

**navigate_to_pose_w_replanning_and_recovery.xml 예시:**

```
PipelineSequence (순차 실행)
├── RateController (재계획 주기 제어: 1Hz)
│   └── RecoveryNode (실패 시 복구)
│       ├── PipelineSequence (메인 네비게이션)
│       │   ├── ComputePathToPose (경로 계획)
│       │   ├── FollowPath (경로 추종)
│       │   └── GoalReached? (목표 도달 확인)
│       └── RecoveryActions (복구 행동)
│           ├── ClearCostmap (맵 초기화)
│           ├── Spin (360도 회전)
│           └── BackUp (후진)
```

**실행 흐름:**
1. **RateController:** 1초마다 경로 재계획 (동적 장애물 대응)
2. **ComputePathToPose:** Planner에 경로 요청
3. **FollowPath:** Controller에 경로 추종 요청
4. **실패 감지:** 
   - 경로 없음 → ClearCostmap 후 재시도
   - 막힘 → Spin (주변 스캔) → 재시도
   - 여전히 실패 → BackUp (후진) → 재시도
5. **GoalReached:** 목표 도달 시 SUCCESS 반환

---

## 문제 3: XML 파일 경로 오류

### 🔴 증상
Global Costmap 경고 메시지 반복:
```
[planner_server-2] [WARN] [1761700074.507873305] [global_costmap.global_costmap]: Sensor origin at (-0.03, 0.00) is out of map bounds (-0.03, -1.50) to (2.55, 3.13). The costmap cannot raytrace for it.
```

### 🔍 원인
로봇이 맵 경계에 위치하여 LiDAR raytracing 실패

### ✅ 해결 방법
이것은 **경고일 뿐**이며 네비게이션 동작에는 영향 없음. 하지만 더 안정적인 동작을 위해:

**옵션 1: 로봇을 맵 중앙으로 이동**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**옵션 2: Global Costmap 설정 조정** (선택사항)
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      track_unknown_space: true
      allow_unknown: true  # 미지의 영역 허용
```

---

## 최종 설정

### 파일 구조
```
transbot_ws_ros2/
└── src/
    └── sllidar_ros2/
        ├── config/
        │   └── nav2_params.yaml          # Nav2 전체 파라미터
        └── launch/
            └── nav2_navigation.launch.py  # Nav2 런치 파일
```

### nav2_params.yaml 핵심 섹션

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    
    # ✅ 절대 경로로 BT XML 지정
    default_nav_to_pose_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"
    
    # ✅ 전체 플러그인 목록 (누락 없음)
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_controller_cancel_bt_node

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: False

bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: False

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    # ... (DWB 설정)

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true  # ✅ 미탐색 영역 허용

# ... (나머지 설정)
```

---

## 사용 방법

### 1. 시스템 실행

**터미널 1: 로봇 + SLAM**
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py use_rviz:=true
```

**터미널 2: Nav2 네비게이션**
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 nav2_navigation.launch.py
```

### 2. RViz2 설정

**추가할 Display:**
1. **Path** → Topic: `/plan` (글로벌 경로, 파란색)
2. **Map** → Topic: `/global_costmap/costmap` (장애물 맵)
3. **Map** → Topic: `/local_costmap/costmap` (로컬 장애물)

**이미 있는 Display:**
- ✅ Map (`/map` - SLAM 맵)
- ✅ LaserScan (`/scan`)
- ✅ TF (로봇 위치)
- ✅ RobotModel

### 3. 네비게이션 사용

1. RViz2 상단 툴바에서 **"2D Goal Pose"** 버튼 클릭
2. 맵에서 목표 위치 **클릭**
3. 마우스 **드래그**하여 로봇이 향할 방향 지정
4. 마우스 놓기 → **자동으로 이동 시작!** 🚀

### 4. 정상 동작 확인

**성공적인 로그 메시지:**
```
[lifecycle_manager_navigation]: Server controller_server connected with bond.
[lifecycle_manager_navigation]: Server planner_server connected with bond.
[lifecycle_manager_navigation]: Server behavior_server connected with bond.
[lifecycle_manager_navigation]: Server bt_navigator connected with bond.
[lifecycle_manager_navigation]: Server velocity_smoother connected with bond.
[lifecycle_manager_navigation]: Managed nodes are active
[lifecycle_manager_navigation]: Creating bond timer...
```

**목표 수신 시:**
```
[bt_navigator]: Begin navigating from current location (x, y) to (goal_x, goal_y)
```

**에러가 없으면 성공!** ✅

---

## 문제 해결 체크리스트

### ❌ "Empty Tree" 에러
```
[bt_navigator] [ERROR]: Behavior tree threw exception: Empty Tree
```
→ `default_nav_to_pose_bt_xml`이 비어있거나 잘못된 경로  
→ **해결:** 절대 경로로 수정

### ❌ "Node not recognized: RemovePassedGoals"
```
[bt_navigator] [ERROR]: Node not recognized: RemovePassedGoals
```
→ `plugin_lib_names`에 필요한 플러그인 누락  
→ **해결:** `nav2_remove_passed_goals_action_bt_node` 추가

### ❌ "Couldn't open input XML file"
```
[bt_navigator] [ERROR]: Couldn't open input XML file: package://...
```
→ `package://` URI가 ROS2에서 인식되지 않음  
→ **해결:** 절대 경로 사용 (`/opt/ros/humble/share/...`)

### ⚠️ "Sensor origin out of map bounds"
```
[global_costmap]: Sensor origin at (-0.03, 0.00) is out of map bounds
```
→ 로봇이 맵 경계에 위치 (경고일 뿐, 치명적이지 않음)  
→ **해결:** 텔레옵으로 맵 중앙으로 이동 또는 무시

---

## 추가 팁

### SLAM + Nav2 동시 사용

**가능합니다!** SLAM Toolbox가 실시간으로 맵을 업데이트하면서 Nav2가 그 맵을 사용하여 네비게이션을 수행합니다.

**워크플로우:**
1. 텔레옵으로 주요 영역 탐색 (맵 생성)
2. Nav2 활성화
3. 이미 탐색한 영역에 목표 설정
4. 새로운 영역 발견 시 맵 자동 업데이트

**주의사항:**
- 미탐색 영역에는 목표 설정 불가 (Unknown 영역)
- CPU 부하 증가 (SLAM + Nav2 동시 실행)
- 천천히 움직여 SLAM 품질 확보

### 맵 저장 (선택)

탐색이 끝나면 맵 저장:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

다음부터는 AMCL 모드로 더 빠르게 사용 가능.

---

## 참고 자료

- [Nav2 공식 문서](https://navigation.ros.org/)
- [Behavior Tree XML 가이드](https://navigation.ros.org/behavior_trees/index.html)
- [Nav2 파라미터 튜닝](https://navigation.ros.org/tuning/index.html)

---

**작성일:** 2025-10-29  
**작성자:** GitHub Copilot  
**버전:** ROS2 Humble  
**테스트 환경:** Jetson (Transbot)
