# Nav2 네비게이션 종합 가이드

> **생성일:** 2025-10-31  
> **통합 문서:** Nav2 설정, 문제 해결, 속도 튜닝, TF 프레임 관리

## 📅 작업 타임라인

**작업 기간:** 2025년 10월 29일 ~ 10월 31일

### 주요 작업 일정
- **2025-10-29**:
  - Nav2 네비게이션 초기 통합 및 문제 해결 가이드 작성
  - SLAM 품질 개선을 위한 속도 제한 설정 (오버슈트/진동 문제 발생)
- **2025-10-30**:
  - 속도 제한 설정 최적화 (진동 문제 해결)
  - 회전 속도만 제한, 가속도 제한 유지 전략 수립
- **2025-10-31**:
  - TF 프레임 일관성 문제 해결 (base_link vs base_footprint)
  - transbot_nav 패키지 생성 및 마이그레이션 완료

### 참조된 원본 문서들
- `Nav2_네비게이션_문제해결_가이드.md` (2025-10-29)
- `Nav2_속도_제한_설정.md` (2025-10-30)

---

## 📋 목차
1. [Nav2 개요](#nav2-개요)
2. [속도 제한 전략](#속도-제한-전략)
3. [진동 문제 해결](#진동-문제-해결)
4. [TF 프레임 관리](#tf-프레임-관리)
5. [문제 해결 가이드](#문제-해결-가이드)

---

## Nav2 개요

### 1.1 Nav2란?

**ROS2 Navigation Stack (Navigation2):**
- 자율 주행을 위한 공식 ROS2 패키지
- 경로 계획, 장애물 회피, 행동 제어 통합

**주요 컴포넌트:**

```
[목표 설정] → [BT Navigator] → [Planner Server] → [전역 경로]
                     ↓
              [Controller Server] → [지역 경로] → [cmd_vel]
                     ↓
              [Behavior Server] → [회전, 후진 등]
                     ↓
              [Costmap] → [장애물 맵]
```

### 1.2 Transbot Nav2 구조

```
[RViz] - 목표 설정 (2D Goal Pose)
   ↓
[BT Navigator] - 행동 트리 의사결정
   ↓
[Planner Server] - 전역 경로 계획 (NavFn)
   ↓
[Controller Server] - 지역 경로 추적 (DWB)
   ↓
[Velocity Smoother] - 속도 명령 부드럽게
   ↓
[/cmd_vel] → [Transbot_Driver] → 모터 제어
```

### 1.3 필수 요구사항

**센서:**
- ✅ LiDAR (장애물 감지)
- ✅ Odometry (위치 추정)
- ✅ IMU (방향 정보)

**맵:**
- ✅ SLAM Toolbox (실시간 맵 생성)
- 또는 사전 생성된 맵 (map_server)

**TF 프레임:**
```
map → odom → base_footprint → base_link → laser_frame
```

---

## 속도 제한 전략

### 2.1 문제 배경

**초기 문제:**
- 너무 빠른 회전 (0.5 rad/s) → SLAM 맵 품질 저하
- 해결 시도: 모든 속도/가속도 제한 → 로봇 진동 발생

**해결 방향:**
- **회전 속도만 제한** (SLAM 품질 향상)
- **가속도는 유지** (DWB 경로 계획 안정성)

### 2.2 속도 제한 설정

**nav2_params.yaml (최적화):**

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # ========== 속도 제한 (핵심) ==========
      
      # 전진 속도 (빠르게 - SLAM에 영향 적음)
      max_vel_x: 0.3              # m/s
      min_vel_x: -0.1             # 후진
      
      # 회전 속도 (천천히 - SLAM 품질 중요) ⭐
      max_vel_theta: 0.5          # rad/s (~29°/s)
      min_vel_theta: -0.5
      min_speed_theta: 0.1
      
      # 전체 속도
      max_speed_xy: 0.3
      min_speed_xy: 0.0
      
      # ========== 가속도 제한 (유지 - 진동 방지) ==========
      
      # 전진 가속도 (높게 유지)
      acc_lim_x: 2.5              # m/s²
      decel_lim_x: -2.5
      
      # 회전 가속도 (높게 유지) ⭐
      acc_lim_theta: 3.2          # rad/s²
      decel_lim_theta: -3.2
      
      # ========== DWB 설정 ==========
      
      # 궤적 생성 (샘플링)
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      
      # 시뮬레이션 시간
      sim_time: 1.7
      
      # 목표 허용 오차
      xy_goal_tolerance: 0.10     # 10cm
      yaw_goal_tolerance: 0.10    # ~6도
      
      # 비용 함수 가중치
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
```

### 2.3 왜 이렇게 설정했나?

**속도 vs 가속도의 차이:**

```python
# 속도 제한 (max_vel_theta)
# → DWB가 생성하는 궤적의 "최대값"
# → SLAM 품질에 직접 영향
# 
# 예: max_vel_theta = 0.5
# → 모든 궤적이 0.5 rad/s 이하
# → 천천히 회전 → 스캔 매칭 정확

# 가속도 제한 (acc_lim_theta)
# → 궤적의 "변화율"
# → DWB 경로 계획의 "유연성"
# 
# 예: acc_lim_theta = 3.2
# → 빠르게 가속/감속 가능
# → 다양한 궤적 생성 → 장애물 회피 능력
```

**잘못된 설정 (진동 발생):**

```yaml
# ❌ 모두 제한 (문제)
max_vel_theta: 0.3
acc_lim_theta: 0.5   # ← 너무 낮음!

# 결과:
# - DWB가 궤적을 거의 생성 못함
# - 좁은 샘플 공간 → 최적 경로 찾기 어려움
# - 로봇이 좌우로 진동
```

**올바른 설정 (안정):**

```yaml
# ✅ 속도만 제한 (해결)
max_vel_theta: 0.5   # 회전은 천천히
acc_lim_theta: 3.2   # 가속은 빠르게 ⭐

# 결과:
# - SLAM 품질 좋음 (천천히 회전)
# - 경로 계획 안정 (다양한 궤적)
# - 진동 없음
```

### 2.4 속도별 성능 비교

| max_vel_theta | SLAM 품질 | 경로 계획 | 주행 시간 | 권장 |
|---------------|-----------|-----------|-----------|------|
| 1.0 rad/s | 나쁨 (맵 왜곡) | 좋음 | 빠름 | ❌ |
| 0.5 rad/s | 좋음 | 좋음 | 보통 | ✅ 권장 |
| 0.3 rad/s | 매우 좋음 | 좋음 | 느림 | ⚠️ 느림 |
| 0.2 rad/s | 최고 | 보통 | 매우 느림 | ❌ 비실용적 |

---

## 진동 문제 해결

### 3.1 진동 증상

**관찰:**
```
로봇이 목표를 향해 이동하다가
좌우로 흔들림 (진동, Oscillation)

  [목표]
    ↑
   /|\
  / | \   ← 지그재그
 /  |  \
[로봇]
```

**원인:**
- DWB가 생성할 수 있는 궤적이 제한됨
- 샘플 공간이 너무 좁음
- 가속도 제한이 너무 낮음

### 3.2 진동 원인 분석

**DWB 동작 원리:**

```python
# DWB (Dynamic Window Approach)
# 1. 현재 속도 확인
current_vx = 0.2  # m/s
current_vtheta = 0.1  # rad/s

# 2. 가능한 속도 범위 계산 (Dynamic Window)
vx_min = current_vx - acc_lim_x * dt
vx_max = current_vx + acc_lim_x * dt
vtheta_min = current_vtheta - acc_lim_theta * dt
vtheta_max = current_vtheta + acc_lim_theta * dt

# 3. 샘플링 (궤적 생성)
for vx in sample(vx_min, vx_max, vx_samples):
    for vtheta in sample(vtheta_min, vtheta_max, vtheta_samples):
        trajectory = simulate(vx, vtheta, sim_time)
        cost = evaluate(trajectory)
        # 최소 비용 궤적 선택

# 4. 문제 발생 조건
if acc_lim_theta is too_low:
    # Dynamic Window가 너무 좁음
    # → 샘플 수가 적음
    # → 최적 경로를 못 찾음
    # → 진동 발생
```

**예시:**

```yaml
# 가속도 높음 (정상)
acc_lim_theta: 3.2
# 
# 현재: 0.1 rad/s
# dt = 0.05s
# Window: 0.1 ± (3.2 × 0.05) = [0.0, 0.26] rad/s
# 20개 샘플: 0.0, 0.013, 0.026, ..., 0.26
# → 다양한 궤적 생성 가능 ✅

# 가속도 낮음 (문제)
acc_lim_theta: 0.5
# 
# 현재: 0.1 rad/s
# Window: 0.1 ± (0.5 × 0.05) = [0.075, 0.125] rad/s
# 20개 샘플: 0.075, 0.078, ..., 0.125
# → 거의 비슷한 궤적만 생성 ❌
# → 목표 추적 어려움 → 진동
```

### 3.3 진동 해결책

**1) 가속도 제한 복원**

```yaml
# nav2_params.yaml

# Before (진동 발생)
acc_lim_theta: 0.5   # 너무 낮음

# After (해결)
acc_lim_theta: 3.2   # 충분히 높음 ⭐
```

**2) 샘플 수 증가**

```yaml
# DWB 샘플링 설정
vx_samples: 20       # 전진 방향
vtheta_samples: 20   # 회전 방향

# 효과:
# - 더 많은 궤적 후보
# - 더 정확한 경로
```

**3) 시뮬레이션 시간 조정**

```yaml
# 궤적 예측 시간
sim_time: 1.7  # 초

# 의미:
# - 1.7초 앞을 예측하여 궤적 생성
# - 너무 짧으면: 근시안적 경로
# - 너무 길면: 계산 부하
```

### 3.4 진동 검증

```bash
# 1. Nav2 실행
ros2 launch transbot_nav nav2_navigation.launch.py

# 2. 목표 설정 (RViz)
# "2D Goal Pose" 버튼 클릭 → 목표 지정

# 3. 경로 관찰
# - 직선에 가까운 부드러운 경로: ✅ 정상
# - 지그재그 경로: ❌ 진동

# 4. 속도 모니터링
ros2 topic echo /cmd_vel

# 정상:
# angular.z: 0.234 (부드럽게 변화)
# 
# 진동:
# angular.z: 0.5, -0.4, 0.5, -0.3... (급변)
```

---

## TF 프레임 관리

### 4.1 TF 프레임 구조

**Transbot TF Tree:**

```
map (SLAM Toolbox가 발행)
 └─ odom (EKF가 발행)
     └─ base_footprint (로봇 지면 투영, EKF base_link_frame)
         └─ base_link (로봇 중심)
             ├─ laser_frame (LiDAR)
             ├─ imu_link (IMU)
             └─ camera_link (카메라)
```

### 4.2 TF 프레임 일관성

**중요:**
- 모든 Nav2 노드가 **동일한 robot_base_frame** 사용해야 함
- EKF의 `base_link_frame`과 일치 필수

**nav2_params.yaml 전체:**

```yaml
# ========== 모든 서버에서 일관성 유지 ==========

planner_server:
  ros__parameters:
    robot_base_frame: base_footprint  # ⭐

controller_server:
  ros__parameters:
    robot_base_frame: base_footprint  # ⭐

behavior_server:
  ros__parameters:
    robot_base_frame: base_footprint  # ⭐

local_costmap:
  local_costmap:
    ros__parameters:
      robot_base_frame: base_footprint  # ⭐
      global_frame: odom

global_costmap:
  global_costmap:
    ros__parameters:
      robot_base_frame: base_footprint  # ⭐
      global_frame: map
```

### 4.3 Odom 드리프트 이해

**정상 동작:**

```
# 시간 0초: 모두 일치
map
 └─ odom (0, 0, 0)
     └─ base_footprint (0, 0, 0)

# 시간 10초: odom 드리프트 발생
map
 └─ odom (0.1, 0.05, 2°)  ← SLAM이 보정
     └─ base_footprint (0, 0, 0)  ← 로봇 위치는 정확

# odom이 base_footprint에서 "분리"되는 것은 정상!
# 이것이 SLAM의 보정 메커니즘입니다.
```

**설명:**

```python
# SLAM 동작:
# 1. Odometry로 예측: "로봇이 (1, 0)로 이동했을 것"
# 2. LiDAR 스캔 매칭: "실제로는 (0.95, 0.02)에 있음"
# 3. 차이 계산: (0.05, 0.02) 드리프트
# 4. map → odom TF 보정
# 
# 결과:
# - odom 프레임이 map에서 (0.05, 0.02) 떨어짐
# - 하지만 base_footprint는 map 기준으로 정확한 위치
```

### 4.4 TF 문제 해결

**"Transform timeout" 에러:**

```bash
# 증상
[controller_server]: Transform from base_footprint to map timeout

# 원인
# 1. 프레임 이름 불일치
# 2. TF가 발행되지 않음
# 3. 네트워크 지연 (드물게)

# 해결
# 1. TF tree 확인
ros2 run tf2_tools view_frames

# 2. TF 발행 확인
ros2 topic echo /tf

# 3. 프레임 이름 검증
# - ekf_config.yaml: base_link_frame
# - nav2_params.yaml: robot_base_frame
# 두 값이 일치해야 함!
```

**"Multiple publishers" 경고:**

```bash
# 증상
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp

# 원인
# 같은 TF를 여러 노드가 발행

# 해결
# 1. TF 발행자 확인
ros2 run tf2_tools view_frames
# frames_*.gv 파일에서 발행자 확인

# 2. 중복 제거
# - Transbot_Driver: odom → base_footprint 발행
# - EKF: odom → base_footprint 발행 (보정된 값)
# → Transbot_Driver의 base_link_frame을 변경하거나
# → EKF가 최종 TF 발행하도록 설정
```

---

## 문제 해결 가이드

### 5.1 "No plan could be generated"

**증상:**
```
[planner_server]: Failed to generate plan
[bt_navigator]: Goal aborted
```

**원인:**
1. 목표가 장애물 안에 있음
2. Costmap에 장애물이 많음
3. 경로를 찾을 수 없음 (막힌 공간)

**해결:**

```yaml
# 1. Inflation 반경 줄이기
# global_costmap/local_costmap
inflation_radius: 0.3  # 0.5 → 0.3
cost_scaling_factor: 5.0  # 10.0 → 5.0

# 2. 목표 허용 오차 증가
xy_goal_tolerance: 0.15  # 0.10 → 0.15
yaw_goal_tolerance: 0.15

# 3. Planner timeout 증가
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_timeout: 2.0  # 초
```

### 5.2 "Costmap out of date"

**증상:**
```
[controller_server]: Costmap is out of date
```

**원인:**
1. LiDAR 데이터 지연
2. TF 타임스탬프 문제
3. Costmap 업데이트 주파수 낮음

**해결:**

```bash
# 1. LiDAR 주파수 확인
ros2 topic hz /scan
# Expected: ~10 Hz

# 2. TF 확인
ros2 run tf2_tools view_frames

# 3. Costmap 주파수 증가
# nav2_params.yaml
update_frequency: 5.0  # Hz
publish_frequency: 2.0
```

### 5.3 로봇이 안 움직여요

**증상:**
- 경로는 계획됨
- 하지만 로봇 정지

**원인:**
1. cmd_vel 토픽 문제
2. Controller 노드 미활성화
3. 속도 제한 너무 낮음

**해결:**

```bash
# 1. cmd_vel 확인
ros2 topic echo /cmd_vel
# 값이 0이 아닌지 확인

# 2. Controller 상태 확인
ros2 lifecycle get /controller_server
# Expected: active (4)

# 3. 수동 제어 테스트
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" -1
# 로봇이 움직이는지 확인

# 4. Velocity Smoother 확인
ros2 topic echo /cmd_vel_nav  # Smoother 입력
ros2 topic echo /cmd_vel      # Smoother 출력
```

### 5.4 목표 근처에서 멈춰요

**증상:**
- 목표 50cm 앞에서 멈춤
- "Goal reached" 안 뜸

**원인:**
- 목표 허용 오차 너무 작음

**해결:**

```yaml
# nav2_params.yaml
xy_goal_tolerance: 0.15   # 0.05 → 0.15 (15cm)
yaw_goal_tolerance: 0.15  # 0.05 → 0.15 (~9도)

# Stateful 옵션 활성화
stateful: true  # 목표 도달 후 유지
```

### 5.5 Recovery 행동 반복

**증상:**
- 로봇이 계속 제자리 회전
- 또는 앞뒤로 반복 이동

**원인:**
- Recovery behavior 무한 루프

**해결:**

```yaml
# behavior_server 설정
behavior_server:
  ros__parameters:
    # Recovery 재시도 제한
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    
    # 각 behavior의 timeout
    spin:
      max_rotational_vel: 0.5
      min_rotational_vel: 0.2
      rotational_acc_lim: 1.0
    
    backup:
      max_linear_vel: 0.2
      min_linear_vel: 0.05
      linear_acc_lim: 1.0
```

---

## 성능 최적화 팁

### 6.1 빠른 주행

```yaml
# 속도 우선 설정
max_vel_x: 0.5          # 빠르게
max_vel_theta: 0.8
controller_frequency: 30.0  # 높은 제어 주파수
```

### 6.2 정확한 주행

```yaml
# 정확도 우선 설정
max_vel_x: 0.2          # 천천히
max_vel_theta: 0.3
xy_goal_tolerance: 0.05  # 5cm 허용 오차
yaw_goal_tolerance: 0.05
```

### 6.3 좁은 공간

```yaml
# 장애물 회피 우선
inflation_radius: 0.2    # 작은 inflation
cost_scaling_factor: 3.0
min_obstacle_dist: 0.1   # 최소 거리
```

---

## 관련 문서

- `04_SLAM_OPTIMIZATION.md` - SLAM 최적화
- `transbot_nav/README.md` - 전체 시스템 통합
- `transbot_nav/QUICK_REFERENCE.md` - 빠른 참조

---

**문서 통합 완료:** 2025-10-31  
**원본 파일들:**
- Nav2_네비게이션_문제해결_가이드.md
- Nav2_회전속도_제한_설정.md
- Nav2_속도_제한_설정.md
- Nav2_진동_문제_분석.md
- Nav2_TF프레임_분리_문제해결.md
- TF프레임_이해_odom_드리프트.md
- NAVIGATION_TUNING_COMPLETE.md
- OSCILLATION_PROBLEM_ANALYSIS.md
- OPTIMIZATION_COMPLETE.md
