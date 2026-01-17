# transbot_nav

**Transbot Navigation and SLAM Integration Package**

이 패키지는 Transbot 로봇의 자율 주행 및 SLAM(Simultaneous Localization and Mapping) 기능을 통합 관리합니다.

## 📦 패키지 개요

`transbot_nav`는 다음 컴포넌트들을 통합하여 Transbot의 자율 주행을 구현합니다:

- **Nav2 (Navigation2)**: ROS2 공식 자율 주행 스택
- **SLAM Toolbox**: 실시간 맵 생성 및 루프 클로저
- **robot_localization (EKF)**: IMU + 오도메트리 센서 퓨전

### 왜 별도 패키지인가?

이전에는 `sllidar_ros2` 패키지에 네비게이션 관련 파일들이 포함되어 있었습니다. 하지만:

- `sllidar_ros2`는 **하드웨어 드라이버** 패키지입니다 (LiDAR 센서 제어)
- 네비게이션 로직은 **응용 프로그램** 레벨입니다

관심사의 분리(Separation of Concerns) 원칙에 따라, 네비게이션 관련 기능을 독립된 패키지로 분리했습니다.

## 📂 패키지 구조

```
transbot_nav/
├── launch/                          # Launch 파일들
│   ├── transbot_full_system.launch.py    # 전체 시스템 통합 런치
│   └── nav2_navigation.launch.py         # Nav2 스택만 실행
├── config/                          # 설정 파일들
│   ├── ekf_config.yaml                   # EKF 센서 퓨전 설정
│   ├── slam_params.yaml                  # SLAM Toolbox 파라미터
│   └── nav2_params.yaml                  # Nav2 스택 설정
├── rviz/                            # RViz 시각화 설정
│   ├── sllidar.rviz
│   └── sllidar_ros2.rviz
├── maps/                            # 저장된 맵 파일 (pgm + yaml)
├── package.xml                      # 패키지 메타데이터
├── setup.py                         # Python 패키지 설정
└── README.md                        # 이 파일
```

## 🚀 사용법

### 1. 전체 시스템 실행 (SLAM 모드)

로봇 하드웨어 + LiDAR + IMU + EKF + SLAM을 한 번에 실행:

```bash
ros2 launch transbot_nav transbot_full_system.launch.py
```

**실행되는 노드들:**
- `robot_state_publisher`: URDF 기반 TF 발행
- `Transbot_Driver`: 모터 제어 및 오도메트리
- `imu_calib_node`: IMU 데이터 보정
- `ekf_filter_node`: IMU + 오도메트리 센서 퓨전
- `sllidar_node`: LiDAR 스캔 데이터
- `async_slam_toolbox_node`: SLAM 맵 생성

### 2. Nav2 자율 주행 실행

SLAM이 실행 중인 상태에서 별도 터미널에서:

```bash
ros2 launch transbot_nav nav2_navigation.launch.py
```

**실행되는 Nav2 노드들:**
- `controller_server`: 경로 추적 제어기
- `planner_server`: 전역 경로 계획
- `behavior_server`: 회전, 후진 등 행동 제어
- `bt_navigator`: Behavior Tree 기반 의사결정
- `velocity_smoother`: 속도 명령 부드럽게 처리
- `lifecycle_manager_navigation`: 노드 생명주기 관리

### 3. RViz로 목표 설정

```bash
rviz2 -d $(ros2 pkg prefix transbot_nav)/share/transbot_nav/rviz/sllidar_ros2.rviz
```

RViz에서:
1. **"2D Goal Pose"** 버튼 클릭
2. 맵에서 원하는 목표 위치 클릭 & 드래그 (방향 설정)
3. 로봇이 자동으로 경로 계획하고 이동

## ⚙️ 주요 설정

### EKF 센서 퓨전 (`ekf_config.yaml`)

IMU를 우선하여 회전 정확도를 높였습니다:

```yaml
# IMU 각속도 신뢰도 (매우 높음)
imu0_angular_velocity_covariance: 0.000025

# 오도메트리 위치 신뢰도 (낮음)
odom0_pose_covariance: 0.0225

# 프로세스 노이즈 (IMU 중심)
process_noise_covariance[11]: 0.00005  # yaw rate
```

### SLAM 파라미터 (`slam_params.yaml`)

루프 클로저 기준을 엄격하게 설정하여 맵 품질 향상:

```yaml
# 루프 클로저 응답 임계값 (0.65 = 매우 엄격)
loop_match_minimum_response_fine: 0.65

# 루프 클로저 체인 크기 (15 = 많은 증거 필요)
loop_match_minimum_chain_size: 15

# 최소 이동 거리/각도
minimum_travel_distance: 0.10  # 10cm
minimum_travel_heading: 0.05   # ~3도
```

### Nav2 속도 제한 (`nav2_params.yaml`)

SLAM 맵 품질을 위해 회전 속도를 제한:

```yaml
# 속도 제한
max_vel_x: 0.3      # 전진: 빠르게 (0.3 m/s)
max_vel_theta: 0.5  # 회전: 느리게 (0.5 rad/s, ~29도/초)

# 가속도 제한 (DWB 계획을 위해 유지)
acc_lim_x: 2.5      # 전진 가속도
acc_lim_theta: 3.2  # 회전 가속도
```

**왜 이렇게 설정했나요?**
- 회전 속도만 제한: SLAM의 스캔 매칭 정확도 향상
- 가속도는 유지: DWB 플래너가 다양한 궤적을 생성할 수 있도록
- 과도하게 제한하면: 로봇이 좌우 진동(oscillation) 발생

## 🔧 TF 프레임 구조

```
map (SLAM Toolbox)
 └─ odom (EKF - IMU + Odometry 퓨전)
     └─ base_footprint (로봇 지면 투영점)
         └─ base_link (로봇 중심)
             ├─ laser_frame (LiDAR)
             ├─ imu_link (IMU)
             └─ camera_link (카메라)
```

**중요:**
- `odom`이 `base_footprint`에서 분리되는 것은 **정상**입니다
- SLAM이 루프 클로저를 수행하면 `map → odom` TF가 조정됩니다
- 이것이 SLAM의 정확한 위치 추정 메커니즘입니다

## 📊 성능 최적화 팁

### SLAM 맵 품질 향상

1. **천천히 회전하세요** (0.5 rad/s 이하)
   - 스캔 매칭 정확도가 높아집니다
   - 루프 클로저 성공률이 올라갑니다

2. **같은 공간을 여러 번 방문하세요**
   - 루프 클로저가 맵을 보정합니다
   - Feature가 많은 환경이 좋습니다 (벽, 모서리)

3. **비어있는 공간은 피하세요**
   - LiDAR가 feature를 찾을 수 없습니다
   - 위치 추정 정확도가 떨어집니다

### Nav2 경로 계획 문제 해결

```bash
# Nav2 노드 상태 확인
ros2 lifecycle list

# Controller 상태 확인
ros2 topic echo /controller_server/transition_event

# 계획된 경로 시각화
ros2 topic echo /plan
```

## 🐛 문제 해결

### 1. "No plan could be generated" 에러

**원인:** 맵에 장애물이 많거나 목표가 장애물 안에 있음

**해결:**
```bash
# Costmap 파라미터 조정
# nav2_params.yaml에서:
inflation_radius: 0.3  # 줄여보기
cost_scaling_factor: 5.0  # 줄여보기
```

### 2. 로봇이 좌우로 진동

**원인:** 속도/가속도 제한이 너무 낮아서 DWB가 궤적 생성 실패

**해결:**
```yaml
# nav2_params.yaml
acc_lim_x: 2.5      # 최소 2.5 유지
acc_lim_theta: 3.2  # 최소 3.2 유지
```

### 3. SLAM 맵이 흐릿하거나 겹침

**원인:** 루프 클로저 실패, IMU 드리프트

**해결:**
1. IMU 캘리브레이션 확인: `ros2 topic echo /imu/data`
2. 회전 속도 줄이기: `max_vel_theta: 0.3`
3. SLAM 파라미터 더 엄격하게: `loop_match_minimum_response_fine: 0.70`

### 4. "Costmap out of date" 경고

**원인:** LiDAR 데이터 지연 또는 TF 타임스탬프 문제

**해결:**
```bash
# LiDAR 주파수 확인
ros2 topic hz /scan

# TF 타임스탬프 확인
ros2 run tf2_tools view_frames
```

## 📚 더 알아보기

- [Nav2 공식 문서](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://docs.ros.org/en/humble/p/robot_localization/)

## 📝 라이센스

Apache-2.0

---

**패키지 버전:** 1.0.0  
**ROS2 버전:** Humble  
**마지막 업데이트:** 2025-10-31
