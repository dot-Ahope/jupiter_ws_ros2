# TIL 2026-03-23: nvblox Nav2 통합 + Astra 카메라 완전 제거

## 목표
1. Astra 카메라 관련 설정 완전 제거 (더 이상 사용하지 않음)
2. Isaac ROS nvblox를 Nav2 costmap에 통합하여 3D 장애물 감지 → 안정성 향상

---

## Phase 1: Astra 카메라 레거시 정리

### 문제
Orbbec Astra 카메라는 더 이상 물리적으로 연결되어 있지 않으나, 코드 전반에 잔존 설정이 산재.

### 수정된 파일들

| 파일 | 변경 내용 |
|------|-----------|
| `~/.bashrc` | `CAMERA_TYPE=astraplus` 삭제, echo 라인 → `RealSense D455F` |
| `jupiter_bringup/launch/bringup.launch.py` | astra_node(~50줄), pointcloud_to_laserscan_node(~25줄) 제거, URDF → `jupiter_simple.urdf` |
| `jupiter_bringup/jupiter_bringup/device_srv.py` | default `'astra'` → `'usb'`, astra 분기 제거 |
| `jupiter_bringup/launch/calibrate_imu.launch.py` | `'CameraDevice': 'astra'` 파라미터 제거 |
| `99-astra-camera.rules` | 파일 삭제 (udev 규칙, 실제 `/etc/udev/rules.d/`에 설치된 적 없음) |

### 남은 레거시 (백업/아카이브, 수정 불요)
- `astra_bringup.launch.py` — 독립 Astra 전용 launch (미사용)
- `jupiter_astra.urdf` — 레거시 URDF
- `display_astra.launch.py` — 레거시 rviz display
- `archive/` 내 문서들 — 히스토리 보존용

---

## Phase 2: nvblox Nav2 통합

### 아키텍처 설계

```
[Docker 컨테이너] ── --network host (DDS 토픽 공유) ──┐
│                                                      │
│  RealSense D455F (emitter_on_off: 60fps)            │
│    ├── RealsenseSplitterNode                        │
│    │    ├── emitter OFF → IR stereo → cuVSLAM      │
│    │    └── emitter ON  → depth    → nvblox_node    │
│    └── color ──────────────────→ nvblox_node         │
│                                                      │
│  Publish:                                            │
│    /visual_slam/tracking/odometry → Native EKF      │
│    /nvblox_node/static_map_slice  → Native Nav2     │
└──────────────────────────────────────────────────────┘

[Native 호스트]
  Nav2 costmap → nvblox_costmap_layer (Docker 토픽 구독)
              → obstacle_layer (RPLidar /scan)
              → inflation_layer
```

### 핵심 설계 결정

1. **emitter_on_off 모드**: RealSense가 프레임마다 emitter ON/OFF 교대
   - VSLAM: emitter OFF IR stereo (30Hz, 기존 60Hz의 절반)
   - nvblox: emitter ON depth (30Hz)
   - RealsenseSplitterNode가 metadata 파싱으로 프레임 분류

2. **Docker 내부 빌드/실행**: nvblox_node, realsense_splitter는 CUDA 의존성으로 Docker 전용
   - `--network host`로 토픽이 호스트에 그대로 전달
   - nvblox_nav2 플러그인 `.so`는 볼륨 마운트로 호스트에서 접근 가능

3. **nvblox_costmap_layer**: `convert_to_binary_costmap: True` (NVIDIA 권장)
   - 거리 ≤ 0 → LETHAL_OBSTACLE, 나머지 → FREE_SPACE
   - 별도 `inflation_layer`가 gradient 제공
   - obstacle_layer 앞, inflation_layer 앞에 배치

4. **Graceful degradation**: nvblox_node 미실행 시 토픽 미수신 → 영향 없음 (LiDAR만 동작)

### 수정/생성된 파일들

| 파일 | 변경 내용 |
|------|-----------|
| `jupiter_nav_VO/config/nav2_params_fused.yaml` | local_costmap + global_costmap에 `nvblox_layer` 플러그인 추가 |
| `isaac_ros_visual_slam/launch/isaac_ros_vslam_nvblox_realsense.launch.py` | **신규** — VSLAM + splitter + nvblox_node 통합 launch |
| `~/.bashrc` | `AMENT_PREFIX_PATH` + `LD_LIBRARY_PATH`에 nvblox_nav2, nvblox_msgs 추가 |
| `install/isaac_ros_common/.../package.sh` | Docker↔Host 경로 오염 수정 (빌드 차단 해결) |

### Docker 빌드 결과

| 패키지 | 상태 | 비고 |
|--------|------|------|
| `nvblox_nav2` | ✅ 빌드 성공 | `libnvblox_nav2.so` — 호스트에서 런타임 deps 모두 resolve |
| `realsense_splitter` | ✅ 빌드 성공 | `librealsense_splitter_component.so` — COLCON_IGNORE 임시 제거 후 빌드 |
| `nvblox_ros` (nvblox_node) | ✅ 기존 빌드 | 이미 빌드되어 있었음 |

### 빌드 트러블슈팅
- **문제**: `install/isaac_ros_common/share/.../package.sh`에 호스트 경로 하드코딩
  - 원인: 호스트에서 colcon build 시도 → isaac_ros_common이 `/home/jetson/isaac_ws_ros`로 캐시
  - Docker 내부에서는 `/workspaces/isaac_ros-dev` 경로만 유효
  - **해결**: `sed -i`로 경로 패치

- **문제**: `realsense_splitter`에 `COLCON_IGNORE` 파일 존재
  - 임시 제거 → 빌드 → 복원

---

## Nav2 costmap 플러그인 설정 상세

### local_costmap (odom frame)
```yaml
plugins: ["obstacle_layer", "nvblox_layer", "inflation_layer"]
nvblox_layer:
  plugin: "nvblox::nav2::NvbloxCostmapLayer"
  enabled: True
  nav2_costmap_global_frame: "odom"
  nvblox_map_slice_topic: "/nvblox_node/static_map_slice"
  convert_to_binary_costmap: True
  max_obstacle_distance: 1.0
  inflation_distance: 0.5
```

### global_costmap (map frame)
```yaml
plugins: ["static_layer", "obstacle_layer", "nvblox_layer", "inflation_layer"]
nvblox_layer:
  plugin: "nvblox::nav2::NvbloxCostmapLayer"
  enabled: True
  nav2_costmap_global_frame: "map"
  nvblox_map_slice_topic: "/nvblox_node/static_map_slice"
  convert_to_binary_costmap: True
  max_obstacle_distance: 1.0
  inflation_distance: 0.5
```

---

## 실행 방법

### VSLAM + nvblox 모드 (3D 장애물 감지 포함)
```bash
# 1. Docker
cd ~/isaac_ws_ros/src/isaac_ros_common/scripts && ./run_dev.sh
# Docker 내부:
source install/setup.bash
ros2 launch isaac_ros_visual_slam isaac_ros_vslam_nvblox_realsense.launch.py

# 2. Native (기존과 동일)
ros2 launch jupiter_nav_VO nav2_vslam_fused.launch.py
ros2 launch jupiter_nav_VO nav2_fused_navigation.launch.py
```

### VSLAM 전용 모드 (기존, nvblox 없이)
```bash
# Docker 내부:
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

---

## 주의사항
- emitter_on_off 모드에서 VSLAM IR framerate가 60Hz → **30Hz로 감소**
  - `image_jitter_threshold_ms`: 34 → 68ms로 조정 완료
  - VSLAM 추적 성능 영향 모니터링 필요
- nvblox_node의 `esdf_slice_min_height: -0.15`, `esdf_slice_max_height: 0.4`
  - D455F 카메라 높이(지면에서 ~13.5cm) 기준 설정
  - 지면 근접 장애물(-1.5cm) ~ 테이블 다리(54cm)까지 감지
- Docker `emitter_on_off`에는 **Linux UVC metadata 지원** 필요 (Isaac ROS Docker에 기본 포함)
