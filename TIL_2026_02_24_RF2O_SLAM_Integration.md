# RF2O + SLAM Toolbox 통합 및 4-센서 EKF 융합 보고서

**날짜**: 2026-02-24  
**환경**: Jetson (aarch64), Ubuntu, ROS 2 Humble  
**로봇**: Jupiter (차동구동, Yahboom x3)  
**센서**: RealSense D455F, MCU IMU (ICM20948), 휠 인코더, RPLidar S2L  
**시스템**: Isaac ROS cuVSLAM (Docker) + RF2O + SLAM Toolbox + robot_localization EKF (호스트)

---

## 1. 작업 목표

이전 세션(02-23~02-24 초반)에서 VSLAM + EKF 센서융합 5대 근본 원인을 수정하고 안정화한 상태에서,
다음 목표를 추진:

1. **RF2O LiDAR Odometry** 추가 — VSLAM 실패(tracking lost) 시 보조 위치 입력
2. **SLAM Toolbox** 추가 — `map→odom` TF 제공 + 실시간 맵 생성
3. **4-센서 EKF 융합** 완성 — Wheel + VSLAM + RF2O + IMU
4. **단일 통합 launch 파일** — `nav2_vslam_fused.launch.py`에 모든 노드 통합

---

## 2. 진단 발견: RF2O 제로 공분산 문제

### 2.1. 문제 발견

RF2O 노드(`rf2o_laser_odometry`)를 EKF에 odom2로 추가한 후 테스트:

```bash
ros2 topic echo /odom_rf2o --once | grep -A 36 "pose_covariance"
```

결과:
```
pose_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist_covariance: [전부 0.0]
```

**RF2O는 pose와 twist의 공분산을 모두 0으로 발행** — VSLAM과 동일한 문제!

### 2.2. 위험 분석

공분산 = 0은 EKF에 "완벽한 측정"을 의미:
- EKF가 RF2O 데이터를 **무한 신뢰**
- 다른 센서(wheel, IMU) 데이터가 무시됨
- RF2O 노이즈/드리프트가 필터링 없이 그대로 출력에 반영
- EKF 발산 위험

### 2.3. 1차 시도: `odom2_pose_covariance` 파라미터 오버라이드

robot_localization에 `odom2_pose_covariance` 파라미터를 설정하여 공분산을 강제 적용 시도:

```yaml
odom2_pose_covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                        ...]
```

결과: **무효 — 파라미터가 무시됨**

```bash
ros2 param list /ekf_filter_node | grep covariance
# odom2_pose_covariance 없음!
```

### 2.4. [TIL] robot_localization ROS2 Humble 제한 사항

**핵심 발견**: `robot_localization` ROS2 Humble 버전은 다음 파라미터를 **지원하지 않음**:

| 파라미터 | 지원 여부 | 비고 |
|----------|-----------|------|
| `odomN_pose_covariance` | ❌ 미지원 | YAML에 설정해도 무시 |
| `odomN_twist_covariance` | ❌ 미지원 | 동일 |
| `imuN_angular_velocity_covariance` | ❌ 미지원 | 동일 |
| `imuN_linear_acceleration_covariance` | ❌ 미지원 | 동일 |
| `odomN_config`, `odomN_differential` | ✅ 지원 | 정상 |
| `odomN_pose_rejection_threshold` | ✅ 지원 | 정상 |

이 파라미터들은 `ros2 param list`에도 나타나지 않으며, YAML에 명시해도 **조용히 무시**된다.

**의미**: EKF 내에서 공분산을 오버라이드할 수 없으므로, **센서 발행 단에서 공분산을 올바르게 설정해야 함**.

---

## 3. 해결: RF2O 공분산 어댑터

### 3.1. 기존 `vslam_covariance_adapter.py` 재활용

이미 VSLAM 제로 공분산 문제를 해결하기 위해 만들어둔 `vslam_covariance_adapter.py`를 재활용:

```python
# rf2o_covariance_adapter (nav2_vslam_fused.launch.py 내)
rf2o_covariance_adapter = Node(
    package='jupiter_nav_VO',
    executable='vslam_covariance_adapter',   # 동일 실행 파일 재사용
    name='rf2o_covariance_adapter',          # 다른 이름으로 실행
    parameters=[{
        'input_topic': '/odom_rf2o',
        'output_topic': '/odom_rf2o_adapted',
        'min_position_cov': 0.1,         # m² (VSLAM 0.05보다 높음 — RF2O가 덜 정확)
        'min_orientation_cov': 0.05,      # rad² (VSLAM 0.02보다 높음)
        'min_linear_vel_cov': 0.01,       # (m/s)²
        'min_angular_vel_cov': 0.01,      # (rad/s)²
        'clamp_existing': True,
    }]
)
```

### 3.2. 공분산 어댑터 파라미터 비교

| 파라미터 | VSLAM 어댑터 | RF2O 어댑터 | 근거 |
|----------|-------------|-------------|------|
| `min_position_cov` | 0.05 m² | 0.1 m² | RF2O는 LiDAR 스캔매칭 기반으로 VSLAM보다 위치 불확실성 높음 |
| `min_orientation_cov` | 0.02 rad² | 0.05 rad² | 2D 스캔은 yaw 추정 정밀도 낮음 |
| `min_linear_vel_cov` | 0.01 | 0.01 | 동일 |
| `min_angular_vel_cov` | 0.005 | 0.01 | RF2O 각속도 추정이 VSLAM보다 덜 정확 |

### 3.3. EKF 설정 변경

```yaml
# ekf_vslam_fusion.yaml
odom2: /odom_rf2o_adapted   # ← /odom_rf2o에서 변경
odom2_config: [true,  true,  false,    # x, y ← 절대 위치
               false, false, true,     # yaw ← 절대 방향
               false, false, false,    # vx, vy, vz
               false, false, false,    # vyaw
               false, false, false]    # ax, ay, az
odom2_differential: false   # RF2O 자체 적분값 (절대 위치) 사용
odom2_pose_rejection_threshold: 5.0
```

### 3.4. 비효과 파라미터 제거

다음 파라미터들은 ROS2 Humble에서 지원되지 않으므로 EKF 설정에서 **제거**:

```yaml
# 삭제된 파라미터 (조용히 무시됨 → 혼란 방지를 위해 삭제)
# odom2_pose_covariance: [...]
# imu0_angular_velocity_covariance: [...]
# imu0_linear_acceleration_covariance: [...]
```

대신 주석으로 이유를 명시:
```yaml
# 참고: robot_localization ROS2 Humble은 imuN_angular_velocity_covariance
# 파라미터를 지원하지 않음. IMU 공분산은 jupiter_driver_compensated.py에서
# 직접 설정됨 (gyro_cov=0.001, accel_cov=0.01)
```

---

## 4. 시스템 통합: nav2_vslam_fused.launch.py

### 4.1. 추가된 노드 (이번 작업)

| # | 노드 | 패키지 | 역할 | 비고 |
|---|------|--------|------|------|
| 8 | sllidar_node | sllidar_ros2 | RPLidar S2L → /scan | baudrate=1000000, Standard 모드 |
| 9 | rf2o_node | rf2o_laser_odometry | /scan → /odom_rf2o (10Hz) | publish_tf=False |
| 10 | rf2o_covariance_adapter | jupiter_nav_VO | /odom_rf2o → /odom_rf2o_adapted | min_pos=0.1, min_orient=0.05 |
| 11 | slam_toolbox | slam_toolbox | /scan → /map + map→odom TF | 5초 지연 시작 |

### 4.2. 기존 노드 (변경 없음)

| # | 노드 | 역할 |
|---|------|------|
| 1 | robot_state_publisher | URDF → TF 정적 변환 |
| 2 | joint_state_publisher | 관절 상태 발행 |
| 3 | tf_base_to_camera | base_link → camera_link 정적 TF |
| 4 | jupiter_driver | MCU 드라이버 (모터, 센서) |
| 5 | imu_calib | IMU 캘리브레이션 |
| 6 | jupiter_base | 휠 오도메트리 (base_node) |
| 7 | odom_covariance_adapter | /odom_raw → /odom_adapted |
| — | vslam_covariance_adapter | VSLAM 공분산 보정 |
| 12 | ekf_filter_node | 4-센서 EKF 융합 (3초 지연) |
| — | foxglove_bridge | 시각화 (선택적) |

### 4.3. 노드 시작 순서

```
[t=0s] URDF + TF + Driver + IMU Calib + Base + Covariance Adapters + LiDAR + RF2O + Foxglove
[t=3s] EKF  (IMU 캘리브레이션 500샘플@100Hz ≈ 5초 대기)
[t=5s] SLAM Toolbox  (EKF TF 준비 대기)
```

### 4.4. 최종 시스템 아키텍처

```
  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
  │ Isaac VSLAM   │  │ RF2O LiDAR   │  │ Wheel Odom   │  │ IMU (Calib)  │
  │ /visual_slam/ │  │  Odometry    │  │ /odom_raw    │  │ /imu/data_   │
  │  tracking/    │  │ /odom_rf2o   │  │ (base_node)  │  │  calibrated  │
  │  odometry     │  │              │  │              │  │              │
  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘
         │                 │                  │                  │
    ┌────▼────┐      ┌────▼────┐      ┌──────▼───────┐         │
    │ VSLAM   │      │ RF2O    │      │ odom_cov_    │         │
    │ cov_    │      │ cov_    │      │ adapter      │         │
    │ adapter │      │ adapter │      │ /odom_adapted│         │
    └────┬────┘      └────┬────┘      └──────┬───────┘         │
         │                │                  │                  │
         └────────────────┼──────────────────┼──────────────────┘
                          ▼                  ▼
           ┌─────────────────────────────────────┐
           │  robot_localization (EKF, 30Hz)      │
           │  odom0: wheel    (vx, vy, vyaw)      │
           │  odom1: VSLAM    (x,y,yaw differential)│
           │  odom2: RF2O     (x,y,yaw absolute)  │
           │  imu0: IMU       (vyaw only)          │
           │  TF: odom → base_footprint            │
           └──────────────┬────────────────────────┘
                          │
  ┌───────────────────────▼───────────────┐
  │  RPLidar S2L (/scan) + EKF TF        │
  │  → SLAM Toolbox                       │
  │  → /map + TF: map → odom             │
  └───────────────────────────────────────┘
```

---

## 5. 성능 검증 결과

### 5.1. 토픽 주파수

| 센서 | 토픽 | 실측 Hz | 기대 Hz |
|------|------|---------|---------|
| Wheel Odom | /odom_adapted | ~50 Hz | 50 Hz ✅ |
| VSLAM | /visual_slam/tracking/odometry_adapted | ~90 Hz | 90 Hz ✅ |
| RF2O | /odom_rf2o_adapted | ~10 Hz | 10 Hz ✅ |
| IMU | /imu/data_calibrated | ~100 Hz | 100 Hz ✅ |
| EKF 출력 | /odom | ~30 Hz | 30 Hz ✅ |
| LiDAR | /scan | ~10 Hz | 10 Hz ✅ |
| Map | /map | 낮은 빈도 (변경 시) | 정상 ✅ |

### 5.2. EKF 안정성 (정지 상태)

```bash
ros2 topic echo /odom --once
# position: x≈0.011, y≈-0.008
# velocity: ~0
```

- **위치 드리프트**: ±1mm (정지 상태) ✅
- **속도**: ~0 m/s ✅
- **발산**: 없음 ✅

### 5.3. EKF 진단

```bash
ros2 topic echo /diagnostics --once
# name: "ekf_filter_node: Filter diagnostic updater"
# level: 0 (OK)
# message: "Filter is healthy"
```

- 공분산 경고: 없음 ✅
- "Filter is healthy" ✅

### 5.4. RF2O 공분산 검증

```bash
# 보정 전 (/odom_rf2o)
pose_covariance[0] = 0.0    ← 위치 x
pose_covariance[35] = 0.0   ← yaw

# 보정 후 (/odom_rf2o_adapted)
pose_covariance[0] = 0.1    ← 위치 x ✅
pose_covariance[7] = 0.1    ← 위치 y ✅
pose_covariance[35] = 0.05  ← yaw ✅
```

### 5.5. RF2O ↔ EKF 위치 일치도

```
RF2O: x=0.0115, y=-0.0077
EKF:  x=0.0111, y=-0.0078
차이:  Δx=0.4mm, Δy=0.1mm  ✅
```

정지 상태에서 RF2O와 EKF 출력이 서브-밀리미터 수준으로 일치.

### 5.6. TF 트리

```
map → odom (SLAM Toolbox)
    → base_footprint (EKF)
        → base_link (URDF, +8cm Z)
            → camera_link (정적 TF)
            → laser (URDF)
            → imu_link (URDF)
```

- map→odom: SLAM Toolbox 발행 ✅
- odom→base_footprint: EKF 발행 ✅
- 전체 TF 체인: 연결 완료 ✅
- NaN TF: 없음 ✅

### 5.7. SLAM Toolbox 맵

```bash
ros2 topic echo /map --once | head -10
# width: 132, height: 185, resolution: 0.05
# 맵 크기: 6.6m × 9.25m
```

---

## 6. TIL (Today I Learned)

### 6.1. RF2O도 제로 공분산을 발행한다

`rf2o_laser_odometry` 패키지는 `nav_msgs/Odometry` 메시지의 `pose.covariance`와 `twist.covariance`를 모두 **0으로 발행**한다. 이는 Isaac VSLAM과 동일한 문제이며, 공분산을 설정하지 않는 오도메트리 소스에 대해 항상 어댑터가 필요하다.

### 6.2. robot_localization의 공분산 오버라이드는 ROS2 Humble에서 미지원

ROS1에서는 `odomN_pose_covariance`, `imuN_angular_velocity_covariance` 등의 파라미터로 EKF 내에서 공분산을 오버라이드할 수 있었으나, **ROS2 Humble 버전에서는 이 파라미터들이 구현되지 않았다**.

- `ros2 param list`에 해당 파라미터가 나타나지 않음
- YAML에 설정해도 **조용히 무시** — 에러도 경고도 없음
- 따라서 공분산은 반드시 **발행 노드 쪽에서** 올바르게 설정해야 함

### 6.3. 공분산 어댑터 패턴의 범용성

`vslam_covariance_adapter.py`를 파라미터화하여 설계한 결과, 별도 코드 수정 없이 다른 오도메트리 소스(RF2O)에도 재활용 가능:

```python
# 동일 executable, 다른 파라미터
executable='vslam_covariance_adapter'
name='rf2o_covariance_adapter'       # 다른 노드 이름
parameters=[{
    'input_topic': '/odom_rf2o',      # 다른 입출력 토픽
    'output_topic': '/odom_rf2o_adapted',
    'min_position_cov': 0.1,          # 다른 공분산 값
    ...
}]
```

### 6.4. 다중 센서 융합에서의 센서 간 공분산 설계 원칙

4개 센서를 융합할 때 공분산 크기 순서:

```
가장 신뢰 (낮은 cov)                    가장 불확실 (높은 cov)
         ←─────────────────────────────────────→
IMU vyaw   VSLAM pos   RF2O pos   Wheel odom
(0.001)    (0.05)      (0.1)      (동적: 0.01~1.0)
```

- IMU vyaw: 가장 정확 → 가장 강하게 신뢰
- VSLAM: 비전 기반 6DoF → 높은 정확도이나 tracking lost 가능
- RF2O: LiDAR 스캔매칭 → 조명 무관하나 환경 의존적
- Wheel: 엔코더 기반 → 슬립/미끄러짐에 취약, 속도에 따라 변동

### 6.5. VSLAM vo_status:0에서도 시스템이 안정적

cuVSLAM이 `vo_status: 0` (tracking lost) 상태에서도 4-센서 EKF 융합은 **안정적**:
- Wheel + RF2O + IMU 3개 센서가 fallback 역할
- VSLAM은 differential 모드이므로, tracking lost 시 새 데이터가 발행되지 않아도 EKF에 영향 없음
- SLAM Toolbox는 RF2O/Wheel→EKF의 odom→base_footprint TF를 사용하므로 VSLAM 무관하게 동작

---

## 7. 현재 상태 요약

| 항목 | 상태 | 비고 |
|------|------|------|
| RPLidar S2L 통합 | ✅ 완료 | /scan 10Hz, sllidar_ros2 |
| RF2O LiDAR 오도메트리 | ✅ 완료 | /odom_rf2o 10Hz |
| RF2O 공분산 어댑터 | ✅ 완료 | /odom_rf2o_adapted, min_pos=0.1 |
| SLAM Toolbox 통합 | ✅ 완료 | map→odom TF, /map 발행 |
| 4-센서 EKF 융합 | ✅ 완료 | wheel + VSLAM + RF2O + IMU |
| EKF 안정성 | ✅ 확인 | ±1mm 드리프트 (정지) |
| TF 트리 완전 연결 | ✅ 확인 | map→odom→base_footprint→base_link→laser |
| 비지원 파라미터 정리 | ✅ 완료 | odomN_pose_covariance 등 제거 |
| 단일 launch 파일 통합 | ✅ 완료 | nav2_vslam_fused.launch.py |
| VSLAM 추적 품질 | ⚠️ vo_status:0 | 추적 안정화 필요 (환경/캘리브 문제) |
| Nav2 네비게이션 | 🔲 미시작 | 다음 단계 |

---

## 8. 다음 단계

### 즉시 (P0)
1. **실주행 테스트** — 4-센서 EKF 동적 환경에서 검증
2. **VSLAM 추적 안정화** — vo_status:0 원인 분석 (RealSense IR 패턴, 캘리브레이션)

### 단기 (P1)
3. RF2O + VSLAM 공분산 파라미터 튜닝 (실측 기반)
4. EKF `process_noise_covariance` 최적화
5. SLAM Toolbox 맵 저장 및 localization 모드 테스트

### 중기 (P2)
6. Nav2 네비게이션 스택 연동
7. 경로 계획 + 장애물 회피 테스트
8. 장시간 운용 안정성 검증

---

## 9. 수정 이력

| 날짜 | 파일 | 변경 내용 |
|------|------|-----------|
| 02-24 | `nav2_vslam_fused.launch.py` | sllidar_node, rf2o_node, rf2o_covariance_adapter, slam_toolbox 추가 |
| 02-24 | `nav2_vslam_fused.launch.py` | lidar_port launch arg 추가 |
| 02-24 | `nav2_vslam_fused.launch.py` | 아키텍처 다이어그램 업데이트 (4-센서 + SLAM Toolbox) |
| 02-24 | `ekf_vslam_fusion.yaml` | odom2 섹션 추가 (/odom_rf2o_adapted, x/y/yaw, differential=false) |
| 02-24 | `ekf_vslam_fusion.yaml` | odom2_pose_covariance 파라미터 제거 (미지원) |
| 02-24 | `ekf_vslam_fusion.yaml` | imu0_angular_velocity_covariance 파라미터 제거 (미지원) |
| 02-24 | `ekf_vslam_fusion.yaml` | imu0_linear_acceleration_covariance 파라미터 제거 (미지원) |
| 02-24 | `ekf_vslam_fusion.yaml` | 헤더 주석 업데이트 (RF2O 설명 추가) |

---

## 10. 참고 파일 경로

| 파일 | 위치 |
|------|------|
| 센서융합 launch | `~/jupiter_ws_ros2/src/jupiter_nav_VO/launch/nav2_vslam_fused.launch.py` |
| EKF 설정 | `~/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf_vslam_fusion.yaml` |
| VSLAM 공분산 어댑터 | `~/jupiter_ws_ros2/src/jupiter_nav_VO/jupiter_nav_VO/vslam_covariance_adapter.py` |
| RF2O 공분산 어댑터 | 위와 동일 (동일 executable, 다른 파라미터로 실행) |
| 휠 공분산 어댑터 | `~/jupiter_ws_ros2/src/jupiter_nav_VO/jupiter_nav_VO/odom_covariance_adapter.py` |
| SLAM Toolbox 설정 | `~/jupiter_ws_ros2/src/jupiter_nav/config/slam_params.yaml` |
| VSLAM launch (Docker) | `~/isaac_ws_ros/src/isaac_ros_visual_slam/.../isaac_ros_visual_slam_realsense.launch.py` |
| IMU 캘리브레이션 | `~/jupiter_ws_ros2/imu_calib.yaml` |
| 이전 TIL (02-23) | `~/isaac_ws_ros/TIL_2026_02_23_VSLAM_Node_Diagnosis.md` |
| 이전 진행과정 | `~/isaac_ws_ros/진행과정.md` |
