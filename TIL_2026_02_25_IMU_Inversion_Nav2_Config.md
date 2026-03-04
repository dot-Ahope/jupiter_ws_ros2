# TIL — 2026-02-25 IMU Gyro Z 반전 버그 수정 + Nav2 설정

**날짜**: 2026-02-25  
**환경**: Jetson (aarch64), Ubuntu, ROS 2 Humble  
**로봇**: Jupiter (차동구동, Yahboom x3)  
**센서**: RealSense D455F, MCU IMU (ICM20948), 휠 인코더, RPLidar S2L  
**시스템**: 4-센서 EKF 융합 + SLAM Toolbox + Nav2

---

## 1. IMU Gyro Z 반전 버그 — 파라미터 선언 ≠ 적용

### 1.1. 문제

`jupiter_driver_compensated.py`에서 IMU 축 반전 파라미터를 **선언만 하고 적용하지 않는** 버그가 있었다.

```python
# jupiter_driver_compensated.py (버그 상태)
self.declare_parameter('imu_invert_x', False)  # ← 선언 ✅
self.declare_parameter('imu_invert_y', False)
self.declare_parameter('imu_invert_z', False)

# ... (수백 줄 후) ...

msg.angular_velocity.x = gx  # ← 반전 없이 바로 대입! ❌
msg.angular_velocity.y = gy
msg.angular_velocity.z = gz
```

반면 원본 `jupiter_driver.py`에는 올바르게 구현되어 있었다:

```python
# jupiter_driver.py (정상)
if self.get_parameter('imu_invert_x').value: gx = -gx  # ← 적용 ✅
if self.get_parameter('imu_invert_y').value: gy = -gy
if self.get_parameter('imu_invert_z').value: gz = -gz

msg.angular_velocity.x = gx
msg.angular_velocity.y = gy
msg.angular_velocity.z = gz
```

### 1.2. 영향 범위

`jupiter_driver_params.yaml`에 `imu_invert_z: true`가 설정되어 있으므로:

1. **IMU angular_velocity.z 부호 반전** — LEFT 회전(CCW)에서 음수 발행
2. **EKF imu0 입력 오염** — IMU vyaw가 반대 방향으로 적용
3. **Wheel odometry yaw 영향** — 내부 yaw 적분에 반전된 IMU 값이 사용됨
4. **전체 용합 결과** Δyaw 부호 불일치 — EKF 출력도 반전 가능

### 1.3. 교훈

> **파라미터를 `declare_parameter()`로 선언했다고 해서, 해당 파라미터가 적용되는 것이 아니다.**  
> 반드시 `get_parameter().value`로 읽어서 로직에 반영해야 한다.

코드를 복사/리팩토링할 때, 파라미터 선언부와 적용부가 분리되어 있으면 적용부가 누락되기 쉽다. 특히 파일이 400줄 이상으로 길어지면 선언(라인 ~100)과 적용(라인 ~354)이 250줄 이상 떨어져 있어 놓치기 쉽다.

**체크리스트**:
```
□ declare_parameter() 했으면 → get_parameter().value로 읽는 코드가 있는가?
□ 원본 파일의 해당 파라미터 적용 로직을 빠짐없이 복사했는가?
□ grep -n "파라미터이름" 으로 선언/적용 모두 확인
```

---

## 2. 회전 테스트를 통한 센서 방향 검증 방법론

### 2.1. 테스트 설계

여러 오도메트리 소스의 방향이 일치하는지 확인하려면, **알려진 방향으로 회전 명령을 보내고 모든 센서의 yaw 응답을 비교**하는 것이 가장 효과적이다.

```python
# rotation_test.py 핵심 구조
class RotationTest(Node):
    def __init__(self, direction='left'):
        # 1. cmd_vel publisher — angular.z = +0.3 (left) or -0.3 (right)
        # 2. 4개 odom 토픽 subscriber — WHEEL, RF2O, VSLAM, EKF
        # 3. IMU subscriber — angular_velocity.z
        # 4. 타이머: 2s wait → 3s rotate → 3s observe
```

### 2.2. 판정 기준

| 회전 방향 | 기대 angular.z | 기대 Δyaw |
|-----------|---------------|-----------|
| LEFT (CCW) | 양수 (+) | 양수 (+) |
| RIGHT (CW) | 음수 (-) | 음수 (-) |

**모든 센서가 동일 부호**를 보여야 정상. 하나라도 반대 부호이면 해당 센서(또는 그 입력)에 문제.

### 2.3. 활용

이 테스트 방법론은 다음 상황에서도 재활용 가능:
- IMU 좌표계 변경 후 검증
- 새 오도메트리 소스 추가 후 방향 일치 확인
- 모터/인코더 방향 변경 후 확인
- EKF 파라미터 튜닝 후 융합 결과 방향 확인

---

## 3. Nav2 설정 시 SLAM Toolbox + EKF 시스템에서의 핵심 주의사항

### 3.1. AMCL 제거 필수

SLAM Toolbox와 AMCL을 동시에 사용하면 **map→odom TF가 충돌**한다:

| 노드 | TF 발행 | 용도 |
|------|---------|------|
| SLAM Toolbox | map → odom | SLAM (맵 생성 + 위치 추정) |
| AMCL | map → odom | 위치 추정만 (기존 맵 필요) |

→ **둘 다 `map→odom` TF를 발행하므로 하나만 사용해야 한다.**

SLAM Toolbox를 사용하는 경우:
- `nav2_bringup/navigation_launch.py` 사용 (Navigation Only)
- `nav2_bringup/localization_launch.py` 사용하지 않음 (AMCL + map_server 포함)
- Nav2 파라미터에서 AMCL 섹션 제거

### 3.2. base_link vs base_footprint

EKF(`robot_localization`)가 TF를 발행하는 프레임을 기준으로 Nav2의 `robot_base_frame`을 설정해야 한다.

| EKF 설정 | Nav2 설정해야 할 값 |
|----------|-------------------|
| `world_frame: odom`, `base_link_frame: base_footprint` | `robot_base_frame: base_footprint` |
| `world_frame: odom`, `base_link_frame: base_link` | `robot_base_frame: base_link` |

EKF가 `odom → base_footprint` TF를 발행하는데 Nav2가 `base_link`를 찾으면 TF 연결이 끊어진다.

주의: Nav2 params에서 `robot_base_frame`이 사용되는 곳이 **여러 군데**이다:

```yaml
bt_navigator.robot_base_frame       # ← 이것만 바꾸면 안 됨!
controller_server.FollowPath.*      # (일부 플러그인에서 참조)
behavior_server.robot_base_frame
local_costmap.robot_base_frame
global_costmap.robot_base_frame
```

→ **전부 일괄 변경 필요**

### 3.3. Costmap 센서 소스 선택

| 센서 | Costmap Layer | 장점 | 단점 |
|------|--------------|------|------|
| LiDAR `/scan` | `ObstacleLayer` (LaserScan) | 안정적, 조명 무관, 2D | 높이 정보 없음 |
| VSLAM pointcloud | `VoxelLayer` (PointCloud2) | 3D 장애물 | VSLAM tracking lost 시 중단 |
| Depth camera | `VoxelLayer` (PointCloud2) | 3D + RGB | 시야 60°, 근거리만 |

VSLAM이 `vo_status:0` (tracking lost) 상태인 경우 pointcloud가 발행되지 않을 수 있으므로, **LiDAR `/scan`을 기본 장애물 소스로 사용**하는 것이 안전하다.

### 3.4. Global Costmap에 StaticLayer 필수

SLAM Toolbox가 `/map` 토픽으로 맵을 발행하므로, Global Costmap에 `StaticLayer`를 추가하면 맵 정보를 활용할 수 있다:

```yaml
global_costmap:
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: True  # /map 토픽 구독
```

StaticLayer 없이 ObstacleLayer만 사용하면, LiDAR 스캔 범위 밖의 벽/장애물 정보가 반영되지 않아 **글로벌 경로 계획이 부정확**해진다.

---

## 4. Jetson에서의 Nav2 성능 고려사항

### 4.1. Controller Frequency

| 값 | 장점 | 단점 |
|---|------|------|
| 20 Hz (기본) | 반응 빠름 | Jetson CPU 부하 높음 |
| 10 Hz (설정값) | CPU 여유, 안정적 | 0.1초 지연 |
| 5 Hz | CPU 최소 | 느린 회피, 불안정 가능 |

Jetson에서 4-센서 EKF + SLAM Toolbox + LiDAR + Nav2를 동시 실행하면 CPU 부하가 높아지므로, **controller_frequency를 10Hz로 낮추는 것이 안전**하다.

### 4.2. Costmap Update Frequency

```yaml
local_costmap:
  update_frequency: 5.0   # LiDAR 10Hz의 절반
  publish_frequency: 2.0

global_costmap:
  update_frequency: 1.0   # 느리게 갱신
  publish_frequency: 0.5
```

LiDAR가 10Hz인 상황에서 costmap update를 10Hz로 하면 불필요한 CPU 사용. 5Hz면 충분.

### 4.3. 보수적 속도 설정

실내 환경 + Jetson 응답 지연을 고려하여:

```yaml
max_vel_x: 0.35      # (이전: 0.45) 실내 안전 속도
max_vel_theta: 1.0    # 기존과 동일
min_speed_theta: 0.15 # 정지마찰 극복 최소 각속도
```

---

## 5. navigation_launch.py vs localization_launch.py 차이

Nav2 bringup에는 두 가지 주요 launch가 있다:

| Launch | 포함 노드 | 용도 |
|--------|----------|------|
| `navigation_launch.py` | controller, planner, behavior, bt_navigator, waypoint_follower, lifecycle_manager | **네비게이션만** (위치 추정은 외부에서) |
| `localization_launch.py` | AMCL + map_server + lifecycle_manager | **위치 추정** (기존 맵 기반) |
| `bringup_launch.py` | 위 두 개 + SLAM (선택) | **전체 스택** |

SLAM Toolbox를 이미 별도로 실행하는 경우:
- `navigation_launch.py`만 사용 ✅
- `localization_launch.py` 사용 ❌ (AMCL 충돌)
- `bringup_launch.py` 사용 시 `slam:=false`, `map:=''` 설정 필요

---

## 6. 수정 이력

| 날짜 | 파일 | 변경 내용 |
|------|------|-----------|
| 02-25 | `jupiter_driver_compensated.py` | IMU gyro X/Y/Z 반전 로직 추가 (라인 354~362) |
| 02-25 | `nav2_params_fused.yaml` | **신규 생성** — AMCL 제거, base_footprint, /odom, /scan 기반 Nav2 설정 |
| 02-25 | `nav2_fused_navigation.launch.py` | **신규 생성** — navigation_launch.py 포함, nav2_params_fused.yaml 사용 |

---

## 7. 관련 파일

| 파일 | 경로 |
|------|------|
| 진행과정 | `~/jupiter_ws_ros2/진행과정_2026_02_25.md` |
| 보정 드라이버 (수정됨) | `~/jupiter_ws_ros2/src/jupiter_bringup/jupiter_bringup/jupiter_driver_compensated.py` |
| 원본 드라이버 (참고) | `~/jupiter_ws_ros2/src/jupiter_bringup/jupiter_bringup/jupiter_driver.py` |
| 드라이버 파라미터 | `~/jupiter_ws_ros2/src/jupiter_bringup/param/jupiter_driver_params.yaml` |
| Nav2 파라미터 (신규) | `~/jupiter_ws_ros2/src/jupiter_nav_VO/config/nav2_params_fused.yaml` |
| Nav2 런치 (신규) | `~/jupiter_ws_ros2/src/jupiter_nav_VO/launch/nav2_fused_navigation.launch.py` |
| Nav2 파라미터 (기존) | `~/jupiter_ws_ros2/src/jupiter_nav_VO/config/nav2_params_vslam.yaml` |
| 회전 테스트 도구 | `~/rotation_test.py` |
| 이전 TIL | `~/jupiter_ws_ros2/TIL_2026_02_24_RF2O_SLAM_Integration.md` |
| 이전 진행과정 | `~/jupiter_ws_ros2/진행과정_2026_02_24.md` |
