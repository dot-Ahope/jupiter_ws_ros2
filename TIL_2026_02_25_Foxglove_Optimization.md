# TIL: Foxglove Bridge 시각화 성능 최적화

**날짜:** 2026-02-25  
**작업 파일:** `src/jupiter_nav_VO/launch/nav2_vslam_fused.launch.py` (Foxglove Bridge 노드)

---

## 1. 문제 현상

- Foxglove Studio에서 로봇 시각화가 **RViz2 대비 매우 느리게** 표시됨
- 로봇 이동 시 화면 갱신이 수 초 지연, 실시간 제어에 부적합

---

## 2. 원인 분석

### 근본 원인: `topic_whitelist: ['.*']` (모든 토픽 구독)

Foxglove Bridge의 `topic_whitelist` 파라미터가 정규식 `['.*']`으로 설정되어 있어서, ROS 2 시스템의 **모든 토픽**이 WebSocket을 통해 Foxglove Studio로 전송됨.

### 문제되는 고속 토픽들

| 토픽 | 주파수 | 용도 |
|------|--------|------|
| `/jupiter/imu` | 100Hz | MCU IMU 원시 데이터 |
| `/imu/data_calibrated` | 100Hz | 보정된 IMU 데이터 |
| `/visual_slam/tracking/odometry` | ~90Hz | VSLAM 오도메트리 |
| `/jupiter/get_vel` | ~50Hz | MCU 엔코더 속도 |
| `/odom_raw` | ~50Hz | Wheel odom 원시 |
| `/odom_adapted` | ~50Hz | Wheel odom (공분산 포함) |
| `/odom` | 30Hz | EKF 융합 출력 |

**합산: 초당 300+ 메시지**가 단일 WebSocket을 포화시켜 시각화 지연 발생.

### 비교: RViz2는 왜 빠른가?

- RViz2는 DDS(로컬 프로세스 간 공유 메모리)로 직접 구독 → 네트워크 병목 없음
- Foxglove는 WebSocket(TCP)을 통한 직렬화/역직렬화 오버헤드 존재
- 고속 토픽이 WebSocket 버퍼를 채워서 실시간성이 있는 토픽의 전송이 지연됨

---

## 3. 해결 방법

### 3.1 토픽 화이트리스트 (핵심 수정)

**변경 전:**
```python
'topic_whitelist': ['.*'],  # 모든 토픽 구독
```

**변경 후:**
```python
'topic_whitelist': [
    '/odom',              # EKF 융합 출력 (30Hz)
    '/map',               # SLAM Toolbox 맵
    '/scan',              # LiDAR (10Hz)
    '/tf',                # TF 트리
    '/tf_static',         # 정적 TF
    '/cmd_vel',           # 속도 명령
    '/diagnostics',       # EKF 진단
    '/robot_description', # URDF
    '/odom_adapted',      # Wheel odom (디버깅용)
    '/odom_rf2o_adapted', # RF2O odom (디버깅용)
    # Nav2 관련
    '/local_costmap/costmap',
    '/global_costmap/costmap',
    '/plan',              # 글로벌 경로
    '/local_plan',        # 로컬 경로
],
```

**효과:** 시각화에 불필요한 고속 토픽(IMU 100Hz, VSLAM 90Hz 등)을 차단하여 WebSocket 트래픽을 **~90% 이상 감소**.

### 3.2 클라이언트 토픽 제한

```python
'client_topic_whitelist': ['/cmd_vel', '/goal_pose', '/initialpose'],
```

Foxglove에서 로봇으로 **발행할 수 있는 토픽을 3개로 제한**:
- `/cmd_vel`: 수동 이동 명령
- `/goal_pose`: Nav2 목표 지점 설정
- `/initialpose`: 초기 위치 설정

**효과:** 의도치 않은 토픽 발행 방지 (안전).

### 3.3 성능 파라미터 추가

```python
'num_threads': 2,              # 메시지 처리 병렬화 (기본값 1)
'send_buffer_limit': 10000000, # 10MB 전송 버퍼 (대용량 맵 메시지 허용)
'max_qos_depth': 5,            # QoS 큐 크기 축소 → 지연 감소
```

| 파라미터 | 기본값 | 설정값 | 효과 |
|---------|--------|--------|------|
| `num_threads` | 1 | 2 | 메시지 직렬화/전송 병렬 처리로 처리량 향상 |
| `send_buffer_limit` | 미설정 | 10MB | `/map` 등 대용량 메시지 전송 시 버퍼 초과 방지 |
| `max_qos_depth` | 미설정 | 5 | 오래된 메시지를 빠르게 버려 최신 데이터만 표시 |

---

## 4. 최종 설정 코드

```python
foxglove_bridge = Node(
    package='foxglove_bridge',
    executable='foxglove_bridge',
    name='foxglove_bridge',
    output='screen',
    parameters=[{
        'port': 8765,
        'address': '0.0.0.0',
        'tls': False,
        'topic_whitelist': [
            '/odom',              # EKF 융합 출력 (30Hz)
            '/map',               # SLAM Toolbox 맵
            '/scan',              # LiDAR (10Hz)
            '/tf',                # TF 트리
            '/tf_static',         # 정적 TF
            '/cmd_vel',           # 속도 명령
            '/diagnostics',       # EKF 진단
            '/robot_description', # URDF
            '/odom_adapted',      # Wheel odom (디버깅용)
            '/odom_rf2o_adapted', # RF2O odom (디버깅용)
            '/local_costmap/costmap',
            '/global_costmap/costmap',
            '/plan',              # 글로벌 경로
            '/local_plan',        # 로컬 경로
        ],
        'service_whitelist': ['.*'],
        'param_whitelist': ['.*'],
        'client_topic_whitelist': ['/cmd_vel', '/goal_pose', '/initialpose'],
        'num_threads': 2,
        'send_buffer_limit': 10000000,
        'max_qos_depth': 5,
    }],
    condition=IfCondition(LaunchConfiguration('use_foxglove'))
)
```

---

## 5. 향후 고려사항

- **토픽 추가 필요 시**: `topic_whitelist`에 토픽 이름 추가 (정규식 지원, 예: `/camera/.*`)
- **카메라 이미지**: Compressed Image를 사용하거나, 별도의 `image_transport` 브리지가 있으면 해당 토픽만 추가
- **멀티 클라이언트**: 여러 Foxglove 인스턴스가 동시 접속하면 `num_threads`를 3~4로 증가 고려
- **지연이 여전히 있다면**: Foxglove Studio의 Layout 설정에서 불필요한 Panel을 제거하거나, 3D Panel의 `Frame Rate`를 낮춰볼 것

---

## 6. 핵심 교훈

> Foxglove Bridge는 **모든** ROS 2 토픽을 WebSocket으로 중계한다.  
> RViz2는 DDS를 직접 사용하므로 성능 차이가 없지만,  
> WebSocket 기반인 Foxglove는 **반드시 토픽 필터링**이 필요하다.  
> 고속 센서 토픽(IMU 100Hz, VSLAM 90Hz)은 시각화에 불필요하므로 차단해야 한다.
