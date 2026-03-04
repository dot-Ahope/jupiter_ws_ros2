# TIL: Isaac ROS Visual SLAM - IMU 토픽 미발행으로 인한 Odometry 출력 불가 문제

**날짜**: 2026-02-12  
**태그**: `Isaac ROS`, `Visual SLAM`, `RealSense`, `IMU`, `ROS2`

---

## 문제 상황

Isaac ROS Visual SLAM을 RealSense D455F 카메라와 함께 실행했을 때, **odometry 토픽이 발행되지 않는 문제** 발생.

### 증상
- `ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py` 실행
- 카메라 노드는 정상 시작 ("RealSense Node Is Up!")
- Visual SLAM 노드도 정상 로드 ("cuVSLAM version: 12.6")
- 그러나 `/visual_slam/tracking/odometry` 토픽에 **데이터 없음**

```bash
$ ros2 topic hz /visual_slam/tracking/odometry
# 타임아웃 - 데이터 없음
```

---

## 디버깅 과정

### 1단계: 토픽 발행 확인

```bash
$ ros2 topic list | grep camera
/camera/infra1/image_rect_raw   # ✅ 존재
/camera/infra2/image_rect_raw   # ✅ 존재
/camera/gyro/sample             # ✅ 존재
/camera/accel/sample            # ✅ 존재
/camera/imu                     # ❌ 없음!
```

### 2단계: 카메라 토픽 데이터 흐름 확인

```bash
$ ros2 topic hz /camera/infra1/image_rect_raw
average rate: 89.944  # ✅ 정상 (640x360@90fps)

$ ros2 topic hz /camera/gyro/sample
average rate: 139.172  # ✅ 정상

$ ros2 topic hz /camera/accel/sample
average rate: 100.784  # ✅ 정상
```

카메라 이미지와 IMU 센서 데이터는 정상적으로 발행되고 있었음.

### 3단계: Visual SLAM 노드 구독 토픽 확인

```bash
$ ros2 node info /visual_slam_node
  Subscribers:
    /camera/imu: sensor_msgs/msg/Imu          # ⚠️ 이 토픽을 구독!
    /camera/infra1/image_rect_raw: sensor_msgs/msg/Image
    /camera/infra2/image_rect_raw: sensor_msgs/msg/Image
    ...
```

**핵심 발견**: Visual SLAM 노드는 `/camera/imu` 토픽을 구독하고 있었음!

### 4단계: Launch 파일 설정 확인

```python
# isaac_ros_visual_slam_realsense.launch.py
realsense_camera_node = Node(
    parameters=[{
        'unite_imu_method': 0,  # ⚠️ 문제의 원인!
        ...
    }],
)

visual_slam_node = ComposableNode(
    parameters=[{
        'enable_imu_fusion': True,  # IMU 융합 활성화
        ...
    }],
)
```

---

## 근본 원인

### `unite_imu_method` 파라미터의 역할

| 값 | 동작 | `/camera/imu` 토픽 |
|----|------|-------------------|
| `0` | None (비활성화) | ❌ 발행 안함 |
| `1` | Copy | ✅ 발행 |
| `2` | Linear Interpolation | ✅ 발행 (권장) |

- **`unite_imu_method=0`**: Gyro와 Accel 데이터가 개별 토픽(`/camera/gyro/sample`, `/camera/accel/sample`)으로만 발행됨
- **`unite_imu_method=2`**: Gyro와 Accel 데이터를 **선형보간**하여 `/camera/imu` 통합 토픽으로 발행

### 문제의 메커니즘

```
[RealSense Camera]
    │
    ├── /camera/gyro/sample (200Hz)  ─┐
    │                                  │  unite_imu_method=0
    └── /camera/accel/sample (100Hz) ─┘  → /camera/imu 미발행!
                                              │
                                              ▼
                                    [Visual SLAM Node]
                                    enable_imu_fusion=True
                                    구독: /camera/imu ← 데이터 없음!
                                              │
                                              ▼
                                    IMU 데이터 부재로 추적 불가
                                    /visual_slam/tracking/odometry 미발행
```

---

## 해결 방법

### Launch 파일 수정

```python
# 변경 전
'unite_imu_method': 0,

# 변경 후
'unite_imu_method': 2,  # linear_interpolation
```

### 전체 수정 코드

```python
realsense_camera_node = Node(
    name='camera',
    namespace='camera',
    package='realsense2_camera',
    executable='realsense2_camera_node',
    parameters=[{
        'enable_infra1': True,
        'enable_infra2': True,
        'enable_color': False,
        'enable_depth': False,
        'depth_module.emitter_enabled': 0,
        'depth_module.profile': '640x360x90',
        'enable_gyro': True,
        'enable_accel': True,
        'gyro_fps': 200,
        'accel_fps': 100,
        'unite_imu_method': 2,  # ✅ 수정: 선형보간으로 IMU 통합
        'enable_sync': True,
    }],
)
```

---

## 검증

수정 후 실행 결과:

```bash
$ ros2 topic hz /camera/imu
average rate: 200.0  # ✅ 정상 발행

$ ros2 topic hz /visual_slam/tracking/odometry
average rate: 30.0  # ✅ SLAM 정상 동작
```

---

## 교훈

1. **토픽 의존성 확인**: 노드가 구독하는 토픽이 실제로 발행되는지 확인 (`ros2 node info`)
2. **파라미터 이해**: `unite_imu_method`처럼 토픽 발행 여부에 영향을 주는 파라미터 숙지
3. **단계별 디버깅**: 데이터 흐름을 상류(센서)에서 하류(출력)까지 순차적으로 확인

---

## 관련 설정 요약

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `unite_imu_method` | `2` | Gyro+Accel → `/camera/imu` 통합 |
| `enable_imu_fusion` | `True` | Visual SLAM에서 IMU 데이터 사용 |
| `imu_frame` | `camera_imu_optical_frame` | IMU TF 프레임 |

---

## 참고

- [RealSense ROS2 Wrapper - IMU](https://github.com/IntelRealSense/realsense-ros)
- [Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/)
