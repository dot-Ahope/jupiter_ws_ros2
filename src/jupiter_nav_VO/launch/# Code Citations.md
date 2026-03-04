# Code Citations

## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```


## License: Apache-2.0
https://github.com/dblanding/ROS2_live_course/blob/504bb387c48a38f79f72e578261988722e2967c4/sensors%26fusion.md

```
# Isaac ROS Visual Odometry 융합 분석

현재 열려 있는 파일은 MCU 레벨의 PID 제어 코드(`app_pid.c`)이며, 질문은 ROS2 워크스페이스(`jupiter_ws_ros2`)의 `jupiter_nav_VO` 패키지에서 Isaac VSLAM 기반 오도메트리를 다른 소스와 융합하는 것에 관한 것입니다.

## 현재 구조 분석

`jupiter_nav_VO`에서 **Isaac ROS Visual SLAM (cuVSLAM)**을 사용하여 Visual Odometry를 생성하고 있다고 가정합니다.

## 활용 가능한 오도메트리 소스

### 1. **Wheel Odometry (바퀴 오도메트리)**
- 현재 MCU의 `app_pid.c`에서 PID 제어되는 모터의 엔코더 데이터를 기반으로 생성 가능
- `nav_msgs/msg/Odometry` 토픽으로 퍼블리시
- 패키지: 직접 구현 또는 `ros2_control` + `diff_drive_controller`

### 2. **Isaac ROS 패키지들로 추가 오도메트리 생성**

| 패키지 | 센서 | 출력 | 특징 |
|--------|------|------|------|
| `isaac_ros_visual_slam` | 스테레오/Depth 카메라 | `visual_slam/tracking/odometry` | **현재 사용 중** |
| `isaac_ros_nvblox` | Depth 카메라 + 포즈 | 3D 맵 + 로컬라이제이션 | VSLAM과 연동 가능 |
| `isaac_ros_imu_bmi088` | IMU | `sensor_msgs/Imu` | IMU 데이터 제공 |

### 3. **기타 ROS2 패키지**

| 패키지 | 용도 |
|--------|------|
| `robot_localization` (EKF/UKF) | **다중 오도메트리 융합의 핵심** |
| `rtabmap_ros` | Visual + LiDAR SLAM |
| `rf2o_laser_odometry` | 2D LiDAR 기반 오도메트리 |
| `imu_filter_madgwick` | IMU 데이터 필터링 |

## 권장 융합 아키텍처

```
┌─────────────────┐   ┌──────────────────┐   ┌─────────────────┐
│  Isaac VSLAM    │   │  Wheel Odometry  │   │  IMU            │
│  (Visual Odom)  │   │  (Encoder기반)    │   │  (가속도/자이로) │
│  /visual_odom   │   │  /wheel_odom     │   │  /imu/data      │
└────────┬────────┘   └────────┬─────────┘   └────────┬────────┘
         │                     │                       │
         └─────────────┬───────┘───────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │   robot_localization    │
         │   (EKF / UKF)          │
         │   /odometry/filtered   │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │   Nav2 Navigation      │
         └─────────────────────────┘
```

## 구현 방법

### Step 1: `robot_localization` EKF 설정 파일

````yaml
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_nav_VO/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true    # 2D 로봇인 경우
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 오도메트리 소스 0: Wheel Odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```

