# WORKSPACE Sensor Summary

작성일: 2025-09-02

이 문서는 현재 워크스페이스에서 오도메트리(nav_msgs/Odometry)와 IMU(sensor_msgs/Imu)를 생산하거나 구독/참조하는 패키지와 핵심 파일들을 정리한 것입니다.

## 요약
- 시스템은 "오도메트리 + IMU"를 함께 사용하도록 설계되어 있습니다.
  - 드라이버(`transbot_driver`)가 IMU(`/transbot/imu`)와 속도(`/transbot/get_vel`)를 퍼블리시합니다.
  - `transbot_base`의 `base_node`가 속도 정보를 받아 `/odom_raw`(nav_msgs/Odometry)를 계산·퍼블리시합니다.
  - EKF/네비게이션/SLAM/rviz 구성은 `/odom_raw` 또는 `/odom`을 주요 입력으로 기대합니다.

## 패키지별 표
| 패키지 | 오도메트리 사용? | IMU 사용? | 핵심 파일(예시 경로) |
|---|---:|---:|---|
| `transbot_base` (ROS2) | 예 — 퍼블리시 `/odom_raw` | 아니요 (직접 퍼블리셔 역할) | `transbot_ws_ros2/src/transbot_base/src/base.cpp`<br>`transbot_ws_ros2/src/transbot_base/src/base_node.cpp` |
| `transbot_nav` | 예 — 구독/참조 (SLAM/네비/rviz) | 일부 보조 사용 가능 | `transbot_ws/src/transbot_nav/src/rrt_save_map.cpp` (구독 `/odom`)<br>`transbot_ws/src/transbot_nav/rviz/transbot_map.rviz`<br>`transbot_ws/src/transbot_nav/rviz/rrt_map.rviz` |
| `transbot_bringup` | 예 — EKF 파라미터가 `odom0: /odom_raw` 참조 | 예 — launch/driver에서 `/transbot/imu` 사용 | `transbot_ws_ros2/src/transbot_bringup/launch/bringup.launch.py`<br>`transbot_ws/src/transbot_bringup/param/ekf/robot_localization.yaml` |
| `transbot_driver` | 아니요 (직접 odom 퍼블리시하지 않음) | 예 — 퍼블리시 `/transbot/imu`, `/transbot/get_vel` | `/home/user/transbot_ws_ros2/install/transbot_bringup/lib/transbot_bringup/transbot_driver` (설치된 실행 스크립트) |
| `transbot_mulity` | 예 — EKF/launch에서 `/odom_raw` 사용 | 예 — launch에서 `/transbot/imu` 파라미터 전달 | `transbot_ws/src/transbot_mulity/launch/transbot_mulity_control.launch`<br>`transbot_ws/src/transbot_mulity/param/ekf/robot_localization.yaml` |
| `transbot_astra` | 아니요 | 예 — 런치에서 `imu` 파라미터 전달 | `transbot_ws/src/transbot_astra/launch/DepthSrv.launch`<br>`transbot_ws/src/transbot_astra/launch/KCFTracker.launch` |
| `transbot_laser` | 아니요 | 예 — 런치에서 `imu` 파라미터 전달 | `transbot_ws/src/transbot_laser/launch/base.launch` |
| `scripts` (도구) | 예(선택적) — 구독 `/odom_raw` 사용 가능 | 예 — 구독 `/transbot/imu` | `transbot_ws_ros2/scripts/log_imu_and_distance.py` (구독: `/transbot/imu`, `/odom_raw`) |
| `robot_localization` (library) | 예 — EKF/UKF 입력으로 odom 사용 (`odom0: /odom_raw`) | 예 — IMU도 입력으로 사용 가능 | `software/library_ws/src/robot_localization/params/ekf_x1_x3.yaml`<br>`software/library_ws/src/robot_localization/launch/ekf.launch.py` |
| `imu_calib` (library) | 아니요 | 예 — `/transbot/imu` 구독/처리 | `software/transbot_library/src/imu_calib/src/apply_calib.cpp`<br>`software/transbot_library/src/imu_calib/src/do_calib.cpp` |

## 기타 참조 파일 (rviz / 설치된 리소스)
- RViz 구성에서 `/odom_raw`를 사용: `transbot_ws/src/transbot_nav/rviz/transbot_map.rviz`, `transbot_ws/src/transbot_nav/rviz/rrt_map.rviz` 등.
- EKF 파라미터들에서 `/odom_raw` 참조: `transbot_ws/src/transbot_bringup/param/ekf/robot_localization.yaml`, `transbot_ws/src/transbot_mulity/param/ekf/robot_localization.yaml` 등.

## 실행/검증 팁
- 런치로 bringup 실행 후 `/odom_raw`와 `/transbot/imu`가 올라오는지 확인:
```bash
. /home/user/transbot_ws_ros2/install/setup.bash
ros2 launch transbot_bringup bringup.launch.py
ros2 topic list | grep -E "(^/odom_raw$|^/transbot/imu$|/transbot/get_vel)"
ros2 node list | grep -E "base_node|transbot_driver"
```

- 로그/분석: `transbot_ws_ros2/scripts/log_imu_and_distance.py`를 실행하면 IMU와 (가능하면) odom 기반 누적거리를 `~/imu_distance_log.csv`에 저장합니다.

## 추가 제공 가능 항목
- 패키지별 관련 파일의 더 상세한 전체 경로와 참조 라인(예: 라인 번호 포함) CSV/마크다운으로 생성 가능
- 현재 런타임 상태(실제 발행 중인 토픽/퍼블리셔 증거) 수집
- `~/imu_distance_log.csv`의 샘플(상위 N행)과 간단한 통계/플롯

---
생성: `/home/user/transbot_ws_ros2/WORKSPACE_SENSOR_SUMMARY.md`
