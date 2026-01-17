# transbot_nav 패키지 - 빠른 참조 가이드 🚀

## 📦 패키지 정보
- **이름:** transbot_nav
- **버전:** 1.0.0
- **타입:** ament_python
- **목적:** Transbot 네비게이션 및 SLAM 통합

## 🎯 핵심 명령어

### 전체 시스템 실행 (SLAM + 모든 센서)
```bash
ros2 launch transbot_nav transbot_full_system.launch.py
```

### Nav2 자율 주행 실행 (별도 터미널)
```bash
ros2 launch transbot_nav nav2_navigation.launch.py
```

### RViz 시각화
```bash
rviz2 -d $(ros2 pkg prefix transbot_nav)/share/transbot_nav/rviz/sllidar_ros2.rviz
```

## 📁 주요 파일 위치

### Launch 파일
```
~/transbot_ws_ros2/src/transbot_nav/launch/
├── transbot_full_system.launch.py   # 전체 시스템
└── nav2_navigation.launch.py        # Nav2 스택
```

### Config 파일
```
~/transbot_ws_ros2/src/transbot_nav/config/
├── ekf_config.yaml      # IMU + 오도메트리 퓨전
├── slam_params.yaml     # SLAM 맵 생성 설정
└── nav2_params.yaml     # 자율 주행 파라미터
```

### 저장된 맵
```
~/transbot_ws_ros2/src/transbot_nav/maps/
├── my_map.pgm          # 맵 이미지
└── my_map.yaml         # 맵 메타데이터
```

## ⚙️ 빠른 설정 조정

### SLAM 맵 품질 향상
```yaml
# config/slam_params.yaml
loop_match_minimum_response_fine: 0.70  # 더 엄격 (0.50 ~ 0.80)
minimum_travel_distance: 0.05           # 더 촘촘하게 스캔
```

### 로봇 속도 조정
```yaml
# config/nav2_params.yaml
max_vel_x: 0.3        # 전진 속도 (m/s)
max_vel_theta: 0.5    # 회전 속도 (rad/s)
```

### IMU 신뢰도 조정
```yaml
# config/ekf_config.yaml
imu0_angular_velocity_covariance: 0.000025  # 낮을수록 IMU 신뢰↑
```

## 🐛 빠른 문제 해결

### 1. Nav2 노드가 안 떠요
```bash
# 상태 확인
ros2 lifecycle list

# lifecycle_manager 확인
ros2 topic echo /lifecycle_manager_navigation/transition_event
```

### 2. 맵이 흐려요/겹쳐요
```bash
# 회전 속도 줄이기
max_vel_theta: 0.3  # nav2_params.yaml에서

# SLAM 기준 더 엄격하게
loop_match_minimum_response_fine: 0.70  # slam_params.yaml에서
```

### 3. 로봇이 진동해요
```bash
# 가속도 제한 확인 (최소값 유지)
acc_lim_x: 2.5       # nav2_params.yaml
acc_lim_theta: 3.2
```

### 4. "Transform timeout" 에러
```bash
# TF 체인 확인
ros2 run tf2_tools view_frames

# 예상 체인: map → odom → base_footprint → base_link
```

## 📊 유용한 모니터링

### TF 트리 저장
```bash
ros2 run tf2_tools view_frames
# frames_<날짜>.gv 파일 생성
evince frames_*.pdf  # 또는
xdot frames_*.gv
```

### Topic 주파수 확인
```bash
ros2 topic hz /scan        # LiDAR: ~10Hz
ros2 topic hz /imu/data    # IMU: ~100Hz
ros2 topic hz /odom        # Odom: ~50Hz
```

### 맵 품질 확인
```bash
# 맵 토픽 시각화
ros2 topic echo /map --once

# 점유율 분포 확인
ros2 topic echo /map | grep -A 10 "data:"
```

## 🎮 수동 제어

### 키보드로 로봇 제어
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 속도 제한:
# x: 0.3 m/s
# z: 0.5 rad/s
```

### 특정 위치로 명령 전송
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: { frame_id: 'map' },
    pose: {
      position: { x: 1.0, y: 0.5, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
  }
}"
```

## 💾 맵 저장/로드

### 맵 저장
```bash
ros2 run nav2_map_server map_saver_cli -f ~/transbot_ws_ros2/src/transbot_nav/maps/my_map
```

### 맵 로드 (Localization 모드)
```bash
# slam_params.yaml에서:
mode: localization

# map_server 실행
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=~/transbot_ws_ros2/src/transbot_nav/maps/my_map.yaml
```

## 🔧 재빌드

### 패키지 재빌드
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_nav --symlink-install
source install/setup.bash
```

### 설정 파일만 변경한 경우
```bash
# symlink-install 사용 시 재빌드 불필요
# 그냥 launch 파일 재실행
```

## 📚 관련 문서

- `README.md` - 상세 패키지 설명
- `MIGRATION_SUMMARY.md` - 패키지 생성 과정
- `Nav2_네비게이션_문제해결_가이드.md` - Nav2 전용 가이드 (있다면)

## 🔗 외부 리소스

- [Nav2 문서](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](http://docs.ros.org/en/humble/p/robot_localization/)

---
**작성일:** 2025-10-31  
**ROS2 버전:** Humble
