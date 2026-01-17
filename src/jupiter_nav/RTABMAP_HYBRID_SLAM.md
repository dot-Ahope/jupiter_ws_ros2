# RTAB-Map Hybrid SLAM System

## 개요

Transbot의 RTAB-Map 통합은 **하이브리드 SLAM 아키텍처**를 사용합니다:
- **RTAB-Map rgbd_odometry**: RGB-D 카메라로 Visual Odometry + 3D 포인트 클라우드 생성
- **SLAM Toolbox**: 2D LiDAR SLAM + 내비게이션 맵 생성
- **Robot Localization EKF**: 다중 센서 융합 (Visual Odom + Wheel Odom + IMU)

## 시스템 구조

```
RGB-D Camera → rgbd_odometry (vo_odom→base_link) → /vo/odom ──┐
Wheel Encoders → base_node → /odom_wheel ──────────────────────┼→ EKF → /odometry/filtered → SLAM Toolbox → /map (2D)
IMU → imu_calib → /imu/data ────────────────────────────────────┘                              ↓
LiDAR → sllidar_node → /scan ──────────────────────────────────────────────────────────────────┘

3D Point Cloud: /rtabmap/cloud_map
```

## TF Tree

```
map (SLAM Toolbox, 5초 후)
  ↓
odom (EKF)
  ↓
base_footprint (EKF/base_node)
  ↓
base_link (robot_state_publisher)
  ├─ camera_link
  │   ├─ camera_depth_optical_frame
  │   └─ camera_color_optical_frame
  ├─ laser
  └─ imu_link

vo_odom (rgbd_odometry, 별도)
  ↓
base_link
```

## 실행 방법

### 기본 실행
```bash
source ~/transbot_ws_ros2/install/setup.bash
ros2 launch transbot_nav transbot_hybrid_slam.launch.py
```

### RViz 포함 실행
```bash
ros2 launch transbot_nav transbot_hybrid_slam.launch.py use_rviz:=true
```

### RGB-D 카메라 없이 실행 (LiDAR SLAM만)
```bash
ros2 launch transbot_nav transbot_hybrid_slam.launch.py use_rgbd:=false
```

## 주요 파일

### Launch 파일
- **`transbot_hybrid_slam.launch.py`**: 메인 하이브리드 SLAM 런치 파일

### 설정 파일
- **`rtabmap_vo_params.yaml`**: Visual Odometry 파라미터
- **`ekf_hybrid_config.yaml`**: 다중 센서 융합 EKF 설정
- **`slam_params.yaml`**: SLAM Toolbox 2D 매핑 설정

### RViz 설정
- **`hybrid_slam.rviz`**: 하이브리드 SLAM 시각화 설정
  - Fixed Frame: `odom`
  - 포함: PointCloud2, Map, LaserScan, TF, Paths, Images, RobotModel

## 주요 토픽

### 입력
- `/camera/color/image_raw` - RGB 이미지
- `/camera/depth/image_raw` - Depth 이미지
- `/camera/color/camera_info` - 카메라 정보
- `/scan` - LiDAR 스캔
- `/imu/data` - IMU 데이터
- `/odom_wheel` - 휠 오도메트리

### 출력
- `/vo/odom` - Visual Odometry
- `/odometry/filtered` - 융합된 오도메트리 (EKF)
- `/map` - 2D 내비게이션 맵
- `/rtabmap/cloud_map` - 3D 포인트 클라우드 맵
- `/vo/odom_path` - Visual Odometry 경로
- `/filtered_odom_path` - 융합된 오도메트리 경로

## 센서 설정

### Astra Pro 카메라
- **RGB**: 640x480 @ 30Hz (UVC)
- **Depth**: 640x480 @ 30Hz
- **IR**: 비활성화 (성능 최적화)

### Visual Odometry 최적화
- **이미지 다운샘플링**: 2x (320x240 처리)
- **키프레임 임계값**: 0.4 (40% 변화만 처리)
- **특징점 수**: 300개

### LiDAR
- **RPLidar A1**: 5Hz, 최대 4m

## 성능 최적화 (Jetson Nano)

1. **카메라**: 30Hz 고정 (하드웨어 제한)
2. **Visual Odometry**: 
   - 이미지 2x 다운샘플링 (4배 속도 향상)
   - 특징점 감소 (400→300)
3. **SLAM Toolbox**:
   - 버퍼 크기: 100 (메시지 드롭 방지)
   - 맵 업데이트: 1초 간격
4. **시작 지연**:
   - rgbd_odometry: 5초 (TF 준비 대기)
   - SLAM Toolbox: 5초 (EKF 준비 대기)

## 문제 해결

### "base_footprint does not exist" 오류
- **원인**: rgbd_odometry가 너무 빨리 시작
- **해결**: `base_link` 프레임 사용 (항상 존재)

### "map frame does not exist" 오류
- **원인**: SLAM Toolbox가 아직 시작 안 함
- **해결**: RViz Fixed Frame을 `odom`으로 설정

### 카메라 토픽 발행 안 됨
- **원인**: 15Hz 미지원 (Astra Pro는 30Hz만 지원)
- **해결**: 30Hz로 설정 유지, 소프트웨어 최적화로 CPU 부하 감소

### SLAM Toolbox 메시지 드롭
- **원인**: 버퍼 크기 부족
- **해결**: `scan_buffer_size: 100`으로 증가

## 장점

1. **안정적인 오도메트리**: Visual Odometry로 드리프트 감소
2. **3D 매핑**: RGB-D 카메라로 3D 포인트 클라우드 생성
3. **2D 내비게이션**: SLAM Toolbox로 Nav2 호환 맵 생성
4. **센서 융합**: EKF로 다중 센서 통합
5. **Nav2 완벽 통합**: 표준 ROS2 내비게이션 스택 사용 가능

## 다음 단계

1. **실제 환경 테스트**: 로봇 주행하며 맵 생성 확인
2. **3D 포인트 클라우드 검증**: `/rtabmap/cloud_map` 품질 확인
3. **Nav2 통합**: 자율 주행 구현
4. **루프 클로저 테스트**: 같은 장소 재방문 시 맵 정합 확인

## 참고 사항

- **최소 시작 시간**: 5초 (모든 센서 및 TF 준비)
- **권장 CPU**: 4코어 이상 (Jetson Nano는 CPU 제한적)
- **메모리**: 최소 2GB 여유 필요
- **저장 공간**: 맵 크기에 따라 가변 (100MB~수GB)

## 작성 정보

- **최종 업데이트**: 2025-11-07
- **ROS2 버전**: Humble
- **테스트 플랫폼**: Jetson Nano (4GB)
