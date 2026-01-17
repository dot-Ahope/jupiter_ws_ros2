# RTAB-Map RGB-D SLAM 완전 설정 가이드

## 📋 시스템 구성

### 센서
- **RGB 카메라**: Astra Pro (UVC 모드, 30Hz)
- **Depth 센서**: Astra Pro IR+Depth (30Hz)
- **IR 센서**: Astra Pro (30Hz)
- **LiDAR**: RPLidar A1 (50Hz)
- **IMU**: MPU6050 (98Hz)

### 주요 컴포넌트
1. **Astra Camera**: RGB+Depth+IR 통합
2. **RTAB-Map**: RGB-D Visual SLAM
3. **robot_localization**: EKF 센서 퓨전
4. **Nav2**: 자율 주행

## 🚀 실행 방법

### 1. 카메라 단독 테스트
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav test_camera.launch.py
```

**센서 확인:**
```bash
# 토픽 리스트
ros2 topic list | grep camera

# 예상 출력:
/camera/color/image_raw          # RGB (UVC)
/camera/depth/image_raw          # Depth
/camera/ir/image_raw             # IR
/camera/depth/color/points       # RGB+Depth 융합
/camera/color/camera_info
/camera/depth/camera_info
/camera/ir/camera_info

# Hz 확인
ros2 topic hz /camera/color/image_raw  # ~30Hz
ros2 topic hz /camera/depth/image_raw  # ~25Hz
ros2 topic hz /camera/ir/image_raw     # ~30Hz
```

### 2. RTAB-Map 전체 시스템 실행

#### 방법 A: 스크립트 사용 (권장)
```bash
~/transbot_ws_ros2/run_rtabmap_rgbd.sh
```

#### 방법 B: 직접 실행
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav transbot_rtabmap.launch.py use_rgbd:=true use_rviz:=true
```

#### Launch 파라미터
- `use_rgbd`: RGB-D 카메라 사용 여부 (default: true)
- `use_rviz`: RViz 시각화 (default: false)
- `lidar_port`: LiDAR 포트 (default: /dev/transbot_lidar)

### 3. RViz 설정

**추가할 Display:**
1. **PointCloud2** - `/camera/depth/color/points` (RGB+Depth 융합)
2. **Image** - `/camera/color/image_raw` (RGB 카메라)
3. **Image** - `/camera/depth/image_raw` (Depth)
4. **Map** - `/rtabmap/grid_map` (RTAB-Map 생성 맵)
5. **MapCloud** - `/rtabmap/mapData` (3D 포인트 클라우드 맵)
6. **LaserScan** - `/scan` (LiDAR)
7. **TF** - 모든 좌표계

**Fixed Frame:** `map` 또는 `odom`

## ⚙️ 핵심 설정

### 카메라 파라미터 (UVC 모드)
```python
'enable_color': True,
'use_uvc_camera': True,         # ✅ RGB를 UVC로 사용
'uvc_vendor_id': '0x2bc5',      # Orbbec
'uvc_product_id': '0x0501',     # RGB 카메라
'uvc_camera_format': 'mjpeg',

'enable_depth': True,
'enable_ir': True,              # ✅ RGB와 동시 사용 가능
'enable_colored_point_cloud': True,  # RGB+Depth 융합
```

### RTAB-Map 파라미터
```yaml
Reg/Strategy: "0"                    # Visual (RGB-D)
RGBD/OptimizeFromGraphEnd: "true"
Mem/IncrementalMemory: "true"
Mem/InitWMWithAllNodes: "false"

# RGB-D 입력
subscribe_rgb: true
subscribe_depth: true
subscribe_scan: true                 # LiDAR 융합

# 동기화
approx_sync: true                    # 타임스탬프 허용 오차
queue_size: 30
```

### TF 트리 구조
```
map
 └─ odom
     └─ base_footprint
         ├─ base_link
         │   ├─ camera_link
         │   │   ├─ camera_color_frame
         │   │   ├─ camera_depth_frame
         │   │   └─ camera_ir_frame
         │   ├─ laser
         │   └─ imu_link
         └─ left_wheel_link
         └─ right_wheel_link
```

## 🔍 문제 해결

### 카메라가 연결되지 않음
```bash
# USB 장치 확인
lsusb | grep 2bc5

# 예상 출력:
# Bus 001 Device 007: ID 2bc5:060f Orbbec ... (Depth+IR)
# Bus 001 Device 009: ID 2bc5:050f Orbbec ... (RGB)

# 뮤텍스 파일 제거
rm -f /tmp/XnCore.Mutex.*

# 카메라 노드 재시작
pkill -9 -f astra_camera_node
```

### RGB 토픽이 발행되지 않음
- `use_uvc_camera: True` 확인
- USB 권한 확인: `ls -la /dev/bus/usb/001/009`
- udev rules 설치: `sudo cp 99-astra-camera.rules /etc/udev/rules.d/`

### RTAB-Map이 데이터를 받지 못함
```bash
# 토픽 확인
ros2 topic echo /camera/color/image_raw --once
ros2 topic echo /camera/depth/image_raw --once

# TF 확인
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_color_frame
```

### 성능 최적화 (Jetson Nano)
1. **해상도 조정**: 640x480 → 320x240
2. **FPS 감소**: 30Hz → 15Hz
3. **Point Cloud 다운샘플링**: `Grid/DepthDecimation: 4`
4. **메모리 관리**: `Mem/ImageKept: false`

## 📊 성능 모니터링

```bash
# CPU/메모리 사용률
jtop

# 토픽 Hz 확인
ros2 topic hz /rtabmap/mapData
ros2 topic hz /odometry/filtered

# TF 지연 확인
ros2 run tf2_ros tf2_monitor

# RTAB-Map 통계
ros2 topic echo /rtabmap/info
```

## 📁 관련 파일

### Launch 파일
- `transbot_rtabmap.launch.py` - 메인 RTAB-Map 시스템
- `test_camera.launch.py` - 카메라 단독 테스트

### 설정 파일
- `config/rtabmap_params.yaml` - RTAB-Map 파라미터
- `99-astra-camera.rules` - USB 권한 규칙

### 스크립트
- `run_rtabmap_rgbd.sh` - RTAB-Map 실행
- `restart_camera_with_uvc.sh` - 카메라 재시작
- `check_camera_sensors.sh` - 센서 확인

### 문서
- `CAMERA_UVC_SOLUTION.md` - UVC 모드 설정 가이드
- `CAMERA_TROUBLESHOOTING.md` - 카메라 문제 해결
- `CAMERA_SENSOR_STRUCTURE.md` - 센서 구조 설명
- `SLAM_COMPARISON.md` - SLAM 알고리즘 비교

## 🎯 다음 단계

1. **매핑**: 로봇을 천천히 이동시켜 환경 맵 생성
2. **맵 저장**: 
   ```bash
   ros2 service call /rtabmap/save_map std_srvs/srv/Empty
   ```
3. **네비게이션**: Nav2로 자율 주행 테스트
4. **루프 클로저**: 같은 장소로 돌아와서 맵 정합 확인

## 💡 팁

- **조명**: RGB 카메라는 밝은 조명 필요
- **속도**: 천천히 이동 (0.2 m/s 이하)
- **특징점**: 텍스처가 많은 환경이 유리
- **거리**: Depth 센서 유효 범위 0.5~4.0m
- **회전**: 급격한 회전 피하기

---

**완료 상태**: ✅ 모든 센서 통합 완료 (RGB+Depth+IR+LiDAR+IMU)
**테스트 완료**: ✅ 카메라 UVC 모드 확인
**빌드 완료**: ✅ transbot_nav 패키지

