# RTAB-Map Depth-Only SLAM 가이드

## 현재 상태

현재 시스템은 다음과 같이 구성되어 있습니다:

### 하드웨어
- **Astra 깊이 카메라**: RGB 없음, Depth + IR만 제공
- **RPLidar A1**: 2D 레이저 스캔
- **MPU6050 IMU**: 가속도 + 자이로
- **휠 엔코더**: 오도메트리

### 문제점
1. **Astra 카메라가 RGB(color) 이미지를 제공하지 않음**
   - 로그: `[astra_camera_node-9] [INFO] color is not enable`
   - Astra AS3G65300V5 모델은 Depth + IR만 지원

2. **RTAB-Map이 RGB+Depth를 요구함**
   - RGB-D SLAM은 RGB 이미지가 필수
   - Depth만으로는 visual features 추출 불가

## 해결 방안

### 방안 1: IR 이미지를 RGB 대신 사용 (권장)
IR(적외선) 이미지를 "RGB" 대신 사용할 수 있습니다.

**장점**:
- Visual features를 추출 가능
- Loop closure detection 가능
- RTAB-Map의 모든 기능 사용 가능

**단점**:
- IR 이미지는 색상 정보 없음 (흑백)
- 특징점이 RGB보다 적을 수 있음

**구현 방법**:
```python
# rgbd_sync 노드에서 IR을 RGB로 remap
remappings=[
    ('rgb/image', '/camera/ir/image_raw'),  # IR을 RGB 대신 사용
    ('rgb/camera_info', '/camera/ir/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),
    ('depth/camera_info', '/camera/depth/camera_info'),
]
```

### 방안 2: LiDAR 기반 SLAM만 사용
RTAB-Map을 사용하지 않고 기존 SLAM Toolbox 사용

**장점**:
- 이미 작동하고 있음
- LiDAR만으로 충분히 정확한 2D SLAM
- 낮은 계산 부하

**단점**:
- 3D 맵 생성 불가
- Visual loop closure 없음

**실행 방법**:
```bash
# 기존 방식 (SLAM Toolbox만 사용)
ros2 launch sllidar_ros2 transbot_full_system.launch.py use_rviz:=true
```

### 방안 3: Depth + LiDAR ICP 기반 SLAM
RTAB-Map을 Depth + LiDAR ICP 모드로 실행 (Visual features 없이)

**장점**:
- RGB 불필요
- 3D 맵 생성 가능
- ICP(Iterative Closest Point)로 정합

**단점**:
- Loop closure가 약함
- 계산 부하 높음
- 특징이 없는 환경에서 취약

## 권장 솔루션: IR 이미지 사용

IR 이미지를 사용하는 것이 가장 실용적입니다. 다음 단계로 구현하겠습니다:

### 1. IR 이미지 확인
```bash
# IR 토픽 확인
ros2 topic list | grep ir
ros2 topic hz /camera/ir/image_raw
ros2 run rqt_image_view rqt_image_view /camera/ir/image_raw
```

### 2. RTAB-Map launch 수정
IR을 RGB 대신 사용하도록 remapping

### 3. 성능 최적화
- `Kp/MaxFeatures`: IR 이미지 특성에 맞게 조정
- `Vis/MinInliers`: Loop closure 임계값 조정
- `RGBD/OptimizeFromGraphEnd`: Graph 최적화 설정

## 다음 단계

1. IR 토픽이 발행되는지 확인
2. IR를 RGB로 remapping하는 launch 파일 생성
3. 테스트 및 파라미터 튜닝

## 참고

ROS1 transbot_nav 패키지도 동일한 Astra 카메라를 사용했을 것이므로,
ROS1 버전의 설정을 확인하면 어떻게 해결했는지 알 수 있습니다.
