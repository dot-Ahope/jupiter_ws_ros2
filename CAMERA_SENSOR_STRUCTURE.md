# Astra 카메라 센서 구조

## 🎥 카메라 센서 구성

Astra Pro 카메라는 **2개의 물리적 센서**를 가지고 있습니다:

### 1️⃣ RGB 센서 (컬러 카메라)
- **독립적인 컬러 센서**
- 일반 웹캠과 유사
- **출력**: `/camera/color/image_raw`

```
┌─────────────────┐
│  RGB 센서       │  ← 독립 센서
│  (컬러 카메라)   │
└─────────────────┘
        ↓
   color/image_raw
```

### 2️⃣ IR 센서 (적외선 + 깊이)
- **적외선 스테레오 센서**
- Depth는 IR 데이터로부터 계산됨
- **출력**: 
  - `/camera/ir/image_raw` (IR 원본)
  - `/camera/depth/image_raw` (계산된 깊이)

```
┌─────────────────┐
│  IR 센서        │  ← 단일 센서
│  (적외선)        │
└─────────────────┘
        ↓
   ┌────┴────┐
   ↓         ↓
ir/image  depth/image
 (원본)    (계산됨)
```

## 📊 토픽 구조

### 현재 설정 (모든 센서 활성화)

```yaml
enable_color: true   # RGB 센서
enable_ir: true      # IR 센서
enable_depth: true   # Depth 계산
```

### 발행되는 토픽

#### RGB (컬러 센서)
- `/camera/color/image_raw` - 640x480 @ 15 Hz
- `/camera/color/camera_info` - 카메라 내부 파라미터

#### IR (적외선 센서)
- `/camera/ir/image_raw` - 640x480 @ 15 Hz
- `/camera/ir/camera_info` - 카메라 내부 파라미터

#### Depth (계산된 깊이)
- `/camera/depth/image_raw` - 640x480 @ 15 Hz
- `/camera/depth/camera_info` - 카메라 내부 파라미터
- `/camera/depth/points` - 3D 포인트 클라우드

## 🔧 TF 프레임 구조

```
base_link
    ↓
camera_link (물리적 카메라 위치)
    ↓
    ├─→ camera_color_frame (RGB 광학 프레임)
    ├─→ camera_depth_frame (Depth 광학 프레임)
    └─→ camera_ir_frame (IR 광학 프레임)
```

**Optical Frame**: 카메라 좌표계 표준
- X: 오른쪽
- Y: 아래
- Z: 앞 (카메라가 보는 방향)

## 🎯 RTAB-Map 토픽 매핑

### RGB-D SLAM에 사용되는 토픽:

```python
remappings=[
    ('rgb/image', '/camera/color/image_raw'),      # RGB 센서
    ('rgb/camera_info', '/camera/color/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),     # IR 센서 (계산)
]
```

### 추가 데이터 (옵션):
- IR 이미지: 저조도 환경에서 유용
- Point Cloud: 3D 시각화

## ✅ 센서 확인 방법

### 1. 모든 토픽 확인
```bash
ros2 topic list | grep camera
```

**예상 출력:**
```
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/points
/camera/ir/camera_info
/camera/ir/image_raw
```

### 2. 각 센서 Hz 확인
```bash
cd ~/transbot_ws_ros2
./check_camera_sensors.sh
```

**예상 출력:**
- RGB: ~15 Hz
- Depth: ~15 Hz
- IR: ~15 Hz

### 3. 센서 데이터 시각화

**RViz로 확인:**
```bash
rviz2
```

**Add → Image → Topic 선택:**
- `/camera/color/image_raw` - 컬러 이미지
- `/camera/depth/image_raw` - 깊이 맵 (흑백)
- `/camera/ir/image_raw` - 적외선 이미지 (흑백)

**Add → PointCloud2 → Topic:**
- `/camera/depth/points` - 3D 포인트 클라우드

## 🔬 센서 특성

### RGB 센서 특성
- ✅ 풀 컬러 (RGB)
- ✅ 일반 조명에서 작동
- ✅ 텍스처 인식 우수
- ❌ 어두운 환경에서 약함

### IR 센서 특성
- ✅ 저조도/어둠에서 작동
- ✅ 정확한 깊이 측정
- ✅ 텍스처 없는 표면 감지
- ❌ 직사광선에 약함
- ❌ 반사 표면 측정 어려움

### Depth 센서 특성
- ✅ 거리: 0.4m ~ 4.0m
- ✅ 정확도: ±2cm @ 2m
- ✅ IR과 동기화
- ❌ RGB와 약간의 시간차

## 🚀 사용 예시

### 카메라 테스트
```bash
cd ~/transbot_ws_ros2
./test_camera.sh
```

### RTAB-Map 실행 (RGB-D SLAM)
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav transbot_rtabmap.launch.py \
  use_rgbd:=true \
  use_rviz:=true
```

### 센서 상태 모니터링
```bash
# 실시간 확인
watch -n 1 "ros2 topic list | grep camera | wc -l"

# 예상: 7개 토픽 (color x2, depth x3, ir x2)
```

## 📝 주의사항

1. **동시 사용**: RGB, IR, Depth를 동시에 활성화하면 대역폭 증가
   - 해결: FPS를 15로 제한 (30에서 감소)

2. **타임스탬프 동기화**: RGB와 Depth는 다른 센서이므로 완벽한 동기화 불가
   - 해결: `approx_sync: true` 사용

3. **조명 조건**: 
   - 밝은 환경: RGB + Depth 사용
   - 어두운 환경: IR + Depth 사용
   - 직사광선: RGB만 사용 (IR 간섭)

4. **리소스**: Jetson Nano에서 모든 센서 활성화 시 CPU 부하 증가
   - 모니터링: `htop` 또는 `jtop`

---

**정리**: Astra는 **RGB 센서(1개) + IR 센서(1개) = 총 2개 물리 센서**이며, Depth는 IR로부터 계산됩니다.
