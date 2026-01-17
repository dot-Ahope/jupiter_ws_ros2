# 카메라 하드웨어 분석 결과

## 실행 날짜
2025-10-28

## 하드웨어 확인

### 1. Astra AS3G65300V5 (Orbbec Depth Sensor)
**USB ID**: 2bc5:060f  
**디바이스 경로**: /dev/bus/usb/001/007

#### 지원 스트림
- ✅ **Depth Stream**: 640x480@30Hz, PIXEL_FORMAT_DEPTH_1_MM
  - 토픽: `/depth/image_raw`
  - 프레임: `camera_depth_optical_frame`
  
- ✅ **IR (Infrared) Stream**: 640x480@30Hz, PIXEL_FORMAT_GRAY8  
  - 토픽: `/ir/image_raw`
  - 프레임: `camera_ir_optical_frame`
  - **주의**: IR과 COLOR는 동시 활성화 불가 (하드웨어 제약)
  
- ❌ **RGB/Color Stream**: **미지원**
  - **이유**: `device_->hasSensor(COLOR)` 반환값이 false
  - 하드웨어에 COLOR 센서가 없음
  - USB 디스크립터에도 비디오 클래스 없음

#### 소스 코드 분석
```cpp
// ob_camera_node.cpp:161
if (device_->hasSensor(stream_index.first) && enable_stream_[stream_index]) {
    // COLOR 센서가 없어서 이 블록이 실행되지 않음
}

// ob_camera_node.cpp:153
if (!use_uvc_camera_ && enable_stream_[INFRA1] && enable_stream_[COLOR]) {
    // IR과 COLOR 동시 활성화 불가
    enable_stream_[INFRA1] = false;
}
```

#### lsusb 출력
```
Bus 001 Device 007: ID 2bc5:060f ORBBEC Depth Sensor
```
- 비디오 클래스(Video Class) 없음
- "Depth Sensor"로만 식별됨

---

### 2. USB 2.0 Camera (별도 장치)
**디바이스 경로**: `/dev/video0`, `/dev/video1`

#### 지원 스트림
- ✅ **RGB/Color Stream**
  - 포맷: MJPG, YUYV
  - 해상도: 최대 1920x1080@30fps
  - 640x480@30fps도 지원
  
#### v4l2 출력
```
Format: 0 MJPG 1920x1080@30fps
Format: 1 YUYV 640x480@30fps
Format: 2 YUYV 1280x720@10fps
Format: 3 YUYV 1920x1080@5fps
```

#### ROS2 통합
- **노드**: `device_srv` (transbot_bringup 패키지)
- **토픽**: `/image/image_raw`, `/image/camera_info`
- **프레임**: `camera_rgb_optical_frame`

---

## 결론

### RGB-D SLAM을 위한 센서 구성
1. **Depth**: Astra AS3G65300V5의 `/depth/image_raw`
2. **RGB**: USB 2.0 Camera의 `/image/image_raw`

### RTAB-Map 설정
```yaml
# RGB-D 동기화
rgb_topic: /image/image_raw
depth_topic: /depth/image_raw
camera_info_topic: /image/camera_info
depth_camera_info_topic: /depth/camera_info
```

### 중요 사항
- ❌ **Astra 카메라에서 RGB를 활성화하려는 시도는 실패함**
  - 하드웨어에 COLOR 센서가 없음
  - `enable_color: true` 설정해도 "color is not enable" 메시지 출력
  
- ✅ **별도의 USB 2.0 Camera를 RGB 소스로 사용**
  - device_srv 노드가 `/dev/video0` 또는 `/dev/video1`에서 RGB 이미지 발행
  - Depth와 RGB가 서로 다른 하드웨어에서 제공됨
  
- ⚠️ **프레임 동기화 필요**
  - Astra depth와 USB camera RGB가 시간적으로 동기화되어야 함
  - RTAB-Map의 `approx_sync: true` 사용 권장
  - 또는 `message_filters::TimeSynchronizer` 사용

---

## 수정 사항

### bringup.launch.py
```python
# Astra 카메라 설정
'enable_depth': True,   # Depth 활성화
'enable_ir': False,     # IR 비활성화
'enable_color': False,  # COLOR 미지원
```

### device_srv.py
```python
# USB 2.0 Camera 사용
# /dev/video0 또는 /dev/video1에서 RGB 이미지 발행
for device_id in [0, 1]:
    self.cap = cv2.VideoCapture(device_id)
    if self.cap.isOpened():
        break
```

---

## 참고 문서
- ros2_astra_camera: https://github.com/orbbec/ros2_astra_camera
- Astra 제품 스펙: AS3G65300V5 Depth Sensor
- OpenNI SDK: COLOR 센서 감지 메커니즘
