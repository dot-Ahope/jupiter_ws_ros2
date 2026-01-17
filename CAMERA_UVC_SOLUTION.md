# Astra Pro 카메라 RGB+Depth+IR 동시 사용 가이드

## 핵심 발견사항
Astra Pro는 **RGB를 별도의 UVC (USB Video Class) 카메라**로 사용합니다!
- 2개의 USB 장치:
  - `2bc5:060f` - Depth + IR 센서
  - `2bc5:050f` - RGB 카메라 (UVC)

## 해결 방법
`use_uvc_camera: True` 파라미터를 사용하여 RGB를 UVC 장치로 활성화하면
IR과 동시에 사용 가능합니다!

## 업데이트된 파라미터
```python
'enable_color': True,
'use_uvc_camera': True,         # ✅ 핵심 설정!
'uvc_vendor_id': '0x2bc5',      # Orbbec
'uvc_product_id': '0x0501',     # RGB 카메라
'uvc_camera_format': 'mjpeg',

'enable_depth': True,
'enable_ir': True,              # ✅ 이제 RGB와 동시 사용 가능!
```

## 테스트 방법

### 1. 현재 실행 중인 카메라 노드 종료
```bash
pkill -9 -f astra_camera_node
rm -f /tmp/XnCore.Mutex.*
```

### 2. 새로운 설정으로 카메라 시작
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav test_camera.launch.py
```

또는 자동 스크립트 사용:
```bash
~/transbot_ws_ros2/restart_camera_with_uvc.sh
```

### 3. 모든 센서 확인
```bash
# 토픽 확인
ros2 topic list | grep camera

# 예상 출력:
# /camera/color/image_raw          ← RGB (UVC)
# /camera/depth/image_raw          ← Depth
# /camera/ir/image_raw             ← IR
# /camera/depth/color/points       ← RGB+Depth 융합

# Hz 확인
ros2 topic hz /camera/color/image_raw &
ros2 topic hz /camera/depth/image_raw &
ros2 topic hz /camera/ir/image_raw &
```

### 4. RTAB-Map RGB-D SLAM 실행
```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav transbot_rtabmap.launch.py use_rviz:=true
```

## 주의사항
- RGB는 UVC 장치(2bc5:050f)로 독립 동작
- Depth와 IR은 같은 센서(2bc5:060f)를 공유
- 모든 센서 30Hz로 동작 (15Hz 미지원)

## 파일 수정 완료
- ✅ `test_camera.launch.py` - UVC 파라미터 추가
- ✅ `transbot_rtabmap.launch.py` - UVC 파라미터 추가
- ✅ 빌드 완료

