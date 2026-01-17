# Astra 카메라 문제 해결 가이드

## 🔍 문제: RTAB-Map이 카메라 데이터를 받지 못함

```
[WARN] rtabmap: Did not receive data since 5 seconds!
rtabmap subscribed to (exact sync):
   /camera/color/image_raw
   /camera/depth/image_raw
   /camera/color/camera_info
   /scan
```

## ✅ 해결 방법

### 1단계: USB 권한 설정 (필수!)

```bash
cd ~/transbot_ws_ros2
./setup_camera_permissions.sh
```

그 다음 **로그아웃 후 다시 로그인** 또는 **카메라 USB 재연결**

### 2단계: 카메라 단독 테스트

```bash
cd ~/transbot_ws_ros2
./test_camera.sh
```

**다른 터미널에서 토픽 확인:**
```bash
# 토픽 목록
ros2 topic list | grep camera

# 토픽 Hz 확인
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# 예상 출력: 약 15 Hz
```

### 3단계: RTAB-Map 실행

카메라가 정상 작동하면 RTAB-Map 실행:

```bash
cd ~/transbot_ws_ros2
source install/setup.bash
ros2 launch transbot_nav transbot_rtabmap.launch.py use_rgbd:=true use_rviz:=true
```

## 🔧 주요 수정 사항

### 1. `approx_sync` 활성화
**파일**: `config/rtabmap_params.yaml`
```yaml
approx_sync: true  # 타임스탬프 근사 동기화 (필수!)
queue_size: 30
sync_queue_size: 30
```

### 2. 카메라 FPS 조정
**파일**: `launch/transbot_rtabmap.launch.py`
```python
'color_fps': 15,  # 30→15 (안정성)
'depth_fps': 15,  # 30→15 (안정성)
```

### 3. USB 권한 규칙
**파일**: `/etc/udev/rules.d/99-astra-camera.rules`
```
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
```

## 🐛 추가 문제 해결

### 카메라가 감지되지 않음

```bash
# USB 디바이스 확인
lsusb | grep -i orbbec

# 예상 출력:
# Bus 001 Device 009: ID 2bc5:050f Orbbec 3D Technology
# Bus 001 Device 007: ID 2bc5:060f Orbbec 3D Technology
```

없으면:
1. USB 케이블 재연결
2. 다른 USB 포트 시도
3. USB 허브 사용 시 → 직접 연결

### 토픽은 있지만 데이터 없음

```bash
# 1. 카메라 노드 상태 확인
ros2 node list | grep camera

# 2. 카메라 노드 로그 확인
ros2 node info /camera/camera

# 3. 카메라 재시작
# Ctrl+C로 종료 후 다시 실행
```

### "exact sync" 경고

이미 `approx_sync: true`로 수정됨. 만약 계속 나타나면:

```bash
# 빌드 확인
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_nav
source install/setup.bash
```

### EKF 업데이트 경고

```
Failed to meet update rate! Took 0.020s
```

**정상입니다!** EKF는 50Hz로 설정되어 있고, 가끔 20ms (50Hz) 이상 걸릴 수 있습니다.

## 📊 정상 작동 확인

### 카메라 토픽 (15 Hz)
```bash
ros2 topic hz /camera/color/image_raw
# average rate: 14.8
```

### RTAB-Map 정보
```bash
ros2 topic echo /rtabmap/info
# nodes: 증가하는 숫자
# loop_closures: 루프 감지 시 증가
```

### RViz에서 확인
- ✅ **MapCloud**: 포인트 클라우드 표시
- ✅ **MapGraph**: 노드와 연결선 표시
- ✅ **Map**: 2D 점유 그리드
- ✅ **LaserScan**: LiDAR 스캔

## 🎯 최종 체크리스트

- [ ] USB 권한 설정 완료
- [ ] 로그아웃/로그인 또는 USB 재연결
- [ ] 카메라 단독 테스트 성공 (15 Hz)
- [ ] RTAB-Map 실행 시 경고 없음
- [ ] RViz에서 포인트 클라우드 확인

---

**문제 지속 시:**
1. `~/transbot_ws_ros2/test_camera.sh` 로그 확인
2. `dmesg | tail -50` (커널 메시지)
3. 시스템 재부팅
