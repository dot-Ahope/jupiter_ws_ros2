# RTAB-Map 통합 가이드

## 📋 개요

RTAB-Map (Real-Time Appearance-Based Mapping)을 Transbot 시스템에 통합했습니다.  
기존 SLAM Toolbox 대신 사용할 수 있는 대안 SLAM 솔루션입니다.

## 🎯 RTAB-Map vs SLAM Toolbox

### RTAB-Map 장점
- ✅ **루프 클로저**: 그래프 최적화로 누적 오차 보정
- ✅ **메모리 관리**: 큰 환경에서도 일정한 메모리 사용
- ✅ **3D 지원**: RGB-D 카메라 추가 시 3D 매핑 가능
- ✅ **Localization 모드**: 기존 맵에서 위치 추정

### SLAM Toolbox 장점
- ✅ **Nav2 통합**: Nav2와 긴밀한 통합
- ✅ **간단함**: 설정과 사용이 단순
- ✅ **가벼움**: 리소스 사용량 적음

## 📁 생성된 파일

```
transbot_nav/
├── config/
│   └── rtabmap_params.yaml       # RTAB-Map 설정
└── launch/
    └── transbot_rtabmap.launch.py # RTAB-Map 런치 파일
```

## 🚀 사용법

### 1. 새 맵 생성 (Mapping Mode)

```bash
# 빌드
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_nav
source install/setup.bash

# 실행
ros2 launch transbot_nav transbot_rtabmap.launch.py
```

### 2. 기존 맵 사용 (Localization Mode)

```bash
# 저장된 맵으로 위치 추정
ros2 launch transbot_nav transbot_rtabmap.launch.py \
  localization:=true \
  database_path:=/path/to/map.db
```

### 3. 맵 저장

RTAB-Map은 자동으로 메모리에 맵을 유지합니다.  
종료 시 저장하려면:

```bash
# database_path 지정하여 실행
ros2 launch transbot_nav transbot_rtabmap.launch.py \
  database_path:=~/maps/my_map.db
```

또는 런타임 중 서비스 호출:

```bash
# 맵 저장
ros2 service call /rtabmap/save_map rtabmap_ros/srv/SaveMap "{filename: '/home/user/maps/my_map.db'}"
```

## 🔧 주요 설정

### rtabmap_params.yaml 핵심 파라미터

```yaml
# 2D SLAM 모드 (LiDAR 전용)
Reg/Strategy: "1"              # ICP (LiDAR 매칭)
Reg/Force3DoF: "true"          # 2D 평면만

# 오도메트리
odom_topic: /odometry/filtered # EKF 융합 오도메트리
scan_topic: /scan              # LiDAR 스캔

# ICP 설정
Icp/MaxTranslation: "0.2"      # 최대 이동 20cm
Icp/MaxRotation: "0.78"        # 최대 회전 45도
Icp/VoxelSize: "0.05"          # 5cm 복셀

# 루프 클로저
Rtabmap/DetectionRate: "1.0"   # 1Hz 검사
Mem/STMSize: "30"              # 단기 메모리 30개 노드

# 그리드 맵
Grid/CellSize: "0.05"          # 5cm 해상도
Grid/RangeMax: "5.0"           # 최대 거리 5m
```

### Launch Arguments

| Argument | Default | 설명 |
|----------|---------|------|
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 |
| `localization` | `false` | Localization 모드 (기존 맵 사용) |
| `database_path` | `""` | DB 파일 경로 (빈 문자열 = 메모리만) |

## 📊 Topic 구조

### 입력 Topics
- `/scan` (sensor_msgs/LaserScan) - LiDAR 스캔
- `/odometry/filtered` (nav_msgs/Odometry) - EKF 융합 오도메트리
- `/imu/data_calibrated` (sensor_msgs/Imu) - 캘리브레이션된 IMU

### 출력 Topics
- `/map` (nav_msgs/OccupancyGrid) - 2D 점유 그리드 맵
- `/rtabmap/grid_map` (nav_msgs/OccupancyGrid) - RTAB-Map 그리드
- `/rtabmap/mapData` (rtabmap_ros/MapData) - 전체 맵 데이터
- `/rtabmap/info` (rtabmap_ros/Info) - SLAM 상태 정보

### TF Frames
- `map` -> `odom` -> `base_footprint` -> `base_link` -> `laser`/`imu_link`

## 🎮 RViz 시각화

### 기본 시각화 (rtabmap_viz)

Launch 파일에 `rtabmap_viz` 노드가 포함되어 있습니다.  
자동으로 RTAB-Map 전용 시각화 창이 열립니다.

### RViz 수동 실행

```bash
rviz2
```

추가할 Display:
1. **Map** (`/map` topic)
2. **LaserScan** (`/scan` topic)
3. **Odometry** (`/odometry/filtered` topic)
4. **TF** (모든 프레임)
5. **MapGraph** (`/rtabmap/mapGraph` topic) - RTAB-Map 그래프
6. **PointCloud2** (`/rtabmap/cloud_map` topic) - 포인트 클라우드

## 🔍 모니터링 & 디버깅

### SLAM 상태 확인

```bash
# RTAB-Map 정보
ros2 topic echo /rtabmap/info

# 루프 클로저 수
ros2 topic echo /rtabmap/info | grep "loop_closures:"

# 맵 크기 (노드 수)
ros2 topic echo /rtabmap/info | grep "nodes:"
```

### Topic 주파수 확인

```bash
# 입력 Topics
ros2 topic hz /scan                 # ~10Hz (RPLidar A1)
ros2 topic hz /odometry/filtered    # ~50Hz (EKF)
ros2 topic hz /imu/data_calibrated  # ~98Hz (IMU)

# 출력 Topics
ros2 topic hz /map                  # ~1Hz (그리드 맵)
ros2 topic hz /rtabmap/info         # ~1Hz (상태 정보)
```

### TF Tree 확인

```bash
ros2 run tf2_tools view_frames
# frames_YYYY-MM-DD_HH.MM.SS.gv 생성
evince frames_*.pdf
```

## ⚙️ 튜닝 가이드

### 맵 품질 개선

**문제: 벽이 두껍게 보임**
```yaml
# rtabmap_params.yaml
Icp/VoxelSize: "0.03"          # 5cm -> 3cm (더 정밀)
Grid/CellSize: "0.03"          # 5cm -> 3cm
```

**문제: 각도 드리프트**
```yaml
# transbot_rtabmap.launch.py (line 96)
'angular_scale': 2.1164,       # 1.8819 -> 2.1164 (ekf_test 결과)
```

**문제: 루프 클로저 실패**
```yaml
# rtabmap_params.yaml
Icp/CorrespondenceRatio: "0.2"  # 0.3 -> 0.2 (더 엄격)
Mem/STMSize: "50"               # 30 -> 50 (더 많은 메모리)
Rtabmap/DetectionRate: "2.0"    # 1Hz -> 2Hz (더 자주 검사)
```

### 성능 최적화

**리소스 부족 (Jetson Nano)**
```yaml
# rtabmap_params.yaml
Icp/VoxelSize: "0.1"           # 복셀 크기 증가 (연산량 감소)
Icp/Iterations: "15"           # 30 -> 15 (반복 감소)
Rtabmap/DetectionRate: "0.5"   # 1Hz -> 0.5Hz (느리게)
```

## 🔄 SLAM Toolbox와 비교

### 전환 방법

**SLAM Toolbox 사용 (기존)**
```bash
ros2 launch transbot_nav transbot_full_system.launch.py
```

**RTAB-Map 사용 (새로 추가)**
```bash
ros2 launch transbot_nav transbot_rtabmap.launch.py
```

### 맵 형식 차이

| | SLAM Toolbox | RTAB-Map |
|---|---|---|
| 파일 형식 | `.yaml` + `.pgm` | `.db` (SQLite) |
| 저장 명령 | `save_map` 서비스 | `save_map` 서비스 |
| 3D 지원 | ❌ | ✅ (RGB-D 카메라 필요) |
| 메모리 사용 | 작음 | 중간 |
| Nav2 통합 | 우수 | 보통 |

## 📈 성능 비교 (예상)

### SLAM Toolbox
- CPU: ~15-20% (Jetson Nano)
- 메모리: ~200MB
- 맵 정확도: ⭐⭐⭐
- 루프 클로저: ⭐⭐

### RTAB-Map
- CPU: ~25-30% (Jetson Nano)
- 메모리: ~300-400MB
- 맵 정확도: ⭐⭐⭐⭐
- 루프 클로저: ⭐⭐⭐⭐⭐

## 🚨 알려진 문제

### 1. angular_scale 보정 필요
**증상**: 회전 시 각도가 12.5% 작게 측정됨

**해결**: `transbot_rtabmap.launch.py` 96번째 줄
```python
'angular_scale': 2.1164,  # ⭐ ekf_comparison_test 결과 적용
```

### 2. 메모리 부족 (큰 환경)
**증상**: Jetson Nano에서 OOM (Out of Memory)

**해결**: 메모리 관리 파라미터 조정
```yaml
Mem/ImageKept: "false"         # 이미지 저장 안 함
Mem/STMSize: "20"              # 단기 메모리 감소
Rtabmap/TimeThr: "700"         # 시간 제한 설정
```

### 3. 초기 위치 점프
**증상**: 시작 직후 위치가 튐

**원인**: EKF 수렴 전에 RTAB-Map 시작

**해결**: EKF 안정화 후 RTAB-Map 시작 (delay 추가)

## 📚 참고 자료

- [RTAB-Map 공식 문서](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map GitHub](https://github.com/introlab/rtabmap_ros)
- [RTAB-Map 튜닝 가이드](https://github.com/introlab/rtabmap/wiki)

## 🎯 다음 단계

1. **테스트**: 실제 환경에서 매핑 테스트
2. **튜닝**: ICP 파라미터 최적화
3. **비교**: SLAM Toolbox vs RTAB-Map 성능 비교
4. **Nav2 통합**: RTAB-Map 맵으로 Nav2 navigation 테스트
5. **RGB-D 추가**: 카메라 추가 시 3D 매핑 활성화

## ✅ 빠른 시작 체크리스트

- [ ] 빌드: `colcon build --packages-select transbot_nav`
- [ ] Source: `source install/setup.bash`
- [ ] 실행: `ros2 launch transbot_nav transbot_rtabmap.launch.py`
- [ ] 로봇 이동: Teleop으로 맵 생성
- [ ] 맵 확인: RViz 또는 rtabmap_viz
- [ ] 맵 저장: `database_path` 인자 사용
- [ ] Localization: `localization:=true`로 재실행

---

**작성일**: 2025-01-XX  
**버전**: 1.0  
**작성자**: GitHub Copilot
