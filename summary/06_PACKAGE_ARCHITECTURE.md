# 패키지 구조 개선 가이드

> **생성일:** 2025-10-31  
> **통합 문서:** transbot_nav 패키지 생성, 아키텍처 개선

## 📅 작업 타임라인

**작업 일자:** 2025년 10월 31일

### 작업 내용
- **문제 인식**: sllidar_ros2(LiDAR 드라이버) 패키지에 네비게이션 통합 파일들이 혼재
- **해결 방안**: 관심사 분리(Separation of Concerns) 원칙에 따라 전용 패키지 생성
- **결과**: transbot_nav 패키지로 네비게이션/SLAM 기능 분리

---

## 📋 목차
1. [문제 배경](#문제-배경)
2. [패키지 구조 설계](#패키지-구조-설계)
3. [마이그레이션 과정](#마이그레이션-과정)
4. [최종 구조](#최종-구조)
5. [사용 방법](#사용-방법)

---

## 문제 배경

### 1.1 초기 상황

**sllidar_ros2 패키지의 역할 혼재:**

```
sllidar_ros2/  (Slamtec 공식 LiDAR 드라이버)
├── launch/
│   ├── sllidar_a1_launch.py          ✅ LiDAR 드라이버 (적절)
│   ├── transbot_full_system.launch.py ❌ 시스템 통합 (부적절)
│   └── nav2_navigation.launch.py      ❌ 네비게이션 (부적절)
├── config/
│   ├── ekf_config.yaml                ❌ EKF 설정 (부적절)
│   ├── slam_params.yaml               ❌ SLAM 설정 (부적절)
│   └── nav2_params.yaml               ❌ Nav2 설정 (부적절)
```

**문제점:**
1. **역할 혼재**: 하드웨어 드라이버 패키지에 응용 로직 포함
2. **유지보수 어려움**: LiDAR 드라이버 업데이트 시 네비게이션 설정 영향
3. **재사용성 부족**: 다른 LiDAR 센서 사용 시 전체 재구성 필요
4. **아키텍처 위반**: 단일 책임 원칙(Single Responsibility Principle) 위배

### 1.2 설계 원칙

**관심사의 분리 (Separation of Concerns):**

```
[하드웨어 계층]     [응용 계층]
    ↓                 ↓
sllidar_ros2    transbot_nav
(LiDAR 제어)    (네비게이션 통합)
```

**각 패키지의 명확한 역할:**
- `sllidar_ros2`: LiDAR 센서 드라이버 (하드웨어 추상화)
- `transbot_nav`: 네비게이션 및 SLAM 통합 (응용 로직)
- `transbot_base`: 모터 제어 및 오도메트리 (하드웨어)
- `transbot_description`: 로봇 URDF (구조 정의)

---

## 패키지 구조 설계

### 2.1 transbot_nav 패키지 개요

**목적:**
- Transbot의 자율 주행 기능 통합
- SLAM, EKF, Nav2 설정 관리
- 시스템 레벨 launch 파일 제공

**패키지 타입:** ament_python

**주요 의존성:**
```xml
<!-- 네비게이션 스택 -->
<depend>nav2_common</depend>
<depend>nav2_bringup</depend>
<depend>nav2_controller</depend>
<depend>nav2_planner</depend>
<depend>nav2_behaviors</depend>
<depend>nav2_bt_navigator</depend>

<!-- SLAM 및 로컬라이제이션 -->
<depend>slam_toolbox</depend>
<depend>robot_localization</depend>

<!-- 로봇 하드웨어 -->
<depend>transbot_description</depend>
<depend>transbot_base</depend>
<depend>sllidar_ros2</depend>
```

### 2.2 디렉토리 구조

```
transbot_nav/
├── launch/                          # Launch 파일
│   ├── transbot_full_system.launch.py    # 전체 시스템 통합
│   └── nav2_navigation.launch.py         # Nav2 스택
│
├── config/                          # 설정 파일
│   ├── ekf_config.yaml                   # EKF 센서 퓨전
│   ├── slam_params.yaml                  # SLAM Toolbox
│   └── nav2_params.yaml                  # Nav2 파라미터
│
├── rviz/                            # RViz 설정
│   ├── sllidar.rviz
│   └── sllidar_ros2.rviz
│
├── maps/                            # 저장된 맵
│   ├── (맵 파일들 저장 위치)
│   └── .gitkeep
│
├── package.xml                      # 패키지 메타데이터
├── setup.py                         # Python 패키지 설정
├── setup.cfg                        # 설정
├── README.md                        # 패키지 설명서
├── QUICK_REFERENCE.md               # 빠른 참조 가이드
└── MIGRATION_SUMMARY.md             # 마이그레이션 기록
```

---

## 마이그레이션 과정

### 3.1 패키지 생성

```bash
cd ~/transbot_ws_ros2/src
ros2 pkg create transbot_nav --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs nav_msgs sensor_msgs \
  nav2_msgs slam_toolbox
```

### 3.2 디렉토리 구조 생성

```bash
cd transbot_nav
mkdir -p launch config rviz maps
```

### 3.3 파일 복사

**Launch 파일:**
```bash
cp ~/transbot_ws_ros2/src/sllidar_ros2/launch/transbot_full_system.launch.py \
   ~/transbot_ws_ros2/src/transbot_nav/launch/

cp ~/transbot_ws_ros2/src/sllidar_ros2/launch/nav2_navigation.launch.py \
   ~/transbot_ws_ros2/src/transbot_nav/launch/
```

**Config 파일:**
```bash
cp ~/transbot_ws_ros2/src/sllidar_ros2/config/ekf_config.yaml \
   ~/transbot_ws_ros2/src/transbot_nav/config/

cp ~/transbot_ws_ros2/src/sllidar_ros2/config/slam_params.yaml \
   ~/transbot_ws_ros2/src/transbot_nav/config/

cp ~/transbot_ws_ros2/src/sllidar_ros2/config/nav2_params.yaml \
   ~/transbot_ws_ros2/src/transbot_nav/config/
```

**RViz 파일:**
```bash
cp ~/transbot_ws_ros2/src/sllidar_ros2/rviz/*.rviz \
   ~/transbot_ws_ros2/src/transbot_nav/rviz/
```

### 3.4 Launch 파일 경로 수정

**transbot_full_system.launch.py 수정:**

```python
# 변경 전
from ament_index_python.packages import get_package_share_directory
sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
slam_params_file = os.path.join(sllidar_ros2_dir, 'config', 'slam_params.yaml')
ekf_config_file = os.path.join(sllidar_ros2_dir, 'config', 'ekf_config.yaml')

# 변경 후
from ament_index_python.packages import get_package_share_directory
transbot_nav_dir = get_package_share_directory('transbot_nav')
slam_params_file = os.path.join(transbot_nav_dir, 'config', 'slam_params.yaml')
ekf_config_file = os.path.join(transbot_nav_dir, 'config', 'ekf_config.yaml')
```

**nav2_navigation.launch.py 수정:**

```python
# 변경 전
pkg_dir = get_package_share_directory('sllidar_ros2')
nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

# 변경 후
pkg_dir = get_package_share_directory('transbot_nav')
nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
```

### 3.5 setup.py 설정

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'transbot_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일들
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config 파일들
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # RViz 설정 파일들
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Maps 디렉토리
        (os.path.join('share', package_name, 'maps'), 
         glob('maps/*') if os.path.exists('maps') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Transbot User',
    maintainer_email='user@transbot.local',
    description='Transbot Navigation and SLAM Integration Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

### 3.6 package.xml 업데이트

```xml
<?xml version="1.0"?>
<package format="3">
  <name>transbot_nav</name>
  <version>1.0.0</version>
  <description>Transbot Navigation and SLAM integration package</description>
  <maintainer email="user@transbot.local">Transbot User</maintainer>
  <license>Apache-2.0</license>

  <!-- 기본 의존성 -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <!-- Nav2 의존성 -->
  <depend>nav2_msgs</depend>
  <depend>nav2_common</depend>
  <depend>nav2_bringup</depend>
  <depend>nav2_lifecycle_manager</depend>
  <depend>nav2_controller</depend>
  <depend>nav2_planner</depend>
  <depend>nav2_behaviors</depend>
  <depend>nav2_bt_navigator</depend>
  <depend>nav2_velocity_smoother</depend>
  
  <!-- SLAM/Localization 의존성 -->
  <depend>slam_toolbox</depend>
  <depend>robot_localization</depend>
  
  <!-- 로봇 하드웨어 -->
  <depend>transbot_description</depend>
  <depend>transbot_base</depend>
  <depend>sllidar_ros2</depend>
  
  <depend>ament_index_python</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 3.7 빌드 및 검증

```bash
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_nav --symlink-install
source install/setup.bash

# 패키지 확인
ros2 pkg list | grep transbot_nav
ros2 pkg prefix transbot_nav

# 설치된 파일 확인
ls -la ~/transbot_ws_ros2/install/transbot_nav/share/transbot_nav/
```

**빌드 결과:**
```
Starting >>> transbot_nav
Finished <<< transbot_nav [2.79s]          
Summary: 1 package finished [3.69s]
```

---

## 최종 구조

### 4.1 전체 워크스페이스 구조

```
transbot_ws_ros2/
├── src/
│   ├── transbot_nav/              ⭐ NEW: 네비게이션 통합 패키지
│   │   ├── launch/                  # 시스템 통합 launch
│   │   ├── config/                  # Nav2, SLAM, EKF 설정
│   │   ├── rviz/                    # 시각화 설정
│   │   ├── maps/                    # 저장된 맵
│   │   └── README.md
│   │
│   ├── transbot_base/             # 모터 제어, 오도메트리
│   ├── transbot_description/      # URDF, 로봇 모델
│   ├── transbot_bringup/          # 하드웨어 초기화
│   │
│   └── sllidar_ros2/              ✅ 역할 명확화: LiDAR 드라이버만
│       ├── launch/
│       │   └── sllidar_a1_launch.py  # LiDAR 센서만
│       └── src/                      # 드라이버 코드
│
├── install/
├── build/
└── log/
```

### 4.2 패키지 간 의존성

```
transbot_nav
  ├── depends on → nav2_*
  ├── depends on → slam_toolbox
  ├── depends on → robot_localization
  ├── depends on → transbot_description
  ├── depends on → transbot_base
  └── depends on → sllidar_ros2
```

### 4.3 실행 흐름

**전체 시스템 실행:**
```bash
ros2 launch transbot_nav transbot_full_system.launch.py
```

**실행되는 노드들:**
1. `robot_state_publisher` (transbot_description)
2. `Transbot_Driver` (transbot_base) - 모터 제어
3. `imu_calib_node` (transbot_base) - IMU 보정
4. `ekf_filter_node` (robot_localization) - 센서 퓨전
5. `sllidar_node` (sllidar_ros2) - LiDAR 스캔
6. `async_slam_toolbox_node` (slam_toolbox) - 맵 생성

**Nav2 자율 주행 실행:**
```bash
ros2 launch transbot_nav nav2_navigation.launch.py
```

---

## 사용 방법

### 5.1 기본 사용법

**1단계: 전체 시스템 실행**
```bash
# 터미널 1
ros2 launch transbot_nav transbot_full_system.launch.py
```

**2단계: Nav2 자율 주행 활성화**
```bash
# 터미널 2
ros2 launch transbot_nav nav2_navigation.launch.py
```

**3단계: RViz 시각화**
```bash
# 터미널 3
rviz2 -d $(ros2 pkg prefix transbot_nav)/share/transbot_nav/rviz/sllidar_ros2.rviz
```

### 5.2 맵 저장

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/transbot_ws_ros2/src/transbot_nav/maps/my_map
```

### 5.3 맵 로드 (Localization 모드)

**slam_params.yaml 수정:**
```yaml
slam_toolbox:
  ros__parameters:
    mode: localization  # mapping → localization
```

**Map Server 실행:**
```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=~/transbot_ws_ros2/src/transbot_nav/maps/my_map.yaml
```

### 5.4 설정 파일 수정

**EKF 튜닝 (config/ekf_config.yaml):**
```yaml
# IMU 신뢰도 조정
imu0_angular_velocity_covariance: 0.000025  # 낮을수록 높은 신뢰도
```

**SLAM 파라미터 (config/slam_params.yaml):**
```yaml
# Loop closure 엄격도
loop_match_minimum_response_fine: 0.65  # 0.50~0.80
```

**Nav2 속도 제한 (config/nav2_params.yaml):**
```yaml
# 속도 제한
max_vel_x: 0.3        # 전진 속도
max_vel_theta: 0.5    # 회전 속도
```

---

## 장점 및 개선 효과

### 6.1 아키텍처 개선

✅ **명확한 역할 분리:**
- 하드웨어 계층 (드라이버)
- 응용 계층 (네비게이션)

✅ **유지보수성 향상:**
- 각 패키지 독립적 업데이트 가능
- 설정 파일 위치 명확

✅ **재사용성 증가:**
- 다른 LiDAR 센서로 교체 용이
- 다른 로봇에 transbot_nav 재사용 가능

✅ **확장성 향상:**
- 새로운 센서 추가 용이
- 새로운 기능 모듈화 가능

### 6.2 개발 효율성

**Before (혼재):**
```bash
# sllidar_ros2에서 모든 것 관리
# - LiDAR 드라이버 업데이트 시 네비게이션 영향
# - 설정 파일 위치 혼란
# - 의존성 불명확
```

**After (분리):**
```bash
# 명확한 패키지 구조
transbot_nav → 네비게이션 설정 및 통합
sllidar_ros2 → LiDAR 드라이버만
transbot_base → 모터 및 오도메트리
```

### 6.3 문서화 개선

**생성된 문서:**
1. `README.md`: 전체 패키지 설명 및 사용법
2. `QUICK_REFERENCE.md`: 자주 쓰는 명령어 모음
3. `MIGRATION_SUMMARY.md`: 마이그레이션 과정 기록

---

## 향후 계획

### 7.1 단기 계획

1. **원본 파일 정리**
   - sllidar_ros2의 네비게이션 파일들 제거 또는 deprecated 표시
   - 중복 제거

2. **문서 업데이트**
   - 모든 가이드 문서의 launch 명령어 업데이트
   - 패키지 구조 다이어그램 추가

3. **테스트 강화**
   - Unit test 추가
   - Integration test 구성

### 7.2 중기 계획

1. **기능 추가**
   - Waypoint navigation
   - Patrol mode
   - Auto-docking

2. **성능 최적화**
   - 파라미터 자동 튜닝
   - 적응형 속도 제어

3. **모니터링 도구**
   - 실시간 성능 대시보드
   - 로그 분석 도구

---

## 참고 자료

### 내부 문서
- `README.md`: 패키지 상세 설명
- `QUICK_REFERENCE.md`: 빠른 참조 가이드
- `MIGRATION_SUMMARY.md`: 마이그레이션 과정

### 외부 문서
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](http://docs.ros.org/en/humble/p/robot_localization/)
- [ROS2 Design Patterns](https://design.ros2.org/)

---

## 결론

`transbot_nav` 패키지 생성을 통해:

✅ **명확한 아키텍처**: 하드웨어와 응용 계층 분리  
✅ **유지보수성 향상**: 독립적 업데이트 가능  
✅ **확장성 증가**: 모듈화된 구조  
✅ **문서화 완비**: 3개의 참조 문서 제공

**패키지 생성일:** 2025-10-31  
**빌드 시간:** 3.69초  
**상태:** ✅ 성공적으로 배포 완료
