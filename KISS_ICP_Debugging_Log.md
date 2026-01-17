# TIL: KISS-ICP 라이다 오도메트리 안정화 및 TF 구조 최적화

**날짜**: 2026년 1월 16일  
**작성자**: GitHub Copilot  
**대상 시스템**: Jupiter Robot (Jetson 기반, Velodyne VLP-16, ROS 2 Humble)

---

## 1. 문제 상황 (Trouble)
**증상 1: 오도메트리 정체**  
로봇을 조이스틱으로 주행시켜도 `/kiss/odometry` 토픽의 좌표값(Position)이 거의 변하지 않고 미세한 진동(노이즈)만 출력됨.

**증상 2: TF 에러**  
Rviz 상에서 `Missing transform from frame <odom_lidar> to frame <base_link>` 경고가 지속적으로 발생하며 시각화가 제대로 되지 않음.

**고려 사항**  
- LIO-SAM, FAST-LIO2 등 고성능 LIO 알고리즘으로 교체해야 하는가?
- 현재 IMU(20Hz) 데이터가 충분한가?

---

## 2. 원인 분석 (Analysis)

### A. 오도메트리 매칭 실패의 원인 (`min_range`)
- **현상**: 실내 좁은 공간 테스트 중 라이다 데이터의 상당 부분이 필터링됨.
- **원인**: `velodyne_transform_node`의 `min_range`가 **0.4m**로 설정되어 있었음. 
- **분석**: 실내 환경에서는 가까운 벽이나 장애물이 중요한 특징점(Feature)이 되는데, 이를 모두 잘라내버리니 ICP 알고리즘이 매칭할 단서가 부족하여 트래킹에 실패함.

### B. 알고리즘 교체 불가 (IMU 제약)
- **제약**: 현재 시스템의 IMU 데이터 주기는 **20Hz**.
- **판단**: LIO-SAM, FAST-LIO2 같은 Tightly-Coupled LIO 알고리즘은 **최소 100Hz 이상(권장 200~400Hz)** 의 IMU 데이터를 요구함. (IMU Pre-integration 필수).
- **결론**: 알고리즘 교체 대신, IMU 의존도가 낮은 **KISS-ICP 파라미터 튜닝**이 유일한 해답.

### C. TF 트리 단절 (`odom_lidar` 이슈)
- **현상**: Rviz나 EKF가 `odom` 프레임을 찾는데, KISS-ICP는 독자적인 `odom_lidar` 프레임을 발행.
- **원인**: EKF(`robot_localization`)가 `odom` -> `base_link` 변환을 담당하도록 설계되었으나, 입력 소스의 프레임 이름이 통일되지 않아 TF 트리가 꼬임.

---

## 3. 해결 과정 (Solution)

### Step 1. 라이다 감지 범위 확장
`jupiter_outdoor_gps.launch.py` 수정:
- **변경**: `min_range: 0.4` → **`0.1`**
- **효과**: 라이다 바로 앞의 근거리 장애물을 특징점으로 활용하게 되어 매칭 성공률이 비약적으로 상승. 오도메트리 수치가 정상적으로 변화하기 시작함.

### Step 2. TF 프레임 표준화
`kiss_icp_node` 파라미터 수정:
- **변경**: `lidar_odom_frame: 'odom_lidar'` → **`'odom'`**
- **효과**: 
    1. KISS-ICP가 표준 `odom` 프레임 기준으로 데이터를 발행.
    2. EKF(`robot_localization`)가 이를 받아 자연스럽게 `odom` -> `base_footprint` TF 관계를 형성.
    3. "Missing transform" 에러 해결.

### Step 3. 디버깅 활성화
- **추가**: `'publish_debug_clouds': True`
- **목적**: `/kiss/local_map` 토픽을 활성화하여 오도메트리 정합 상태를 Rviz로 실시간 검증 가능하도록 조치.

---

## 4. 결과 (Result)
1. **정상 주행 확인**: 로봇 이동 시 `odom` 좌표계에서 `base_footprint`가 멀어지는(이동하는) 현상이 정상적으로 관측됨.
2. **시스템 최적화**: 고속 IMU 없이도 VLP-16 라이다만으로 안정적인 실내외 오도메트리 확보.
3. **가시성 확보**: `/kiss/local_map`을 통해 실시간 매핑 상태 확인 가능.

> **교훈**: 복잡한 알고리즘(LIO-SAM) 도입을 고민하기 전에, 기본 센서 파라미터(`min_range` 등) 설정이 환경에 맞게 되어 있는지 먼저 확인하자.
