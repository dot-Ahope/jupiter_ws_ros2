# 오도메트리 최소화 SLAM 테스트 설정

## 개요
라이다 센서의 포인트가 SLAM 맵을 벗어나는 문제를 해결하기 위해, 오도메트리 의존성을 최소화하고 스캔 매칭에 더 의존하는 테스트 설정을 만들었습니다.

## 생성된 파일

### 1. Launch 파일
**파일**: `src/sllidar_ros2/launch/transbot_minimal_odom_test.launch.py`

**제거된 노드들**:
- `robot_localization_node` (EKF 노드) - 오도메트리와 IMU 융합 제거
- `imu_calib_node` - IMU 보정 노드 제거
- `imu_filter_node` - IMU 필터링 노드 제거
- `transbot_base_node` - 휠 오도메트리 계산 노드 제거

**유지된 노드들**:
- `robot_state_publisher` - TF 트리 구성용 (필수)
- `joint_state_publisher` - 조인트 상태 발행 (필수)
- `transbot_driver_node` - 기본 모터 제어용
- `sllidar_node` - 라이다 데이터 수집 (핵심)
- `slam_toolbox_node` - 스캔 매칭 기반 SLAM

### 2. SLAM 파라미터 파일
**파일**: `src/sllidar_ros2/config/slam_minimal_odom.yaml`

**주요 변경사항**:
- `minimum_travel_distance: 0.02` (기존: 0.05) - 스캔 매칭 의존도 증가
- `minimum_travel_heading: 0.02` (기존: 0.05) - 회전 감지 민감도 증가
- `scan_buffer_size: 20` (기존: 10) - 더 많은 스캔 데이터 활용
- `link_match_minimum_response_fine: 0.10` (기존: 0.15) - 매칭 조건 완화
- `distance_variance_penalty: 0.3` (기존: 0.6) - 오도메트리 페널티 감소
- `angle_variance_penalty: 0.5` (기존: 1.0) - 오도메트리 페널티 감소
- `correlation_search_space_dimension: 0.8` (기존: 0.6) - 검색 공간 확장
- `loop_search_space_dimension: 10.0` (기존: 9.0) - 루프 클로저 검색 확장

### 3. 실행 스크립트
**파일**: `run_minimal_odom_test.sh`

간단한 실행 명령:
```bash
./run_minimal_odom_test.sh
```

또는 직접 실행:
```bash
cd /home/user/transbot_ws_ros2
source install/setup.bash
ros2 launch sllidar_ros2 transbot_minimal_odom_test.launch.py use_rviz:=true
```

## 설정 철학

### 오도메트리 의존성 최소화
1. **휠 오도메트리 제거**: `transbot_base_node` 제거로 휠 엔코더 기반 위치 추정 비활성화
2. **IMU 융합 제거**: EKF 노드 제거로 IMU 데이터 융합 비활성화
3. **센서 융합 제거**: robot_localization 제거로 다중 센서 융합 비활성화

### 스캔 매칭 강화
1. **민감도 증가**: 최소 이동/회전 임계값 감소로 미세한 변화도 감지
2. **버퍼 확장**: 더 많은 스캔 데이터를 버퍼에 저장하여 매칭 정확도 향상
3. **검색 공간 확장**: 스캔 매칭 시 더 넓은 영역 탐색
4. **페널티 완화**: 오도메트리 불일치에 대한 페널티 감소

## 예상 효과

### 긍정적 효과
- 오도메트리 오차에 의한 맵 왜곡 감소
- 회전 시 라이다 포인트의 맵 이탈 문제 완화
- 스캔 매칭만으로 안정적인 위치 추정 가능

### 잠재적 문제
- 특징이 없는 환경(빈 복도 등)에서 성능 저하 가능
- 빠른 움직임 시 스캔 매칭 실패 가능
- 루프 클로저 성능 저하 가능 (오도메트리 힌트 없음)

## 테스트 방법

### 1. 기본 테스트
```bash
./run_minimal_odom_test.sh
```

### 2. 회전 테스트
- RViz에서 맵을 관찰하며 로봇을 제자리에서 회전
- 라이다 포인트가 맵에서 이탈하는지 확인
- 맵의 일관성 확인

### 3. 이동 테스트
- 직선 이동 후 회전
- 복잡한 경로 주행
- 같은 위치로 복귀 (루프 클로저 테스트)

## 원본 설정과 비교

### 원본 (transbot_full_system.launch.py)
- 노드 수: 9개 (driver, base, imu_calib, imu_filter, ekf, lidar, robot_state_pub, joint_state_pub, slam)
- 오도메트리 소스: 휠 엔코더 + IMU (EKF 융합)
- 복잡도: 높음

### 테스트 설정 (transbot_minimal_odom_test.launch.py)
- 노드 수: 5개 (driver, lidar, robot_state_pub, joint_state_pub, slam)
- 오도메트리 소스: 없음 (스캔 매칭만 사용)
- 복잡도: 낮음

## 다음 단계

테스트 결과에 따라:

1. **개선된 경우**: 이 설정을 기반으로 최적화
2. **악화된 경우**: 오도메트리 보정 문제 해결 필요
3. **비슷한 경우**: 라이다 캘리브레이션 또는 TF 설정 검토 필요

## 문제 해결

### SLAM이 시작되지 않는 경우
- TF 트리 확인: `ros2 run tf2_tools view_frames`
- base_footprint 프레임 존재 확인

### 스캔 매칭 실패 메시지
- 환경에 충분한 특징(벽, 장애물 등)이 있는지 확인
- `link_match_minimum_response_fine` 값을 더 낮춤 (예: 0.05)

### 맵이 불안정한 경우
- `scan_buffer_size` 증가
- `correlation_search_space_dimension` 증가
- 로봇 이동 속도 감소

## 참고사항

이 설정은 **테스트 목적**으로 만들어졌습니다. 프로덕션 환경에서는 적절한 오도메트리와 센서 융합을 사용하는 것이 권장됩니다.
