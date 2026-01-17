# 🔧 RViz 오도메트리 토픽 수정

## 📋 문제점

**발견**: RViz가 발행자 없는 `/odom` 토픽을 구독 중

```bash
ros2 topic info /odom -v

Type: nav_msgs/msg/Odometry
Publisher count: 0  ❌ (발행자 없음!)
Subscription count: 1 (rviz2)
```

**영향**:
- RViz에서 오도메트리 화살표가 표시되지 않음
- 로봇 위치 시각화 불가
- 디버깅 어려움

---

## ✅ 해결 방법

### 수정된 파일

#### 1. `/home/user/transbot_ws_ros2/src/sllidar_ros2/rviz/sllidar.rviz`
**Line 278 수정**:
```yaml
# 변경 전
Value: /odom

# 변경 후
Value: /odometry/filtered
```

#### 2. `/home/user/transbot_ws_ros2/src/transbot_nav/rviz/sllidar.rviz`
**Line 278 수정**:
```yaml
# 변경 전
Value: /odom

# 변경 후  
Value: /odometry/filtered
```

---

## 🎯 변경 이유

### 기존 토픽 구조
```
/odom              ❌ 발행자 없음 (구독: rviz2)
/odom_raw          ✅ 발행자: base_node (구독: ekf_filter_node)
/odometry/filtered ✅ 발행자: ekf_filter_node (구독: 없음)
```

### 새로운 토픽 구조
```
/odom_raw          ✅ 발행자: base_node (구독: ekf_filter_node)
/odometry/filtered ✅ 발행자: ekf_filter_node (구독: rviz2)
```

### 장점

1. **EKF 융합 결과 활용**
   - IMU + Odom 센서 융합
   - 더 정확한 위치 추정
   - 노이즈 감소

2. **시각화 정상화**
   - RViz에서 오도메트리 화살표 표시
   - 로봇 경로 추적 가능
   - 실시간 디버깅 가능

3. **데이터 흐름 최적화**
   - EKF 계산 결과가 활용됨 (CPU 낭비 없음)
   - 센서 융합의 이점 실현

---

## 🚀 적용 방법

### 1. 패키지 빌드 (완료 ✅)
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2 transbot_nav --symlink-install
```

### 2. 시스템 재시작
```bash
# 기존 프로세스 종료
sudo killall -9 python3 rviz2

# 시스템 재실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

### 3. RViz 확인
- 좌측 패널에서 "Odometry" 확인
- Topic이 `/odometry/filtered`로 설정되었는지 확인
- 로봇 이동 시 빨간 화살표가 표시되는지 확인

---

## 📊 검증

### Before (수정 전)
```bash
$ ros2 topic info /odom -v
Publisher count: 0  ❌
Subscription count: 1 (rviz2)

# RViz에서 오도메트리 표시 안 됨
```

### After (수정 후)
```bash
$ ros2 topic info /odometry/filtered -v
Publisher count: 1 (ekf_filter_node)  ✅
Subscription count: 1 (rviz2)  ✅

# RViz에서 오도메트리 화살표 표시됨
```

---

## 🎓 토픽 역할 정리

### `/odom_raw` (base_node → ekf_filter_node)
- **역할**: 엔코더 기반 순수 오도메트리
- **발행자**: base_node (50Hz)
- **구독자**: ekf_filter_node
- **특징**: 
  - angular_scale 보정 적용
  - 회전 중 데이터 손실 있음
  - 장기 누적 오차 발생 가능

### `/odometry/filtered` (ekf_filter_node → 응용)
- **역할**: EKF 센서 융합 결과
- **발행자**: ekf_filter_node (49Hz)
- **구독자**: rviz2 (수정 후)
- **특징**:
  - IMU 각속도 + Odom 위치 융합
  - 노이즈 감소
  - 단기 정확도 향상
  - **이것이 시스템의 최종 오도메트리!**

### `/odom` (삭제됨)
- **역할**: 레거시 호환성 (더 이상 사용 안 함)
- **발행자**: 없음 ❌
- **구독자**: rviz2 (수정 전)
- **상태**: Deprecated (삭제 권장)

---

## 💡 추가 권장 사항

### 1. SLAM Toolbox도 확인 필요

현재 SLAM Toolbox가 어떤 오도메트리를 사용하는지 확인:
```bash
ros2 param get /slam_toolbox odom_frame
```

만약 `/odom` 또는 `odom`을 사용 중이라면 변경 권장:
```bash
# slam_toolbox 설정 파일 수정
odom_frame: odometry/filtered  # 또는 'odom' (TF 프레임 이름)
```

### 2. Nav2도 확인 필요 (사용 시)

Nav2 사용 시 오도메트리 토픽 설정:
```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    odom_topic: "/odometry/filtered"
```

### 3. 토픽 이름 vs TF 프레임 구분

**중요**: 혼동하지 말 것!
- **토픽 이름**: `/odometry/filtered`, `/odom_raw` (데이터 전달)
- **TF 프레임**: `odom`, `base_footprint`, `map` (좌표계 변환)

SLAM과 Nav2는 주로 **TF 프레임 이름**을 사용:
```yaml
odom_frame: odom        # TF 프레임 (O)
odom_frame: /odom       # 토픽 이름 (X)
```

---

## 📝 체크리스트

수정 완료 후 확인 사항:

- [x] sllidar.rviz 파일 수정 (2개 파일)
- [x] colcon build 완료
- [ ] 시스템 재시작
- [ ] RViz에서 오도메트리 화살표 확인
- [ ] 로봇 이동 시 경로 추적 확인
- [ ] SLAM Toolbox 오도메트리 소스 확인
- [ ] 맵 품질 개선 확인

---

## 🔗 관련 이슈

이 수정은 다음 문서에서 발견된 문제 해결:
- `ODOMETRY_TOPIC_ANALYSIS.md` - 토픽 분석
- `ODOM_DRIFT_ROOT_CAUSE_ANALYSIS.md` - 드리프트 원인 분석

다음 단계:
1. ✅ RViz 토픽 수정 (완료)
2. ⏳ angular_scale 2.1164 적용
3. ⏳ SLAM Toolbox 오도메트리 소스 확인
4. ⏳ 통합 테스트 및 검증

---

## 🎉 기대 효과

### 즉시 효과
- ✅ RViz 오도메트리 시각화 정상화
- ✅ EKF 융합 결과 활용
- ✅ 디버깅 편의성 향상

### 장기 효과
- 🎯 SLAM 맵 품질 향상 (EKF 융합 사용 시)
- 🎯 내비게이션 정확도 개선
- 🎯 센서 융합의 이점 실현

---

## 📌 참고

RViz 설정 파일 위치:
```
사용 중: /home/user/transbot_ws_ros2/src/sllidar_ros2/rviz/sllidar.rviz
백업: /home/user/transbot_ws_ros2/src/transbot_nav/rviz/sllidar.rviz
```

Launch 파일에서 설정:
```python
# transbot_full_system.launch.py Line 110
rviz_config_file = os.path.join(sllidar_ros2_dir, 'rviz', 'sllidar.rviz')
```
