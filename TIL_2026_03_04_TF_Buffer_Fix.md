# TIL 2026-03-04: Nav2 TF Buffer Eviction 근본 수정 + RF2O Respawn

## 1. 문제 현상 (오늘 검증 결과)

### 1.1 NTP 동기화 확인
- PC: `w32tm /query /status` → Stratum 4, time.windows.com 동기화
- Jetson: chrony → Stratum 3, 1ms 이내 정확도
- **결과**: stamp_offset이 +0.04s ~ -0.70s로 크게 개선됨 (이전 ~2.4s)

### 1.2 Nav2 실주행 결과
- **Goal 1** (0.27m, 단거리): 성공 ✅ — 6초 소요
- **Goal 2** (-1.79m, 중거리): **실패** ❌ — 13초 후 planner TF 조회 실패
  - Controller는 10+초간 정상 동작
  - Planner 주기적 replan 시 goal stamp=309.957가 TF buffer에서 evicted
  - Recovery cascade → backup 발생
- **Goal 3, 4**: 동일 패턴으로 실패

### 1.3 근본 원인: TF Buffer Eviction
```
[planner_server] Extrapolation Error: Requested time 1772587309.957117
                 but earliest data is at 1772587313.193746
                 when looking up transform from [odom] to [map]
```
- Nav2 TF buffer 기본값: **10초**
- Goal stamp이 "현재 시간"으로 찍혀도, 13초+ 후 replan 시점에는 이미 evicted
- **NTP 동기화와 무관한 구조적 문제**

## 2. 해결: stamp=Time(0,0)

### 2.1 원리
- ROS2 TF2에서 `Time(sec=0, nanosec=0)` = **"latest available transform"**
- TF buffer가 항상 최신 데이터로 조회 → eviction 불가
- Nav2 recovery가 아무리 오래 걸려도 TF 조회 성공

### 2.2 이전 방식 vs 새 방식
| | 이전 (stamp=now) | 새 방식 (stamp=0) |
|---|---|---|
| 최초 planner 호출 | ✅ (~0s offset) | ✅ (latest TF) |
| 10초 후 replan | ❌ TF buffer evicted | ✅ (latest TF) |
| 30초 후 recovery retry | ❌ 확실히 실패 | ✅ (latest TF) |
| BT recovery 무한 루프 | 발생 | **불가** |

### 2.3 수정 파일
- `goal_pose_restamper.py`: `msg.header.stamp = Time(sec=0, nanosec=0)`

## 3. 센서 데이터 분석 (plot_data.csv)

### 3.1 데이터 요약
| 센서 | 샘플수 | 유효 시간 범위 | 상태 |
|---|---|---|---|
| RF2O | 279 | 6.2 ~ **34.1s** | ❌ 28초 후 사망 |
| VSLAM | 8643 | 6.2 ~ 110.6s | ⚠️ 7.9s gap (42.5~50.4s) |
| Wheel odom (raw) | 3308 | 6.2 ~ 116.3s | ✅ 안정 |
| Wheel odom (adapted) | 5503 | 6.2 ~ 116.3s | ✅ 안정 |

### 3.2 RF2O 사망 (elapsed=34.1s)
- 로봇이 Goal 2(뒤쪽 목표)로 180° 회전하는 도중 사망
- yaw가 ±180° 경계를 넘는 시점과 일치
- RF2O 노드 자체 크래시 가능성 (C++ rf2o_laser_odometry)
- **respawn=True 추가**로 자동 재시작 대응

### 3.3 VSLAM Tracking Loss (elapsed=42.5~50.4s)
- RF2O 사망 후 8초 뒤 VSLAM도 7.9초간 tracking loss
- 이 시간대에 Nav2 recovery backup 진행 중
- Robot motion (backup) + 센서 이탈 = 일시적 EKF 정확도 저하

### 3.4 센서 장애 타임라인
```
t=0        t=26.1      t=34.1    t=42.5   t=50.4         t=110.6   t=116.3
|─ 정상 ──|── Goal2 ──|── RF2O ──|────────|──────── ... ──|─────────|
  시작       시작       DIES      VSLAM    VSLAM            VSLAM     끝
                                 LOST     RECOVERED        STOPS
                       
 4센서 융합  ↓3센서     ↓2센서    ↓2센서    ↓3센서           ↓2센서
 (W+V+R+I)  (W+V+R+I)  (W+V+I)  (W+I)    (W+V+I)         (W+I)
```

## 4. RF2O 안정성 대응

### 4.1 respawn 추가
```python
rf2o_node = Node(
    ...
    respawn=True,       # 크래시 시 자동 재시작
    respawn_delay=2.0,  # 재시작 전 2초 대기
    ...
)
```

### 4.2 RF2O Adapter 설정 수정
RF2O adapter가 VSLAM용 anomaly detection을 사용하고 있었음:
- `enable_vo_state_gating`: True → **False** (RF2O에는 vo_state 없음)
- `max_yaw_delta`: 0.15 rad → **99.0** (anomaly detection 사실상 비활성)
- `max_pos_delta`: 0.5m → **99.0** (RF2O는 자체 필터링으로 충분)

### 4.3 RF2O adapter도 respawn 추가
RF2O 노드 재시작 시 adapter도 유지되어야 함.

## 5. 3-Layer Defense (최종)
1. **NTP chrony**: PC↔Jetson 시계 오차 <10ms
2. **stamp=Time(0,0)**: TF2 latest available → eviction 불가
3. **transform_tolerance=3.0**: 최후 안전망 (이제 거의 필요 없음)

## 6. 수정 파일 목록
- `goal_pose_restamper.py`: stamp=now → stamp=Time(0,0)
- `nav2_vslam_fused.launch.py`:
  - rf2o_node: respawn=True, respawn_delay=2.0
  - rf2o_covariance_adapter: respawn=True, enable_vo_state_gating=False,
    max_yaw_delta=99.0, max_pos_delta=99.0

## 7. 검증 방법
- 중거리(2m+) 목표 전송 → 13초 이상 소요 → planner replan 성공 여부 확인
- RF2O 노드 강제 kill → 자동 재시작 확인
- Goal recovery 시 backup 없이 path 재계산 성공 확인

---

## 8. 2차/3차 실주행 검증 결과

### 8.1 TF Buffer Fix (stamp=0) 검증 ✅
- Goal 1~4 (최대 ~2m): planner replan 시 TF 에러 **없음**
- Goal 5 (2.77m): 장시간 소요에도 planner TF 조회 **모두 성공**
- stamp=Time(0,0) 전략 효과 확인

### 8.2 RF2O Respawn 검증 ✅
- 2차 테스트: RF2O가 **전 구간(241초)** 동작 (1차에서 34초에 사망했던 것과 대비)
- respawn=True 설정이 효과적으로 동작

## 9. 새 발견: 목표 도착 후 회전 진동 (Oscillation at Goal)

### 9.1 현상
- 먼 목표(2.7m+) 도착 후 DWB RotateToGoal이 **75초간 풀스핀 16회 반복**
- angular.z ≈ 0.042 rad/s 출력 시 모터 데드존 진입 → 회전 불가

### 9.2 근본 원인 분석

#### A. DWB RotateToGoal 토글링
DWB의 `xy_goal_tolerance=0.10m`이 너무 좁아서:
```
도착(0.10m 이내) → RotateToGoal 활성 → 제자리 회전 중 XY 드리프트
→ 0.10m 벗어남 → RotateToGoal 비활성 → GoalDist가 전진 명령
→ 다시 진입 → 다른 각도에서 RotateToGoal → 연속 스핀
```

#### B. 차동구동 모터 데드존
| 상황 | 바퀴 동작 | 마찰 | 결과 |
|------|----------|------|------|
| **정지→회전** | 양쪽 반대 방향 | **정지마찰** 동시 극복 | 높은 토크 필요 |
| **전진 중 회전** | 차속 변경만 | **동마찰** (이미 회전 중) | 낮은 토크로 충분 |

DWB의 `slowing_factor` 적용 후 최종 출력 ≈ 0.042 rad/s → 정지마찰 미극복 → 회전 불가

### 9.3 해결: RotationShimController + DWB 파라미터 개선

#### RotationShimController 도입
```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "dwb_core::DWBLocalPlanner"
  angular_dist_threshold: 0.785    # 경로와 45° 이상 차이 시
  rotate_to_heading_angular_vel: 0.5  # 고정 0.5 rad/s (데드존 초과)
  max_angular_accel: 1.6
  simulate_ahead_time: 1.0
```

**동작 흐름:**
1. 목표 수신 → 경로 방향과 현재 헤딩 비교
2. 차이 > 45° → **고정 0.5 rad/s로 제자리 회전** (모터 데드존 무시)
3. 방향 맞춰지면 → DWB에 경로 추종 위임
4. 목표 도착 → DWB RotateToGoal이 최종 yaw 정렬 (min_speed_theta=0.30 보장)

#### DWB 파라미터 변경
| 파라미터 | 이전 | 변경 | 이유 |
|---------|------|------|------|
| DWB xy_goal_tolerance | 0.10 | **0.25** | RotateToGoal 토글 방지 |
| min_speed_theta | 0.05 | **0.30** | 모터 데드존 확실히 초과 |
| RotateToGoal.slowing_factor | 5.0 | **2.0** | min_speed_theta=0.30이 바닥으로 작용 |
| general_goal_checker yaw_tol | 0.12 | **0.35** (~20°) | 오버슛 허용 |
| general_goal_checker xy_tol | 0.10 | **0.15** | 여유 확보 |
| required_movement_radius | 0.5 | **0.10** | 제자리 회전 시 progress 실패 방지 |
| movement_time_allowance | 10.0 | **15.0** | 회전 정렬에 충분한 시간 |
| **TwirlingCritic 추가** | — | scale=5.0 | 불필요한 스핀 페널티 |
| **OscillationCritic 강화** | — | reset_dist=0.15 | 리셋 임계값 상향 |

## 10. VSLAM Frozen Data Detection 구현

### 10.1 문제
- cuVSLAM이 동일 pose를 23초간 반복 발행 (qz=+0.993252 고정)
- delta=0이므로 기존 anomaly detection 통과 → stale 데이터가 EKF에 유입

### 10.2 해결
`vslam_covariance_adapter.py`에 `_check_frozen()` 메서드 추가:
- 연속 동일 pose 10회 이상 → frozen 판정 → EKF 전달 차단
- 파라미터: `frozen_threshold=10`, `frozen_pos_tolerance=1e-6`, `frozen_yaw_tolerance=1e-6`

## 11. 수정 파일 목록 (최종)
1. `goal_pose_restamper.py` — stamp=Time(0,0)
2. `nav2_vslam_fused.launch.py` — RF2O respawn + adapter 설정
3. `vslam_covariance_adapter.py` — frozen data detection
4. `nav2_params_fused.yaml` — RotationShimController + DWB 파라미터 전면 개선
5. `jupiter_base/src/base.cpp` — heading [-π,π] 정규화

## 12. SimpleProgressChecker 대안 검토
| ProgressChecker | 상태 | 설명 |
|----------------|------|------|
| `SimpleProgressChecker` | ✅ 사용 중 | XY만 측정, 회전은 진행으로 미인정 |
| `PoseProgressChecker` | ❌ Humble 미지원 | Iron/Rolling에서 추가 (회전 포함) |
| 커스텀 C++ 플러그인 | 가능 | `nav2_core::ProgressChecker` 상속 |

**결론:** RotationShimController + 넓은 xy_goal_tolerance로 스핀이 억제되면 SimpleProgressChecker의 XY 이동 감지가 정상 작동하므로 교체 불필요.

## 13. 4차 실주행 검증 및 로그 분석

### 13.1 네비게이션 결과
4개 목표 모두 성공 — RotationShimController 도입 후 회전 진동 제거 확인:
- Goal 1 (1.29m): 6초, Goal 2 (0.72m): 3초, Goal 3 (-1.52m U턴): 13초, Goal 4 (-0.73m): 4초

### 13.2 VSLAM 안정성
| 구간 | 상태 | 설명 |
|------|------|------|
| 23.5~33.4s | ✅ 동작 | 초기 10초 |
| 33.4~66.7s | ❌ gap | 33초 loss |
| 66.7~94.8s | ✅ 동작 | 28초 |
| 94.8~113.8s | ❌ gap | 19초 loss |
| 113.8~120.7s | ✅ 동작 | 7초 |
| 120.7~133.4s | ❌ gap | 13초 loss |
| 133.4~138.2s | ✅ 동작 | 5초 → 영구 종료 |

- 165초 세션 중 **66초만 유효** (40%)
- 짧은 frozen(5 samples, t=135)도 감지됨

### 13.3 TF Buffer Flush 이벤트
Goal 4 완료 후(t≈144~163) **19초간 모든 laser 메시지 버려짐:**
```
Message Filter dropping: frame 'laser' at time T
reason: 'the timestamp on the message is earlier than all the data in the transform cache'
```
**학습 포인트:**
- TF buffer가 flush되면 과거 timestamp의 sensor data가 모두 reject됨
- costmap이 장애물을 감지 못하는 blind window 발생
- SLAM Toolbox의 map→odom TF 점프 또는 Jetson CPU 과부하가 원인으로 추정
- 현재 네비게이션 미진행 구간이라 실질적 영향 없었으나, 주행 중 발생 시 위험

### 13.4 Costmap Raytrace 초기 경고
`Sensor origin at (-0.03, -0.00) is out of map bounds` — 30초간 반복 후 자연 해소.
SLAM Toolbox의 map이 아직 로봇 위치를 포함하지 않는 초기 상태 문제.

## 14. base.cpp heading 정규화

### 14.1 배경
wheel encoder odometry의 `heading_ += delta_heading`이 ±π를 넘으면 quaternion qz 부호 반전.
4차 실주행에서 t=118.7부터 EKF(qz=-0.9999)와 odom_adapted(qz=+0.9987) 차이 ≈ 2.0 확인.

### 14.2 수정
```cpp
heading_ += delta_heading;
heading_ = std::atan2(std::sin(heading_), std::cos(heading_));  // [-π, π] 정규화
```

### 14.3 학습 포인트
- `atan2(sin(h), cos(h))`는 임의 각도를 [-π, π]로 사상하는 표준 기법
- EKF는 twist만 사용하므로 기능적 영향 없으나, pose 일관성과 향후 확장성 확보
- sin/cos/atan2 3회 추가는 ~20Hz callback에서 성능 영향 무시 가능
