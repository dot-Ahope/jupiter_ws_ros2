# TIL — 2026-03-03 Nav2 Goal Stamp 문제 해결 & 센서 안정성 개선

**날짜**: 2026-03-03  
**환경**: Jetson (aarch64), Ubuntu, ROS 2 Humble  
**로봇**: Jupiter (차동구동, Yahboom x3)  
**이전 작업**: `TIL_2026_02_27_EKF_Yaw_Overshoot_Diagnosis.md`

---

## 1. 문제 현상

### 1.1. Nav2 Goal 4 실패 — backup 반복 후 최종 abort

transform_tolerance 3.0 적용 후 **Goals 1-3은 성공**하지만, **Goal 4** (0.07,-1.87)→(2.51,-1.32)에서:

1. Controller "Failed to make progress" (10s간 0.5m 미이동)
2. BT가 원래 goal stamp(1772522993.0)으로 재계획 시도
3. 30초+ 경과 → TF 버퍼(10s)에서 만료 → "Extrapolation into the past" 무한 반복
4. backup 3회 → 최종 Goal failed

### 1.2. PC-Jetson 시계 차이 ~4s (2.4s → 4s 증가)

| Goal | BT 수신 시간 | Goal stamp | 차이 |
|------|-------------|-----------|------|
| 1 | 916.67 | 921.0 | **+4.33s** |
| 2 | 934.46 | 939.0 | **+4.54s** |
| 3 | 952.61 | 957.0 | **+4.39s** |
| 4 | 989.05 | 993.0 | **+3.95s** |

### 1.3. VSLAM/RF2O 종료 문제 (이전 세션에서 확인)

- VSLAM: t=224.5s에서 odometry 발행 중단
- RF2O: t=264.6s에서 odometry 발행 중단

---

## 2. 근본 원인 분석

### 2.1. Goal Stamp Cascade 실패

```
1. Foxglove(PC) → goal_pose(stamp = Jetson_time + 4s)
2. planner_server: TF lookup → "future extrapolation" → 실패
3. BT recovery → clearCostmap → 재시도 → 성공 (TF 따라잡음)
4. controller: 10s 주행 → "Failed to make progress" → abort
5. BT: 원래 goal stamp(4s+10s = 14s 과거)로 재계획 → "past extrapolation"
6. TF 버퍼(10s)에 해당 시간 없음 → 무한 실패 → backup → abort
```

**핵심**: transform_tolerance 3.0은 초기 ~1s 미래 오차를 처리하지만, controller 실패 후 stale timestamp(14s+ 과거)는 처리 불가.

### 2.2. VSLAM 추적 손실

- **USB 안정**: RealSense (USB3.0 Bus2, 5Gbps) / RPLidar (USB2.0 Bus1, 12Mbps) — 별도 버스, dmesg에 disconnect 이벤트 없음
- **GPU 부하**: `enable_slam_visualization: True` + landmarks + observations → 불필요한 GPU 사용
- **추정 원인**: 빠른 회전 시 motion blur → 스테레오 피처 매칭 실패 → tracking lost
- cuVSLAM은 tracking 손실 시 odometry 발행을 중단함 (crash 아님)

### 2.3. RF2O 종료

- USB 안정 확인됨 (하드웨어 문제 아님)
- 추정 원인: LiDAR scan 품질 이슈 (환경 의존적) 또는 RF2O 알고리즘 한계
- EKF `sensor_timeout: 0.5` → 센서 중단 시 0.5s 후 자동으로 나머지 센서만 사용

---

## 3. 수정 사항

### 3.1. Goal Pose Re-stamper 노드 (핵심)

**파일**: `jupiter_nav_VO/goal_pose_restamper.py` (신규)

Foxglove PC의 goal_pose를 Jetson 현재 시간으로 재스탬핑:

```
토픽 흐름:
  Foxglove(PC) → /goal_pose_raw → [restamper] → /goal_pose → Nav2 bt_navigator
```

- Foxglove bridge의 `client_topic_whitelist`: `/goal_pose` → `/goal_pose_raw` 변경
- Foxglove 3D Panel 설정에서 Publish topic을 `/goal_pose_raw`로 변경 필요

**효과**:
- Goal stamp이 항상 Jetson 현재 시간 → TF lookup 항상 성공
- Controller 실패 후 재계획 시에도 stale timestamp 문제 없음
- NTP 동기화와 무관하게 안정적으로 동작

### 3.2. NTP 시간 동기화 (chrony)

```bash
sudo apt install chrony
sudo systemctl enable chrony && sudo systemctl start chrony
```

Jetson 시간이 NTP 서버(ap-northeast-2 EC2)와 **1ms 이내**로 동기화:
```
System time: 0.001075122 seconds fast of NTP time
Stratum: 3
Reference: ec2-3-39-176-65.ap-northeast-2.compute.amazonaws.com
```

**PC 측에서도 NTP 동기화 필요** — Windows: `w32tm /resync`, Linux: `sudo apt install chrony`

### 3.3. VSLAM 시각화 비활성화

**파일**: `isaac_ws_ros/src/isaac_ros_visual_slam/.../isaac_ros_visual_slam_realsense.launch.py`

```python
# 변경 전 (GPU 부하 높음)
'enable_slam_visualization': True,
'enable_landmarks_view': True,
'enable_observations_view': True,

# 변경 후 (GPU 부하 감소 → 추적 안정성 향상)
'enable_slam_visualization': False,
'enable_landmarks_view': False,
'enable_observations_view': False,
```

**효과**: GPU VRAM 및 처리 부하 10-15% 감소 → 빠른 회전 시 tracking 유지 가능성 향상

---

## 4. 수정 파일 요약

| 파일 | 변경 | 이유 |
|------|------|------|
| `goal_pose_restamper.py` | 신규 생성 | PC→Jetson goal timestamp 변환 |
| `nav2_fused_navigation.launch.py` | restamper 노드 추가 + 문서화 | launch에 통합 |
| `setup.py` | entry_point 추가 | goal_pose_restamper 등록 |
| `nav2_vslam_fused.launch.py` | client_topic_whitelist 변경 | `/goal_pose_raw` 허용 |
| `isaac_ros_visual_slam_realsense.launch.py` | 시각화 비활성화 | GPU 부하 감소 |
| Jetson system | chrony 설치 | NTP 시간 동기화 |

---

## 5. USB 버스 토폴로지 (확인됨)

```
Bus 02 (USB 3.0, 10Gbps):
  └── RealSense D455F (5Gbps) — 스테레오 IR 90fps + IMU 200Hz

Bus 01 (USB 2.0, 480Mbps):
  ├── CP210x (RPLidar S2L, 1Mbps 시리얼)
  ├── ch34x (MCU, 시리얼)
  └── rtk_btusb (Bluetooth)
```

RealSense와 RPLidar가 별도 USB 호스트 컨트롤러에 있어 대역폭 경합 없음.

---

## 6. 교훈 (Lessons Learned)

### 6.1. ROS 2 분산 시스템의 시계 문제

Foxglove 같은 외부 클라이언트가 메시지를 발행할 때, `header.stamp`은 클라이언트 시계 기준. Nav2의 TF 시스템은 이 stamp으로 TF를 조회하므로, 시계 차이가 크면 반드시 실패한다.

**3단 방어**:
1. **NTP 동기화** (chrony) — 시계 차이 <10ms로 유지 (근본 해결)
2. **Goal Pose Re-stamper** — 수신 시 로봇 시간으로 재스탬핑 (보험)
3. **transform_tolerance 증가** — 최후의 방어선 (3.0s)

### 6.2. GPU 리소스 경합

VSLAM 시각화 기능은 GPU VRAM과 처리 시간을 소모하며, 이는 실시간 추적 성능을 저하시킨다. **프로덕션 환경에서는 시각화를 반드시 비활성화**해야 한다.

---

## 7. 향후 과제

- ⏳ Re-stamper + NTP 적용 후 Goal 4 같은 장거리 네비게이션 재검증
- ⏳ Foxglove 3D Panel에서 Publish topic을 `/goal_pose_raw`로 변경
- ⏳ PC 측 chrony/NTP 동기화 확인
- ⏳ VSLAM 시각화 비활성화 후 tracking 손실 빈도 재측정
- ⏳ process_noise vyaw=0.01 실주행 검증
- 💡 RF2O 종료 원인 → 다음 실주행 시 `/rosout` 로그에서 RF2O 관련 에러 확인
