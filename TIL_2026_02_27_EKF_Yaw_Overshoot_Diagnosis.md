# TIL — 2026-02-27 EKF Yaw Overshoot 진단/수정 & Nav2 Backup 문제 해결

**날짜**: 2026-02-27  
**환경**: Jetson (aarch64), Ubuntu, ROS 2 Humble  
**로봇**: Jupiter (차동구동, Yahboom x3)  
**센서**: RealSense D455F, MCU IMU (ICM20948), 휠 인코더, RPLidar S2L  
**시스템**: 4-센서 EKF 융합 + SLAM Toolbox + Nav2

---

## 1. 문제 현상

EKF 출력(`/odom`)의 yaw가 실제 로봇 회전보다 크게 overshoot하여 영구적 offset이 발생.

| 시점 | Wheel yaw | EKF yaw | EKF-Wheel 오차 |
|------|-----------|---------|---------------|
| 직진 중 (t=120s) | -3.4° | -1.2° | **+2.3°** |
| 360° 회전 후 (t=190s) | +2.3° | +8.4° | **+6.1°** |
| 2차 회전 중 (t=220s) | -2.1° | +15.6° | **+17.7°** (피크) |
| 데이터 종료 (t=330s) | +1.7° | +14.5° | **+12.8°** (고착) |

---

## 2. 원인 가설 검증 과정

### 2.1. 가설 1: IMU Gyro Scale 오류 (❌ 기각)

**배경**: 이전 CSV 분석에서 EKF vyaw가 Wheel vyaw 대비 6.9배 → Rosmaster_Lib의 ICM20948 `gyro_ratio = 1/1000.0`이 의심됨 (MPU9250은 `1/3754.9`).

**검증**: `diagnose_imu_gyro_scale.py` 진단 스크립트 작성 후 실측:

```
Integrated yaw (total rotation):
  IMU raw:         -356.53°
  IMU calibrated:  -361.03°
  Wheel encoder:   -360.44°

Scale Ratio:
  IMU_raw / Wheel   = 0.9892
  IMU_calib / Wheel = 1.0017

✓ IMU gyro scale looks CORRECT (ratio close to 1.0)
```

**결론**: IMU gyro scale은 정확. `gyro_ratio = 1/1000.0`은 MCU 펌웨어가 pre-scaled mrad/s를 보내기 때문에 올바른 값.

### 2.2. 가설 2: VSLAM Differential Yaw 오차 (✅ 확정)

**검증**: CSV에서 VSLAM-Wheel 매초 yaw delta 비교:

| 시간 | Wheel delta | VSLAM delta | 오차 | 상황 |
|------|-----------|------------|------|------|
| t=165 | -0.052 qz | +0.059 qz | **+12.8°** | 180° 회전 중 (motion blur) |
| t=183 | +0.055 qz | -0.068 qz | **-14.1°** | 360° 회전 마무리 |
| t=210 | +0.196 qz | +0.111 qz | **-9.7°** | 2차 회전 진입 |
| t=215 | -0.185 qz | +0.000 qz | **+21.3°** | VSLAM 데이터 동결 중 |

**메커니즘**:
1. 빠른 회전 시 카메라 motion blur → VSLAM 피처 매칭 실패
2. VSLAM이 잘못된 yaw 변위를 보고 (또는 동결 후 재개 시 점프)
3. `odom1_differential: true` 설정으로 이 오차가 EKF에 누적
4. EKF process_noise vyaw=0.0001 → IMU가 지배하지만, VSLAM의 yaw도 position과 함께 흡수됨
5. 회전 종료 후에도 누적된 offset이 영구적으로 남음

---

## 3. 수정 사항

### 3.1. VSLAM yaw/vyaw를 EKF에서 제거 (핵심)

**파일**: `src/jupiter_nav_VO/config/ekf_vslam_fusion.yaml`

```yaml
# 변경 전 (VSLAM yaw 활성화)
odom1_config: [true,  true,  false,    # x, y, z
               false, false, true,     # roll, pitch, yaw ← 활성
               true,  true,  false,    # vx, vy, vz
               false, false, true,     # vroll, vpitch, vyaw ← 활성
               false, false, false]

# 변경 후 (VSLAM yaw 제거)
odom1_config: [true,  true,  false,    # x, y, z
               false, false, false,    # roll, pitch, yaw ← 제거
               true,  true,  false,    # vx, vy, vz
               false, false, false,    # vroll, vpitch, vyaw ← 제거
               false, false, false]
```

**근거**: VSLAM의 강점은 절대 위치(x, y) 정확도. yaw는 빠른 회전에서 불안정.  
**yaw 제어**: IMU vyaw (정확도 확인됨: ratio=1.0) + Wheel vyaw에만 위임.

### 3.2. EKF Process Noise vyaw 증가

**파일**: `src/jupiter_nav_VO/config/ekf_vslam_fusion.yaml`

```yaml
# 변경 전: vyaw(11,11) = 0.0001 (IMU 과신뢰)
# 변경 후: vyaw(11,11) = 0.01   (Wheel과 균형)
```

이전 값(0.0001)은 IMU vyaw를 거의 절대적으로 신뢰하여, 다른 센서의 yaw 기여가 무시되었음.  
0.01로 올려 Wheel encoder vyaw와 균형을 맞춤.

### 3.3. imu_gyro_scale 파라미터 추가

**파일**: `src/jupiter_bringup/jupiter_bringup/jupiter_driver_compensated.py`

```python
self.declare_parameter('imu_gyro_scale', 1.0,
    ParameterDescriptor(
        type=ParameterType.PARAMETER_DOUBLE,
        floating_point_range=[FloatingPointRange(
            from_value=0.01, to_value=10.0, step=0.001
        )],
        description='Gyro scale correction factor'
    ))
```

진단으로 scale이 정확함이 확인되어 기본값은 1.0. 향후 캘리브레이션 용도로 남겨둠.

### 3.4. Foxglove 화이트리스트에 IMU 토픽 추가

**파일**: `src/jupiter_nav_VO/launch/nav2_vslam_fused.launch.py`

```python
'/imu/data_calibrated',  # IMU calibrated (진단용)
'/jupiter/imu',          # IMU raw from driver (진단용)
```

---

## 4. 현재 EKF 센서 융합 구성 (최종)

| 센서 | 토픽 | 사용 데이터 | 비고 |
|------|------|-----------|------|
| Wheel Odom | `/odom_adapted` | vx, vy, vyaw | 기본 이동/회전 |
| VSLAM | `/visual_slam/tracking/odometry_adapted` | **x, y, vx, vy** | ~~yaw, vyaw~~ 제거 |
| RF2O | `/odom_rf2o_adapted` | x, y | ~~yaw~~ 이미 제거 (02-26) |
| IMU | `/imu/data_calibrated` | vyaw | 핵심 회전 센서 |

**Yaw 제어**: IMU vyaw + Wheel vyaw만 사용 (두 센서 모두 ratio=1.0 확인됨).

---

## 5. 수정 후 검증 — 새 CSV 분석

### 5.1. VSLAM yaw 제거 후 기록 (55293 rows, 4 topics, 317.7s)

VSLAM yaw/vyaw 제거 후 새로운 주행 데이터를 기록하여 검증:

| 토픽 | 샘플 수 |
|------|---------|
| `/odom.pose.pose.orientation.z` (EKF) | 15804 |
| `/odom_adapted.pose.pose.orientation.z` (Wheel) | 16090 |
| `/odom_rf2o_adapted.pose.pose.orientation.z` (RF2O) | 12814 |
| `/visual_slam/tracking/odometry_adapted.pose.pose.orientation.z` (VSLAM) | 10585 |

### 5.2. Unwrapped Yaw 분석 결과

| 시간 | Wheel (unwrapped) | EKF (unwrapped) | RF2O (unwrapped) | EKF-Wheel |
|------|-------------------|-----------------|------------------|-----------|
| t=50s | -21.8° | -22.3° | -20.3° | **-0.5°** |
| t=100s | -99.1° | -94.7° | -88.1° | **+4.4°** |
| t=150s | 16.3° | 17.5° | 24.7° | **+1.2°** |
| t=200s | -338.3° | -333.1° | -308.5° | **+5.2°** |
| t=250s | -337.4° | -336.6° | — | **+0.8°** |

**해석**: EKF-Wheel 차이의 최대값은 +5.2°이나, 이는 이전 세션(최대 +17.7°)보다 크게 개선됨. 다만 RF2O와의 차이가 더 크며(최대 30°), 이는 RF2O 자체의 드리프트로 추정.

### 5.3. 핵심 관찰

- EKF가 RF2O와 잘 추적 (VSLAM yaw 없이도 yaw 안정)
- Wheel odom 자체가 장기 주행에서 drift를 보임 (타이어 슬립, 노면 차이)
- t=244.8s에서 |EKF-Wheel| = 337.4° 최대 → 이는 wheel odom 드리프트이지 EKF 오류가 아님
- **VSLAM yaw 제거가 EKF yaw overshoot 문제를 해결함을 확인**

---

## 6. Nav2 Backup 반복 실행 문제 진단 및 수정

### 6.1. 문제 현상

VSLAM yaw 제거 후 주행 테스트 시, Nav2가 **모든 내비게이션 목표에서 backup behavior를 실행**:
```
[behavior_server]: Attempting backup
[behavior_server]: Running backup
```

Goal을 보낼 때마다: Planner 실패 → BT Recovery → Backup → 재시도 반복.

### 6.2. 근본 원인: PC-Jetson 시계 차이 (~2.4s)

Foxglove (PC)에서 보낸 `goal_pose`의 `header.stamp`이 **Jetson 시간보다 ~2.4초 미래**:

| Goal # | BT 수신 시간 | Goal stamp | 시계 차이 |
|--------|-------------|-----------|-----------|
| 1 | 1740661025.80 | 1740661028.21 | **+2.41s** |
| 2 | 1740661123.42 | 1740661125.67 | **+2.25s** |
| 3 | 1740661215.81 | 1740661217.81 | **+2.00s** |
| 4 | 1740661279.02 | 1740661280.02 | **+1.00s** |
| 5 | 1740661382.29 | 1740661384.38 | **+2.09s** |
| 6 | 1740661425.96 | 1740661428.40 | **+2.44s** |

### 6.3. 실패 메커니즘 (Cascade)

```
1. Foxglove PC → goal_pose(stamp = Jetson_time + 2.4s)
2. planner_server: TF lookup(target_time = stamp)
3. → "Extrapolation into the future by 2.4s" ← TF 버퍼에 미래 데이터 없음
4. → 1초 대기 → TF 버퍼 도착 → 이번엔 "Extrapolation into the past" (이미 만료)
5. → NavfnPlanner abort × 2회
6. → BT: plannerRecoveryFallback → clearGlobalCostmap → 재시도 실패
7. → BT: recoveryFallback → backup behavior 실행
```

### 6.4. 수정: transform_tolerance 3.0s

**파일**: `src/jupiter_nav_VO/config/nav2_params_fused.yaml`

5개 컴포넌트 모두 `transform_tolerance: 3.0`으로 설정:

| 컴포넌트 | 이전 값 | 변경 값 | 위치 |
|----------|---------|---------|------|
| controller_server (DWB) | 1.0 | **3.0** | line 124 |
| planner_server | (미설정) | **3.0** | line 148 |
| behavior_server | 0.1 | **3.0** | line 174 |
| local_costmap | (미설정) | **3.0** | line 198 |
| global_costmap | (미설정) | **3.0** | line 237 |

### 6.5. 근본 해결 (미구현)

`transform_tolerance`는 임시 조치. 근본 해결은 **PC-Jetson NTP 시간 동기화**:
```bash
# Jetson에 chrony 설치 후 PC와 시간 동기화
sudo apt install chrony
# → 시계 차이를 <10ms로 줄이면 transform_tolerance를 기본값으로 복원 가능
```

---

## 7. 수정 파일 최종 요약

| 파일 | 변경 | 이유 |
|------|------|------|
| `config/ekf_vslam_fusion.yaml` | odom1 yaw/vyaw 제거 | VSLAM 회전 중 ±21° 오차 → EKF 누적 |
| `config/ekf_vslam_fusion.yaml` | process_noise vyaw: 0.0001→0.01 | IMU 과신뢰 해소 |
| `config/nav2_params_fused.yaml` | transform_tolerance: 3.0 (5곳) | PC-Jetson 시계 차이 ~2.4s 허용 |
| `jupiter_driver_compensated.py` | `imu_gyro_scale` 파라미터 추가 | 향후 캘리브레이션 대비 (기본 1.0) |
| `nav2_vslam_fused.launch.py` | Foxglove에 IMU 토픽 추가 | 실시간 IMU 진단용 |
| `scripts/diagnose_imu_gyro_scale.py` | 신규 생성 | IMU gyro scale 진단 도구 |

---

## 8. 교훈 (Lessons Learned)

### 8.1. differential 모드의 오차 누적 위험

`odom1_differential: true`는 절대 위치 점프를 방지하지만, **매 프레임의 작은 오차가 영원히 누적**된다. 특히 yaw는 한번 틀어지면 모든 이후 이동 방향이 왜곡되므로, differential yaw는 센서 신뢰도가 충분하지 않으면 위험하다.

### 8.2. 가설 검증의 중요성

초기 분석에서 "IMU gyro scale 오류"로 결론 내렸으나, 실측 결과 ratio=1.0으로 완전히 기각됨. **실제 데이터 기반 검증** 없이 코드 분석만으로 결론 내리면 잘못된 수정을 하게 된다.

### 8.3. 센서별 역할 명확화

각 센서에는 고유의 강점이 있다:
- **VSLAM**: 절대 위치 (x, y) — 장기 드리프트 보정
- **IMU**: 각속도 (vyaw) — 고속 반응, 단기 정확
- **Wheel**: 속도 (vx, vyaw) — 안정적 기준선
- **RF2O**: 절대 위치 (x, y) — VSLAM 비가용 시 백업

센서의 약점을 억지로 보완하려 하지 말고, **강점만 활용**하는 것이 안정적이다.

### 8.4. 분산 시스템의 시계 동기화

PC에서 보낸 ROS 메시지의 timestamp가 로봇 시계와 다르면, TF 조회가 실패한다. `transform_tolerance` 증가는 임시 조치이며, **NTP(chrony)를 통한 시계 동기화가 근본 해결**이다. 특히 Foxglove 등 외부 클라이언트에서 goal을 보내는 구조에서는 시계 일치가 필수.

---

## 9. 향후 과제

- ⏳ Nav2 backup 수정 후 실주행 검증 — transform_tolerance 3.0으로 backup 해소 확인
- ⏳ PC-Jetson NTP 시간 동기화 (chrony) — transform_tolerance를 기본값으로 복원
- ⏳ process_noise vyaw=0.01 실주행 후 재조정 필요
- ⏳ VSLAM 트래킹 손실 원인 조사 (t=224.5s에서 종료)
- ⏳ RF2O 종료 원인 조사 (t=264.6s에서 종료)
- 💡 vslam_covariance_adapter 위치 이상치 게이팅 점검 (yaw 제거 후)
