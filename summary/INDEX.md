# Transbot 문서 종합 인덱스

> **최종 업데이트:** 2025-10-31  
> **문서 정리:** 70+ 개별 MD 파일 → 6개 종합 가이드  
> **전체 개발 기간:** 2025년 9월 30일 ~ 10월 31일

---

## �️ 전체 개발 타임라인

```
2025-09-30 ────────────────────────────────────────────────── 2025-10-31
    │                                                              │
    ├─ Phase 1: 센서 캘리브레이션 (09/30 ~ 10/21)
    │   └─ IMU, Odometry 정확도 개선
    │
    ├─ Phase 2: EKF 센서 퓨전 (10/21 ~ 10/24)
    │   └─ 센서 데이터 통합 문제 해결
    │
    ├─ Phase 3: 회전 정확도 (10/16 ~ 10/17)
    │   └─ Angular scale 보정
    │
    ├─ Phase 4: SLAM 최적화 (10/17 ~ 10/21)
    │   └─ 맵 품질 향상, Loop closure
    │
    ├─ Phase 5: Nav2 통합 (10/29 ~ 10/31)
    │   └─ 자율 주행, 속도 튜닝
    │
    └─ Phase 6: 패키지 구조 개선 (10/31)
        └─ transbot_nav 패키지 생성
```

---

## �📚 문서 구조

### 🎯 핵심 문서 (권장 순서)

1. **[IMU 및 Odometry 캘리브레이션](01_IMU_ODOMETRY_CALIBRATION.md)** 📅 `09/30 ~ 10/21`
   - IMU 센서 캘리브레이션 (MPU6050)
   - LiDAR-Odometry 좌표계 정렬
   - 센서 파라미터 분석
   - 테스트 및 검증 방법
   
2. **[EKF 센서 퓨전 문제 해결](02_EKF_SENSOR_FUSION.md)** 📅 `10/21 ~ 10/24`
   - EKF 각속도 전달 문제 진단
   - Phase 1-2 근본 원인 분석
   - TF 프레임 충돌 해결
   - Process Noise Covariance 최적화
   
3. **[회전 정확도 개선](03_ROTATION_ACCURACY.md)** 📅 `10/16 ~ 10/17`
   - Angular Scale 분석 및 보정
   - 270도 회전 문제 해결
   - 10바퀴 회전 테스트
   - Wheelbase 실측 및 적용
   
4. **[SLAM 최적화](04_SLAM_OPTIMIZATION.md)** 📅 `10/17 ~ 10/21`
   - 맵 중첩 문제 해결
   - Loop Closure 파라미터 튜닝
   - Odometry 정확도 개선
   - 드리프트 최소화 전략
   
5. **[Nav2 네비게이션](05_NAV2_NAVIGATION.md)** 📅 `10/29 ~ 10/31`
   - Nav2 설정 및 통합
   - 속도 제한 전략 (회전 vs 전진)
   - 진동 문제 해결
   - TF 프레임 관리

6. **[패키지 구조 개선](06_PACKAGE_ARCHITECTURE.md)** 📅 `10/31`
   - transbot_nav 패키지 생성
   - 아키텍처 개선 (관심사 분리)
   - 마이그레이션 과정
   - 사용법 및 참조 문서

---

## 🗂️ 문서별 주요 내용

### 01_IMU_ODOMETRY_CALIBRATION.md

**다루는 주제:**
- ✅ IMU 자동/수동 캘리브레이션
- ✅ Gyro/Accel 바이어스 보정
- ✅ LiDAR-Odometry 방향 정렬
- ✅ TF 프레임 체인 검증
- ✅ 90도 회전 테스트

**주요 파일:**
- `imu_calib.yaml` - IMU 캘리브레이션 결과
- `transbot_params.yaml` - 하드웨어 파라미터

**통합된 원본 문서:**
```
IMU_CALIB_GUIDE.md
IMU_OPTIMIZATION_GUIDE.md
LIDAR_ODOMETRY_ALIGNMENT_GUIDE.md
SENSOR_ACCURACY_PARAMETERS_ANALYSIS.md
HARDWARE_PARAMETER_ANALYSIS.md
CALIBRATION_*.md (다수)
```

---

### 02_EKF_SENSOR_FUSION.md

**다루는 주제:**
- ✅ EKF 각속도 0 문제 진단
- ✅ Phase 1: 런치 파일 분석 (1.5618)
- ✅ Phase 2: TF 충돌 해결
- ✅ IMU 우선 센서 퓨전 설정
- ✅ Process Noise Covariance 튜닝

**주요 파일:**
- `ekf_config.yaml` - EKF 최종 설정

**핵심 해결책:**
```yaml
# IMU 우선 (회전 정보)
imu0_angular_velocity_covariance: 0.000025  # 매우 높은 신뢰

# Odometry (위치 정보만)
odom0_config: [true, true, false, ...]  # 각속도 사용 안 함

# Process Noise (IMU 중심)
process_noise_covariance[11]: 0.00005  # vyaw - 매우 낮음
```

**통합된 원본 문서:**
```
EKF_DIAGNOSIS.md
EKF_FIX_STRATEGY.md
EKF_FIX_SUMMARY.md
EKF_ANGULAR_VELOCITY_FIX.md
EKF_ADDITIONAL_FIX.md
EKF_FREQUENCY_ADJUSTMENT.md
EKF_분석_보고서.md
EKF_ROOT_CAUSE.md
PHASE1_*.md (다수)
PHASE2_*.md (다수)
DATA_FLOW_ANALYSIS.md
TF_CONFLICT_*.md (다수)
```

---

### 03_ROTATION_ACCURACY.md

**다루는 주제:**
- ✅ Angular Scale 측정 및 분석
- ✅ 과회전 문제 원인 (Wheelbase 오측정)
- ✅ 270도 회전 시 오차율 감소 이유
- ✅ 10바퀴 회전 테스트 (누적 오차)
- ✅ 타이어 슬립 최소화 전략

**주요 파일:**
- `transbot_params.yaml` - wheelbase 보정

**성능 개선:**
| 항목 | Before | After | 개선 |
|------|--------|-------|------|
| 90° 회전 오차 | 22% | 2% | 91% ↓ |
| 10바퀴 오차 | 9.7% | 0.8% | 92% ↓ |

**통합된 원본 문서:**
```
README_ANGULAR_SCALE_ANALYSIS.md
FINAL_SUMMARY_ANGULAR_SCALE.md
FIX_270_DEGREE_PROBLEM.md
FIX_APPLIED_270_DEGREE.md
ROTATION_270_DEGREE_ROOT_CAUSE_ANALYSIS.md
ROTATION_10LAPS_ANALYSIS.md
ROTATION_CALIBRATION_ROADMAP.md
ROTATION_FIX_APPLIED.md
ROTATION_LOCALIZATION_FIX.md
ROTATION_PARAMETER_SUMMARY.md
ROTATION_TEST_*.md (다수)
ROTATION_TRACKING_ANALYSIS.md
GYRO_DRIFT_FIX.md
odom_rotation_analysis.md
```

---

### 04_SLAM_OPTIMIZATION.md

**다루는 주제:**
- ✅ 맵 중첩 문제 원인 및 해결
- ✅ Loop Closure 파라미터 의미
- ✅ 잘못된 Loop Closure 방지
- ✅ Odometry 정확도가 SLAM에 미치는 영향
- ✅ SLAM 품질 향상을 위한 속도 제한

**주요 파일:**
- `slam_params.yaml` - SLAM Toolbox 설정

**핵심 파라미터:**
```yaml
# 엄격한 Loop Closure
loop_match_minimum_response_fine: 0.65  # 65% 유사도
loop_match_minimum_chain_size: 15       # 15개 체인

# 안정적인 스캔
minimum_travel_distance: 0.10  # 10cm
minimum_travel_heading: 0.05   # ~3도
```

**통합된 원본 문서:**
```
SLAM_ANGLE_MISMATCH_SOLUTION.md
SLAM_DUAL_STRATEGY.md
SLAM_LOCALIZATION_RELIABILITY_IMPROVEMENT.md
SLAM_드리프트_해결_최종보고서.md
SLAM_맵중첩_문제해결_가이드.md
SLAM_ODOMETRY_ACCURACY_IMPROVEMENT_METHODOLOGY.md
SLAM_OPTIMIZATION_APPLIED.md
SLAM_PARAMETER_TUNING_ANALYSIS.md
SLAM_ROTATION_DIAGNOSIS.md
localization_strategy_guide.md
LOCALIZATION_DATA_FLOW.md
```

---

### 05_NAV2_NAVIGATION.md

**다루는 주제:**
- ✅ Nav2 구조 및 컴포넌트
- ✅ 속도 제한 전략 (회전만 제한)
- ✅ 진동 문제 원인 (가속도 제한 과다)
- ✅ TF 프레임 일관성 유지
- ✅ Odom 드리프트 정상 동작 이해

**주요 파일:**
- `nav2_params.yaml` - Nav2 스택 설정

**핵심 전략:**
```yaml
# 회전 속도만 제한 (SLAM 품질)
max_vel_theta: 0.5  # 천천히 회전

# 가속도는 유지 (진동 방지)
acc_lim_theta: 3.2  # DWB 경로 계획 유연성
```

**통합된 원본 문서:**
```
Nav2_네비게이션_문제해결_가이드.md
Nav2_회전속도_제한_설정.md
Nav2_속도_제한_설정.md
Nav2_진동_문제_분석.md
Nav2_TF프레임_분리_문제해결.md
TF프레임_이해_odom_드리프트.md
NAVIGATION_TUNING_COMPLETE.md
OSCILLATION_PROBLEM_ANALYSIS.md
OPTIMIZATION_COMPLETE.md
```

---

## 🚀 빠른 시작 가이드

### 전체 시스템 실행

```bash
# 1. IMU 캘리브레이션 (최초 1회)
ros2 run imu_calib imu_calib_node

# 2. 전체 시스템 실행 (SLAM + EKF + 센서)
ros2 launch transbot_nav transbot_full_system.launch.py

# 3. Nav2 자율 주행 (별도 터미널)
ros2 launch transbot_nav nav2_navigation.launch.py

# 4. RViz 시각화
rviz2 -d $(ros2 pkg prefix transbot_nav)/share/transbot_nav/rviz/sllidar_ros2.rviz
```

### 주요 설정 파일 위치

```bash
# IMU 캘리브레이션
~/transbot_ws_ros2/imu_calib.yaml

# EKF 센서 퓨전
~/transbot_ws_ros2/src/transbot_nav/config/ekf_config.yaml

# SLAM Toolbox
~/transbot_ws_ros2/src/transbot_nav/config/slam_params.yaml

# Nav2 네비게이션
~/transbot_ws_ros2/src/transbot_nav/config/nav2_params.yaml

# 하드웨어 파라미터
~/transbot_ws_ros2/src/transbot_base/config/transbot_params.yaml
```

---

## 📖 문서 사용 가이드

### 처음 시작하시나요?

**권장 순서:**
1. `01_IMU_ODOMETRY_CALIBRATION.md` - IMU 캘리브레이션 실행
2. `02_EKF_SENSOR_FUSION.md` - EKF 설정 이해
3. `transbot_nav/README.md` - 전체 시스템 실행
4. `transbot_nav/QUICK_REFERENCE.md` - 빠른 참조

### 특정 문제 해결

**맵이 흐릿하거나 겹쳐요:**
→ `04_SLAM_OPTIMIZATION.md` 참조

**로봇이 진동해요:**
→ `05_NAV2_NAVIGATION.md` § 진동 문제 해결

**회전 각도가 안 맞아요:**
→ `03_ROTATION_ACCURACY.md` § Angular Scale 보정

**EKF가 IMU를 무시해요:**
→ `02_EKF_SENSOR_FUSION.md` § Phase 2 분석

**TF 프레임 에러:**
→ `05_NAV2_NAVIGATION.md` § TF 프레임 관리

### 고급 튜닝

**SLAM 파라미터 세밀 조정:**
→ `04_SLAM_OPTIMIZATION.md` § Loop Closure 최적화

**센서 정확도 분석:**
→ `01_IMU_ODOMETRY_CALIBRATION.md` § 센서 파라미터 분석

**Nav2 성능 최적화:**
→ `05_NAV2_NAVIGATION.md` § 성능 최적화 팁

---

## 🔍 주요 개념 설명

### TF 프레임 체인

```
map (SLAM Toolbox) - 절대 좌표계
 └─ odom (EKF) - 드리프트 있지만 연속적
     └─ base_footprint (로봇 지면 투영)
         └─ base_link (로봇 중심)
             ├─ laser_frame (LiDAR)
             ├─ imu_link (IMU)
             └─ camera_link (카메라)
```

**중요:**
- `odom`이 `base_footprint`에서 분리되는 것은 **정상**
- SLAM이 루프 클로저로 `map → odom` TF를 보정
- 이것이 SLAM의 위치 보정 메커니즘

### 센서 역할 분담

| 센서 | 역할 | 정확도 | 사용처 |
|------|------|--------|--------|
| **IMU** | 회전 정보 | 매우 높음 | EKF (yaw rate) |
| **Odometry** | 위치 정보 | 보통 (슬립) | EKF (x, y) |
| **LiDAR** | 절대 위치 | 높음 | SLAM |

### 파라미터 우선순위

**속도 제한 (SLAM 품질):**
```yaml
max_vel_theta: 0.5  # 회전은 천천히 ⭐
max_vel_x: 0.3      # 전진은 빠르게
```

**가속도 (경로 계획):**
```yaml
acc_lim_theta: 3.2  # 높게 유지 (진동 방지) ⭐
acc_lim_x: 2.5
```

**Loop Closure (맵 품질):**
```yaml
loop_match_minimum_response_fine: 0.65  # 엄격하게 ⭐
loop_match_minimum_chain_size: 15
```

**IMU 신뢰도 (회전 정확도):**
```yaml
imu0_angular_velocity_covariance: 0.000025  # 매우 낮음 = 높은 신뢰 ⭐
process_noise_covariance[11]: 0.00005       # vyaw - IMU 우선
```

---

## 📊 성능 벤치마크

### 전체 시스템

| 항목 | 초기 (Before) | 최적화 (After) | 개선율 |
|------|---------------|----------------|--------|
| 90° 회전 오차 | ±15° | ±2° | 87% ↓ |
| 10바퀴 누적 오차 | 9.7% | 0.8% | 92% ↓ |
| SLAM Loop Closure | 50% | 90% | 80% ↑ |
| 맵 중첩 발생 | 빈번 | 거의 없음 | 95% ↓ |
| Nav2 진동 | 발생 | 없음 | - |
| EKF 각속도 전달 | 0% | 97% | - |

---

## 🛠️ 문제 해결 플로우차트

```
문제 발생
    ↓
┌─────────────────┐
│ 어떤 문제인가요? │
└─────────────────┘
    ↓
    ├─→ [맵 문제] → 04_SLAM_OPTIMIZATION.md
    ├─→ [회전 오차] → 03_ROTATION_ACCURACY.md
    ├─→ [Nav2 에러] → 05_NAV2_NAVIGATION.md
    ├─→ [EKF 이상] → 02_EKF_SENSOR_FUSION.md
    ├─→ [센서 문제] → 01_IMU_ODOMETRY_CALIBRATION.md
    └─→ [시스템 전반] → transbot_nav/README.md
```

---

## 🗑️ 삭제된 원본 파일 목록

**총 70+ 개의 MD 파일이 6개로 통합되었습니다.**

### 삭제 예정 파일들

```bash
# EKF 관련 (→ 02_EKF_SENSOR_FUSION.md)
EKF_DIAGNOSIS.md
EKF_FIX_STRATEGY.md
EKF_FIX_SUMMARY.md
EKF_ANGULAR_VELOCITY_FIX.md
EKF_ADDITIONAL_FIX.md
EKF_FREQUENCY_ADJUSTMENT.md
EKF_분석_보고서.md
EKF_ROOT_CAUSE.md
PHASE1_ANALYSIS_LAUNCH_1.5618.md
PHASE1_DETAILED_ANALYSIS.md
PHASE1_DIAGNOSIS_GUIDE.md
PHASE2_ANALYSIS.md
PHASE2_COMPLETE_ANALYSIS.md
PHASE2_FIX_VERIFICATION.md
DATA_FLOW_ANALYSIS.md
TF_CONFLICT_ANALYSIS.md
TF_CONFLICT_RESOLUTION_COMPLETE.md

# Rotation 관련 (→ 03_ROTATION_ACCURACY.md)
README_ANGULAR_SCALE_ANALYSIS.md
FINAL_SUMMARY_ANGULAR_SCALE.md
FIX_270_DEGREE_PROBLEM.md
FIX_APPLIED_270_DEGREE.md
ROTATION_270_DEGREE_ROOT_CAUSE_ANALYSIS.md
ROTATION_10LAPS_ANALYSIS.md
ROTATION_CALIBRATION_ROADMAP.md
ROTATION_FIX_APPLIED.md
ROTATION_LOCALIZATION_FIX.md
ROTATION_PARAMETER_SUMMARY.md
ROTATION_TEST_270_DEGREE_ANALYSIS.md
ROTATION_TEST_IMPROVEMENT.md
ROTATION_TEST_OSCILLATION_ANALYSIS.md
ROTATION_TEST_RESULT_20251016_140235.md
ROTATION_TRACKING_ANALYSIS.md
GYRO_DRIFT_FIX.md
odom_rotation_analysis.md

# SLAM 관련 (→ 04_SLAM_OPTIMIZATION.md)
SLAM_ANGLE_MISMATCH_SOLUTION.md
SLAM_DUAL_STRATEGY.md
SLAM_LOCALIZATION_RELIABILITY_IMPROVEMENT.md
SLAM_드리프트_해결_최종보고서.md
SLAM_맵중첩_문제해결_가이드.md
SLAM_ODOMETRY_ACCURACY_IMPROVEMENT_METHODOLOGY.md
SLAM_OPTIMIZATION_APPLIED.md
SLAM_PARAMETER_TUNING_ANALYSIS.md
SLAM_ROTATION_DIAGNOSIS.md
localization_strategy_guide.md
LOCALIZATION_DATA_FLOW.md

# Nav2 관련 (→ 05_NAV2_NAVIGATION.md)
Nav2_네비게이션_문제해결_가이드.md
Nav2_회전속도_제한_설정.md
Nav2_속도_제한_설정.md
Nav2_진동_문제_분석.md
Nav2_TF프레임_분리_문제해결_md
TF프레임_이해_odom_드리프트.md
NAVIGATION_TUNING_COMPLETE.md
OSCILLATION_PROBLEM_ANALYSIS.md
OPTIMIZATION_COMPLETE.md

# Calibration 관련 (→ 01_IMU_ODOMETRY_CALIBRATION.md)
IMU_CALIB_GUIDE.md
IMU_OPTIMIZATION_GUIDE.md
LIDAR_ODOMETRY_ALIGNMENT_GUIDE.md
SENSOR_ACCURACY_PARAMETERS_ANALYSIS.md
HARDWARE_PARAMETER_ANALYSIS.md
CALIBRATION_SCRIPT_FIX_COMPLETE.md
CALIBRATION_SCRIPT_IMPROVEMENTS.md
CALIBRATION_USAGE.md
CALIBRATION_VISUAL_DIAGRAMS.md
COMPLETE_CALIBRATION_FLOW_ANALYSIS.md
CORRECT_CALIBRATION_STRATEGY.md
SENSOR_BASED_CALIBRATION_APPROACH.md

# 기타
REAL_PROBLEM_ANALYSIS.md
TF_ANALYSIS.md
TEMPORARY_IMU_TOPIC.md
TEST_90DEGREE_ROTATION_GUIDE.md
MINIMAL_ODOM_TEST_README.md
테스트_결과_분석_2025-10-24.md
```

**보관할 파일:**
- `src/*/README.md` - 각 패키지 설명
- `summary/*` - 이번에 생성한 종합 문서들

---

## 📝 문서 유지보수

### 문서 업데이트 시

1. **관련 종합 문서 수정** (01~05)
2. **INDEX.md 업데이트** (필요시)
3. **transbot_nav/README.md 반영** (시스템 레벨 변경 시)

### 새 기능 추가 시

1. 해당 카테고리 종합 문서에 섹션 추가
2. 또는 새로운 카테고리 문서 생성 (06_*.md)
3. INDEX.md에 링크 추가

---

**문서 정리 완료:** 2025-10-31  
**정리 전:** 70+ 개별 MD 파일  
**정리 후:** 6개 종합 가이드 + 패키지 문서
