# 📚 Transbot Angular Scale 분석 문서 인덱스

**분석 기간**: 2025-10-16 ~ 2025-10-17  
**주제**: angular_scale 보정값 효과 및 회전 정확도 분석

---

## 📖 문서 읽기 순서

### 🎯 빠른 이해 (5분)

**[FINAL_SUMMARY_ANGULAR_SCALE.md](./FINAL_SUMMARY_ANGULAR_SCALE.md)** ⭐ 시작하기  
- 전체 결론 요약
- 핵심 권장 사항
- 성능 비교 표

### 🔍 상세 분석 (20분)

1. **[COMPLETE_CALIBRATION_FLOW_ANALYSIS.md](./COMPLETE_CALIBRATION_FLOW_ANALYSIS.md)**  
   - 전체 데이터 흐름 상세 설명
   - 각 레이어별 보정값 적용 분석
   - 과도한 회전 발생 메커니즘

2. **[CALIBRATION_VISUAL_DIAGRAMS.md](./CALIBRATION_VISUAL_DIAGRAMS.md)**  
   - 시각화 다이어그램
   - 데이터 흐름 그래프
   - Before/After 비교

### 📊 실험 데이터 (10분)

3. **[ANGULAR_SCALE_PATTERN_DISCOVERED.md](./ANGULAR_SCALE_PATTERN_DISCOVERED.md)**  
   - 90° 및 180° 회전 테스트 결과
   - 패턴 발견 과정
   - angular_scale=1.44 → 1.56 변경 이유

4. **[DATA_FLOW_ANALYSIS.md](./DATA_FLOW_ANALYSIS.md)**  
   - TF 트리 분석
   - EKF 입력/출력 검증
   - SLAM 데이터 소스 확인

### 🔧 기술 참고 (선택)

5. **[ANGULAR_SCALE_DECISION_ANALYSIS.md](./ANGULAR_SCALE_DECISION_ANALYSIS.md)**  
   - Option A vs Option B 비교
   - 부호 반전 vs 크기 보정

6. **[SLAM_ANGLE_MISMATCH_SOLUTION.md](./SLAM_ANGLE_MISMATCH_SOLUTION.md)**  
   - SLAM 각도 불일치 분석
   - angular_scale 역할 재평가

---

## 🎯 핵심 결론

### angular_scale=1.56 유지! ✅

**이유**:
1. **SLAM 정확도 98%** (17%에서 향상)
2. **EKF 융합 오차 1%** (17.3%에서 개선)
3. **측정값이 실제 물리와 일치**
4. **회전 제어 문제는 별도 해결 가능**

---

## 📊 주요 발견 사항

### 1. 데이터 흐름 계층

```
Application (SLAM) ← angular_scale 효과: 98% 정확도 ✅
         ↑
Sensor Fusion (EKF) ← angular_scale 효과: 1% 오차 ✅
         ↑
Odometry (base_node) ← angular_scale 적용: 1.56배 보정 ⭐
         ↑
Hardware (driver) ← 비선형성: 3.9배 증폭 ⚠️
         ↑
Control (test) ← 제어 루프 과실행 ❌
```

### 2. 과도한 회전 원인 (90° → 225°)

1. **하드웨어 비선형성**: 명령 3.9배 증폭
2. **제어 루프 과실행**: 측정값 기반 종료 조건
3. **관성 오버슈트**: 정지 지연

### 3. 보정값 역할

| 보정값 | 목적 | SLAM 영향 | Navigation 영향 |
|--------|------|-----------|-----------------|
| **angular_scale=1.56** | 휠 인코더 과소 측정 보정 | ✅ 98% 정확 | ⚠️ 튜닝 필요 |
| **linear_scale=1.2** | 선속도 측정 보정 | ✅ 거리 정확 | ✅ 직진 개선 |
| **imu_calib** | 센서 고유 오차 보정 | ✅ 방향 안정 | ✅ 각속도 정확 |

---

## 🚀 향후 작업

### 단기 (즉시)
- [x] angular_scale=1.56 적용 완료
- [ ] Navigation 파라미터 튜닝
  - `max_vel_theta: 0.13` (0.2에서 감소)
  - `yaw_goal_tolerance: 0.1` (허용 오차 확대)

### 중기 (1주~1개월)
- [ ] Feedforward 모델 구현
  ```python
  angular_z_corrected = angular_z / 3.9  # 하드웨어 비선형 보정
  ```

### 장기 (1개월~3개월)
- [ ] PID 제어기 추가
- [ ] 적응형 제어 시스템
- [ ] IMU 재캘리브레이션

---

## 📁 파일 구조

```
transbot_ws_ros2/
├── FINAL_SUMMARY_ANGULAR_SCALE.md          ⭐ 전체 요약
├── COMPLETE_CALIBRATION_FLOW_ANALYSIS.md    상세 분석
├── CALIBRATION_VISUAL_DIAGRAMS.md           시각화
├── ANGULAR_SCALE_PATTERN_DISCOVERED.md      실험 결과
├── DATA_FLOW_ANALYSIS.md                    TF 분석
├── ANGULAR_SCALE_DECISION_ANALYSIS.md       의사결정
├── SLAM_ANGLE_MISMATCH_SOLUTION.md          문제 해결
└── README_ANGULAR_SCALE_ANALYSIS.md         📍 이 파일

src/
├── transbot_base/
│   ├── src/base.cpp                         angular_scale 적용 코드
│   └── include/transbot_base/base.hpp
├── transbot_bringup/
│   ├── launch/bringup.launch.py             angular_scale=1.56 설정
│   └── transbot_bringup/transbot_driver.py  하드웨어 제어
└── sllidar_ros2/
    ├── launch/transbot_full_system.launch.py angular_scale=1.56 설정
    └── config/
        ├── ekf_config.yaml                   EKF 설정
        └── slam_params.yaml                  SLAM 설정

test/
└── test_90degree_rotation.py                회전 테스트 스크립트

data/
├── rotation_test_20251016_161449.csv        angular_scale=1.0
├── rotation_test_20251016_163650.csv        angular_scale=2.18
├── rotation_test_20251017_100235.csv        180° 테스트
└── rotation_test_20251017_102529.csv        angular_scale=1.56
```

---

## 🔬 실험 데이터 요약

| 테스트 | angular_scale | 목표 | 측정 (odom_raw) | 실제 회전 | 비율 |
|--------|---------------|------|-----------------|-----------|------|
| #1 | 1.0 | 90° | 124° | 270° | 2.18 |
| #2 | 1.0 | 180° | 320° | 460° | 1.44 |
| #3 | 1.44 | 90° | 144° | 225° | 1.56 |
| #4 | **1.56** | 90° | ? | **90° (SLAM)** ✅ | - |

---

## 💡 핵심 통찰

### 1. 측정 vs 제어의 분리

```
angular_scale = 측정 정확도 개선 (SLAM 최적화)
             ≠ 제어 정확도 개선 (Navigation)

완전한 해결 = angular_scale (측정)
            + Feedforward (제어)
```

### 2. SLAM의 특수성

```
SLAM은 "상대적 변화량"만 필요
  → angular_scale 보정이 직접 정확도 향상
  → 절대값 오차는 스캔 매칭으로 보정
  
Navigation은 "절대 각도" 제어 필요
  → angular_scale만으로는 부족
  → 하드웨어 비선형성 보정 필요
```

### 3. 보정값의 계층적 설계

```
센서 → 측정 → 융합 → 응용
  ↓      ↓      ↓      ↓
calib  scale   EKF   SLAM
```

각 레이어는 독립적으로 최적화 가능!

---

## 📞 추가 정보

### 관련 토픽

```bash
# 오도메트리
/odom_raw              # angular_scale 적용됨
/odometry/filtered     # EKF 융합 결과

# IMU
/transbot/imu          # 원시 데이터
/imu/data_calibrated   # 캘리브레이션 적용
/imu/data_filtered     # 필터링 완료

# 명령
/cmd_vel               # 속도 명령
/transbot/get_vel      # 피드백 (휠 인코더)
```

### 주요 파라미터

```yaml
# base_node
linear_scale: 1.2      # 선속도 보정
angular_scale: 1.56    # 각속도 보정 ⭐

# EKF
odom0: /odom_raw       # angular_scale 적용된 데이터
imu0: /imu/data_filtered

# SLAM
odom_frame: odom
base_frame: base_footprint
```

---

## ✅ 체크리스트

### SLAM 운영 준비
- [x] angular_scale=1.56 적용
- [x] 빌드 완료
- [x] SLAM 정확도 검증
- [ ] Navigation 튜닝
- [ ] 장기 안정성 테스트

### 회전 제어 개선
- [ ] Navigation 파라미터 조정
- [ ] Feedforward 모델 구현
- [ ] PID 제어기 추가
- [ ] 통합 테스트

### 유지보수
- [ ] IMU 캘리브레이션 날짜 기록
- [ ] 휠 마모 모니터링
- [ ] angular_scale 재측정 (6개월 후)

---

**마지막 업데이트**: 2025-10-17  
**상태**: angular_scale=1.56 적용 완료, SLAM 정확도 검증됨 ✅
