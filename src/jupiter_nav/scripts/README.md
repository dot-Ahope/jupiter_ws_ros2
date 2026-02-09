# Transbot 센서 진단 스크립트 모음

이 디렉토리에는 센서 검증 및 진단을 위한 스크립트들이 있습니다.

---

## 📁 스크립트 목록

### 1. `ekf_comparison_test.py` ⭐ 주요 스크립트
> **목적:** Odom, IMU, EKF 융합 결과를 종합적으로 비교 및 분석

### 2. `quick_encoder_test.py` 🆕 빠른 진단
> **목적:** 인코더 데이터 손실 문제 빠른 진단 (3초 × 2방향)

### 3. `diagnose_encoder.py` 🔬 상세 진단
> **목적:** 인코더 메시지 주기, 간격, 연속성 상세 분석

---

## 🚨 인코더 데이터 손실 문제

### 증상
- 특정 방향(주로 CCW) 회전 시 Odom 값이 누락되거나 적게 측정됨
- 예: IMU 86° 회전했는데 Odom은 8° 만 측정

### 즉시 진단

```bash
cd ~/transbot_ws_ros2/src/transbot_nav/scripts

# 1. 빠른 테스트 (1분)
python3 quick_encoder_test.py

# 2. 전체 테스트 (원인 규명 필요 시)
python3 ekf_comparison_test.py
```

### 해결 가이드

📖 **[인코더 디버그 완전 가이드](../../../ENCODER_DEBUG_GUIDE.md)**
- 1단계: 즉시 진단 (토픽 Hz 확인)
- 2단계: 하드웨어 점검 (케이블, 커넥터)
- 3단계: 소프트웨어 점검 (로그, CPU)
- 4단계: 구체적 해결책 (6가지 방법)
- 5단계: 검증 테스트

---

## 🎯 ekf_comparison_test.py (메인 스크립트)

---

## 🎯 개요

이 스크립트는 센서 정렬과 EKF 성능을 한 번에 검증합니다:

1. **IMU 기준 회전**: IMU 적분값을 기준으로 90도 회전
2. **Odom 기준 회전**: Odom 측정값을 기준으로 90도 회전
3. **4방향 테스트**: 반시계/시계 방향 각각 테스트
4. **센서 비교**: IMU, Odom, EKF 각각의 측정값 비교
5. **Angular Scale**: Odom angular_scale 자동 계산 및 제안

---

## 🚀 사용법

### 1단계: 시스템 실행

```bash
# 터미널 1
ros2 launch transbot_nav transbot_full_system.launch.py
```

### 2단계: 테스트 실행

```bash
# 터미널 2
cd ~/transbot_ws_ros2/src/transbot_nav/scripts
python3 ekf_comparison_test.py
```

### 3단계: 준비 확인

스크립트가 다음을 확인합니다:
- 로봇 주변 2m 공간
- 센서 데이터 수신 확인

준비되면 **Enter** 키를 눌러 시작

---

## 📊 테스트 과정

### Test 1/4: IMU 기준 반시계 회전
```
진행  50.2% | IMU적분:  45.18° | Odom:  47.23° | EKF:  46.15°
진행 100.4% | IMU적분:  90.36° | Odom:  94.50° | EKF:  91.82°
✅ IMU 목표 도달!
```

### Test 2/4: IMU 기준 시계 회전
```
진행  49.8% | IMU적분:  44.82° | Odom:  46.91° | EKF:  45.73°
진행  99.6% | IMU적분:  89.64° | Odom:  93.67° | EKF:  91.02°
✅ IMU 목표 도달!
```

### Test 3/4: Odom 기준 반시계 회전
```
진행  50.1% | Odom:  45.09° | IMU적분:  43.12° | EKF:  44.28°
진행 100.2% | Odom:  90.18° | IMU적분:  86.34° | EKF:  88.59°
✅ Odom 목표 도달!
```

### Test 4/4: Odom 기준 시계 회전
```
진행  49.9% | Odom:  44.91° | IMU적분:  42.88° | EKF:  44.01°
진행  99.8% | Odom:  89.82° | IMU적분:  85.96° | EKF:  88.11°
✅ Odom 목표 도달!
```

---

## 📈 결과 분석

### 1. 개별 측정값 비교

```
📊 IMU 기준 회전 결과
----------------------------------------------------------
목표:            90.00°

📍 측정값:
  IMU 적분:       90.36° (오차: +0.36°)
  Odom (raw):     94.50° (오차: +4.50°)
  EKF (융합):     91.82° (오차: +1.82°)

⚙️  Odom 원시값 (scale 적용 전):
   50.21° (측정 94.50° / 1.8819)

🔄 IMU vs Odom 비율:
  0.9562 (IMU 90.36° / Odom 94.50°)
```

**해석:**
- IMU 적분이 가장 정확 (오차 +0.36°)
- Odom이 과대 측정 (오차 +4.50°) → angular_scale 재조정 필요
- EKF가 중간값으로 융합 (오차 +1.82°)

### 2. EKF 성능 분석

```
3️⃣  EKF 성능 분석:
   IMU 기준 EKF 차이: 0.80° (반시계 vs 시계)
   Odom 기준 EKF 차이: 0.48° (반시계 vs 시계)
   EKF 전체 범위: 3.71° (최대 91.82° - 최소 88.11°)
   ✅ EKF가 일관되게 작동 (범위 < 10°)
```

**해석:**
- EKF가 방향에 관계없이 일관된 결과
- 전체 범위 < 10° → 센서 융합 성공
- IMU와 Odom 사이 적절한 균형

### 3. 센서 비대칭 분석

```
4️⃣  센서 비대칭 분석:
   IMU 적분 비대칭: 0.72° (반시계 90.36° vs 시계 89.64°)
   Odom 측정 비대칭: 0.83° (반시계 94.50° vs 시계 93.67°)
```

**해석:**
- 비대칭 < 1° → 센서 정렬 우수
- 반시계/시계 회전이 거의 동일
- 기계적 편향 없음

---

## 🔧 Angular Scale 조정

### 계산 방법

**IMU 기준 테스트에서:**
```
IMU 적분: 90.36° (실제 회전으로 간주)
Odom 측정: 94.50° (과대 측정)

필요한 보정:
new_scale = current_scale × (IMU / Odom)
new_scale = 1.8819 × (90.36 / 94.50)
new_scale = 1.8819 × 0.9562
new_scale = 1.7995
```

### 적용 방법

#### 방법 A: transbot_base 파라미터 (영구)

```bash
# 파라미터 파일 찾기
find ~/transbot_ws_ros2/src -name "*.yaml" -path "*/transbot_base/*"

# 수정 (예시)
# transbot_base/config/robot_params.yaml
transbot_driver:
  ros__parameters:
    angular_scale: 1.7995  # 1.8819 → 1.7995
```

#### 방법 B: Launch 파라미터 (임시 테스트)

```python
# transbot_full_system.launch.py
driver_node = Node(
    package='transbot_base',
    executable='driver_node',
    parameters=[{
        'angular_scale': 1.7995,  # 수정된 값
        # ... 기타 파라미터
    }]
)
```

### 검증

```bash
# 다시 테스트 실행
python3 ekf_comparison_test.py

# 기대 결과:
# Odom 측정: 90.xx° (오차 < 2°)
```

---

## ✅ 판단 기준

### 우수 (✅)
- **IMU-Odom 차이**: < 5°
- **EKF 범위**: < 10°
- **비대칭**: < 2°

### 양호 (⚠️)
- **IMU-Odom 차이**: 5~10°
- **EKF 범위**: 10~20°
- **비대칭**: 2~5°

### 불량 (❌)
- **IMU-Odom 차이**: > 10°
- **EKF 범위**: > 20°
- **비대칭**: > 5°

---

## 🐛 문제 해결

### 문제 1: IMU 적분이 목표에 도달하지 못함

**증상:**
```
진행  85.3% | IMU적분:  76.77° | ...
⚠️ 타임아웃!
```

**원인:** IMU 드리프트 또는 각속도 스케일 문제

**해결:**
1. IMU 재캘리브레이션
2. IMU angular_velocity scale 확인

---

### 문제 2: Odom 과대/과소 측정

**증상:**
```
Odom (raw):    105.23° (오차: +15.23°)  ← 과대
```

**원인:** angular_scale 부정확

**해결:**
- 스크립트 출력의 제안 값 사용
- 또는 수동 계산: `new_scale = old_scale × (IMU / Odom)`

---

### 문제 3: EKF 범위가 큼

**증상:**
```
EKF 전체 범위: 25.33° (최대 102.15° - 최소 76.82°)
❌ EKF 일관성 낮음 (범위 > 20°)
```

**원인:** 센서 신뢰도 설정 불균형

**해결:**
1. `ekf_config.yaml` 확인
2. IMU covariance 낮추기 (더 신뢰)
3. Odom covariance 높이기 (덜 신뢰)

---

### 문제 4: 비대칭이 큼

**증상:**
```
IMU 적분 비대칭: 8.52° (반시계 95.26° vs 시계 86.74°)
```

**원인:** 
- 바퀴 마찰 차이
- 기계적 편향
- 자기장 간섭 (IMU)

**해결:**
1. 바닥 상태 확인
2. 바퀴 압력 확인
3. IMU 주변 금속 제거

---

## 📚 관련 문서

- [센서 각도 정렬 완전 가이드](../summary/SENSOR_ANGLE_ALIGNMENT.md)
- [IMU & Odometry 캘리브레이션](../summary/01_IMU_ODOMETRY_CALIBRATION.md)
- [회전 정확도 개선](../summary/03_ROTATION_ACCURACY.md)
- [EKF 센서 퓨전](../summary/02_EKF_SENSOR_FUSION.md)

---

**작성일:** 2025-10-31  
**스크립트 버전:** 기존 검증 완료
