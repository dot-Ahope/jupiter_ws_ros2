# SLAM 비교: SLAM Toolbox vs RTAB-Map

## 🎯 빠른 비교표

| 특성 | SLAM Toolbox | RTAB-Map |
|------|--------------|----------|
| **난이도** | ⭐ 쉬움 | ⭐⭐⭐ 중간 |
| **설정 복잡도** | 간단 | 복잡 |
| **메모리 사용** | 낮음 (200MB) | 중간 (300-400MB) |
| **CPU 사용** | 낮음 (15-20%) | 중간 (25-30%) |
| **맵 정확도** | ⭐⭐⭐ 좋음 | ⭐⭐⭐⭐ 우수 |
| **루프 클로저** | ⭐⭐ 기본 | ⭐⭐⭐⭐⭐ 매우 우수 |
| **큰 환경** | ⭐⭐ 보통 | ⭐⭐⭐⭐⭐ 매우 좋음 |
| **Nav2 통합** | ⭐⭐⭐⭐⭐ 완벽 | ⭐⭐⭐ 보통 |
| **3D 지원** | ❌ 없음 | ✅ RGB-D 카메라 필요 |
| **로컬라이제이션** | ⭐⭐⭐ 보통 | ⭐⭐⭐⭐⭐ 매우 우수 |

## 🚀 실행 명령 비교

### SLAM Toolbox
```bash
ros2 launch transbot_nav transbot_full_system.launch.py
```

### RTAB-Map
```bash
ros2 launch transbot_nav transbot_rtabmap.launch.py
```

## 📊 언제 무엇을 사용할까?

### SLAM Toolbox를 사용해야 할 때 ✅

1. **Nav2와 긴밀한 통합 필요**
   - Nav2 기본 SLAM 솔루션
   - 설정 없이 바로 작동

2. **리소스가 제한적**
   - Jetson Nano 같은 임베디드 시스템
   - 메모리/CPU 절약 필요

3. **간단한 환경**
   - 작은 방, 복도
   - 루프 클로저 불필요

4. **빠른 프로토타입**
   - 설정이 단순
   - 디버깅 쉬움

### RTAB-Map을 사용해야 할 때 ✅

1. **큰 환경 매핑**
   - 여러 방, 층
   - 긴 복도, 넓은 공간

2. **루프 클로저 중요**
   - 반복 방문 환경
   - 누적 오차 보정 필요

3. **정확한 맵 필요**
   - 산업용, 상업용 애플리케이션
   - 높은 정확도 요구

4. **3D 매핑 계획**
   - 향후 RGB-D 카메라 추가
   - 3D 지도 필요

5. **로컬라이제이션 모드**
   - 기존 맵에서 위치 추정
   - 맵 업데이트 불필요

## 🧪 성능 테스트 시나리오

### 시나리오 1: 작은 방 (5m × 5m)

**SLAM Toolbox**
- ✅ 빠른 매핑 (30초)
- ✅ 낮은 리소스 사용
- ⚠️ 약간의 각도 드리프트

**RTAB-Map**
- ✅ 정확한 맵
- ⚠️ 오버킬 (과한 성능)
- ⚠️ 높은 리소스 사용

**추천**: SLAM Toolbox 👍

---

### 시나리오 2: 긴 복도 (50m)

**SLAM Toolbox**
- ⚠️ 누적 오차 증가
- ⚠️ 끝부분 정렬 오류
- ❌ 루프 클로저 실패

**RTAB-Map**
- ✅ 정확한 맵 유지
- ✅ 루프 클로저로 보정
- ✅ 일정한 메모리 사용

**추천**: RTAB-Map 👍

---

### 시나리오 3: 여러 방 + 반복 방문

**SLAM Toolbox**
- ❌ 같은 방을 다른 방으로 인식
- ❌ 맵이 계속 커짐
- ⚠️ 각도 누적 오차

**RTAB-Map**
- ✅ 같은 방 인식 (루프 클로저)
- ✅ 맵 최적화
- ✅ 정확한 위치 추정

**추천**: RTAB-Map 👍👍

---

### 시나리오 4: Jetson Nano + 배터리 동작

**SLAM Toolbox**
- ✅ 낮은 전력 소비
- ✅ 안정적 동작
- ✅ 긴 배터리 수명

**RTAB-Map**
- ⚠️ 높은 전력 소비
- ⚠️ 발열 증가
- ⚠️ 짧은 배터리 수명

**추천**: SLAM Toolbox 👍

## 🔧 설정 파일 비교

### SLAM Toolbox 주요 설정
```yaml
# slam_toolbox_params.yaml
solver_plugin: solver_plugins::CeresSolver
mode: mapping
resolution: 0.05
max_laser_range: 5.0
```

### RTAB-Map 주요 설정
```yaml
# rtabmap_params.yaml
Reg/Strategy: "1"              # ICP
Reg/Force3DoF: "true"          # 2D
Icp/VoxelSize: "0.05"
Rtabmap/DetectionRate: "1.0"
Grid/CellSize: "0.05"
```

## 📈 맵 품질 비교 (예상)

### SLAM Toolbox
- **벽 두께**: 5-10cm (단일 스캔)
- **각도 정확도**: ±2-5도 (angular_scale 보정 전)
- **위치 정확도**: ±5-10cm
- **루프 클로저**: 때때로 실패

### RTAB-Map
- **벽 두께**: 3-5cm (ICP 정렬)
- **각도 정확도**: ±0.5-1도 (그래프 최적화)
- **위치 정확도**: ±2-5cm
- **루프 클로저**: 거의 항상 성공

## 🎮 Nav2 통합 차이

### SLAM Toolbox
```bash
# 1. SLAM 실행
ros2 launch transbot_nav transbot_full_system.launch.py

# 2. Nav2 바로 사용 가능 (동일 launch 파일에 포함)
# -> 별도 작업 불필요
```

### RTAB-Map
```bash
# 1. SLAM 실행
ros2 launch transbot_nav transbot_rtabmap.launch.py

# 2. Nav2 별도 실행 필요
ros2 launch nav2_bringup navigation_launch.py \
  map:=/map \
  params_file:=nav2_params.yaml
```

## 💡 권장 사항

### 초보자 / 프로토타입
```
SLAM Toolbox 👍
- 설정 간단
- Nav2 통합 우수
- 리소스 효율적
```

### 고급 사용자 / 프로덕션
```
RTAB-Map 👍
- 정확도 우수
- 큰 환경 적합
- 로컬라이제이션 모드
```

### 하이브리드 접근
```
1. 개발/테스트: SLAM Toolbox
2. 최종 매핑: RTAB-Map
3. 운영: RTAB-Map Localization 모드
```

## 🔄 전환 가이드

### SLAM Toolbox → RTAB-Map 전환

1. **맵 생성**
```bash
# SLAM Toolbox로 빠르게 초기 맵 생성
ros2 launch transbot_nav transbot_full_system.launch.py
# -> maps/my_map.yaml, my_map.pgm
```

2. **정밀 매핑**
```bash
# RTAB-Map으로 정밀한 최종 맵 생성
ros2 launch transbot_nav transbot_rtabmap.launch.py \
  database_path:=~/maps/final_map.db
```

3. **운영**
```bash
# RTAB-Map Localization 모드
ros2 launch transbot_nav transbot_rtabmap.launch.py \
  localization:=true \
  database_path:=~/maps/final_map.db
```

## 📚 학습 곡선

```
난이도
  ^
  |                    🔺 RTAB-Map (고급)
  |              🔺
  |        🔺
  |  🔺 SLAM Toolbox (기본)
  +-------------------------> 시간
```

## ✅ 결론

**당신의 상황에 맞는 선택**

| 상황 | 추천 |
|------|------|
| 처음 시작 | SLAM Toolbox |
| 작은 환경 | SLAM Toolbox |
| 큰 환경 | RTAB-Map |
| 정확도 중요 | RTAB-Map |
| 리소스 제한 | SLAM Toolbox |
| 루프 클로저 필요 | RTAB-Map |
| Nav2 사용 | SLAM Toolbox |
| 3D 매핑 계획 | RTAB-Map |

**우리 시스템(Transbot) 추천**:
- 테스트/개발: **SLAM Toolbox** (현재 사용 중)
- 프로덕션: **RTAB-Map** (새로 추가됨)

---

**작성일**: 2025-01-XX  
**버전**: 1.0
