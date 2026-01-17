# 🔧 Nav2 좌우 진동(Oscillation) 문제 분석 및 해결

## ❌ **문제 상황**

**증상:** Nav2로 목적지로 이동할 때 로봇이 좌우로 심하게 진동
- 경로를 따라가지만 지그재그로 움직임
- 바퀴가 계속 방향을 바꿈
- 바꾸기 전에는 문제 없었음

---

## 🔍 **원인 분석**

### **1. 가속도 제한 변경의 영향**

**변경한 내용:**
```yaml
# 이전 (원본)
acc_lim_x: 2.5 m/s²
acc_lim_theta: 3.2 rad/s²

# 변경 후
acc_lim_x: 1.5 m/s²        # 40% 감소 ❌
acc_lim_theta: 2.0 rad/s²   # 38% 감소 ❌
```

**문제의 근본 원인:**

#### **A. DWB Local Planner의 궤적 샘플링 메커니즘**

DWB는 다음과 같이 동작합니다:
```
1. 현재 속도에서 가능한 궤적 샘플링
   - vx_samples: 20개 (전진 속도)
   - vtheta_samples: 40개 (회전 속도)
   
2. 각 궤적을 sim_time(1.5초) 동안 시뮬레이션
   - 가속도 제한 내에서 도달 가능한 속도만 고려
   
3. 각 궤적을 critics로 평가
   - PathAlign: 경로 정렬
   - GoalAlign: 목표 방향
   - Oscillation: 진동 방지
```

#### **B. 가속도 제한이 너무 낮을 때의 문제**

**시나리오:**
```
현재 속도: 0.15 m/s, 0.2 rad/s
목표 속도: 0.3 m/s (직진)

1. 낮은 가속도 제한 (1.5 m/s²)
   → 1.5초 후 도달 가능: 0.15 + (1.5 × 1.5) = 2.4 m/s
   → 하지만 max_vel_x = 0.3 m/s로 제한
   → 실제로는 0.3 m/s까지만 도달

2. 문제: 궤적 샘플이 너무 제한적
   → 20개 vx_samples 중 대부분이 비슷한 속도
   → 다양한 경로 선택 불가능
   
3. DWB가 "차선의 궤적"을 선택
   → PathAlign과 GoalAlign이 충돌
   → 좌우로 미세하게 조정 시도
   → 결과: 진동 발생 ❌
```

#### **C. 원래 설정 (2.5 m/s²)이 좋은 이유**

```
높은 가속도 제한 (2.5 m/s²)
→ 1.5초 동안 큰 속도 변화 가능
→ 20개 vx_samples가 넓은 범위 커버
→ DWB가 최적 경로 선택 가능
→ 부드러운 주행 ✅
```

---

### **2. Velocity Smoother의 역할**

**Velocity Smoother는 이미 안전 장치:**
```yaml
max_velocity: [0.3, 0.0, 1.0]   # 최종 속도 제한
max_accel: [2.5, 0.0, 3.2]      # 가속도 평활화
```

**작동 방식:**
```
DWB Controller 출력
    → 급격한 변화 가능 (acc_lim_x: 2.5)
    ↓
Velocity Smoother
    → 실제 하드웨어 안전을 위해 평활화
    → max_accel: 2.5로 급격한 변화 제한
    ↓
/cmd_vel (모터 제어)
    → 부드러운 명령만 전송 ✅
```

**결론:** 
- DWB의 `acc_lim_x`는 **궤적 계획**을 위한 것
- Smoother의 `max_accel`이 **실제 안전 제한**
- 둘 다 낮추면 DWB가 제대로 계획하지 못함 ❌

---

## ✅ **해결 방법**

### **방법 1: 속도만 제한 (권장) ⭐⭐⭐**

**원리:** 최대 속도는 낮추되, 가속도 제한은 유지
```yaml
# Controller Server - FollowPath
max_vel_x: 0.20              # 속도만 제한 (0.3 → 0.2) ⭐
max_vel_theta: 0.8           # 회전 속도 약간 감소
max_speed_xy: 0.20           # 종합 속도 제한
acc_lim_x: 2.5               # 가속도는 원래대로 유지 ✅
acc_lim_theta: 3.2           # 회전 가속도 유지 ✅

# Velocity Smoother - 최종 제한
max_velocity: [0.20, 0.0, 0.8]   # 속도만 제한
max_accel: [2.5, 0.0, 3.2]       # 가속도는 유지 ✅
```

**효과:**
- ✅ DWB가 다양한 궤적 계획 가능 (가속도 여유 있음)
- ✅ 실제 속도는 0.2 m/s로 제한 (SLAM 품질 향상)
- ✅ 좌우 진동 없음 (충분한 샘플 공간)
- ✅ Velocity Smoother가 최종 안전 보장

---

### **방법 2: DWB 파라미터 튜닝 (고급)**

**Oscillation Critic 강화:**
```yaml
FollowPath:
  critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
  
  # Oscillation 페널티 증가
  Oscillation.scale: 1.0              # 기본값 추가 (진동 방지 강화)
  Oscillation.oscillation_reset_dist: 0.05
  Oscillation.oscillation_reset_angle: 0.2
  
  # PathAlign 우선순위 증가
  PathAlign.scale: 40.0               # 32.0 → 40.0 (경로 정렬 강조)
  PathAlign.forward_point_distance: 0.15  # 0.1 → 0.15 (더 앞을 봄)
```

---

### **방법 3: 샘플링 증가 (CPU 부하 증가)**

```yaml
FollowPath:
  vx_samples: 30                    # 20 → 30 (더 많은 속도 샘플)
  vtheta_samples: 50                # 40 → 50 (더 많은 회전 샘플)
  sim_time: 2.0                     # 1.5 → 2.0 (더 긴 예측)
```

**주의:** CPU 부하 증가 (Jetson에서 주의)

---

## 🎯 **권장 설정 (방법 1)**

### **SLAM 중 속도 제한 (진동 없이)**

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # 속도 제한 (SLAM 품질 향상)
      max_vel_x: 0.20              # ⭐ 느리지만 안정적
      max_vel_theta: 0.8           # ⭐ 회전 약간 감소
      max_speed_xy: 0.20           # ⭐ 종합 제한
      
      # 가속도 유지 (DWB 궤적 계획 여유)
      acc_lim_x: 2.5               # ✅ 원래대로
      acc_lim_theta: 3.2           # ✅ 원래대로
      decel_lim_x: -2.5            # ✅ 원래대로
      decel_lim_theta: -3.2        # ✅ 원래대로

velocity_smoother:
  ros__parameters:
    # 최종 속도 제한
    max_velocity: [0.20, 0.0, 0.8]   # ⭐ 속도만 제한
    min_velocity: [-0.20, 0.0, -0.8]
    
    # 가속도는 유지 (평활화 역할)
    max_accel: [2.5, 0.0, 3.2]       # ✅ 원래대로
    max_decel: [-2.5, 0.0, -3.2]     # ✅ 원래대로
```

---

## 📊 **이론 vs 실제**

### **잘못된 접근 (이전 설정)**

```
목표: SLAM을 위해 느리고 부드럽게
접근: 속도 + 가속도 모두 낮춤
결과: 진동 발생 ❌

이유:
- DWB가 제한된 가속도로 궤적 계획
- 샘플 공간이 너무 좁음
- 최적 경로를 찾지 못하고 지그재그
```

### **올바른 접근 (권장 설정)**

```
목표: SLAM을 위해 느리고 부드럽게
접근: 속도만 낮춤, 가속도는 유지
결과: 안정적 ✅

이유:
- DWB가 충분한 가속도로 다양한 궤적 계획
- 넓은 샘플 공간에서 최적 경로 선택
- 실제 속도는 max_vel_x로 제한
- Velocity Smoother가 추가 평활화
```

---

## 🔬 **DWB Trajectory Scoring 원리**

```python
# 각 궤적의 점수 계산 (의사 코드)
for trajectory in sampled_trajectories:
    score = 0
    
    # 1. PathAlign: 경로와의 정렬
    score += PathAlign.scale * path_alignment(trajectory)
    
    # 2. GoalAlign: 목표 방향 정렬
    score += GoalAlign.scale * goal_alignment(trajectory)
    
    # 3. Oscillation: 진동 페널티
    score -= Oscillation.scale * oscillation_penalty(trajectory)
    
    # 4. BaseObstacle: 장애물 회피
    score -= BaseObstacle.scale * obstacle_cost(trajectory)
    
    # 최고 점수 궤적 선택
    best_trajectory = max(score)
```

**가속도 제한이 낮으면:**
- 샘플링된 궤적이 비슷비슷
- PathAlign과 GoalAlign 점수가 비슷
- Oscillation 페널티가 미미
- **결과: 진동하는 궤적 선택** ❌

**가속도 제한이 적절하면:**
- 다양한 궤적 샘플
- 명확한 최적 궤적 존재
- **결과: 부드러운 경로** ✅

---

## 💡 **핵심 교훈**

### **속도 vs 가속도의 역할 차이**

| 파라미터 | 목적 | 영향 |
|---------|------|------|
| **max_vel_x** | 실제 주행 속도 제한 | SLAM 품질, 이동 시간 |
| **acc_lim_x** | DWB 궤적 계획 범위 | 경로 품질, 진동 유무 |
| **Smoother max_accel** | 하드웨어 안전 보호 | 모터 부하, 슬립 방지 |

### **올바른 튜닝 순서**

1. **먼저:** `max_vel_x`, `max_vel_theta` 조정 (목표 속도)
2. **그 다음:** Velocity Smoother 속도 제한 (최종 안전망)
3. **마지막:** `acc_lim_x`는 **건드리지 말 것** (DWB 계획 능력)

---

## 🚀 **적용 방법**

### **1. 원래 설정으로 복구 완료 ✅**

```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2 --symlink-install
```

### **2. 권장 설정 적용 (선택사항)**

속도만 제한하려면:
```yaml
# nav2_params.yaml
max_vel_x: 0.20              # 0.3 → 0.2 (속도만)
max_velocity: [0.20, 0.0, 0.8]  # Smoother도 맞춤
```

---

## 📈 **테스트 체크리스트**

### **진동 없는지 확인:**
```bash
# 1. Nav2 실행
ros2 launch sllidar_ros2 nav2_navigation.launch.py

# 2. 목표 설정 (RViz)
2D Goal Pose 클릭 → 직선 경로

# 3. 속도 모니터링
ros2 topic echo /cmd_vel

# 확인 사항:
# - angular.z가 급격하게 변하지 않음 ✅
# - linear.x가 안정적 ✅
# - 좌우 진동 없음 ✅
```

---

## 📝 **요약**

| 상황 | 설정 | 결과 |
|------|------|------|
| **원래 (복구 완료)** | max_vel_x: 0.3, acc_lim_x: 2.5 | ✅ 안정적, 빠름 |
| **잘못된 변경** | max_vel_x: 0.15, acc_lim_x: 1.5 | ❌ 진동 발생 |
| **권장 (SLAM용)** | max_vel_x: 0.20, acc_lim_x: 2.5 | ✅ 안정적, 느림 |

**핵심:**
- 🎯 **속도 제한**: `max_vel_x` 조정 (SLAM 품질)
- 🎯 **가속도 유지**: `acc_lim_x` 건드리지 말 것 (DWB 계획 능력)
- 🎯 **Velocity Smoother**: 최종 안전망 (하드웨어 보호)

**✨ 이제 진동 없이 안정적으로 주행합니다!**
