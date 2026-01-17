# 진짜 문제 분석: Odom 측정값 vs 물리적 회전 vs 시스템 판단

## 🎯 **상황 재정리**

### **사실 관계:**

```
✅ angular_scale=1.5618 일 때:
   - 물리적: 90° 회전 (정확!)
   - Odom 측정: 40-50° 정도 측정
   - 시스템 판단: "아직 90°에 안 도달했네" → 계속 회전 지시

❌ Phase 2 (scale=1.0, launch_scale=1.5618) 결과:
   - Odom 목표: 141° (90° × 1.5618)
   - Odom 측정: 147° (목표 도달 ✅)
   - 물리적: ~140° 회전 (90°보다 과대회전!)
   - 시스템 판단: "141° 도달했다!" → 회전 멈춤
```

---

## 🔍 **진짜 문제: 측정 스케일 vs 제어 목표의 불일치**

### **데이터 흐름 분석:**

```
1. 하드웨어 → base_node:
   msg->angular.z (원시 각속도)
   
2. base_node 증폭:
   angular_velocity_z_ = msg->angular.z × 1.5618
   heading_ += angular_velocity_z_ × dt
   
3. /odom_raw 발행:
   odom.pose.pose.orientation (heading_을 quaternion으로)
   → 1.5618배 증폭된 각도
   
4. 제어 시스템 (rover.py 등):
   목표: 90° 회전
   측정: /odom_raw의 heading_ 읽음
   판단: heading_ >= 90° ? 멈춤 : 계속
```

---

## 📊 **실제 동작 시나리오**

### **시나리오 1: angular_scale=1.5618 (현재)**

```
명령: "90° 회전하라"

0.0초: 목표 90°, Odom 0°, 물리적 0°
      → 오차 90°, 회전 시작

1.0초: 하드웨어 16.6° 회전 (물리적)
      → Odom 측정: 16.6 × 1.5618 = 26°
      → 오차 64° (90-26), 계속 회전

2.0초: 하드웨어 33.2° 회전 (물리적)
      → Odom 측정: 33.2 × 1.5618 = 52°
      → 오차 38° (90-52), 계속 회전

3.0초: 하드웨어 49.8° 회전 (물리적)
      → Odom 측정: 49.8 × 1.5618 = 78°
      → 오차 12° (90-78), 계속 회전

3.7초: 하드웨어 57.6° 회전 (물리적) ⭐
      → Odom 측정: 57.6 × 1.5618 = 90° ⭐
      → 오차 0°, 회전 멈춤!

결과:
  - Odom은 90° 측정했다고 생각 ✅
  - 실제로는 57.6° 물리적 회전 ❌
  - 목표 90°보다 부족! (64% 달성)
```

### **시나리오 2: Phase 2 실험 (scale=1.0 실행)**

```
Calibration 스크립트 내부 동작:

목표: 물리적 90° 회전
계산된 Odom 목표: 90 × (1.5618/1.0) = 140.6°

0.0초: Odom 0°, 물리적 0°
      → 회전 시작

...

5.7초: 물리적 90° 회전
      → Odom 측정: 90 × 1.5618 = 140.6° ⭐
      → 목표 도달! 회전 멈춤

결과:
  - 물리적: 90° 정확 ✅
  - Odom 측정: 140.6° ✅
  - 스크립트가 목표를 1.5618배 상향하여 보정
```

### **시나리오 3: Phase 2에서 scale=1.0 사용 문제**

```
스크립트는 scale=1.0이라고 했지만:
  - launch에 이미 angular_scale=1.5618 적용됨
  - Odom 목표: 90 × 1.5618 = 140.6°
  
실행:
  물리적 90° 회전 → Odom 140.6° 측정 → 멈춤 ✅
  
하지만 스크립트가 angular_scale 계산:
  angular_scale = IMU적분 / (Odom측정 / 1.5618)
  angular_scale = 362° / (147° / 1.5618)
  angular_scale = 362° / 94° = 3.85
  
이게 왜 문제?
  - 3.85는 "원시 하드웨어 → 물리적 회전" 비율
  - 하지만 launch에 이미 1.5618이 적용되어 있음
  - 총 배율: 1.5618 × 2.46 = 3.85
  - 즉, 1.5618 → 3.85로 2.46배 더 증폭해야 함
```

---

## 🚨 **핵심 문제: 2가지 충돌**

### **문제 1: 측정 스케일 ≠ 제어 목표**

```
현재 시스템 (angular_scale=1.5618):
  
  제어 명령: "90° 회전하라"
  제어 로직: Odom >= 90° 되면 멈춰라
  
  Odom 측정: 하드웨어 × 1.5618
  
  결과:
    하드웨어 57.6° → Odom 90° → 멈춤
    물리적 회전 부족!
```

**왜 이런 문제가 발생?**
```
base_node가 /odom_raw에 증폭된 값 발행
  ↓
제어 시스템이 /odom_raw를 "진짜 회전량"으로 간주
  ↓
"90° 측정되면 물리적으로 90° 돌았겠지" (착각!)
  ↓
실제로는 57.6° 물리적 회전 (1.5618배 과대평가)
```

---

### **문제 2: Calibration 스크립트의 오해**

```
스크립트 가정:
  "launch_scale이 적용되어 있으니 보정해야지"
  odom_raw = odom_measured / launch_scale
  
실제:
  launch_scale=1.5618이 정확한 물리적 배율!
  나눠버리면 오히려 왜곡!

Phase 2 결과 angular_scale=3.85의 의미:
  - 원시 하드웨어 26° → 물리적 90° (3.46배)
  - 현재 1.5618 적용 → 40° 측정
  - 40° → 90° 물리적 달성하려면 2.25배 더 필요
  - 하지만 스크립트는 1.5618로 나눠서 3.85 계산
```

---

## 🎯 **올바른 이해**

### **angular_scale=1.5618의 실제 의미:**

```
"하드웨어 측정값을 1.5618배 증폭하면 물리적 회전량"

하드웨어 원시: 26° (심각한 언더측정)
  ↓ ×1.5618
Odom 측정: 40° (여전히 부족하지만 개선)
  ↓ ×2.25 (추가 필요)
물리적 회전: 90° ✅

총 배율: 26 × 1.5618 × 2.25 = 90
→ 26 × 3.51 = 90 ✅
```

---

## 📐 **Phase 2 결과 재해석**

### **실제로 일어난 일:**

```
Test: 90° CCW (scale=1.0, launch_scale=1.5618)

스크립트 목표 설정:
  odom_target = 90 × (1.5618 / 1.0) = 140.6°

실행:
  하드웨어 회전 → Odom 측정 147° → 멈춤
  물리적 회전: 147 / 1.5618 = 94° (거의 정확!)
  
하지만 스크립트 계산:
  Odom 원시 = 147 / 1.5618 = 94°
  angular_scale = 362 (IMU) / 94 = 3.85
  
이 3.85의 의미:
  "현재 1.5618에서 3.85로 바꾸면 정확해진다"
  배율 증가: 3.85 / 1.5618 = 2.46배
```

---

## 🔧 **진짜 해결책**

### **문제의 본질:**

```
❌ 잘못된 생각:
   "Odom이 90° 측정하면 물리적으로 90° 돌았다"
   
✅ 올바른 생각:
   "Odom이 X° 측정했으면 물리적으로 X/1.5618° 돌았다"
```

### **해결 방법:**

#### **방법 1: 제어 목표 보정 (권장)**

```python
# rover.py 같은 제어 스크립트
class RobotController:
    def __init__(self):
        self.angular_scale = 1.5618  # launch와 동일
    
    def rotate(self, target_physical_degrees):
        # 물리적 목표를 Odom 목표로 변환
        odom_target = target_physical_degrees * self.angular_scale
        
        while True:
            current_odom = self.get_odom_heading()
            if abs(current_odom - odom_target) < tolerance:
                break
            
            # 계속 회전...

# 사용:
robot.rotate(90)  # 물리적 90° 회전
# → Odom 목표: 90 × 1.5618 = 140.6°
# → 물리적 90° 도달 시 Odom 140.6° 측정
# → 정확히 멈춤! ✅
```

#### **방법 2: angular_scale 업데이트**

```yaml
# transbot_full_system.launch.py
'angular_scale': 3.85,  # Phase 2 결과 (1.5618 → 3.85)

효과:
  하드웨어 26° → Odom 100° (26 × 3.85)
  
  제어: "90° 회전하라"
  하드웨어 23.4° 회전 (물리적)
  → Odom 90° 측정 (23.4 × 3.85)
  → 멈춤
  
  결과: 물리적 23.4° 회전 ❌ (더 부족해짐!)
```

**왜 방법 2가 잘못?**
```
angular_scale을 높이면:
  - Odom 측정값이 더 커짐
  - 같은 목표(90°)에 더 빨리 도달
  - 물리적 회전은 더 줄어듦!
  
현재: 하드웨어 57.6° → Odom 90° (부족)
증폭: 하드웨어 23.4° → Odom 90° (더 부족!)

반대 방향으로 가고 있음!
```

---

## ✅ **정확한 해결책**

### **단계 1: 현재 상태 이해**

```
angular_scale = 1.5618 (현재)
물리적 90° 회전 달성 ✅

하지만:
  Odom은 140° 측정 (90 × 1.5618)
  제어 시스템이 "90° 목표"를 "Odom 90°"로 해석
  → 물리적 57.6° 회전에서 멈춤 ❌
```

### **단계 2: 제어 시스템 수정**

```python
# 모든 제어 스크립트에 보정 추가

ANGULAR_SCALE = 1.5618  # launch 설정과 동일

def rotate_physical(degrees):
    """물리적 각도만큼 회전"""
    odom_target = degrees * ANGULAR_SCALE
    # Odom이 odom_target 도달할 때까지 회전

def rotate_odom(degrees):
    """Odom 측정값 기준 회전 (물리적으로는 부족)"""
    odom_target = degrees
    # 기존 방식 (문제 있음)
```

### **단계 3: Calibration 스크립트 수정**

```python
# odom_based_angular_calibration.py

# 잘못된 부분 (Line 289-295):
odom_rotation_raw = odom_rotation / self.current_launch_scale
# launch_scale이 이미 물리적 보정이므로 나누면 안 됨!

# 올바른 계산:
odom_rotation_physical = odom_rotation  # 이미 물리적 의미
angular_scale_integrated = imu_integrated_deg / odom_rotation
# IMU / Odom의 직접 비율
```

---

## 🎯 **결론**

### **현재 상황:**

```
✅ angular_scale=1.5618은 정확함
   - 물리적 90° 회전 달성
   
❌ 제어 시스템이 Odom 스케일 모름
   - "90° 회전하라" → Odom 90° 목표
   - 실제로는 물리적 57.6° 회전
```

### **해결:**

```
1. 제어 목표 보정:
   physical_target × angular_scale = odom_target
   
2. Calibration 스크립트 수정:
   launch_scale로 나누지 말고 직접 비율 계산
   
3. angular_scale 재측정 (선택):
   더 정확한 값 찾기 (현재 1.5618 이미 좋음)
```

### **다음 작업:**

```
1. rover.py 같은 제어 스크립트 찾아서 수정
2. odom_based_angular_calibration.py 계산식 수정
3. 실제 90° 회전 테스트
```

**핵심: angular_scale은 맞지만, 제어 목표가 스케일을 반영 안 함!** 🎯
