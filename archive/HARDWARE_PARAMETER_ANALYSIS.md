# 🔧 Transbot 하드웨어 파라미터 분석

## 🚨 **핵심 문제**

**증상:**
```
Odom 목표: 230.77° (회전 명령)
실제 측정 결과:
- Odom:   207.8° (90% 도달)
- IMU적분: 527.7° (229% 과다)
```

**비율 계산:**
```
IMU / Odom = 527.7° / 207.8° = 2.54배
```

**결론:** 오도메트리가 IMU 대비 **2.5배 과소 측정**

---

## 📊 문제 원인 분석

### 시나리오 1: 휠베이스 설정 오류 (가장 가능성 높음)
**원리:**
```
angular_velocity = (v_right - v_left) / wheelbase

실제 wheelbase > 설정된 wheelbase  →  angular 과소 측정
```

**예시:**
```
실제 wheelbase = 0.15m (추정)
설정 wheelbase = 0.06m (잘못된 값)
→ 각속도가 실제의 40%만 측정됨 (0.06/0.15 = 0.4)
```

---

### 시나리오 2: 엔코더 해상도 설정 오류
**원리:**
```
실제 엔코더 pulse/rev > 설정 값  →  거리/회전 과소 측정
```

**예시:**
```
실제 encoder = 1560 pulses/rev
설정 encoder = 624 pulses/rev (2.5배 차이)
→ 회전량이 실제의 40%만 측정됨 (624/1560 = 0.4)
```

---

### 시나리오 3: 바퀴 직경 설정 오류
**원리:**
```
실제 wheel_diameter > 설정 값  →  거리 과소 측정
→ 같은 엔코더 변화량에서 실제 이동 거리 더 큼
```

**예시:**
```
실제 diameter = 0.065m
설정 diameter = 0.026m (2.5배 차이)
→  이동 거리가 실제의 40%만 측정됨
```

---

## 🔍 확인해야 할 하드웨어 파라미터

### 1. 휠베이스 (Wheelbase)
**정의:** 좌우 바퀴 중심 간 거리  
**측정 방법:** 자로 직접 측정 (바퀴 중심 사이)  
**일반적 값:** 0.13m ~ 0.17m  

**코드 위치 찾기:**
```bash
# ROS1 코드에서 찾기
grep -r "wheelbase\|WHEELBASE\|wheel.*base" ~/transbot_ws/src/

# ROS2 코드에서 찾기
grep -r "wheelbase\|WHEELBASE\|wheel.*base" ~/transbot_ws_ros2/src/

# Python 라이브러리에서 찾기
grep -r "wheelbase\|WHEELBASE" ~/Transbot/
```

---

### 2. 바퀴 직경 (Wheel Diameter)
**정의:** 바퀴의 직경  
**측정 방법:**  
1. 바퀴 1회전 시 이동 거리 측정
2. diameter = 이동 거리 / π

**일반적 값:** 0.06m ~ 0.08m

**코드 위치:**
```bash
# 바퀴 관련 파라미터 찾기
grep -r "wheel.*diameter\|WHEEL_DIAMETER\|wheel_radius" ~/transbot_ws/src/
grep -r "wheel.*diameter\|WHEEL_DIAMETER\|wheel_radius" ~/transbot_ws_ros2/src/
```

---

### 3. 엔코더 해상도 (Encoder Resolution)
**정의:** 바퀴 1회전당 펄스 수  
**일반적 값:**  
- 저해상도: 330 pulses/rev
- 중해상도: 600-800 pulses/rev
- 고해상도: 1500-2000 pulses/rev

**코드 위치:**
```bash
# 엔코더 관련 파라미터 찾기
grep -r "encoder.*resolution\|ENCODER_RESOLUTION\|pulse.*per.*rev\|PPR" ~/transbot_ws/src/
grep -r "encoder.*resolution\|ENCODER_RESOLUTION\|pulse.*per.*rev" ~/Transbot/
```

---

## 🧮 계산식 확인

### 오도메트리 계산 공식

**선속도 계산:**
```python
# 왼쪽/오른쪽 바퀴 속도 (m/s)
v_left = (encoder_left_delta / encoder_resolution) * wheel_circumference / dt
v_right = (encoder_right_delta / encoder_resolution) * wheel_circumference / dt

# 로봇 중심 선속도
v_linear = (v_left + v_right) / 2
```

**각속도 계산 (핵심!)**
```python
# 로봇 중심 각속도 (rad/s)
v_angular = (v_right - v_left) / wheelbase
```

**만약 wheelbase가 실제보다 작으면:**
```
v_angular_measured = (v_right - v_left) / wheelbase_small
v_angular_actual = (v_right - v_left) / wheelbase_actual

→ v_angular_measured > v_angular_actual (과대 측정)

하지만 현재는 과소 측정이므로...
→ wheelbase_설정 > wheelbase_실제 (가능성 낮음)
```

**만약 encoder_resolution이 실제보다 작으면:**
```
# 엔코더 틱당 이동 거리가 크게 계산됨
distance_per_tick_설정 > distance_per_tick_실제

→ 같은 틱 수에서 더 많이 이동한 것으로 계산
→ 오도메트리 과대 측정

하지만 현재는 과소 측정이므로...
→ encoder_resolution_설정 > encoder_resolution_실제
```

---

## 📝 현재 시스템 값 역산

**IMU가 정확하다고 가정:**
```
IMU 적분: 527.7° (실제 회전량)
Odom 측정: 207.8° (시스템 측정값)

보정 계수 = 527.7 / 207.8 = 2.54
```

**현재 angular_scale = 1.56 적용 시:**
```
실제 측정값 = 207.8° * 1.56 = 324.2°
여전히 부족 (목표 527.7°)

필요한 scale = 527.7 / 207.8 = 2.54
```

**하지만 사용자가 2.4123을 사용하면:**
```
207.8° * 2.4123 = 501.4°
IMU 527.7°와 약간 차이 있지만 합리적
```

---

## 🔍 의심되는 문제

### 가설 1: Encoder Resolution 오류
**근거:**
- IMU/Odom 비율 = 2.54배
- 일반적으로 Transbot은 **520-600 PPR** 엔코더 사용
- 만약 시스템이 **1300-1500 PPR**로 설정되어 있다면:
  ```
  1300 / 520 = 2.5배 (일치!)
  ```

**확인 방법:**
```python
# Transbot_Lib.py에서 get_velocity() 함수 분석
# 속도 계산식에서 엔코더 상수 찾기
```

---

### 가설 2: Wheelbase 설정 오류
**근거:**
- 각속도 = (v_right - v_left) / wheelbase
- wheelbase가 2.5배 크게 설정되어 있다면:
  ```
  실제 wheelbase = 0.15m
  설정 wheelbase = 0.375m (2.5배)
  → 각속도가 0.4배만 계산됨
  ```

**가능성:** 낮음 (0.375m는 비현실적)

---

### 가설 3: 펌웨어 내부 스케일 계수
**근거:**
- Transbot_Lib.py에서 보면:
  ```python
  self.__velocity = float(struct.unpack('b', bytearray(ext_data[0:1]))[0]) / 100.0
  self.__angular = float(struct.unpack('h', bytearray(ext_data[1:3]))[0]) / 100.0
  ```
- 펌웨어에서 이미 `/100.0` 스케일링
- 펌웨어 내부에 추가 스케일 계수가 있을 수 있음

**확인 필요:**
- 펌웨어 버전 확인
- 제조사 스펙 문서 확인

---

## 🛠️ 해결 방안

### 방안 1: 물리적 측정 (권장)
```bash
# 1. 휠베이스 측정
# - 자로 좌우 바퀴 중심 간 거리 측정
# - 정확도: ±1mm

# 2. 바퀴 직경 측정
# - 바퀴 1회전 시 이동 거리 측정
# - diameter = 이동 거리 / π

# 3. 값을 코드에 반영
```

### 방안 2: 소프트웨어 캘리브레이션 (현재 사용 중)
```python
# angular_scale = 2.4123 사용
# 장점: 간단, 빠름
# 단점: 근본 원인 해결 X
```

### 방안 3: 펌웨어 확인
```bash
# Transbot 제조사 문서 확인
# - 엔코더 스펙
# - 휠베이스 스펙
# - 바퀴 직경 스펙
```

---

## ✅ 권장 조치

### 즉시 조치:
1. **angular_scale = 2.4123 적용** (이미 완료)
2. **테스트 및 검증**

### 추후 조치:
1. **Transbot 제조사 스펙 문서 확인**
   - 공식 wheelbase 값
   - 공식 wheel diameter 값
   - 공식 encoder resolution 값

2. **물리적 측정**
   ```bash
   # 휠베이스 측정
   실제 측정 → 코드 비교

   # 바퀴 직경 측정  
   1회전 거리 측정 → diameter 계산

   # 엔코더 검증
   10회전 후 엔코더 틱 수 확인
   ```

3. **코드 수정**
   ```python
   # 정확한 하드웨어 파라미터로 수정
   # angular_scale을 1.0에 가깝게 만들기
   ```

---

## 📚 참고 자료

### 코드 위치:
- **ROS2 Odom**: `/home/user/transbot_ws_ros2/src/transbot_base/src/base.cpp`
- **Python 드라이버**: `/home/user/transbot_ws_ros2/src/transbot_bringup/transbot_bringup/transbot_driver.py`
- **하드웨어 라이브러리**: `/home/user/Transbot/py_install/Transbot_Lib/Transbot_Lib.py`

### 캘리브레이션 결과:
- **Phase 2**: angular_scale = 2.4123
- **IMU/Odom 비율**: 2.54배
- **추정 원인**: Encoder resolution 또는 wheelbase 설정 오류

---

## 🎯 결론

**현재 상태:**
- ✅ angular_scale = 2.4123 적용으로 **임시 해결**
- ⚠️ 근본 원인 (하드웨어 파라미터 오류) **미해결**

**다음 단계:**
1. Transbot 제조사 스펙 문서 확인
2. 물리적 측정 (wheelbase, wheel diameter)
3. 펌웨어 버전 및 엔코더 스펙 확인
4. 정확한 파라미터로 코드 수정

**기대 효과:**
- angular_scale을 1.0에 가깝게 만들기
- 더 정확한 오도메트리
- IMU와 Odom 일치율 향상
