# 🔍 270도 회전 문제 근본 원인 분석

## 📊 테스트 데이터 (rotation_test_20251016_161449.csv)

### 측정 결과
```
목표:          90° 회전
────────────────────────────────────────
odom_raw:      123.97° (33.97° 오차)
odom_filtered: 121.68° (31.68° 오차)
실제 물리:     270° 회전 ❌❌❌
────────────────────────────────────────
비율: 실제 / 측정 = 270° / 124° = 2.18배
```

### 센서 데이터 상세

#### 1. odom_raw (휠 오도메트리)
```
시간(s)  yaw(°)   angular_z(rad/s)  상태
─────────────────────────────────────────────
0.059     0.00     0.000            정지
0.885     0.00     0.000            정지 유지
1.092     1.95     0.340            회전 시작
1.504    18.88     0.750            가속 완료
2.949    78.64     0.780            목표 근접
3.156    91.73     0.750            목표 초과
3.467   105.47     0.760            계속 회전
3.672   113.90     0.710            감속 시작
3.881   120.41     0.420            급감속
4.086   123.29     0.210            거의 정지
4.291   123.97     0.010            최종 정지
```

**특징**:
- ✅ 부드러운 가속/감속
- ✅ 역회전 없음 (이전 개선 효과)
- ✅ 안정적인 제어
- ❌ 측정값 약 124°, 하지만 실제는 270°

#### 2. odom_filtered (EKF 융합)
```
시간(s)  yaw(°)   angular_z(rad/s)  상태
─────────────────────────────────────────────
0.111    -0.09    -0.000176        초기
0.936    -0.09    -0.000133        거의 변화 없음
3.313    -0.08     0.000101        여전히 0° 근처
3.518    56.35     0.760001        갑자기 점프!
3.724    64.73     0.710001        추적 시작
4.343    89.13     0.710000        거의 90°
4.548    97.27     0.710000        약간 초과
5.166   121.68     0.710000        최종 값
```

**특징**:
- 3.5초까지 거의 무시 (0° → -0.08°)
- 3.5초에 **56.35°로 점프**
- 이후 안정적으로 추적
- 최종: 121.68° (odom_raw와 유사)

#### 3. IMU 각속도
```
시간(s)  angular_z(rad/s)  상태
────────────────────────────────
0.162     0.000293         정지
0.368     2.950566         회전 인식
1.195     3.376906         안정적
2.435     3.199442         계속 3.2 rad/s
2.846     3.174394         유지
3.052     2.674511         감속
3.364     0.589175         거의 정지
3.570    -0.003437         완전 정지
```

**분석**:
- 명령: 0.2 rad/s
- IMU 측정: 3.2~3.4 rad/s
- **비율: 16~17배 증폭!**

## 🔍 보정 파라미터 분석

### 1. 캘리브레이션 파라미터 (ROS2)

#### A. `calibrate_angular_params.yaml`
```yaml
odom_angular_scale_correction: 0.75
```

**의미**:
- 측정값에 0.75를 곱해서 실제 값 추정
- 예: 측정 120° × 0.75 = 90° (실제 예상)

**실제 적용**:
```python
# calibrate_angular.py Line 94
delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
```

**문제**: 이 값은 `calibrate_angular.py` 스크립트에서만 사용되고, **실제 드라이버나 오도메트리 계산에는 적용되지 않음!**

#### B. `bringup.launch.py`
```python
# Line 152: angular_scale 설정
'angular_scale': 1.08,  # ROS1 캘리브레이션 값
```

**적용 위치**: `imu_filter_madgwick` 노드
- IMU 각속도에 1.08 곱함
- 3.2 rad/s × 1.08 = 3.456 rad/s

**문제**: IMU를 더 증폭시킴!

#### C. `base_node` - linear_scale
```python
# Line 114: linear_scale 파라미터
parameters=[{'linear_scale': 1.2, 'is_multi_robot': False}]
```

**적용 위치**: `base.cpp` Line 43
```cpp
linear_velocity_x_ = msg->linear.x * linear_scale_;
linear_velocity_y_ = msg->linear.y * linear_scale_;
angular_velocity_z_ = msg->angular.z;  // angular는 곱하지 않음!
```

**문제**: 
- **angular_velocity_z에는 linear_scale이 적용되지 않음**
- 각속도 보정이 전혀 없음!

### 2. ROS1 캘리브레이션 값 (비교)

#### `transbot_ws/src/transbot_bringup/launch/bringup.launch`
```xml
<param name="angular_scale" value="1.08"/>
<param name="linear_scale" type="double" value="1.2"/>
```

**ROS1과 ROS2 비교**:
```
파라미터           ROS1    ROS2    적용 여부
─────────────────────────────────────────────
angular_scale      1.08    1.08    ✓ (IMU 필터)
linear_scale       1.2     1.2     ✓ (base_node)
angular correction 없음    0.75    ✗ (미적용)
```

## 🔧 드라이버 속도 변환 분석

### `transbot_driver.py` - cmd_vel_callback

```python
# Line 173-174
base_speed = linear_x * (100.0 / 0.2)   # 0.2 m/s → 100%
turn_speed = angular_z * (100.0 / 0.5)  # 0.5 rad/s → 100%
```

**계산 예시 (명령: 0.2 rad/s)**:
```
turn_speed = 0.2 * (100.0 / 0.5)
           = 0.2 * 200
           = 40% PWM
```

**문제**:
- 40% PWM이 실제로 어느 정도 각속도를 만드는지 알 수 없음
- 엔코더가 없어서 피드백 없음
- PWM과 실제 속도의 관계가 비선형

### `Transbot_Lib.py` - set_car_motion

```python
# Line 289-312
def set_car_motion(self, velocity, angular):
    # velocity=[-0.45, 0.45], angular=[-2.00, 2.00]
    velocity_params = bytearray(struct.pack('h', int(velocity * 100)))
    angular_params = bytearray(struct.pack('h', int(angular * 100)))
    # 하드웨어로 전송
```

**변환**:
- velocity: m/s × 100 (정수로 변환)
- angular: rad/s × 100 (정수로 변환)
- 예: 0.2 rad/s → 20

**문제**: 하드웨어가 이 값을 어떻게 해석하는지 불명확

## 💡 근본 원인 분석

### 원인 1: **angular_scale_correction이 적용되지 않음** ⚠️⚠️⚠️

**발견**:
```yaml
# calibrate_angular_params.yaml
odom_angular_scale_correction: 0.75
```

**적용 위치**:
- ✅ `calibrate_angular.py` 스크립트 (테스트용)
- ❌ `transbot_driver.py` (드라이버)
- ❌ `base.cpp` (오도메트리 계산)
- ❌ `Transbot_Lib.py` (하드웨어 통신)

**결과**:
- 캘리브레이션 값이 실제 주행에 반영되지 않음
- 테스트 스크립트에서만 사용됨

### 원인 2: **base.cpp에서 angular_velocity 보정 없음** ⚠️⚠️

**코드 분석** (`base.cpp` Line 43):
```cpp
linear_velocity_x_ = msg->linear.x * linear_scale_;  // 1.2배
linear_velocity_y_ = msg->linear.y * linear_scale_;  // 1.2배
angular_velocity_z_ = msg->angular.z;  // 보정 없음!
```

**문제**:
- linear는 1.2배 보정
- angular는 **보정 없이 그대로 사용**
- odom_raw 계산 시 angular가 과소평가됨

### 원인 3: **PWM과 실제 속도의 비선형 관계** ⚠️

**측정 데이터**:
```
명령: 0.2 rad/s
PWM:  40% (계산값)
────────────────────────────────────────
odom_raw:      0.75 rad/s (평균, 측정)
IMU:           3.2 rad/s (측정)
물리 실제:     약 5.5 rad/s (270°를 3초에)
────────────────────────────────────────
비율 (실제/명령):  27.5배!!!
```

**계산**:
```
실제 각속도 = 270° ÷ 3초 = 90°/s = 1.57 rad/s
측정 odom   = 124° ÷ 3초 = 41.3°/s = 0.72 rad/s
비율        = 1.57 / 0.72 = 2.18배

또는:
270° / 124° = 2.18배
```

### 원인 4: **IMU sensitivity_gain 2.0** ⚠️

**코드** (`transbot_driver.py` Line 298):
```python
sensitivity_gain = 2.0
msg.angular_velocity.z = float(gz) * sensitivity_gain
```

**실제 데이터**:
- 하드웨어 원본: ~1.6 rad/s (추정)
- gain 2.0 적용: 3.2 rad/s
- angular_scale 1.08 적용: 3.456 rad/s

**문제**: IMU가 실제보다 과대 측정

### 원인 5: **하드웨어 펌웨어의 비선형 응답** ⚠️⚠️⚠️

**가설**:
```
PWM 40% 명령 시:
→ 하드웨어가 실제로는 훨씬 더 빠르게 회전
→ 비선형 모터 응답 곡선
→ 캘리브레이션 필요
```

**증거**:
```
odom_angular_scale_correction: 0.75

의미: 측정값 × 0.75 = 실제값
즉: 측정값이 실제의 1.33배 (1/0.75)

하지만:
실제 측정: 124°
실제 물리: 270°
비율: 270 / 124 = 2.18배

예상 (0.75 적용): 124° × 0.75 = 93°
실제: 270°

차이: 270 / 93 = 2.9배 추가 오차!
```

## 📋 보정 값 계산

### 필요한 angular_scale_correction

**현재 상황**:
```
명령: 90° 목표
측정: 124° (odom_raw)
실제: 270° 회전
```

**계산**:
```
correction = 실제 / 측정
           = 270° / 124°
           = 2.18

또는 역으로:
측정값 × correction = 실제값
124° × 2.18 = 270°
```

**적용해야 할 값**:
```yaml
# ROS2에서 필요한 값
odom_angular_scale_correction: 2.18

# 또는 드라이버에서:
turn_speed = angular_z * (100.0 / 0.5) * (1.0 / 2.18)
           = angular_z * 91.74
```

### base.cpp에서 필요한 수정

**현재**:
```cpp
angular_velocity_z_ = msg->angular.z;  // 보정 없음
```

**수정 필요**:
```cpp
angular_velocity_z_ = msg->angular.z * angular_scale_;  // 보정 추가
```

**파라미터**:
```yaml
# launch 파일에서
parameters=[{
    'linear_scale': 1.2,
    'angular_scale': 2.18,  # 또는 0.459 (역수)
    'is_multi_robot': False
}]
```

## 🎯 결론

### 270도 회전의 근본 원인

**1단계: 명령 생성**
```
test_90degree_rotation.py:
angular_speed = 0.2 rad/s
→ /cmd_vel 발행
```

**2단계: 드라이버 변환**
```
transbot_driver.py:
turn_speed = 0.2 * (100.0 / 0.5) = 40% PWM
→ Transbot_Lib: angular = 20 (0.2 × 100)
→ 하드웨어로 전송
```

**3단계: 하드웨어 실행** ⚠️⚠️⚠️
```
하드웨어:
20 값 수신
→ 실제 회전: 약 1.57 rad/s (270°/3초)
→ 명령의 7.85배!
```

**4단계: 오도메트리 추정** ⚠️
```
base.cpp:
/transbot/get_vel 구독 (0.2 rad/s 그대로)
→ angular_velocity_z_ = 0.2 (보정 없음)
→ odom_raw 계산: 0.2 rad/s 기준
→ 실제는 1.57 rad/s인데 0.2로 가정
→ 7.85배 과소평가!
```

**5단계: 실제 vs 측정 차이**
```
실제 회전: 270° (1.57 rad/s × 3초)
측정 odom: 124° (0.72 rad/s × 3초)
비율: 2.18배 차이
```

### 왜 2.18배인가?

**계산**:
```
1. 하드웨어 실제 속도: 1.57 rad/s
2. 명령 속도: 0.2 rad/s
3. 하드웨어 증폭: 7.85배

4. odom 측정 평균: 0.72 rad/s
   (데이터: 0.75~0.80 rad/s)

5. odom vs 실제: 1.57 / 0.72 = 2.18배

왜 0.72 rad/s인가?
→ /transbot/get_vel이 실제 값의 일부만 보고
→ 또는 base.cpp 계산 시 dt 오차
→ 또는 드라이버에서 속도 피드백 부정확
```

### 핵심 문제

**✗ odom_angular_scale_correction: 0.75가 적용되지 않음**
- 파일에만 존재, 코드에 미반영
- calibrate_angular.py 스크립트용

**✗ base.cpp에서 angular_velocity 보정 없음**
- linear_scale은 있지만 angular_scale 없음
- angular_velocity_z_ = msg->angular.z (그대로)

**✗ 하드웨어 펌웨어의 비선형 응답**
- PWM 40% → 실제 7.85배 속도
- 캘리브레이션 필요

**✗ IMU 과증폭**
- sensitivity_gain: 2.0
- angular_scale: 1.08
- 총 2.16배 증폭

## 🛠️ 해결 방법 (권장)

### 방법 1: base.cpp에 angular_scale 추가 ⭐⭐⭐

**장점**: 
- 근본적 해결
- 모든 오도메트리에 적용
- ROS1 방식과 동일

**수정**:
1. `base.hpp`에 `angular_scale_` 추가
2. `base.cpp`에서 `angular_velocity_z_` 계산 시 곱하기
3. launch 파일에서 파라미터 전달

### 방법 2: transbot_driver.py에서 보정 ⭐⭐

**장점**:
- 빠른 수정
- 재빌드 필요 없음 (Python)

**수정**:
```python
# Line 174
turn_speed = angular_z * (100.0 / (0.5 * 2.18))
```

### 방법 3: 하드웨어 펌웨어 캘리브레이션 ⭐

**장점**:
- 가장 정확
- 하드웨어 레벨 해결

**방법**:
1. calibrate_angular.py 실행
2. 360° 회전 시간 측정
3. correction 값 계산
4. 드라이버나 base.cpp에 적용

## 📝 다음 단계

1. **측정**: calibrate_angular.py로 정확한 correction 값 확인
2. **적용**: base.cpp 또는 driver.py 수정
3. **검증**: test_90degree_rotation.py 재실행
4. **미세조정**: 필요 시 correction 값 조정
