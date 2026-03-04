# TIL: 휠 인코더 폐루프 제어 마이그레이션

**날짜:** 2026년 2월 11일  
**작성자:** Jupiter 프로젝트  
**주제:** 오픈루프 보상 로직을 폐루프 PID 제어로 전환

---

## 📋 배경

### 기존 시스템 (오픈루프 제어)
- **문제:** 휠 인코더가 없어 모터 속도 피드백 불가
- **해결 방법:** `jupiter_driver_compensated.py`에 수동 보상 로직 구현
  - 마찰력 보상 (Static/Dynamic Bias 이원화)
  - 각속도 제곱 보정
  - 주행 상태별 파라미터 분리

### 새 시스템 (폐루프 제어)
- **개선:** MCU에 휠 인코더 연동 완료 ([extbd](https://github.com/p-trck/extbd) - transbot-encoder 브랜치)
- **장점:** 
  - 실시간 속도 피드백
  - PID 자동 보정
  - 마찰/부하/배터리 전압 변화 자동 대응

---

## 🔍 MCU 코드 분석

### 1. 제어 흐름 (app_differential.c)

```c
// Differential_Ctrl() - 차동 구동 로봇 속도 제어
void Differential_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust)
{
    float robot_APB = Motion_Get_APB();  // 휠베이스 절반
    speed_fb = V_x;
    speed_spin = (V_z / 1000.0f) * robot_APB;  // 회전 → 휠 속도 차이 변환
    
    // 좌우 모터 목표 속도 계산
    speed_L1_setup = speed_fb - speed_spin;  // 왼쪽: 전진 - 회전
    speed_R1_setup = speed_fb + speed_spin;  // 오른쪽: 전진 + 회전
    
    Differential_Set_Speed(speed_L1_setup, speed_R1_setup);
}
```

### 2. PID 폐루프 제어 (app_motion.c + app_pid.c)

```c
// 10ms마다 실행되는 제어 루프
void Motion_Get_Speed(car_data_t* car)
{
    // 1. 인코더에서 휠 속도 읽기
    Motion_Get_Encoder();
    
    for (i = 0; i < 4; i++)
    {
        // 2. 펄스 → mm/s 변환
        speed_mm[i] = -1 * (g_Encoder_All_Offset_Filtered[i]) * 100 
                      * circle_mm / circle_pulse;
    }
    
    if (g_start_ctrl)
    {
        // 3. PID 계산: 목표 vs 실제 속도
        PID_Calc_Motor(&motor_data);
        
        // 4. PWM 출력 자동 조정
        // - 에러 = 목표 - 실제
        // - 출력 += Kp*Δerr + Ki*err + Kd*Δ²err
    }
}
```

**핵심 구조:**
```
Nav2/Teleop → Python Driver → MCU
                               ↓
                        [목표 속도 설정]
                               ↓
                    ┌─────────┴─────────┐
                    ↓                   ↑
              [인코더 측정]         [PWM 조정]
                    ↓                   ↑
              [PID 계산] ──────────────┘
                 (오차 보정)
```

### 3. 증분형 PID 알고리즘 (app_pid.c)

```c
float PID_Incre_Calc(pid_t *pid, float actual_val)
{
    // 오차 계산
    pid->err = pid->target_val - actual_val;
    
    // 증분형 PID
    pid->pwm_output += pid->Kp * (pid->err - pid->err_next)     // P: 오차 변화량
                     + pid->Ki * pid->err                        // I: 오차 누적
                     + pid->Kd * (pid->err - 2*pid->err_next + pid->err_last);  // D: 오차 가속도
    
    // PWM 출력 제한 (±MOTOR_MAX_PULSE)
    // ...
    
    return pid->pwm_output;
}
```

**특징:**
- **비례(P):** 현재 오차 크기에 반응
- **적분(I):** 정상상태 오차 제거 (마찰 극복)
- **미분(D):** 오버슈트 방지 (진동 감소)

---

## 🛠️ Python 드라이버 수정

### 문제점 진단

**기존 `cmd_vel_callback` (183~245라인):**
```python
# 복잡한 보상 로직 (오픈루프 시절)
if abs_vx > 0.01:
    BIAS = 0.05     # 동적 마찰
    SLOPE = 0.191
else:
    BIAS = 0.13     # 정적 마찰
    SLOPE = 0.111

angular_comp = (abs_angular * SLOPE) + BIAS  # 수동 보정
```

**문제:**
1. MCU가 PID로 자동 보정하는데, 드라이버 단에서 또 보정 시도
2. 이중 보정으로 인한 오버슈트, 진동, 불안정성
3. 마찰/부하 변화에 대응 불가 (고정 파라미터)

### 해결 방안

**1단계: 보상 로직 전체 제거**

```python
def cmd_vel_callback(self, msg):
    """
    MCU now has encoder feedback and PID control
    - No manual compensation needed
    - MCU PID automatically handles friction, inertia, battery voltage
    """
    vx = msg.linear.x
    angular = msg.angular.z
    
    # 단순 데드밴드만 적용
    if abs(vx) < 0.001:
        vx = 0.0
    if abs(angular) < 0.001:
        angular = 0.0
    
    # 직접 전달 → MCU PID가 알아서 처리
    self.bot.set_car_motion(vx, vy, angular)
```

**2단계: 각속도 캘리브레이션**

**테스트 결과:**
- 명령: `angular = 1.0 rad/s` × 10초
- 기대: 1.59회 회전 (10 rad ÷ 2π)
- 실제: **4.5회 회전** (283% 과회전)

**원인:** MCU의 `set_car_motion()` 내부 게인이 과도함

**보정:**
```python
# 보정 계수 계산
ANGULAR_SCALE_FACTOR = 1.59 / 4.5 = 0.353

# 적용
angular_corrected = angular * 0.353
self.bot.set_car_motion(vx, vy, angular_corrected)
```

### 최종 코드

**파일:** `jupiter_driver_compensated.py:179-225`

```python
def cmd_vel_callback(self, msg):
    """
    Handle incoming velocity commands from Nav2 or teleop
    
    Angular Velocity Calibration (2026-02-11):
    - Test: angular = 1.0 rad/s for 10 seconds
    - Expected: 1.59 rotations (10 rad ÷ 2π)
    - Actual: 4.5 rotations (283% over-rotation)
    - Correction factor: 1.59 / 4.5 = 0.353
    """
    vx = msg.linear.x
    angular = msg.angular.z
    
    # 각속도 스케일링 보정
    ANGULAR_SCALE_FACTOR = 0.353  # Calibrated 2026-02-11
    angular_corrected = angular * ANGULAR_SCALE_FACTOR
    
    # 데드밴드 필터
    if abs(vx) < 0.001:
        vx = 0.0
    if abs(angular_corrected) < 0.001:
        angular_corrected = 0.0
    
    # MCU로 전달 (PID가 나머지 처리)
    self.bot.set_car_motion(vx, vy, angular_corrected)
```

---

## 📊 변경 사항 요약

| 항목 | 이전 (오픈루프) | 이후 (폐루프) |
|------|----------------|--------------|
| **속도 피드백** | ❌ 없음 | ✅ 10ms마다 인코더 측정 |
| **보상 방식** | 수동 Bias/Slope 조정 | PID 자동 보정 (Kp=0.8, Ki=0.06, Kd=0.5) |
| **마찰 대응** | 고정 파라미터 | 실시간 적응 |
| **코드 복잡도** | 259줄 (보상 로직 70줄) | 223줄 (단순 전달) |
| **각속도 정확도** | 미검증 | 0.353 스케일 보정 적용 |
| **튜닝 방법** | 코드 수정 후 재빌드 | ROS2 파라미터 실시간 조정 |

---

## ✅ 검증 절차

### 1. 빌드 및 배포
```bash
cd /home/jetson/jupiter_ws_ros2
colcon build --packages-select jupiter_bringup
source install/setup.bash
sudo systemctl restart jupiter_joystick.service
```

### 2. 각속도 정밀 테스트
```bash
# 10초 회전 테스트
# 목표: 1.5~1.6회 회전
# - 더 빠르면: ANGULAR_SCALE_FACTOR를 0.28로 감소
# - 더 느리면: ANGULAR_SCALE_FACTOR를 0.56으로 증가
```

### 3. PID 튜닝 (필요시)
```bash
# 현재 파라미터: Kp=0.8, Ki=0.06, Kd=0.5
ros2 param set /jupiter_driver_compensated Kp 1.0  # 응답성 증가
ros2 param set /jupiter_driver_compensated Ki 0.1   # 정상상태 오차 감소
ros2 param set /jupiter_driver_compensated Kd 0.3   # 진동 감소
```

---

## 🎯 기대 효과

### 성능 개선
- ✅ **정확한 속도 제어:** 목표 속도와 실제 속도 오차 < 5%
- ✅ **부하 자동 보상:** 경사로, 카펫 등에서 속도 유지
- ✅ **배터리 변화 대응:** 전압 하락 시에도 일정한 속도
- ✅ **진동 감소:** 이중 보정 제거로 오버슈트 최소화

### 유지보수성
- ✅ **코드 단순화:** 70줄의 보상 로직 삭제
- ✅ **실시간 튜닝:** 재빌드 없이 파라미터 조정 가능
- ✅ **명확한 책임 분리:** 
  - Python 드라이버 → 명령 전달
  - MCU PID → 저수준 제어

---

## 🔗 참고 자료

### MCU 펌웨어
- **저장소:** https://github.com/p-trck/extbd
- **브랜치:** transbot-encoder
- **핵심 파일:**
  - `/Source/APP/app_differential.c` - 차동구동 제어
  - `/Source/APP/app_motion.c` - 인코더 피드백
  - `/Source/APP/app_pid.c` - PID 알고리즘

### ROS2 드라이버
- **파일:** `/home/jetson/jupiter_ws_ros2/src/jupiter_bringup/jupiter_bringup/jupiter_driver_compensated.py`
- **주요 변경:** 
  - `cmd_vel_callback()` 단순화 (라인 179~225)
  - 각속도 스케일 보정 추가 (0.353)

---

## 📝 다음 단계

1. **각속도 미세 조정:** 실제 테스트 후 0.353 계수 검증
2. **선속도 캘리브레이션:** vx 명령과 실제 이동거리 검증
3. **Nav2 통합 테스트:** SLAM 및 자율주행 정확도 확인
4. **PID 최적 튜닝:** 다양한 지형에서 파라미터 최적화
5. **오도메트리 검증:** 인코더 기반 오도메트리 정확도 측정

---

## 🐛 알려진 이슈 및 해결

### ✅ 해결됨: Nav2 과회전 문제 (2026-02-11)

**증상:**
- Nav2 자율주행 시 로봇이 목표 회전각도를 초과하여 과회전
- 제자리 회전 테스트는 정상이지만, Nav2 주행 중 회전 불안정

**근본 원인:**
- `cmd_vel_callback`: Nav2 명령을 0.353로 스케일링하여 MCU 전송 ✅
- `publish_velocity`: MCU 피드백을 **스케일링 없이** 그대로 퍼블리시 ❌

**결과적 동작:**
```
1. Nav2: "1.0 rad/s로 회전해!" → cmd_vel 발행
2. Driver: 1.0 × 0.353 = 0.353 → MCU로 전송
3. MCU: 0.353 rad/s로 실제 회전, 피드백 vz=0.353
4. Driver: 0.353 그대로 → /jupiter/get_vel 발행 ❌ 
5. Nav2: "1.0 명령했는데 0.353만 돌고 있네? 더 세게!"
6. Nav2: 2.0 rad/s 명령 → 결국 과회전
```

**해결 방법:**
- **명령 스케일링:** `angular × 0.353` → MCU
- **피드백 역스케일링:** `vz ÷ 0.353` → Nav2

```python
class JupiterDriver(Node):
    # Class-level constant for consistency
    ANGULAR_SCALE_FACTOR = 0.353
    
    def cmd_vel_callback(self, msg):
        angular_corrected = msg.angular.z * self.ANGULAR_SCALE_FACTOR
        self.bot.set_car_motion(vx, vy, angular_corrected)
    
    def publish_velocity(self):
        vz = self.bot._Rosmaster__vz  # MCU feedback
        vz_scaled = vz / self.ANGULAR_SCALE_FACTOR  # Inverse scale
        msg.angular.z = float(vz_scaled)  # Publish to Nav2
```

**검증:**
```bash
# 빌드 및 재시작
cd /home/jetson/jupiter_ws_ros2
colcon build --packages-select jupiter_bringup
source install/setup.bash
sudo systemctl restart jupiter_joystick.service

# Nav2 테스트
# 터미널 1: SLAM
ros2 launch jupiter_nav jupiter_full_system.launch.py use_rviz:=true

# 터미널 2: Nav2
ros2 launch jupiter_nav nav2_navigation.launch.py

# 회전 정확도 확인: 목표각 도달 시 정지해야 함
```

### 미해결 이슈

- [ ] 각속도 보정 계수(0.353)가 속도에 따라 달라질 가능성
- [ ] 저속(< 0.1 m/s)에서 PID 데드존 확인 필요
- [ ] 갑작스런 가속/감속 시 오버슈트 모니터링 필요

---

**작성 완료:** 2026-02-11  
**최종 업데이트:** 2026-02-11 - Nav2 과회전 문제 해결
