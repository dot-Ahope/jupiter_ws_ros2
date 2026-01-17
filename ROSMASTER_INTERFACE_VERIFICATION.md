# Rosmaster_Lib 인터페이스 사용 확인 보고서

**확인 일자**: 2025-11-14  
**대상 시스템**: yahboomcar_ros2_ws

---

## ✅ 확인 결과: yahboomcar_ros2_ws는 Rosmaster_Lib를 적극적으로 사용 중

### 1. Rosmaster_Lib 설치 상태

**전역 설치 확인됨**:
```
Location: /usr/local/lib/python3.10/dist-packages/Rosmaster_Lib-3.3.1-py3.10.egg/
```

- ✓ 시스템에 전역으로 설치되어 있음
- ✓ Python에서 `from Rosmaster_Lib import Rosmaster`로 직접 import 가능

### 2. 소스 라이브러리 위치

**원본 라이브러리 경로**:
```
/home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1/Rosmaster_Lib/Rosmaster_Lib.py
```

**주요 파일**:
- `Rosmaster_Lib.py` - 메인 라이브러리 (1314 lines)
- `__init__.py` - 패키지 초기화
- `test.py` - 테스트 파일

### 3. yahboomcar_ros2_ws에서의 사용 현황

#### 3.1 사용하는 ROS2 노드들

Rosmaster를 import하고 사용하는 파일 (9개):

1. **yahboomcar_bringup 패키지**:
   - `Mcnamu_driver_X3.py` - X3 차량 드라이버
   - `Mcnamu_driver_x1.py` - X1 차량 드라이버  
   - `Ackman_driver_R2.py` - R2 차량 드라이버

2. **yahboomcar_voice_ctrl 패키지**:
   - `Voice_Ctrl_Mcnamu_driver_X3.py` - X3 음성 제어
   - `Voice_Ctrl_Ackman_driver.py` - Ackman 음성 제어

#### 3.2 초기화 방식

```python
from Rosmaster_Lib import Rosmaster

# 기본 초기화 (기본 포트 사용)
self.car = Rosmaster()

# 차종 설정
self.car.set_car_type(1)  # 1=X3, 5=R2

# 수신 스레드 시작
self.car.create_receive_threading()
```

**기본 설정** (Rosmaster_Lib.py에서):
- 기본 포트: `/dev/myserial` (symbolic link)
- 보드레이트: 115200
- 지연: 0.002초

#### 3.3 실제 사용하는 API

**모터 제어**:
```python
self.car.set_car_motion(vx, vy, angular)  # 속도 명령
```

**센서 데이터 읽기**:
```python
ax, ay, az = self.car.get_accelerometer_data()  # 가속도
gx, gy, gz = self.car.get_gyroscope_data()      # 자이로
mx, my, mz = self.car.get_magnetometer_data()   # 자력계
vx, vy, angular = self.car.get_motion_data()    # 현재 속도
battery = self.car.get_battery_voltage()        # 배터리 전압
version = self.car.get_version()                # 펌웨어 버전
```

**부가 기능**:
```python
self.car.set_beep(1)                           # 부저 ON
self.car.set_beep(0)                           # 부저 OFF
self.car.set_colorful_effect(effect, speed, parm)  # RGB LED 효과
```

### 4. Rosmaster_Lib의 주요 API

#### 4.1 초기화 및 통신
```python
__init__(car_type=1, com="/dev/myserial", delay=0.002, debug=False)
create_receive_threading()           # 수신 스레드 시작
set_auto_report_state(enable, forever=False)  # 자동 리포트 설정
```

#### 4.2 모터 제어
```python
set_motor(speed_1, speed_2, speed_3, speed_4)  # 개별 모터 제어
set_car_run(state, speed, adjust=False)        # 차량 주행 (전진/후진/회전)
set_car_motion(v_x, v_y, v_z)                  # 속도 벡터 제어
set_pid_param(kp, ki, kd, forever=False)       # PID 파라미터 설정
```

#### 4.3 센서 데이터
```python
get_accelerometer_data()    # 가속도계 (ax, ay, az)
get_gyroscope_data()        # 자이로스코프 (gx, gy, gz)
get_magnetometer_data()     # 자력계 (mx, my, mz)
get_imu_attitude_data()     # IMU 자세 (roll, pitch, yaw)
get_motion_data()           # 현재 속도 (vx, vy, angular)
get_motor_encoder()         # 엔코더 값
get_battery_voltage()       # 배터리 전압
get_version()               # 펌웨어 버전
```

#### 4.4 부가 기능
```python
set_beep(on_time)                              # 부저 제어
set_pwm_servo(servo_id, angle)                 # PWM 서보 제어
set_colorful_lamps(led_id, red, green, blue)   # 개별 LED 제어
set_colorful_effect(effect, speed, parm)       # LED 효과
set_uart_servo_angle(s_id, s_angle, run_time)  # UART 서보 제어
```

#### 4.5 설정 관리
```python
set_car_type(car_type)      # 차종 설정 (1=X3, 5=R2)
reset_car_state()           # 차량 상태 리셋
reset_flash_value()         # 플래시 값 리셋
```

### 5. 통신 프로토콜

**직렬 통신**:
- 포트: `/dev/ttyTHS1` (Jetson UART) 또는 `/dev/ttyUSB0`
- 보드레이트: 115200
- 프로토콜: 커스텀 바이너리 프로토콜
  - 헤더: 0xFF
  - 장치 ID: 0xFC
  - 길이 + 기능 코드 + 데이터 + 체크섬

**지원 기능 코드** (일부):
- `0x01`: AUTO_REPORT - 자동 보고
- `0x0A`: REPORT_SPEED - 속도 보고
- `0x0B`: REPORT_MPU_RAW - MPU9250 원시 데이터
- `0x0C`: REPORT_IMU_ATT - IMU 자세
- `0x0D`: REPORT_ENCODER - 엔코더
- `0x0E`: REPORT_ICM_RAW - ICM20948 원시 데이터
- `0x10`: MOTOR - 모터 제어
- `0x11`: CAR_RUN - 차량 주행
- `0x12`: MOTION - 모션 제어
- `0x13`: SET_MOTOR_PID - 모터 PID
- `0x14`: SET_YAW_PID - YAW PID

### 6. 차종별 설정

**지원 차종**:
```python
CARTYPE_X3 = 0x01      # Yahboom X3 (Mecanum wheel)
CARTYPE_X3_PLUS = 0x02 # X3 Plus
CARTYPE_X1 = 0x04      # X1
CARTYPE_R2 = 0x05      # R2 (Ackermann steering)
```

### 7. transbot_ws_ros2 적용 가능성

#### ✅ 호환성 높음

**이유**:
1. Rosmaster_Lib가 이미 시스템에 설치되어 있음
2. 동일한 하드웨어 플랫폼 (Jetson + STM32 기반 제어 보드)
3. 동일한 통신 방식 (UART 시리얼)
4. 유사한 센서 구성 (IMU, 엔코더)

#### 적용 방법

**옵션 1**: 직접 Rosmaster 사용
```python
from Rosmaster_Lib import Rosmaster

self.bot = Rosmaster(com="/dev/ttyTHS1", delay=0.002)
self.bot.set_car_type(1)
self.bot.create_receive_threading()
```

**옵션 2**: 래퍼 클래스 작성
```python
# 기존 Transbot API를 Rosmaster API로 변환
class TransbotRosmasterAdapter:
    def __init__(self):
        self.bot = Rosmaster(com="/dev/ttyTHS1")
        self.bot.create_receive_threading()
```

### 8. 주요 차이점

| 항목 | 기존 Transbot | Rosmaster_Lib |
|------|---------------|---------------|
| Import | `from Transbot_Lib import Transbot` | `from Rosmaster_Lib import Rosmaster` |
| 초기화 | `Transbot(com, delay)` | `Rosmaster(car_type, com, delay)` |
| 속도 제어 | `set_motor(id, speed)` | `set_car_motion(vx, vy, vz)` |
| IMU 데이터 | 개별 메서드 | `get_accelerometer_data()` 등 |
| 수신 스레드 | `create_receive_threading()` | `create_receive_threading()` (동일) |
| PID 설정 | `set_pid_param()` | `set_pid_param()` (동일) |

### 9. 권장 사항

#### ✅ Rosmaster_Lib 적용 권장

**장점**:
1. 이미 시스템에 설치되어 있어 추가 설치 불필요
2. 실제 동작이 검증된 라이브러리
3. 더 많은 기능 지원 (LED, 부저, 서보 등)
4. 자동 데이터 수신 지원
5. 여러 차종 지원

**작업 순서**:
1. `transbot_driver.py`에서 Transbot_Lib import를 Rosmaster_Lib로 변경
2. 초기화 코드 수정
3. API 호출 메서드 매핑
4. 테스트 및 검증

### 10. 다음 단계

1. ✅ **확인 완료**: yahboomcar_ros2_ws가 Rosmaster_Lib 사용 중
2. ⏳ **대기 중**: transbot_ws_ros2에 Rosmaster_Lib 적용 승인
3. 🔄 **예정**: 코드 수정 및 빌드
4. 🧪 **예정**: 하드웨어 테스트

---

## 결론

**yahboomcar_ros2_ws는 Rosmaster_Lib를 핵심 하드웨어 인터페이스로 사용하고 있으며**, transbot_ws_ros2에도 동일하게 적용 가능합니다. 

Rosmaster_Lib는:
- ✓ 시스템에 전역 설치됨
- ✓ 실제 운영 환경에서 검증됨
- ✓ 풍부한 API 제공
- ✓ 직렬 통신 안정성 확보
- ✓ 멀티 스레드 지원

**적용 준비 완료 상태입니다.**
