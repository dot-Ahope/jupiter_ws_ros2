#!/usr/bin/env python3
# encoding: utf-8
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, ParameterType
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from jupiter_msgs.srv import RobotArm
from . import utilities
import math
import threading
import time
# from Rosmaster_Lib import Rosmaster
from .Rosmaster_Lib import Rosmaster
from . import arm_jupiter
from .arm_jupiter import Jupiter_ARM

class JupiterDriver(Node):
    # [2026-04-17] 모터 교체로 캘리브레이션 무효화 → 주석처리. 재캘리브레이션 필요.
    # Angular velocity scale factor (recalibrated 2026-03-19 for 방안A-4 ESC deadzone skip)
    # MCU has internal gain causing over-rotation, compensate in both directions
    # 방안A-4 후 ESC 매핑 변경 → MCU 게인 2.83→2.49 (Part2: 0.5rad/s×3s=85.9° 예상, 75.59° 실측)
    # 보정: 0.353 × (85.9/75.59) = 0.401
    # ANGULAR_SCALE_FACTOR = 0.401  # 방안A-4 기준 재캘리브레이션
    # ANGULAR_SCALE_FACTOR = 1.0    # [2026-04-17] 모터 교체 → 1:1 (재캘리브 필요 명시)
    # [2026-05-11 Stage 8] 1.0 → 0.93. calibrate_angular.py 측정 (cmd ω=0.5 rad/s, 5 cycles
    # 평균 11.64s/360° → ratio 1.079 — robot 이 cmd 보다 8% 더 회전). 명령을 0.93× 로 보내
    # 의도대로 회전. publish_velocity 의 vz_scaled = vz / SCALE 도 자동 정합.
    # 주의: CCW 만 측정 — CW 비대칭 가능. 양방향 측정으로 추후 재조정 권장.
    ANGULAR_SCALE_FACTOR = 0.93     # 회전 명령 보정 — robot 이 cmd 의 1.079× 응답

    # 방안C: MCU 엔코더 분해능 한계 보정 (정지 회전 시 최소 각속도)
    # 엔코더: 1320 pulse/rev, 10ms 측정주기 → 1 tick = 16.3 mm/s
    # PID target < 16.3 mm/s → 0 tick과 1 tick 사이 진동 → 전진↔후진 반복 → 순회전 0
    # MCU_MIN_ANGULAR = 0.12 → 19.7 mm/s → 1.21 ticks/period
    # 주의: min_speed_theta(0.30) × SCALE(0.401) = 0.120 ≥ 0.12 → DWB 명령에 클램프 미적용
    # (0.15일 때 0.120 < 0.15 → 클램프 0.15 → 실효 0.374 = 24% 증폭 → 오버슈트→헌팅)
    # MCU_MIN_ANGULAR = 0.12
    # MCU_MIN_ANGULAR = 0.0  # [2026-04-17] 모터 교체 → 클램프 비활성화
    MCU_MIN_ANGULAR = 0.0  # [2026-04-20] 정지 상태 최소 회전 angular 실측값. 주행 중에는 미적용 (line 271 조건)

    # [2026-04-21] 정지 회전 상태기계 (2단계: STARTUP 키크 + RUNNING 패스스루)
    #
    # 문제: 채터링 억제를 위해 active 상태에서 max(raw, 0.22)를 강제하면
    #       RotateToGoal이 near-goal에서 보내는 작은 값(e.g. 0.0526)이 항상
    #       0.22로 부풀려져 오버슈트 → yaw 미수렴 영구 루프 발생.
    #
    # 해결: 시작 키크(0.37, 0.12s) → 이후 nav2 요청값 그대로 통과
    #   - 채터링 방지: ROT_MIN_REQUEST(0.04) 미만 신호는 시작 자체를 차단
    #     (velocity_smoother deadband=0.05가 0.04 미만 신호를 이미 0으로 만들므로 안전)
    #   - 부호 전환 홀드(0.25s): 좌우 교번 억제는 유지
    # [2026-04-30 update] 정지 회전 키크 강화 + adaptive 종료 (encoder 기반)
    # 이전 (시간 기반 0.12s): MCU PID 적분이 deadzone 까지 올라오기 전 kick 종료 → 무회전 지속.
    # 현재: kick 강도 0.55 로 상향 + 최대 0.40s 지속, 단 actual ω 가 CONFIRM 임계 도달 시 즉시 전환.
    # [2026-05-08] 0.55 → 0.30. bag 분석으로 짧은 회전 phase (0.15s) 직후 actual ω 잔류
    # 6.4x 까지 (cmd=-0.113 → actual=-0.721) over-rotation 관찰. kick 0.55 이 모터에 강한
    # inertia 부여 → kick 종료 후에도 0.5~0.7 rad/s 잔류 회전 → nav2 의 작은 ω 명령 무시.
    # 새 모터 + deadzone 환원 (700/700) 환경에서 0.30 으로도 정지마찰 극복 가능 가설.
    # 위험: 정지마찰 못 극복 시 회전 시작 안 됨 — stall. 발생 시 0.40 으로 단계적 상향.
    MCU_ROT_START_ANGULAR = 0.30     # [2026-05-08] 0.55 → 0.30. inertia 감소 — over-rotation 차단
    # [2026-05-06] 0.12 → 0.30. bag 분석으로 좌회전 명령 cmd_z 평균 0.204 rad/s 시 wheel-level
    # diff = 0.025 m/s = 1.5 tick (16.3 mm/s/tick) 으로 모터 deadzone 미달 → 68.8% 정지 확인.
    # 0.30 으로 boost 하면 wheel-level diff = 0.037 m/s = 2.3 tick → 통과 가능.
    # 부작용: MPPI 의 trajectory rollout 예측 (작은 ω) 과 실제 거동 (0.30) 불일치 → 진동 가능.
    # 임시 해결책. 근본은 펌웨어 양방향 deadzone 캘리브레이션 (옵션 B).
    #
    # [2026-05-08] 0.30 → 0.0 (DISABLED). bag 분석으로 cmd ω ∈ [0.04, 0.30] 표본 55개에서
    # actual/cmd ratio 평균 2.14x (최대 8x: cmd=-0.052, actual=-0.420). 회전→전진 전환 시
    # nav2 의 ramping ω (0.250→0.125→0.116) 에 actual 이 -0.541 → -0.481 → -0.120 으로
    # over-rotation. 사용자 가설 (정지 회전 floor 가 nav2 와 충돌) 검증됨.
    # MCU_ROT_START_ANGULAR=0.55 의 startup kick 만으로 정지마찰 극복 후, RUNNING 단계는
    # nav2 명령 그대로 통과 (max(raw, 0.0) = raw 패스스루).
    # 위험: kick 종료 후 작은 ω 명령에 모터 deadzone 미달로 stall 가능 (0.12 시절 문제).
    #       단 startup kick 의 adaptive 종료 (encoder confirm) 가 작동 중이라 motion 확인된
    #       상태에서 RUNNING 진입 → deadzone 통과 가능성 ↑. 만약 stall 자주 발생하면
    #       0.10 ~ 0.15 정도로 작은 floor 재도입.
    # [2026-05-11 Stage 5 우선 적용] 0.0 → 0.20. Stage 1+2 검증으로 핵심 문제 = RUNNING 단계
    # cmd 0.125~0.20 영역의 motor deadzone 미달 stall 확인 (8~12s 무응답).
    # [2026-05-11 update 2] 0.20 → 0.30. Stage 3 검증으로 cmd 0.150 영역 5회 stall (총 17.7s)
    # 잔존 확인. 0.20 boost 도 motor deadzone 미달. bag 데이터: cmd 0.30 부터 motor 정상 응답.
    # 0.30 으로 boost 하면 nav2 작은 cmd (0.04~0.30) 모두 motor deadzone 확실 통과.
    # trade-off: over-rotation 추가 악화 가능 — Stage 4 (wz_max clamp) 또는 Stage 6
    # (sign-flip 0 cycle) 으로 흡수 예정.
    ROT_RUNNING_MIN_ANGULAR = 0.30   # RUNNING 단계 floor — motor deadzone 확실 통과
    ROT_MIN_REQUEST_ANGULAR = 0.04   # 이 이상이어야 회전 시작 (smoother deadband 0.05보다 낮아 안전)
    ROT_RELEASE_ANGULAR = 0.03       # 이 미만이면 회전 종료 (노이즈 구간)
    # [2026-05-11 Stage 2 REVERTED] 0.25 → 0.40 환원. Stage 2 검증으로 motion 안 시작 폭증
    # (0/10 → 13/19 = 68%) + stuck_status True 전환 10회 발생. 0.25s 안에 RUNNING 진입 시
    # cmd 그대로 통과 → deadzone 미달 stall. 0.40s 유지로 kick 시간 보장.
    ROT_STARTUP_KICK_SEC = 0.40      # 기동 키크 최대 지속 시간 (encoder 확인 전 안전 상한)
    # [2026-05-11 Stage 1] 0.10 → 0.06. bsp_motor filter α (0.3) 의 응답 지연으로 actual_z 가
    # 0.10 까지 도달하기 100ms 이상 → kick 0.40s 끝까지 유지 → PID 적분 누적 → over-rotation.
    # 0.06 으로 낮추면 100~150ms 안에 confirm → kick 즉시 종료 → RUNNING 패스스루 진입.
    ROT_KICK_CONFIRM_THRESHOLD = 0.06  # |actual_z| 가 이 값 이상이면 motion 확인 → kick 조기 종료
    ROT_SIGN_HOLD_SEC = 0.25         # 부호 전환 금지 시간
    # [2026-05-14 옵션 B] cmd |ω| 가 이 값 이상이면 STARTUP_KICK 우회. 직진→회전 transition 시
    # MCU_ROT_START_ANGULAR=0.30 의 작은 cmd 가 wheel-level 약 0.035 m/s 로 산출되어 M1 의
    # 정지마찰+deadzone 통과 못 함 → M1 1.5s 이상 stall 관찰 (bag t=311s, BWD→CW).
    # cmd |ω| ≥ 0.50 이면 RUNNING 즉시 진입 → raw cmd magnitude 그대로 통과 → wheel-level
    # 0.157 m/s 정도 → M1 deadzone 통과 확률 향상.
    # PID windup 우려 없음 (incremental form + output saturation). balance correction 도 회전
    # 모드라 비동작 → 충돌 없음.
    ROT_BYPASS_KICK_THRESHOLD = 0.50

    # [2026-04-30] 이동 + 회전 데드존 보상
    # DWB 가 ω≈0.16 같은 작은 angular 을 내면 wheel-level 명령(left=0.10, right=0.06 m/s)이
    # 모터 데드존 근처에 머물러 PID 적분 지연 → 실효 회전 거의 안 됨 → lethal 으로 직진.
    # 정지 회전에 쓰는 ROT_RUNNING_MIN_ANGULAR=0.12 의 이동 버전. DWB min_speed_theta=0.20
    # 과 동기화하여 controller 의도와 driver 출력이 일치하도록.
    MOVING_ROT_MIN_ANGULAR = 0.20    # 이동 중 최소 사용 가능한 angular (모터 데드존 위)
    MOVING_ROT_VX_THRESHOLD = 0.02   # 이 이상의 vx 일 때 적용 (정지 회전 분기와 분리)
    MOVING_ROT_RELEASE = 0.05        # 이 미만 명령은 그대로 0 처리 (smoother deadband 와 정렬)
    
    def __init__(self):
        super().__init__('jupiter_driver_compensated')
        
        # Initialize hardware with retry mechanism
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # Initialize Rosmaster with X3 car type (differential drive)
                self.bot = Rosmaster(car_type=1, com="/dev/myserial", delay=0.002)
                # Create receive thread to handle IMU and velocity data
                self.bot.create_receive_threading()
                
                # Wait for thread to start
                time.sleep(0.2)
                
                # Enable auto reporting for IMU and velocity data
                # Try multiple times to ensure it sticks
                for _ in range(3):
                    self.bot.set_auto_report_state(True, forever=False)
                    time.sleep(0.1)
                
                # Test communication - stop all motors
                self.bot.set_motor(0, 0, 0, 0)
                self.get_logger().info("Rosmaster hardware initialized successfully on /dev/ttyTHS1")
                
                # Initialize zero data counter for watchdog
                self.zero_data_count = 0
                break
            except Exception as e:
                if attempt < max_retries - 1:
                    self.get_logger().warn(f"Failed to initialize hardware (attempt {attempt + 1}): {str(e)}")
                    time.sleep(1)
                else:
                    self.get_logger().error(f"Failed to initialize hardware after {max_retries} attempts: {str(e)}")
                    raise
        
        # Declare parameters with constraints
        self.declare_parameter('Kp', 0.8,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=3.0,
                    step=0.01
                )],
                description='Proportional gain'
            ))
            
        # Ki: 0.06→0.02 [2026-03-19] 방안A-4 ESC 스킵 후 적분 불필요하게 큼
        # 높은 Ki → incremental PID 누적 출력 과다 → 방향전환 시 ~2초 지연 → 오버슈트→헌팅
        # 0.02: 적분 누적 3x 감소 → 방향전환 ~0.7초
        # [2026-05-11] 0.02 → 0.06 환원. 방안A-4 는 CAR_DIFFERENTIAL 만 적용 (bsp_motor.c:41
        # 분기), 현재 CAR_MECANUM 사용 중이라 방안A-4 가정 무관. bag 분석으로 cmd vx 0.2 → actual
        # 5초 ramp-up 확인 → progress_checker 못 미달 → BT Spin recovery 잘못 trigger.
        # Ki=0.06 이 정상상태 오차 누적 빠르게 → 응답 1초 이내 도달 → progress 정상.
        self.declare_parameter('Ki', 0.06,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=1.0,
                    step=0.01
                )],
                description='Integral gain'
            ))
        
        # Kd: 0.5→0.3 [2026-03-19] incremental PID에서 Kd는 출력 변화에 저항
        # 높은 Kd → 방향전환 시 pwm 감소를 억제 → 감속 지연
        # 0.3: 출력 변경 저항 감소 → 빠른 감속. 약간의 진동은 Nav2가 10Hz 보정으로 흡수
        self.declare_parameter('Kd', 0.3,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=1.0,
                    step=0.01
                )],
                description='Derivative gain'
            ))
            
        # IMU Inversion Parameters
        self.declare_parameter('imu_invert_x', False)
        self.declare_parameter('imu_invert_y', False)
        self.declare_parameter('imu_invert_z', False)

        # IMU Enable/Disable Parameter
        self.declare_parameter('enable_imu', True)  # IMU 퍼블리싱 활성화

        # IMU Gyro Scale Correction
        # ICM20948 gyro_ratio=1/1000 in Rosmaster_Lib may be incorrect.
        # MPU9250 uses 1/3754.9 — ratio: 1000/3754.9 ≈ 0.2663
        # If MCU sends raw INT16 at ±500dps, correction = 1000/3754.9 ≈ 0.2663
        # Use 1.0 for no correction, adjust after live verification.
        self.declare_parameter('imu_gyro_scale', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.01,
                    to_value=10.0,
                    step=0.001
                )],
                description='Gyro scale correction factor (applied to all axes)'
            ))


        # Create publishers
        self.imu_pub = self.create_publisher(
            Imu, 
            '/jupiter/imu', 
            10
        )
        self.vel_pub = self.create_publisher(
            Twist,
            '/jupiter/get_vel',
            10
        )
        # [2026-05-06] 좌/우 휠 속도 (m/s) publish.
        # data[0]=LEFT (M1), data[1]=RIGHT (M3). MCU FUNC_REPORT_WHEEL_SPEED (0x08) 25Hz 송출.
        # 진단 목적: 비대칭 deadzone, path 추종 시 좌우 회전 방향 검증.
        self.wheel_speeds_pub = self.create_publisher(
            Float32MultiArray,
            '/jupiter/wheel_speeds',
            10
        )
        # [2026-05-12] Encoder raw count (4 motor) publish — 우측 wheel 변동성 진단용.
        # data = [M1, M2, M3, M4] int32 누적 count. MCU FUNC_REPORT_ENCODER (0x0D) 송출.
        # 진단 목적: wheel_speeds (m/s) 만으로 잡히지 않는 raw encoder 의 jitter / 비대칭 측정.
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/jupiter/encoder_counts',
            10
        )
        # 별도의 가속도 변화량 토픽 추가 (움직임 감지용)
        self.accel_delta_pub = self.create_publisher(
            Imu,
            '/jupiter/accel_delta',
            10
        )
        
        # Initialize robot arm
        try:
            self.arm = Jupiter_ARM()
            # self.get_logger().info("Robot arm initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize robot arm: {str(e)}")
            raise

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create RobotArm service
        self.srv_CurrentAngle = self.create_service(
            RobotArm,
            '/CurrentAngle',
            self.RobotArmCallback
        )
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create timers - IMU 데이터의 빈도 더 높게 설정
        self.create_timer(0.01, self.publish_imu)  # 100Hz (0.01s)
        self.create_timer(0.02, self.publish_velocity)  # 50Hz (0.02s) - 10Hz에서 증가
        # [2026-05-06] 휠 속도 publish — MCU FUNC_REPORT_WHEEL_SPEED 가 25Hz 라 그에 맞춰 40ms.
        self.create_timer(0.04, self.publish_wheel_speeds)  # 25Hz
        # [2026-05-12] Encoder raw count publish — wheel_speeds 와 별도. MCU 가 report_count==31
        #   마다 송신 (약 30Hz). 40ms (25Hz) 주기로 read 충분.
        self.create_timer(0.04, self.publish_encoder_counts)  # 25Hz
        self.create_timer(5.0, self.check_serial_health)  # 5초마다 시리얼 상태 확인
        self._serial_was_ok = True  # 최초 상태 추적 (로그 중복 방지)
        
        # Initialize PID parameters
        try:
            kp = self.get_parameter('Kp').value
            ki = self.get_parameter('Ki').value
            kd = self.get_parameter('Kd').value
            self.bot.set_pid_param(kp, ki, kd, forever=True)
            # self.get_logger().info(f"Initial PID parameters set: Kp={kp}, Ki={ki}, Kd={kd}")
        except Exception as e:
            self.get_logger().error(f"Failed to set initial PID parameters: {str(e)}")
        
        self.get_logger().info('Jupiter Driver Node has been initialized')

        # 정지 회전 상태기계 변수
        self._rot_active = False
        self._rot_sign = 0
        self._rot_sign_hold_until = 0.0
        self._rot_startup_until = 0.0    # 기동 키크 최대 종료 시각 (안전 상한)
        self._rot_motion_confirmed = False  # encoder 로 motion 확인됐는지

        # [2026-05-11 Stage 6] sign-flip 0 cycle 삽입용. 직전 angular 명령 기억.
        # brake → 반대 방향 회전 transition 시 motor inertia 잔류 방지.
        self._last_cmd_omega = 0.0
        
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name in ['Kp', 'Ki', 'Kd']:
                # Update PID parameters
                kp = self.get_parameter('Kp').value
                ki = self.get_parameter('Ki').value
                kd = self.get_parameter('Kd').value
                # Update PID values in robot hardware
                try:
                    self.bot.set_pid_param(kp, ki, kd, forever=True)
                    # self.get_logger().info(f"PID parameters updated: Kp={kp}, Ki={ki}, Kd={kd}")
                except Exception as e:
                    self.get_logger().error(f"Failed to update PID parameters: {str(e)}")
        return True
        
    def check_serial_health(self):
        """주기적 시리얼 상태 확인 — 단절 감지 시 ROS 로그 출력"""
        is_ok = self.bot.is_serial_ok()
        if not is_ok and self._serial_was_ok:
            self.get_logger().error(
                'USB serial disconnected! MCU communication lost. '
                'Rosmaster_Lib will auto-reconnect when device reappears.')
        elif is_ok and not self._serial_was_ok:
            self.get_logger().info(
                'USB serial reconnected! MCU communication restored.')
        self._serial_was_ok = is_ok

    def cmd_vel_callback(self, msg):
        """
        Handle incoming velocity commands from Nav2 or teleop
        
        Note: MCU now has encoder feedback and PID control (CAR_DIFFERENTIAL mode)
        - MCU PID automatically handles friction, inertia, battery voltage
        - Driver applies angular velocity scaling due to MCU internal gain mismatch
        - PID tuning is done via ROS2 parameters (Kp, Ki, Kd) which update MCU in real-time
        
        MCU Control Flow:
        1. Differential_Ctrl() converts vx, angular → left/right motor speeds (±1000 range)
        2. Motion_Set_Speed() sets target speeds for each motor
        3. Motion_Get_Speed() reads wheel encoder feedback every 10ms
        4. PID_Calc_Motor() computes error and adjusts PWM output automatically
        5. Closed-loop control maintains accurate speed regardless of load/friction
        
        Angular Velocity Calibration (2026-02-11):
        - Test: angular = 1.0 rad/s for 10 seconds
        - Expected: 1.59 rotations (10 rad ÷ 2π)
        - Actual: 4.5 rotations (283% over-rotation)
        - Correction factor: 1.59 / 4.5 = 0.353
        
        CRITICAL: Both command and feedback must use same scale factor!
        - Command: Nav2_angular × SCALE → MCU
        - Feedback: MCU_angular ÷ SCALE → Nav2 (see publish_velocity)
        """
        try:
            # Get velocity commands from twist message
            vx = msg.linear.x       # m/s - forward/backward linear velocity
            vy = msg.linear.y       # m/s - lateral velocity (ignored for differential drive)
            angular = msg.angular.z # rad/s - rotational velocity (CCW positive)
            
            # [2026-04-17] 모터 교체 → SCALE=1.0, MIN_ANGULAR=0.0 으로 캘리브레이션 무효화됨
            # 재캘리브레이션 후 ANGULAR_SCALE_FACTOR, MCU_MIN_ANGULAR 값 업데이트 필요
            angular_corrected = angular * self.ANGULAR_SCALE_FACTOR
            
            # ESC 데드존은 펌웨어 방안A-4 (ESC deadzone skip)가 처리
            # PID 출력≠0이면 즉시 ESC 111/105로 점프 → 적분 지연 없음
            
            # 정지 회전 상태기계 (2단계: STARTUP 키크 → RUNNING 패스스루)
            if abs(vx) < 0.02:
                now = time.monotonic()
                raw_abs = abs(angular_corrected)
                req_sign = 0
                if angular_corrected > 0.0:
                    req_sign = 1
                elif angular_corrected < 0.0:
                    req_sign = -1

                # encoder 의 actual angular (MCU 보고값). publish_velocity 와 동일 소스.
                # cmd_vel_callback 직접 read 로 매 명령마다 fresh 값 사용.
                try:
                    actual_z = float(self.bot._Rosmaster__vz)
                except Exception:
                    actual_z = 0.0

                if self._rot_active:
                    if raw_abs < self.ROT_RELEASE_ANGULAR or req_sign == 0:
                        # 신호가 노이즈 수준으로 떨어졌으면 회전 종료
                        self._rot_active = False
                        self._rot_sign = 0
                        self._rot_motion_confirmed = False
                        angular_corrected = 0.0
                    else:
                        # 부호 전환 방지 (채터링 억제)
                        if req_sign != self._rot_sign and now < self._rot_sign_hold_until:
                            req_sign = self._rot_sign  # hold 기간 중이면 이전 방향 유지
                        elif req_sign != self._rot_sign:
                            # hold 만료 후 방향 전환 허용 → 새 기동 키크 재시작
                            self._rot_sign = req_sign
                            self._rot_sign_hold_until = now + self.ROT_SIGN_HOLD_SEC
                            self._rot_startup_until = now + self.ROT_STARTUP_KICK_SEC
                            self._rot_motion_confirmed = False

                        # [2026-04-30] adaptive kick: encoder actual_z 가 CONFIRM 임계 도달
                        #   → motion 확인 → kick 즉시 종료 → RUNNING 진입
                        # confirm 안 되면 ROT_STARTUP_KICK_SEC (0.40s) 까지 kick 유지.
                        # 이 후엔 안전 상한으로 강제 RUNNING 전환 (모터 고장 등 예외 상황 대비).
                        if not self._rot_motion_confirmed:
                            sign_match = (
                                (self._rot_sign > 0 and actual_z >= self.ROT_KICK_CONFIRM_THRESHOLD) or
                                (self._rot_sign < 0 and actual_z <= -self.ROT_KICK_CONFIRM_THRESHOLD)
                            )
                            if sign_match:
                                self._rot_motion_confirmed = True

                        if (not self._rot_motion_confirmed) and now < self._rot_startup_until:
                            # [옵션 B] STARTUP 중이라도 cmd 가 충분히 크면 즉시 RUNNING 전환
                            # raw_abs (=|angular_corrected|) >= BYPASS 면 작은 kick 0.30 보다 큰 raw 가 우선.
                            if raw_abs >= self.ROT_BYPASS_KICK_THRESHOLD:
                                self._rot_motion_confirmed = True
                                angular_corrected = req_sign * max(raw_abs, self.ROT_RUNNING_MIN_ANGULAR)
                            else:
                                # STARTUP 단계: 정지마찰 극복용 키크 유지
                                angular_corrected = self._rot_sign * self.MCU_ROT_START_ANGULAR
                        else:
                            # RUNNING 단계:
                            # 정지 회전에서는 0.0526 같은 DWB 최소 샘플이 실제 하드웨어를 못 움직일 수 있다.
                            # 따라서 완전 패스스루 대신, 기동 후 유지 가능한 하한(0.12)만 보장한다.
                            if not self._rot_motion_confirmed:
                                # kick timeout 인데 encoder 미확인 → 안전 상한 도달, 강제 RUNNING.
                                # 모터 stall 가능성 — stuck_detector 가 별도로 처리.
                                self._rot_motion_confirmed = True
                            angular_corrected = req_sign * max(raw_abs, self.ROT_RUNNING_MIN_ANGULAR)
                else:
                    if raw_abs >= self.ROT_MIN_REQUEST_ANGULAR and req_sign != 0:
                        # 유의미한 회전 요청 → 기동 키크와 함께 시작
                        self._rot_active = True
                        self._rot_sign = req_sign
                        self._rot_sign_hold_until = now + self.ROT_SIGN_HOLD_SEC
                        self._rot_startup_until = now + self.ROT_STARTUP_KICK_SEC
                        # [옵션 B] cmd 가 충분히 크면 STARTUP_KICK 우회 → 바로 RUNNING 진입
                        # 직진→회전 transition 시 M1 정지마찰+deadzone 통과 보장.
                        if raw_abs >= self.ROT_BYPASS_KICK_THRESHOLD:
                            self._rot_motion_confirmed = True
                            angular_corrected = req_sign * max(raw_abs, self.ROT_RUNNING_MIN_ANGULAR)
                        else:
                            self._rot_motion_confirmed = False
                            angular_corrected = self._rot_sign * self.MCU_ROT_START_ANGULAR
                    else:
                        angular_corrected = 0.0
            
            # [2026-04-30] 이동 + 회전 데드존 보상
            # 정지 회전 분기 (vx<0.02) 에서는 위 STARTUP/RUNNING 키크 처리됨.
            # 이동 중 (vx≥MOVING_ROT_VX_THRESHOLD) 회전은 여기서 floor 적용:
            #   |ω| < RELEASE(0.05)  → 0 (smoother deadband 와 일치, 노이즈 컷)
            #   RELEASE ≤ |ω| < MIN(0.20) → MIN 으로 boost (모터 데드존 안정 통과)
            #   |ω| ≥ MIN            → 패스스루
            if abs(vx) >= self.MOVING_ROT_VX_THRESHOLD:
                raw_abs = abs(angular_corrected)
                if raw_abs < self.MOVING_ROT_RELEASE:
                    angular_corrected = 0.0
                elif raw_abs < self.MOVING_ROT_MIN_ANGULAR:
                    sign = 1.0 if angular_corrected > 0 else -1.0
                    angular_corrected = sign * self.MOVING_ROT_MIN_ANGULAR

            # Apply minimal deadband to reduce command jitter and motor noise
            # Small values near zero often result from Nav2 controller noise
            if abs(vx) < 0.001:
                vx = 0.0
            if abs(angular_corrected) < 0.001:
                angular_corrected = 0.0
            
            # Save last command timestamp for odometry and timeout detection
            self.last_cmd_vel = msg
            self.last_cmd_time = self.get_clock().now()

            # [2026-05-11 Stage 6] Sign-flip 0 cycle 삽입 — motor inertia 배출
            # 부호 뒤집힘 + 직전이 의미있는 회전 (|ω|>0.10) 이면 이번 cycle 0 송신.
            # 다음 cycle 부터 angular_corrected 적용 → motor 가 inertia 배출 후 반대 방향 시작.
            # premature forward 시 actual_ω 잔류 (0.5~1.0) 의 직접 원인 제거.
            if self._last_cmd_omega * angular_corrected < 0 and abs(self._last_cmd_omega) > 0.10:
                self.bot.set_car_motion(vx, vy, 0.0)
                self._last_cmd_omega = 0.0
                return
            self._last_cmd_omega = angular_corrected

            # Send velocity command to MCU with corrected angular velocity
            # MCU's PID controller will handle all low-level control:
            # - Converts velocity → target encoder counts
            # - Measures actual wheel speeds via encoders
            # - Adjusts motor PWM to minimize error
            # - Compensates for friction, load, battery voltage automatically
            self.bot.set_car_motion(vx, vy, angular_corrected)
            
            # Optional: Log commands for debugging (disabled by default)
            # if abs(vx) > 0.01 or abs(angular) > 0.01:
            #     self.get_logger().info(
            #         f'cmd_vel: vx={vx:.3f} m/s, angular={angular:.3f} → {angular_corrected:.3f} rad/s'
            #     )
            
        except Exception as e:
            self.get_logger().error(f'Failed to set velocity: {str(e)}')
            
    # 이동 평균 필터를 위한 데이터 저장 배열
    imu_data_buffer = {
        'ax': [], 'ay': [], 'az': [],
        'gx': [], 'gy': [], 'gz': []
    }
    buffer_size = 2  # 버퍼 크기 (필터링 정도) - 민감도 향상을 위해 5에서 2로 감소

    def apply_moving_average(self, value, buffer_name):
        """이동 평균 필터 적용 - 비활성화됨, 원본 값 반환"""
        # 버퍼에 현재 값 추가 (로깅 목적으로만 유지)
        if len(self.imu_data_buffer[buffer_name]) >= self.buffer_size:
            self.imu_data_buffer[buffer_name].pop(0)
        self.imu_data_buffer[buffer_name].append(value)
        
        # 원시 값을 직접 반환 (이동 평균 필터 비활성화)
        return value

    def publish_imu(self):
        """Publish IMU data"""
        if not self.get_parameter('enable_imu').value:
            return

        try:
            # Get accelerometer and gyroscope data
            ax, ay, az = self.bot.get_accelerometer_data()
            gx, gy, gz = self.bot.get_gyroscope_data()
            
            # Watchdog: Check if data is all zeros (which indicates communication failure)
            # Gravity (az) should be around 9.8 or -9.8, so 0.0 is definitely wrong
            if ax == 0.0 and ay == 0.0 and az == 0.0 and gx == 0.0 and gy == 0.0 and gz == 0.0:
                self.zero_data_count += 1
                if self.zero_data_count > 100:  # 1 second of zeros
                    self.get_logger().warn("IMU data is all zeros. Attempting to re-enable auto-report...")
                    self.bot.set_auto_report_state(True, forever=False)
                    self.zero_data_count = 0
            else:
                self.zero_data_count = 0
            
            # IMU 데이터를 저장하여 움직임 감지에 활용
            # 이전 값과 현재 값의 차이를 계산 (가속도 변화량)
            if not hasattr(self, 'prev_accel'):
                self.prev_accel = {'ax': ax, 'ay': ay, 'az': az}
                self.accel_delta = {'ax': 0.0, 'ay': 0.0, 'az': 0.0}
                
                # 가속도 데이터 윈도우 저장 (노이즈 필터링 위한 이동평균용)
                self.accel_window = {
                    'ax': [ax], 'ay': [ay], 'az': [az]
                }
                self.window_size = 5  # 이동평균 윈도우 크기
            else:
                # 이전 값과의 차이 계산 (가속도의 변화량)
                self.accel_delta = {
                    'ax': ax - self.prev_accel['ax'],
                    'ay': ay - self.prev_accel['ay'],
                    'az': az - self.prev_accel['az']
                }
                # 현재 값을 이전 값으로 저장
                self.prev_accel = {'ax': ax, 'ay': ay, 'az': az}
                
                # 가속도 데이터 윈도우 업데이트
                for axis in ['ax', 'ay', 'az']:
                    if len(self.accel_window[axis]) >= self.window_size:
                        self.accel_window[axis].pop(0)
                    self.accel_window[axis].append(locals()[axis])
            
            # 이동 평균 계산 (노이즈 감소)
            smooth_ax = sum(self.accel_window['ax']) / len(self.accel_window['ax'])
            smooth_ay = sum(self.accel_window['ay']) / len(self.accel_window['ay'])
            smooth_az = sum(self.accel_window['az']) / len(self.accel_window['az'])
            
            # 디버깅을 위해 원시 IMU 데이터와 변화량 출력
            # self.get_logger().info(f"Raw IMU: ax={ax:.6f}, ay={ay:.6f}, az={az:.6f}, gx={gx:.6f}, gy={gy:.6f}, gz={gz:.6f}")
            # self.get_logger().info(f"Smooth accel: ax={smooth_ax:.6f}, ay={smooth_ay:.6f}, az={smooth_az:.6f}")
            # self.get_logger().info(f"Delta: dax={self.accel_delta['ax']:.6f}, day={self.accel_delta['ay']:.6f}, daz={self.accel_delta['az']:.6f}")
            
            # 이동 평균으로 보정된 값 사용
            ax = smooth_ax
            ay = smooth_ay
            az = smooth_az
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            
            # 적절한 공분산 값 설정 (노이즈 수준 추정)
            accel_cov = 0.01  # 가속도 측정 불확실성 (m/s^2)^2
            gyro_cov = 0.005  # 각속도 측정 불확실성 (rad/s)^2 — 0.001→0.005: Wheel vyaw 비중 ↑
            
            # 초기 공분산 행렬 설정
            msg.orientation_covariance = [-1.0] * 9  # 방향은 여전히 제공되지 않음
            
            # 각속도 공분산 (대각선에 불확실성 값 설정)
            msg.angular_velocity_covariance = [
                gyro_cov, 0.0, 0.0,
                0.0, gyro_cov, 0.0,
                0.0, 0.0, gyro_cov
            ]
            
            # 가속도 공분산
            msg.linear_acceleration_covariance = [
                accel_cov, 0.0, 0.0,
                0.0, accel_cov, 0.0,
                0.0, 0.0, accel_cov
            ]
            
            # Apply IMU gyro scale correction
            # (ICM20948 gyro_ratio may be wrong — adjust via parameter)
            gyro_scale = self.get_parameter('imu_gyro_scale').value
            gx = gx * gyro_scale
            gy = gy * gyro_scale
            gz = gz * gyro_scale

            # Apply IMU axis inversion if configured
            # (jupiter_driver_params.yaml에서 imu_invert_z: true로 설정됨
            #  — IMU Z축이 물리적으로 아래를 향해 있어 angular_velocity.z 부호 반전 필요)
            if self.get_parameter('imu_invert_x').value:
                gx = -gx
            if self.get_parameter('imu_invert_y').value:
                gy = -gy
            if self.get_parameter('imu_invert_z').value:
                gz = -gz

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            
            self.imu_pub.publish(msg)
            
            # 가속도 변화량 토픽 발행
            delta_msg = Imu()
            delta_msg.header = msg.header
            delta_msg.linear_acceleration.x = self.accel_delta['ax']
            delta_msg.linear_acceleration.y = self.accel_delta['ay']
            delta_msg.linear_acceleration.z = self.accel_delta['az']
            self.accel_delta_pub.publish(delta_msg)
            
        except Exception as e:
            self.zero_data_count += 1
            if self.zero_data_count > 50:
                 self.get_logger().warn(f"Failed to read/publish IMU data: {str(e)}")
                 self.zero_data_count = 0
            
    def publish_velocity(self):
        """
        Publish current velocity feedback from MCU encoders
        
        CRITICAL: Apply inverse scaling to angular velocity feedback!
        - MCU reports actual wheel angular velocity (after scaling)
        - Nav2 expects velocity in its command reference frame
        - Without inverse scaling, Nav2 thinks robot is rotating slower than commanded
        - This causes over-rotation as Nav2 keeps sending larger commands
        
        Example:
        - Nav2 commands: 1.0 rad/s
        - Driver sends to MCU: 1.0 × 0.353 = 0.353
        - MCU rotates at: 0.353 rad/s (actual)
        - MCU reports back: vz = 0.353
        - Driver MUST report: 0.353 ÷ 0.353 = 1.0 (back to Nav2 frame)
        - Nav2 sees: "I commanded 1.0, getting 1.0, good!"
        """
        try:
            # Get velocity feedback from MCU encoder measurements
            vx = self.bot._Rosmaster__vx  # Linear velocity (m/s)
            vy = self.bot._Rosmaster__vy  # Lateral velocity (m/s)
            vz = self.bot._Rosmaster__vz  # Angular velocity (rad/s) - MCU scaled
            
            # [2026-04-17] CAR_MECANUM Motion_Get_Speed(): speed_mm = -1 * offset
            # → MCU Vx가 전진 시 음수. ROS 좌표(전진=+x)와 반대이므로 부호 반전 필요.
            vx = -vx
            
            # Apply inverse scaling to angular velocity for Nav2 feedback
            # This ensures command-feedback consistency in Nav2's reference frame
            if self.ANGULAR_SCALE_FACTOR > 0.001:  # Avoid division by zero
                vz_scaled = vz / self.ANGULAR_SCALE_FACTOR
            else:
                vz_scaled = vz
            
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(vz_scaled)  # Inverse scaled for consistency
            
            self.vel_pub.publish(msg)

        except Exception as e:
            # Silently fail to avoid log spam
            pass

    def publish_wheel_speeds(self):
        """
        [2026-05-06] 좌/우 휠 속도 (m/s) 를 /jupiter/wheel_speeds 로 publish.
        Float32MultiArray.data = [left_mps, right_mps].

        매핑: M1=LEFT, M3=RIGHT (Jupiter 차동구동, M2/M4 미사용).
        소스: MCU FUNC_REPORT_WHEEL_SPEED (25Hz). Rosmaster_Lib.get_wheel_speeds().

        Note: vx 와 다르게 부호 반전 안 함. MCU encoder PID feedback 의 wheel-frame 그대로.
        """
        try:
            left_mps, right_mps = self.bot.get_wheel_speeds()
            msg = Float32MultiArray()
            msg.data = [float(left_mps), float(right_mps)]
            self.wheel_speeds_pub.publish(msg)
        except Exception:
            pass

    def publish_encoder_counts(self):
        """
        [2026-05-12] 4 motor raw encoder count 를 /jupiter/encoder_counts 로 publish.
        Int32MultiArray.data = [M1, M2, M3, M4] 누적 count.

        소스: MCU FUNC_REPORT_ENCODER (0x0D). Rosmaster_Lib.get_motor_encoder().
        주기: ~30Hz (firmware 의 report_count==31 마다 auto report).

        진단 목적:
          - wheel_speeds (m/s) 만으로 안 잡히는 raw encoder 의 jitter 측정
          - 4 motor 각각의 회전수 차이 (M2/M4 의 회전 정도 확인)
          - 동일 cmd 에 대한 motor 별 변동성 (특히 우측 wheel 의 변동)

        Note: count 는 누적값. 측정 시 diff 로 변환 필요.
        """
        try:
            m1, m2, m3, m4 = self.bot.get_motor_encoder()
            msg = Int32MultiArray()
            msg.data = [int(m1), int(m2), int(m3), int(m4)]
            self.encoder_pub.publish(msg)
        except Exception:
            pass

    def RobotArmCallback(self, request, response):
        """Handle Robot Arm service requests"""
        try:
            # Implement your arm control logic here
            # For demonstration, just returning success
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error(f"Robot Arm service failed: {str(e)}")
            response.success = False
            return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = JupiterDriver()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        # Cleanup
        try:
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()