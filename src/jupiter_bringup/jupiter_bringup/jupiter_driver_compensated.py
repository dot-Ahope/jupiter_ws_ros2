#!/usr/bin/env python3
# encoding: utf-8
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, ParameterType
from std_msgs.msg import Float32
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
    ANGULAR_SCALE_FACTOR = 1.0  # 모터 교체 → 보정 없이 1:1 전달

    # 방안C: MCU 엔코더 분해능 한계 보정 (정지 회전 시 최소 각속도)
    # 엔코더: 1320 pulse/rev, 10ms 측정주기 → 1 tick = 16.3 mm/s
    # PID target < 16.3 mm/s → 0 tick과 1 tick 사이 진동 → 전진↔후진 반복 → 순회전 0
    # MCU_MIN_ANGULAR = 0.12 → 19.7 mm/s → 1.21 ticks/period
    # 주의: min_speed_theta(0.30) × SCALE(0.401) = 0.120 ≥ 0.12 → DWB 명령에 클램프 미적용
    # (0.15일 때 0.120 < 0.15 → 클램프 0.15 → 실효 0.374 = 24% 증폭 → 오버슈트→헌팅)
    # MCU_MIN_ANGULAR = 0.12
    MCU_MIN_ANGULAR = 0.0  # 모터 교체 → 클램프 비활성화
    
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
        self.declare_parameter('Ki', 0.02,
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
            
            # 방안C: MCU 엔코더 분해능 한계 보정 (정지 회전 전용) - 모터 교체로 비활성화
            if abs(vx) < 0.02:  # 정지 상태에서만 클램프
                if abs(angular_corrected) > 0.001 and abs(angular_corrected) < self.MCU_MIN_ANGULAR:
                    angular_corrected = math.copysign(self.MCU_MIN_ANGULAR, angular_corrected)
            
            # Apply minimal deadband to reduce command jitter and motor noise
            # Small values near zero often result from Nav2 controller noise
            if abs(vx) < 0.001:
                vx = 0.0
            if abs(angular_corrected) < 0.001:
                angular_corrected = 0.0
            
            # Save last command timestamp for odometry and timeout detection
            self.last_cmd_vel = msg
            self.last_cmd_time = self.get_clock().now()
            
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