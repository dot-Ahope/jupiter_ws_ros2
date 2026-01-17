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
    def __init__(self):
        super().__init__('jupiter_driver')
        
        # Initialize hardware with retry mechanism
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # Initialize Rosmaster with X3 car type (mecanum wheel)
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
        self.declare_parameter('Kp', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=2.0,
                    step=0.01
                )],
                description='Proportional gain'
            ))
            
        self.declare_parameter('Ki', 0.1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=1.0,
                    step=0.01
                )],
                description='Integral gain'
            ))
            
        self.declare_parameter('Kd', 0.05,
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
        
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        try:
            # Get velocity commands from twist message
            vx = msg.linear.x  # m/s
            vy = msg.linear.y  # m/s (for mecanum wheels)
            angular = msg.angular.z  # rad/s
            
            # Save last command for fake odometry
            self.last_cmd_vel = msg
            self.last_cmd_time = self.get_clock().now()
            
            # Use Rosmaster's set_car_motion for direct velocity control
            # This handles all the motor calculations internally
            self.bot.set_car_motion(vx, vy, angular)
            
            # Debug output (optional)
            # self.get_logger().info(f'Velocity command: vx={vx:.3f}, vy={vy:.3f}, angular={angular:.3f}')
            
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
            gyro_cov = 0.001  # 각속도 측정 불확실성 (rad/s)^2
            
            # 초기 공분산 행렬 설정
            msg.orientation_covariance = [-1.0] * 9  # 방향은 여전히 제공되지 않음
            
            # 각속도 공분산 (대각선에 불확실성 값 설정)
            msg.angular_velocity_covariance = [
                gyro_cov, 0.0, 0.0,
                0.0, gyro_cov, 0.0,
                0.0, 0.0, gyro_cov
            ]
            
            # 선형 가속도 공분산 (대각선에 불확실성 값 설정)
            msg.linear_acceleration_covariance = [
                accel_cov, 0.0, 0.0,
                0.0, accel_cov, 0.0,
                0.0, 0.0, accel_cov
            ]
            
            # Fill in angular velocity (rad/s)
            # Calibration Result (2026-01-05):
            # Current Gain: 0.5
            # Measured: 179.62 deg for 360 deg physical rotation
            # Correction Factor: 2.0042
            # New Gain: 0.5 * 2.0042 = 1.0021
            sensitivity_gain = 1.0021
            
            # Apply Inversion if requested
            if self.get_parameter('imu_invert_x').value:
                gx = -gx
            if self.get_parameter('imu_invert_y').value:
                gy = -gy
            if self.get_parameter('imu_invert_z').value:
                gz = -gz
                
            msg.angular_velocity.x = float(gx) * sensitivity_gain
            msg.angular_velocity.y = float(gy) * sensitivity_gain
            msg.angular_velocity.z = float(gz) * sensitivity_gain
            
            # Fill in linear acceleration (m/s^2)
            # IMU에서 제공하는 가속도 값은 g 단위로 정규화되어 있음
            # 1g = 9.80665 m/s^2 이므로 이 값을 곱해서 m/s^2 단위로 변환
            GRAVITY = 9.80665  # 표준 중력가속도 (m/s^2)
            
            # IMU 좌표계를 로봇 좌표계로 변환
            # Jupiter의 경우 일반적으로:
            # x축: 로봇의 전진/후진 방향
            # y축: 로봇의 좌/우 방향
            # z축: 로봇의 위/아래 방향
            
            # 로봇 좌표계 기준으로 재정렬 
            # IMU 축 보정 결과에 따른 매핑:
            # - 로봇 전후 방향(cmd_vel의 linear.x)은 IMU의 Y축
            # - 로봇 좌우 방향(cmd_vel의 linear.y)은 IMU의 X축
            # - 로봇 상하 방향(cmd_vel의 linear.z)은 IMU의 Z축
            robot_ax = float(ax)  # 전진/후진 방향은 IMU의 Y축
            robot_ay = float(ay)  # 좌/우 방향은 IMU의 X축
            robot_az = float(az)  # 상/하 방향은 IMU의 Z축
            
            # g 단위 가속도를 m/s^2 단위로 변환
            msg.linear_acceleration.x = robot_ax * GRAVITY
            msg.linear_acceleration.y = robot_ay * GRAVITY
            msg.linear_acceleration.z = robot_az * GRAVITY
            
            # 디버깅을 위해 변환된 선형 가속도 출력
            #self.get_logger().info(f"Linear accel: x={msg.linear_acceleration.x:.6f}, y={msg.linear_acceleration.y:.6f}, z={msg.linear_acceleration.z:.6f}")
            
            # Publish the message
            self.imu_pub.publish(msg)
            
            # 가속도 변화량 메시지 발행 (별도의 토픽)
            if hasattr(self, 'accel_delta'):
                delta_msg = Imu()
                delta_msg.header.stamp = self.get_clock().now().to_msg()
                delta_msg.header.frame_id = "imu_link"
                
                # 변화량을 m/s^2 단위로 변환
                GRAVITY = 9.80665  # 표준 중력가속도 (m/s^2)
                delta_msg.linear_acceleration.x = float(self.accel_delta['ax']) * GRAVITY
                delta_msg.linear_acceleration.y = float(self.accel_delta['ay']) * GRAVITY
                delta_msg.linear_acceleration.z = float(self.accel_delta['az']) * GRAVITY
                
                # 변화량 임계값 설정 (노이즈 필터링) - 값 감소로 더 민감하게 반응
                threshold_g = 0.0005  # g 단위 임계값 (0.001 → 0.0005로 감소)
                threshold = threshold_g * GRAVITY  # m/s^2 단위로 변환
                
                if (abs(delta_msg.linear_acceleration.x) > threshold or
                    abs(delta_msg.linear_acceleration.y) > threshold or
                    abs(delta_msg.linear_acceleration.z) > threshold):
                    # 임계값을 초과하는 경우만 메시지 발행 (노이즈 제거)
                    self.accel_delta_pub.publish(delta_msg)
                    # self.get_logger().info(f"Movement detected: dx={delta_msg.linear_acceleration.x:.6f}, "
                    #                        f"dy={delta_msg.linear_acceleration.y:.6f}, "
                    #                        f"dz={delta_msg.linear_acceleration.z:.6f}")
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish IMU data: {str(e)}')
            
    def publish_velocity(self):
        """Publish current velocity"""
        try:
            # Get current velocity from hardware
            vx, vy, angular = self.bot.get_motion_data()
            
            # If hardware reports zero velocity (e.g. encoders removed), use commanded velocity (Open Loop)
            # Only if we have received a command recently (within 0.5s)
            if vx == 0.0 and vy == 0.0 and angular == 0.0:
                if hasattr(self, 'last_cmd_vel') and hasattr(self, 'last_cmd_time'):
                    time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
                    if time_diff < 0.5:
                        # Apply Deadzone Logic for Fake Odometry
                        # Nav2 ramps velocity, but robot doesn't move until threshold is reached.
                        # This prevents integrating velocity when robot is actually stationary.
                        
                        cmd_vx = self.last_cmd_vel.linear.x
                        cmd_vy = self.last_cmd_vel.linear.y
                        cmd_az = self.last_cmd_vel.angular.z
                        
                        # Linear Deadzone (User reported: 0.20 m/s)
                        if abs(cmd_vx) < 0.20: cmd_vx = 0.0
                        if abs(cmd_vy) < 0.20: cmd_vy = 0.0
                        
                        # Angular Deadzone (User reported: 2.5 rad/s)
                        if abs(cmd_az) < 2.5: cmd_az = 0.0
                        
                        vx = cmd_vx
                        vy = cmd_vy
                        angular = cmd_az
            
            # Create and fill velocity message
            msg = Twist()
            msg.linear.x = float(vx)   # linear velocity in m/s
            msg.linear.y = float(vy)   # lateral velocity in m/s (mecanum wheels)
            msg.angular.z = float(angular)  # angular velocity in rad/s
            
            # Publish velocity data
            self.vel_pub.publish(msg)
            
            # Also publish battery voltage
            battery_voltage = self.bot.get_battery_voltage()
            # Optional: publish battery data if needed
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish velocity data: {str(e)}')
            
    def RobotArmCallback(self, request, response):
        """Handle RobotArm service requests"""
        try:
            if request.command == "getJoint":
                # Get current joint angles
                current_angles = self.arm.get_current_angles()
                if current_angles is not None and len(current_angles) >= 3:
                    response.angles = [float(angle) for angle in current_angles[:3]]
                    response.result = True
                    self.get_logger().info(f'Current joint angles: {response.angles}')
                else:
                    response.result = False
                    self.get_logger().warn('Failed to get joint angles')
            else:
                # Handle other commands if needed
                response.result = False
                self.get_logger().warn(f'Unknown RobotArm command: {request.command}')
        except Exception as e:
            response.result = False
            self.get_logger().error(f'RobotArm service error: {str(e)}')
        
        return response

    def destroy_node(self):
        """Cleanup when node is shut down"""
        try:
            # Stop all motors
            self.bot.set_motor(0, 0, 0, 0)
            # Or use car motion with zero velocity
            self.bot.set_car_motion(0.0, 0.0, 0.0)
        except:
            self.get_logger().warn('Failed to stop motors during shutdown')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # 로그 레벨 설정 - INFO 이상의 모든 로그 메시지 표시
    node = JupiterDriver()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
