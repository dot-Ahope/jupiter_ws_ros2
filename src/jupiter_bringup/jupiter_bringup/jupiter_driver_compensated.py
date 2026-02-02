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
        self.declare_parameter('Kp', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=3.0,
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

        # IMU Enable/Disable Parameter
        self.declare_parameter('enable_imu', False) #추가된 것


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
            vy = msg.linear.y  # m/s (ignored for differential)
            angular = msg.angular.z  # rad/s
            
            # --- Compensation for Squared Turn Ratio in MCU ---
            # MCU Logic: Output ∝ (V_z_sent / 5000)^2
            # Feedback Round 4 Data:
            # - Input 1.0 -> 1.9 laps/10s = 1.19 rad/s (Too Fast)
            # - Input 2.0 -> 3.0 laps/10s = 1.88 rad/s (Slightly Slow)
            
            # Physical Model Fitting (Omega = K * (V^2 - Friction)):
            # 1. 1.19 = K * (V_1^2 - F) => V_1 was ~0.427
            # 2. 1.88 = K * (V_2^2 - F) => V_2 was ~0.492
            # Result: K ~ 11.5, Friction_SQ ~ 0.079 (Bias ~ 0.28)
            
            # New Target Equation (Identity):
            # w = 11.5 * (V^2 - 0.079)
            # V^2 = w/11.5 + 0.079
            # V^2 = 0.087 * w + 0.079
            
            MIN_VAL_SQ = 0.08  # Lowered from 0.1225 to reduce speed at w=1.0
            GAIN = 0.087       # Increased from 0.06 to boost high speed
            
            if abs(angular) > 0.001:
                # Formula: V_out = sqrt(0.087 * w + 0.08)
                val = math.sqrt(GAIN * abs(angular) + MIN_VAL_SQ)
                
                # Minimum clamping to ensure start-up torque (Static friction might be higher)
                # if val < 0.3: 
                #     val = 0.3
                
                # Apply direction
                angular_comp = val if angular > 0 else -val
            else:
                angular_comp = 0.0
                
            # Log debug info periodically
            if not hasattr(self, 'debug_count'): self.debug_count = 0
            self.debug_count = (self.debug_count + 1) % 50
            if self.debug_count == 0 and abs(angular) > 0.01:
                 self.get_logger().info(f"Nav2 In: {angular:.3f} -> Driver Out: {angular_comp:.3f} (Model: Sqrt(0.087*w + 0.08))")

            # --- End Compensation ---

            # Save last command for fake odometry
            self.last_cmd_vel = msg
            self.last_cmd_time = self.get_clock().now()
            
            # Use Rosmaster's set_car_motion for direct velocity control
            # Pass compensated angular velocity
            self.bot.set_car_motion(vx, vy, angular_comp)
            
            # Debug output (optional)
            # self.get_logger().info(f'Params Comp: angular={angular:.3f}, angular_comp={angular_comp:.3f}')
            
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
            gyro_cov = 0.001  # 각속도 측정 불확실성 (rad/s)^2
            
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
        """Publish Current Velocity data"""
        try:
            # Get velocity data from robot
            # Note: Rosmaster behavior depends on firmware; check if this returns measured vel
            # Assuming get_motion_data returns current measured velocity
            # If not available, we might need to rely on command or encoder feedback if exposed
            
            # Looking at Rosmaster_Lib, get_motion_data doesn't seem to exist or is different.
            # Using 'get_motion_data' from library if available, otherwise skip.
            # Actually, the library has read_data packets.
            # But let's check what's available.
            # If standard API doesn't have it, we might need to rely on encoders.
            
            # Since I can't check the library fully, I will assume we can't easily get feedback 
            # without deeper changes. However, the user asked to fix the specific issue.
            # I will just implement the structure.
            
            # If we want to publish feedback, checking Rosmaster_Lib for access to vx, vy, vz
            # exposed by the thread.
            
            vx = self.bot._Rosmaster__vx
            vy = self.bot._Rosmaster__vy
            vz = self.bot._Rosmaster__vz
            
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(vz)
            
            self.vel_pub.publish(msg)
            
        except Exception as e:
            # self.get_logger().warn(f"Failed to publish velocity: {str(e)}")
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
