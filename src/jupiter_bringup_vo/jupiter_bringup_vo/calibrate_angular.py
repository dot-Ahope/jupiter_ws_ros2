#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, ParameterType
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import math
from . import utilities

class CalibrateAngular(Node):
    def __init__(self):
        super().__init__('calibrate_angular')
        
        # Initialize variables
        self.stop_rotation = False
        self.start_test = False
        self.test_angle = 0.0
        self.actual_angle = 0.0
        self.speed = 0.0
        self.tolerance = 0.0
        self.odom_angular_scale_correction = 1.0
        self.start_time = None
        self.timestamp = None
        
        # Declare parameters with constraints
        self.declare_parameter('test_angle', 360.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=360.0,
                    step=0.1
                )],
                description='Test angle in degrees'
            ))
            
        self.declare_parameter('speed', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=-2.0,
                    to_value=2.0,
                    step=0.1
                )],
                description='Angular speed in radians per second'
            ))
            
        self.declare_parameter('tolerance', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=10.0,
                    step=0.1
                )],
                description='Error tolerance in degrees'
            ))
            
        self.declare_parameter('odom_angular_scale_correction', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=3.0,
                    step=0.01
                )],
                description='Angular correction factor'
            ))
            
        self.declare_parameter('start_test', False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Start the calibration test'
            ))
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Initialize other variables
        self.initial_angle = None
        
        # Create timer for test monitoring
        self.create_timer(0.05, self.test_monitor)  # 20Hz
        
        self.get_logger().info('Angular Calibration Node has been initialized')
        
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            self.get_logger().info(f'Received parameter update for: {param.name}, type: {param.type_}')
            
            if param.name == 'start_test':
                try:
                    # 명시적으로 타입 체크 및 변환
                    start_test_value = bool(param.get_parameter_value().bool_value)
                    self.get_logger().info(f'start_test parameter set to: {start_test_value}')
                    
                    if start_test_value and not self.start_test:
                        self.start_test = True
                        self.test_angle = self.get_parameter('test_angle').get_parameter_value().double_value
                        self.speed = self.get_parameter('speed').get_parameter_value().double_value
                        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
                        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
                        
                        # 로그 추가
                        self.get_logger().info(f'Starting rotation test with parameters:')
                        self.get_logger().info(f'  test_angle: {self.test_angle} degrees')
                        self.get_logger().info(f'  speed: {self.speed} rad/s')
                        self.get_logger().info(f'  tolerance: {self.tolerance} degrees')
                        self.get_logger().info(f'  odom_angular_scale_correction: {self.odom_angular_scale_correction}')
                        
                        self.start_rotation()
                except Exception as e:
                    self.get_logger().error(f'Error processing start_test parameter: {e}')
                    return SetParametersResult(successful=False)
        
        return SetParametersResult(successful=True)
        
    def start_rotation(self):
        """Start the rotation test"""
        self.get_logger().info('Starting rotation test...')
        self.get_logger().info('Waiting for first odometry message...')
        
        # 로깅 변수 초기화
        self._last_log_angle = -1  # 매 10도마다 로그를 출력하기 위한 변수
        
        # 테스트 변수 초기화
        self.initial_angle = None
        self.actual_angle = 0.0
        self.stop_rotation = False
        
    def odom_callback(self, msg):
        """Handle odometry updates"""
        # 오도메트리 데이터 수신 시간 업데이트
        self._odom_received_time = self.get_clock().now().to_msg().sec
        
        if self.start_test:
            current_angle = utilities.quat_to_angle(msg.pose.pose.orientation)
            
            if self.initial_angle is None:
                self.initial_angle = current_angle
                self.get_logger().info(f'Initial angle set to: {self.initial_angle * 180.0 / math.pi:.2f} degrees')
            else:
                # Calculate change in angle
                delta = utilities.normalize_angle(current_angle - self.initial_angle)
                self.actual_angle = abs(delta) * 180.0 / math.pi
                
                # 로그 출력 로직 개선
                # 매 0.5초마다 또는 최소 5도 변화가 있을 때 로그 출력
                current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
                
                if not hasattr(self, '_last_log_time'):
                    self._last_log_time = current_time
                    self._last_log_angle = self.actual_angle
                
                time_diff = current_time - self._last_log_time
                angle_diff = abs(self.actual_angle - self._last_log_angle)
                
                if time_diff >= 0.5 or angle_diff >= 5.0:
                    self.get_logger().info(f'Current rotation: {self.actual_angle:.1f} degrees (target: {self.test_angle} degrees)')
                    self._last_log_time = current_time
                    self._last_log_angle = self.actual_angle
                
                if self.actual_angle >= self.test_angle - self.tolerance:
                    self.get_logger().info(f'Reached target angle: {self.actual_angle:.1f} degrees')
                    self.stop_rotation = True
                    
    def test_monitor(self):
        """Monitor and control the test"""
        if self.start_test and not self.stop_rotation:
            # Send rotation command
            twist = Twist()
            twist.angular.z = self.speed
            self.cmd_vel_pub.publish(twist)
            
            # 첫 번째 명령 실행 시에만 로그 출력
            if not hasattr(self, '_command_sent'):
                self.get_logger().info(f'Sending rotation command: angular.z = {self.speed} rad/s')
                self._command_sent = True
                
            # 오도메트리 데이터 수신 확인
            if not hasattr(self, '_odom_received_time'):
                self._odom_received_time = self.get_clock().now().to_msg().sec
            
            current_time = self.get_clock().now().to_msg().sec
            if current_time - self._odom_received_time > 3:  # 3초 이상 오도메트리 업데이트 없으면 경고
                self.get_logger().warn("오도메트리 데이터가 3초 이상 수신되지 않았습니다!")
                self._odom_received_time = current_time
        
        elif self.stop_rotation:
            # Stop rotation
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            if self.start_test:
                self.start_test = False
                self.stop_rotation = False
                
                # 결과 계산
                error = abs(self.test_angle - self.actual_angle)
                correction_factor = self.test_angle / self.actual_angle if self.actual_angle > 0 else 1.0
                new_scale = self.odom_angular_scale_correction * correction_factor
                
                # 결과 출력
                self.get_logger().info(
                    f'\n========== Rotation Test Results ==========\n' +
                    f'Target angle: {self.test_angle} degrees\n' +
                    f'Actual angle: {self.actual_angle:.1f} degrees\n' +
                    f'Error: {error:.1f} degrees ({(error/self.test_angle)*100:.1f}%)\n' +
                    f'Current scale: {self.odom_angular_scale_correction}\n' +
                    f'Recommended scale: {new_scale:.4f}\n' +
                    f'==========================================='
                )
                
                # 새로운 스케일 값을 파라미터로 설정
                try:
                    self.set_parameters([Parameter('odom_angular_scale_correction', Parameter.Type.DOUBLE, new_scale)])
                    self.get_logger().info(f'Updated odom_angular_scale_correction parameter to {new_scale:.4f}')
                except Exception as e:
                    self.get_logger().error(f'Failed to update parameter: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateAngular()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 키보드 인터럽트 발생 시 로그 메시지 출력
        print("\n키보드 인터럽트 감지: 모터를 정지합니다...")
        if hasattr(node, 'get_logger'):
            node.get_logger().warning('KeyboardInterrupt 발생: 모터를 정지합니다.')
    finally:
        # 어떤 상황에서도 로봇이 안전하게 정지하도록 보장
        if hasattr(node, 'cmd_vel_pub'):
            # 모터 정지를 위해 0 속도 명령 전송
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
            # 명령이 전달되도록 잠시 대기
            import time
            time.sleep(0.1)
            # 한번 더 정지 명령 전송하여 확실히 정지
            node.cmd_vel_pub.publish(stop_cmd)
            print("모터 정지 명령을 전송했습니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
