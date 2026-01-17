#!/usr/bin/env python3
"""
IMU 자이로 바이어스 제거 노드

MPU6050 자이로스코프의 정적 바이어스를 실시간으로 제거합니다.
gyro_bias_calibration.py로 측정한 바이어스 값을 사용합니다.

데이터 흐름:
  jupiter_driver → /jupiter/imu → [이 노드] → /imu/data_raw → EKF

저장된 바이어스를 각속도에서 빼서 정지 상태의 드리프트를 제거합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import yaml
import os


class ImuBiasRemover(Node):
    def __init__(self):
        super().__init__('imu_bias_remover')
        
        # 바이어스 파일 경로
        self.declare_parameter('bias_file', '/home/user/jupiter_ws_ros2/imu_gyro_bias.yaml')
        bias_file = self.get_parameter('bias_file').value
        
        # 바이어스 로드
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0
        
        if os.path.exists(bias_file):
            try:
                with open(bias_file, 'r') as f:
                    bias_data = yaml.safe_load(f)
                    self.bias_x = bias_data['gyro_bias']['x']
                    self.bias_y = bias_data['gyro_bias']['y']
                    self.bias_z = bias_data['gyro_bias']['z']
                    
                self.get_logger().info('=' * 60)
                self.get_logger().info('✅ 자이로 바이어스 보정 활성화')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'바이어스 파일: {bias_file}')
                self.get_logger().info(f'Gyro X bias: {self.bias_x:+.6f} rad/s')
                self.get_logger().info(f'Gyro Y bias: {self.bias_y:+.6f} rad/s')
                self.get_logger().info(f'Gyro Z bias: {self.bias_z:+.6f} rad/s ({self.bias_z * 57.2958:+.2f}°/s)')
                self.get_logger().info('=' * 60)
            except Exception as e:
                self.get_logger().error(f'바이어스 파일 로드 실패: {e}')
                self.get_logger().warn('⚠️  바이어스 보정 없이 실행됩니다')
        else:
            self.get_logger().warn('=' * 60)
            self.get_logger().warn('⚠️  바이어스 파일이 없습니다')
            self.get_logger().warn(f'   {bias_file}')
            self.get_logger().warn('')
            self.get_logger().warn('먼저 캘리브레이션을 실행하세요:')
            self.get_logger().warn('  python3 gyro_bias_calibration.py')
            self.get_logger().warn('')
            self.get_logger().warn('바이어스 보정 없이 실행됩니다 (드리프트 발생 가능)')
            self.get_logger().warn('=' * 60)
        
        # 구독 및 발행 설정
        self.subscription = self.create_subscription(
            Imu,
            '/jupiter/imu',
            self.imu_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )
        
        self.get_logger().info('IMU 바이어스 제거 노드 시작')
        self.get_logger().info('  입력: /jupiter/imu')
        self.get_logger().info('  출력: /imu/data_raw')
        
    def imu_callback(self, msg):
        # 바이어스 제거
        corrected_msg = Imu()
        corrected_msg.header = msg.header
        
        # 각속도 보정 (바이어스 제거)
        corrected_msg.angular_velocity.x = msg.angular_velocity.x - self.bias_x
        corrected_msg.angular_velocity.y = msg.angular_velocity.y - self.bias_y
        corrected_msg.angular_velocity.z = msg.angular_velocity.z - self.bias_z
        corrected_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        # 나머지는 그대로 복사
        corrected_msg.orientation = msg.orientation
        corrected_msg.orientation_covariance = msg.orientation_covariance
        corrected_msg.linear_acceleration = msg.linear_acceleration
        corrected_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # 발행
        self.publisher.publish(corrected_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasRemover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
