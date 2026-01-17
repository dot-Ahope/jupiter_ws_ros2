#!/usr/bin/env python3
"""
자이로 바이어스 측정 스크립트

사용법:
1. 로봇을 완전히 정지시킴
2. python3 measure_gyro_bias.py
3. 100개 샘플 수집 후 평균 출력
4. imu_calib.yaml에 수동으로 추가
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class GyroBiasMeasurer(Node):
    def __init__(self):
        super().__init__('gyro_bias_measurer')
        
        self.samples_x = []
        self.samples_y = []
        self.samples_z = []
        self.target_samples = 100
        
        self.subscription = self.create_subscription(
            Imu,
            '/transbot/imu',  # 원시 IMU 데이터
            self.imu_callback,
            10
        )
        
        self.get_logger().info('🔍 자이로 바이어스 측정 시작...')
        self.get_logger().info('⚠️  로봇을 완전히 정지시키세요!')
        self.get_logger().info(f'   {self.target_samples}개 샘플 수집 중...')
        
    def imu_callback(self, msg):
        if len(self.samples_z) >= self.target_samples:
            return
            
        self.samples_x.append(msg.angular_velocity.x)
        self.samples_y.append(msg.angular_velocity.y)
        self.samples_z.append(msg.angular_velocity.z)
        
        if len(self.samples_z) % 10 == 0:
            self.get_logger().info(f'   진행: {len(self.samples_z)}/{self.target_samples}')
        
        if len(self.samples_z) == self.target_samples:
            self.calculate_bias()
            rclpy.shutdown()
    
    def calculate_bias(self):
        bias_x = np.mean(self.samples_x)
        bias_y = np.mean(self.samples_y)
        bias_z = np.mean(self.samples_z)
        
        std_x = np.std(self.samples_x)
        std_y = np.std(self.samples_y)
        std_z = np.std(self.samples_z)
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('📊 자이로 바이어스 측정 결과:')
        self.get_logger().info('-' * 70)
        self.get_logger().info(f'gyro_bias:')
        self.get_logger().info(f'  x: {bias_x:.10f}  (std: {std_x:.6f})')
        self.get_logger().info(f'  y: {bias_y:.10f}  (std: {std_y:.6f})')
        self.get_logger().info(f'  z: {bias_z:.10f}  (std: {std_z:.6f}) ⭐')
        self.get_logger().info('')
        self.get_logger().info('예상 드리프트 (정지 시):')
        self.get_logger().info(f'  X축: {bias_x * 60:.4f}°/min')
        self.get_logger().info(f'  Y축: {bias_y * 60:.4f}°/min')
        self.get_logger().info(f'  Z축: {bias_z * 60:.4f}°/min ({bias_z * 3600:.1f}°/hour) ⚠️')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('📝 다음 내용을 imu_calib.yaml에 추가하세요:')
        self.get_logger().info('')
        self.get_logger().info('gyro_bias:')
        self.get_logger().info(f'  x: {bias_x:.10f}')
        self.get_logger().info(f'  y: {bias_y:.10f}')
        self.get_logger().info(f'  z: {bias_z:.10f}')
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        
def main(args=None):
    rclpy.init(args=args)
    measurer = GyroBiasMeasurer()
    
    try:
        rclpy.spin(measurer)
    except KeyboardInterrupt:
        pass
    finally:
        measurer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
