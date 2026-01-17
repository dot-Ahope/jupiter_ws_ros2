#!/usr/bin/env python3
"""
자이로 바이어스 보정 스크립트

IMU 센서(MPU6050)의 자이로스코프 바이어스를 측정하고 저장합니다.
로버를 완전히 정지시킨 상태에서 실행해야 합니다.

사용법:
    python3 gyro_bias_calibration.py

출력:
    imu_gyro_bias.yaml - 측정된 바이어스 값
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import yaml
import numpy as np
from collections import deque


class GyroBiasCalibrator(Node):
    def __init__(self):
        super().__init__('gyro_bias_calibrator')
        
        # 샘플 수집 설정
        self.sample_count = 200  # 20초 동안 수집 (10Hz)
        self.samples_x = deque(maxlen=self.sample_count)
        self.samples_y = deque(maxlen=self.sample_count)
        self.samples_z = deque(maxlen=self.sample_count)
        
        # IMU 구독
        self.subscription = self.create_subscription(
            Imu,
            '/transbot/imu',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('자이로 바이어스 보정 시작')
        self.get_logger().info('=' * 60)
        self.get_logger().info('⚠️  로버를 완전히 정지시키고 움직이지 마세요!')
        self.get_logger().info(f'📊 {self.sample_count}개 샘플 수집 중... (약 20초)')
        self.get_logger().info('')
        
    def imu_callback(self, msg):
        # 각속도 샘플 저장
        self.samples_x.append(msg.angular_velocity.x)
        self.samples_y.append(msg.angular_velocity.y)
        self.samples_z.append(msg.angular_velocity.z)
        
        # 진행 상황 표시
        if len(self.samples_z) % 20 == 0:
            progress = len(self.samples_z) / self.sample_count * 100
            self.get_logger().info(f'진행: {progress:.0f}% ({len(self.samples_z)}/{self.sample_count})')
        
        # 충분한 샘플 수집 완료
        if len(self.samples_z) >= self.sample_count:
            self.calculate_and_save_bias()
            rclpy.shutdown()
    
    def calculate_and_save_bias(self):
        # 평균 및 표준편차 계산
        bias_x = float(np.mean(self.samples_x))
        bias_y = float(np.mean(self.samples_y))
        bias_z = float(np.mean(self.samples_z))
        
        std_x = float(np.std(self.samples_x))
        std_y = float(np.std(self.samples_y))
        std_z = float(np.std(self.samples_z))
        
        # 결과 출력
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ 자이로 바이어스 측정 완료')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Gyro X bias: {bias_x:+.6f} rad/s (std: {std_x:.6f})')
        self.get_logger().info(f'Gyro Y bias: {bias_y:+.6f} rad/s (std: {std_y:.6f})')
        self.get_logger().info(f'Gyro Z bias: {bias_z:+.6f} rad/s (std: {std_z:.6f})')
        self.get_logger().info('')
        self.get_logger().info(f'📈 예상 드리프트 (바이어스 보정 전):')
        self.get_logger().info(f'   Z축 (Yaw): {abs(bias_z * 57.2958):.2f}°/sec = {abs(bias_z * 57.2958 * 60):.1f}°/min')
        self.get_logger().info('')
        
        # YAML 파일로 저장
        bias_data = {
            'gyro_bias': {
                'x': bias_x,
                'y': bias_y,
                'z': bias_z
            },
            'gyro_std': {
                'x': std_x,
                'y': std_y,
                'z': std_z
            },
            'sample_count': self.sample_count,
            'calibration_info': 'Measured with robot stationary. Apply before EKF.'
        }
        
        output_file = '/home/user/transbot_ws_ros2/imu_gyro_bias.yaml'
        with open(output_file, 'w') as f:
            yaml.dump(bias_data, f, default_flow_style=False)
        
        self.get_logger().info(f'💾 바이어스 값 저장: {output_file}')
        self.get_logger().info('')
        self.get_logger().info('다음 단계:')
        self.get_logger().info('  1. 시스템을 재시작하세요')
        self.get_logger().info('  2. imu_bias_remover 노드가 자동으로 바이어스를 제거합니다')
        self.get_logger().info('  3. 정지 상태에서 드리프트가 사라졌는지 확인하세요')
        self.get_logger().info('=' * 60)


def main():
    rclpy.init()
    
    calibrator = GyroBiasCalibrator()
    
    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
