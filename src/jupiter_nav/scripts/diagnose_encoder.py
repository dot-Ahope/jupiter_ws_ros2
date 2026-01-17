#!/usr/bin/env python3
"""
인코더 데이터 손실 진단 스크립트
- 반시계/시계 회전 시 wheel_odom 발행 빈도 확인
- 좌우 휠 속도 비대칭 검사
- 메시지 드롭 및 지연 분석
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import math

class EncoderDiagnostic(Node):
    def __init__(self):
        super().__init__('encoder_diagnostic')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_callback, 10)
        
        # 통계
        self.odom_count = 0
        self.odom_last_time = None
        self.odom_dt_list = []
        
        self.imu_count = 0
        self.imu_last_time = None
        self.imu_dt_list = []
        
        # 각속도 누적
        self.odom_angular_vel = []
        self.imu_angular_vel = []
        
        self.get_logger().info('인코더 진단 노드 초기화 완료')
    
    def odom_callback(self, msg):
        """Odometry 콜백"""
        current_time = time.time()
        
        if self.odom_last_time is not None:
            dt = current_time - self.odom_last_time
            self.odom_dt_list.append(dt)
        
        self.odom_last_time = current_time
        self.odom_count += 1
        self.odom_angular_vel.append(msg.twist.twist.angular.z)
    
    def imu_callback(self, msg):
        """IMU 콜백"""
        current_time = time.time()
        
        if self.imu_last_time is not None:
            dt = current_time - self.imu_last_time
            self.imu_dt_list.append(dt)
        
        self.imu_last_time = current_time
        self.imu_count += 1
        self.imu_angular_vel.append(msg.angular_velocity.z)
    
    def stop_robot(self):
        """로봇 정지"""
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
    
    def test_rotation(self, direction_name, speed, duration=3.0):
        """회전 테스트"""
        print(f'\n{"="*60}')
        print(f'테스트: {direction_name} (속도: {speed:.2f} rad/s)')
        print(f'{"="*60}')
        
        # 초기화
        self.odom_count = 0
        self.odom_dt_list = []
        self.odom_angular_vel = []
        self.imu_count = 0
        self.imu_dt_list = []
        self.imu_angular_vel = []
        
        # 데이터 수집 대기
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # 회전 시작
        twist = Twist()
        twist.angular.z = speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
        
        # 정지
        self.stop_robot()
        time.sleep(0.5)
        
        # 결과 분석
        self.print_statistics(direction_name)
    
    def print_statistics(self, direction_name):
        """통계 출력"""
        print(f'\n📊 {direction_name} 결과:')
        print(f'{"-"*60}')
        
        # Odom 통계
        if len(self.odom_dt_list) > 0:
            odom_hz = 1.0 / (sum(self.odom_dt_list) / len(self.odom_dt_list))
            odom_max_dt = max(self.odom_dt_list)
            odom_min_dt = min(self.odom_dt_list)
            odom_avg_vel = sum(self.odom_angular_vel) / len(self.odom_angular_vel)
            
            print(f'\n🔵 Odometry (/odom_raw):')
            print(f'  총 메시지 수: {self.odom_count}')
            print(f'  평균 빈도: {odom_hz:.1f} Hz')
            print(f'  메시지 간격: min={odom_min_dt*1000:.1f}ms, max={odom_max_dt*1000:.1f}ms')
            print(f'  평균 각속도: {odom_avg_vel:.3f} rad/s')
            
            # 이상 감지
            if odom_max_dt > 0.2:
                print(f'  ⚠️  메시지 지연 감지: {odom_max_dt*1000:.1f}ms')
            if odom_hz < 20:
                print(f'  ⚠️  발행 빈도 낮음: {odom_hz:.1f} Hz (정상: 30-50 Hz)')
        else:
            print(f'\n❌ Odometry 데이터 없음!')
        
        # IMU 통계
        if len(self.imu_dt_list) > 0:
            imu_hz = 1.0 / (sum(self.imu_dt_list) / len(self.imu_dt_list))
            imu_max_dt = max(self.imu_dt_list)
            imu_min_dt = min(self.imu_dt_list)
            imu_avg_vel = sum(self.imu_angular_vel) / len(self.imu_angular_vel)
            
            print(f'\n🟢 IMU (/imu/data_calibrated):')
            print(f'  총 메시지 수: {self.imu_count}')
            print(f'  평균 빈도: {imu_hz:.1f} Hz')
            print(f'  메시지 간격: min={imu_min_dt*1000:.1f}ms, max={imu_max_dt*1000:.1f}ms')
            print(f'  평균 각속도: {imu_avg_vel:.3f} rad/s')
        else:
            print(f'\n❌ IMU 데이터 없음!')
        
        # 비교
        if len(self.odom_angular_vel) > 0 and len(self.imu_angular_vel) > 0:
            odom_avg = sum(self.odom_angular_vel) / len(self.odom_angular_vel)
            imu_avg = sum(self.imu_angular_vel) / len(self.imu_angular_vel)
            
            if abs(imu_avg) > 0.01:
                ratio = odom_avg / imu_avg
                print(f'\n🔄 각속도 비교:')
                print(f'  Odom / IMU = {ratio:.4f}')
                
                if abs(ratio - 1.0) > 0.3:
                    print(f'  ❌ 각속도 불일치! (정상: 0.7~1.3)')


def main():
    rclpy.init()
    
    print('\n' + '='*60)
    print('인코더 데이터 손실 진단')
    print('='*60)
    print('\n📋 테스트 순서:')
    print('  1. 반시계 회전 (3초)')
    print('  2. 시계 회전 (3초)')
    print('  3. 메시지 빈도 및 지연 분석')
    print('\n⚠️  시스템이 실행 중이어야 합니다:')
    print('     ros2 launch sllidar_ros2 jupiter_full_system.launch.py')
    print('\n시작하려면 Enter를 누르세요...')
    input()
    
    node = EncoderDiagnostic()
    
    try:
        # 센서 대기
        print('\n센서 데이터 대기 중...')
        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.odom_last_time is not None and node.imu_last_time is not None:
                print('✅ 센서 준비 완료\n')
                time.sleep(1.0)
                break
        else:
            print('❌ 센서 타임아웃!')
            return
        
        # 테스트 1: 반시계
        node.test_rotation('반시계 회전 (CCW)', speed=0.3, duration=3.0)
        time.sleep(2.0)
        
        # 테스트 2: 시계
        node.test_rotation('시계 회전 (CW)', speed=-0.3, duration=3.0)
        
        print('\n' + '='*60)
        print('진단 완료')
        print('='*60)
        
    except KeyboardInterrupt:
        print('\n사용자 중단')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
