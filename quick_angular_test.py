#!/usr/bin/env python3
"""
빠른 Angular Scale 테스트 - 90도만
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time


class QuickAngularTest(Node):
    def __init__(self):
        super().__init__('quick_angular_test')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_yaw = 0.0
        self.odom_received = False
        
        self.imu_angular_vel_z = 0.0
        self.last_imu_time = None
        self.integrated_imu_yaw = 0.0
        self.imu_received = False
        
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(q)
        self.odom_received = True
        
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        self.imu_angular_vel_z = msg.angular_velocity.z
        
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            if dt < 1.0:
                self.integrated_imu_yaw += self.imu_angular_vel_z * dt
        
        self.last_imu_time = current_time
        self.imu_received = True
    
    def stop(self):
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
    
    def test_90_degrees(self):
        print('\n' + '='*70)
        print('90도 회전 테스트')
        print('='*70)
        
        # 센서 대기
        print('\n센서 데이터 대기 중...')
        timeout = time.time() + 5.0
        while time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_received and self.imu_received:
                print('✅ 센서 준비 완료\n')
                break
        else:
            print('❌ 센서 데이터 수신 실패!')
            return
        
        time.sleep(1.0)
        
        # 초기값 기록
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        start_odom = self.odom_yaw
        self.integrated_imu_yaw = 0.0
        start_time = time.time()
        
        print(f'시작 Odom yaw: {math.degrees(start_odom):.2f}°')
        print('회전 시작...\n')
        
        # 0.3 rad/s로 회전
        twist = Twist()
        twist.angular.z = 0.3
        
        target_rad = math.radians(90)
        
        while True:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            elapsed = time.time() - start_time
            imu_deg = math.degrees(abs(self.integrated_imu_yaw))
            
            # 0.5초마다 진행률 표시
            if int(elapsed * 2) != int((elapsed - 0.02) * 2):
                print(f'진행: {imu_deg:6.1f}° | 시간: {elapsed:.1f}s')
            
            # IMU 기준 95% 도달
            if abs(self.integrated_imu_yaw) >= target_rad * 0.95:
                print(f'\n✅ 목표 도달! (IMU: {imu_deg:.1f}°)')
                break
            
            # 타임아웃 (10초)
            if elapsed > 10.0:
                print('\n⚠️  타임아웃!')
                break
        
        # 정지
        self.stop()
        time.sleep(1.0)
        
        # 최종 측정
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        final_odom = self.odom_yaw
        final_imu_integrated = self.integrated_imu_yaw
        
        # 회전량 계산
        odom_delta = final_odom - start_odom
        while odom_delta > math.pi:
            odom_delta -= 2 * math.pi
        while odom_delta < -math.pi:
            odom_delta += 2 * math.pi
        
        odom_deg = math.degrees(abs(odom_delta))
        imu_deg = math.degrees(abs(final_imu_integrated))
        
        # angular_scale 계산
        if odom_deg > 1.0:
            angular_scale = imu_deg / odom_deg
        else:
            angular_scale = 0.0
        
        # 결과 출력
        print('\n' + '='*70)
        print('📊 측정 결과')
        print('='*70)
        print(f'목표:            90.0°')
        print(f'')
        print(f'IMU (적분):     {imu_deg:6.2f}°')
        print(f'Odom (raw):     {odom_deg:6.2f}° (보정 전)')
        print(f'')
        print(f'angular_scale:  {angular_scale:.4f} ⭐')
        print(f'')
        print(f'소요 시간:      {time.time() - start_time:.1f}초')
        print('='*70)
        
        return angular_scale


def main():
    rclpy.init()
    
    node = QuickAngularTest()
    
    try:
        print('\n빠른 Angular Scale 측정 (90도 단일 테스트)')
        print('시작하려면 Enter를 누르세요...')
        input()
        
        scale = node.test_90_degrees()
        
        if scale:
            print(f'\n✅ 측정 완료!')
            print(f'⭐ angular_scale: {scale:.4f}')
            print(f'\nPhase 2를 실행하려면:')
            print(f'  python3 odom_based_angular_calibration.py --phase 2 --scale {scale:.4f}')
    
    except KeyboardInterrupt:
        print('\n중단됨')
        node.stop()
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
