#!/usr/bin/env python3
"""
Odom 각도 추적 디버그 테스트
- 실제 odom_yaw 값의 변화 추적
- Wrap-around 문제 확인
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

class OdomDebugTest(Node):
    def __init__(self):
        super().__init__('odom_debug_test')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        
        self.odom_yaw = 0.0
        self.start_yaw = None
        self.cumulative_angle = 0.0
        self.last_yaw = None
        
    def odom_callback(self, msg):
        quat = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(quat)
        
        if self.start_yaw is None:
            self.start_yaw = self.odom_yaw
            self.last_yaw = self.odom_yaw
        else:
            # 누적 각도 계산 (wrap-around 고려)
            delta = self.odom_yaw - self.last_yaw
            
            # Wrap-around 감지 및 보정
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            
            self.cumulative_angle += delta
            self.last_yaw = self.odom_yaw
    
    def quaternion_to_yaw(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def test_rotation(self, speed=0.3, duration=3.0):
        """회전 테스트"""
        print(f"\n{'='*70}")
        print(f"Odom 디버그 테스트 - {duration:.1f}초 동안 속도 {speed:.2f} rad/s로 회전")
        print(f"{'='*70}\n")
        
        # 초기화
        self.cumulative_angle = 0.0
        self.start_yaw = None
        
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        print(f"시작 yaw: {math.degrees(self.odom_yaw):7.2f}°")
        print(f"\n{'시간':>6} {'Raw Yaw':>10} {'누적각도':>10} {'예상각도':>10} {'차이':>8}")
        print(f"{'-'*70}")
        
        twist = Twist()
        twist.angular.z = speed
        
        start_time = time.time()
        last_print = start_time
        
        while time.time() - start_time < duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            # 0.5초마다 출력
            if time.time() - last_print > 0.5:
                elapsed = time.time() - start_time
                expected = speed * elapsed
                diff = self.cumulative_angle - expected
                
                print(f"{elapsed:5.1f}s {math.degrees(self.odom_yaw):9.2f}° "
                      f"{math.degrees(self.cumulative_angle):9.2f}° "
                      f"{math.degrees(expected):9.2f}° "
                      f"{math.degrees(diff):+7.2f}°")
                
                last_print = time.time()
        
        # 정지
        twist.angular.z = 0.0
        for _ in range(20):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        
        # 최종 측정
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        elapsed = time.time() - start_time
        expected = speed * duration
        
        print(f"\n{'='*70}")
        print(f"📊 최종 결과:")
        print(f"  명령 시간:    {duration:.1f}초")
        print(f"  명령 속도:    {speed:.2f} rad/s")
        print(f"  예상 회전:    {math.degrees(expected):7.2f}° ({expected:.3f} rad)")
        print(f"  누적 회전:    {math.degrees(self.cumulative_angle):7.2f}° ({self.cumulative_angle:.3f} rad)")
        print(f"  Raw delta:    {math.degrees(self.odom_yaw - self.start_yaw):7.2f}°")
        print(f"  오차:         {math.degrees(self.cumulative_angle - expected):+7.2f}°")
        print(f"  정확도:       {(self.cumulative_angle/expected)*100:.1f}%")
        print(f"{'='*70}\n")


def main():
    rclpy.init()
    
    print("\n" + "="*70)
    print("Odom 디버그 테스트 - Wrap-around 및 누적 각도 추적")
    print("="*70)
    print("\n⚠️  준비사항:")
    print("  1. 로봇 주변 2m 이상 공간 확보")
    print("  2. 시스템 실행: ros2 launch sllidar_ros2 transbot_full_system.launch.py")
    print("\n📋 테스트:")
    print("  - 3초 동안 0.3 rad/s로 회전 (약 51.6°)")
    print("  - Odom raw yaw 값의 wrap-around 확인")
    print("  - 누적 각도 vs 예상 각도 비교")
    print("\n시작하려면 Enter를 누르세요...")
    input()
    
    node = OdomDebugTest()
    
    try:
        node.test_rotation(speed=0.3, duration=3.0)
        
        print("\n추가 테스트 (장시간 회전)?")
        print("  - 10초 회전 (약 172°, wrap-around 테스트)")
        print("y/N: ", end='')
        if input().lower() == 'y':
            node.test_rotation(speed=0.3, duration=10.0)
            
    except KeyboardInterrupt:
        print('\n사용자 중단')
    finally:
        # 정지
        twist = Twist()
        for _ in range(20):
            node.cmd_pub.publish(twist)
            time.sleep(0.05)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
