#!/usr/bin/env python3
"""
빠른 인코더 품질 테스트
- 3초 회전하면서 /odom_raw 메시지 품질 확인
- CCW vs CW 비교
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import math

class QuickEncoderTest(Node):
    def __init__(self):
        super().__init__('quick_encoder_test')
        self.odom_sub = self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg_count = 0
        self.start_yaw = None
        self.current_yaw = None
        
    def odom_cb(self, msg):
        self.msg_count += 1
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        
    def test(self, direction, duration=3.0):
        self.msg_count = 0
        self.start_yaw = self.current_yaw
        speed = 0.3 if direction == 'CCW' else -0.3
        
        print(f'\n{"="*50}')
        print(f'{direction} 회전 테스트 ({duration}초)')
        print(f'{"="*50}')
        
        twist = Twist()
        twist.angular.z = speed
        
        start = time.time()
        while time.time() - start < duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
        
        twist.angular.z = 0.0
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        
        rotation = abs(self.current_yaw - self.start_yaw) if self.start_yaw else 0
        expected_msgs = int(duration * 50)  # 50Hz 예상
        msg_rate = self.msg_count / duration
        
        print(f'\n결과:')
        print(f'  회전량:      {math.degrees(rotation):.2f}°')
        print(f'  수신 메시지: {self.msg_count}개')
        print(f'  예상 메시지: {expected_msgs}개')
        print(f'  메시지 주기: {msg_rate:.1f} Hz')
        
        if self.msg_count < expected_msgs * 0.8:
            print(f'  ❌ 심각한 메시지 손실! ({(1-self.msg_count/expected_msgs)*100:.1f}%)')
        elif self.msg_count < expected_msgs * 0.95:
            print(f'  ⚠️  일부 메시지 손실 ({(1-self.msg_count/expected_msgs)*100:.1f}%)')
        else:
            print(f'  ✅ 정상')
        
        if rotation < math.radians(60):
            print(f'  ❌ 회전량 부족! (예상 90° 대비 {math.degrees(rotation):.1f}°)')
        
        return rotation, self.msg_count

def main():
    rclpy.init()
    node = QuickEncoderTest()
    
    print('\n인코더 빠른 진단')
    print('시스템이 실행 중인지 확인하세요.')
    print('Enter를 눌러 시작...')
    input()
    
    # 센서 대기
    print('\n센서 준비 중...')
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if node.current_yaw is None:
        print('❌ /odom_raw 토픽 수신 실패!')
        return
    
    print('✅ 준비 완료\n')
    time.sleep(1)
    
    # 테스트
    ccw_rot, ccw_msgs = node.test('CCW', 3.0)
    time.sleep(2)
    cw_rot, cw_msgs = node.test('CW', 3.0)
    
    # 비교
    print(f'\n{"="*50}')
    print('비교 분석')
    print(f'{"="*50}')
    print(f'CCW 회전: {math.degrees(ccw_rot):.2f}° ({ccw_msgs} msgs)')
    print(f'CW  회전: {math.degrees(cw_rot):.2f}° ({cw_msgs} msgs)')
    print(f'비대칭:   {abs(math.degrees(ccw_rot - cw_rot)):.2f}°')
    
    if abs(ccw_msgs - cw_msgs) > 10:
        print(f'\n⚠️  방향별 메시지 수 차이: {abs(ccw_msgs - cw_msgs)}개')
        print('→ 특정 방향 인코더 문제 의심')
    
    if abs(ccw_rot - cw_rot) > 0.3:
        print(f'\n⚠️  회전량 비대칭: {abs(math.degrees(ccw_rot - cw_rot)):.2f}°')
        print('→ angular_scale 보정 필요')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
