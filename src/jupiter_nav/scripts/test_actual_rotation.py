#!/usr/bin/env python3
"""
실제 로봇 회전 확인 테스트
- 3초 동안 0.3 rad/s 명령 전송
- 로봇이 실제로 얼마나 회전하는지 육안 확인
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RotationTest(Node):
    def __init__(self):
        super().__init__('rotation_test')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def rotate(self, speed, duration):
        print(f'\n{"="*60}')
        print(f'회전 속도: {speed:.2f} rad/s')
        print(f'지속 시간: {duration:.1f}초')
        print(f'예상 회전: {abs(speed * duration * 57.3):.1f}°')
        print(f'{"="*60}')
        print('\n⚠️  로봇을 관찰하고 실제 회전 각도를 육안으로 확인하세요!')
        print('3초 후 시작합니다...\n')
        time.sleep(3)
        
        twist = Twist()
        twist.angular.z = speed
        
        start = time.time()
        print('회전 시작!')
        
        while time.time() - start < duration:
            self.pub.publish(twist)
            time.sleep(0.05)
        
        twist.angular.z = 0.0
        for _ in range(10):
            self.pub.publish(twist)
            time.sleep(0.05)
        
        print('회전 종료!\n')

def main():
    rclpy.init()
    node = RotationTest()
    
    print('\n' + '='*60)
    print('실제 로봇 회전 테스트')
    print('='*60)
    print('\n📋 테스트:')
    print('  1. 반시계 방향 3초 회전')
    print('  2. 시계 방향 3초 회전')
    print('\n⚠️  주의:')
    print('  - 로봇 주변 2m 공간 확보')
    print('  - 실제 회전 각도를 육안으로 관찰')
    print('  - 약 90° 회전해야 정상')
    print('  - 10° 미만이면 명령 전달 문제')
    print('\nEnter를 눌러 시작...')
    input()
    
    try:
        # 테스트 1: 반시계
        node.rotate(speed=0.3, duration=3.0)
        
        print('실제로 몇 도 회전했습니까? (숫자만 입력):')
        actual_ccw = input('CCW 회전: ')
        
        time.sleep(2)
        
        # 테스트 2: 시계
        node.rotate(speed=-0.3, duration=3.0)
        
        print('실제로 몇 도 회전했습니까? (숫자만 입력):')
        actual_cw = input('CW 회전: ')
        
        # 분석
        print(f'\n{"="*60}')
        print('분석 결과')
        print(f'{"="*60}')
        
        try:
            ccw_deg = float(actual_ccw)
            cw_deg = float(actual_cw)
            expected = 51.6  # 0.3 rad/s * 3s * 57.3
            
            print(f'예상 회전: {expected:.1f}°')
            print(f'실제 CCW:  {ccw_deg:.1f}°')
            print(f'실제 CW:   {cw_deg:.1f}°')
            
            if ccw_deg > expected * 1.5:
                # 과도 회전: 명령보다 훨씬 많이 회전
                over_ratio = ccw_deg / expected
                print(f'\n❌ 과도 회전: 명령({expected:.1f}°)보다 {over_ratio:.1f}배 많이 회전!')
                print('→ jupiter_driver의 angular 스케일이 너무 큼')
                print(f'→ jupiter_driver.py 라인 175:')
                print(f'   turn_speed = angular_z * (100.0 / 0.5)')
                print(f'→ 수정 필요: (100.0 / 0.5) → (100.0 / {0.5 * over_ratio:.2f})')
                print(f'   즉, 0.5를 {0.5 * over_ratio:.2f}로 변경')
            elif ccw_deg < 15:
                print('\n❌ 심각: 로봇이 거의 회전하지 않음!')
                print('→ cmd_vel 명령이 제대로 전달되지 않거나')
                print('→ 모터 드라이버 문제 가능성')
            elif ccw_deg < expected * 0.7:
                print(f'\n⚠️  회전 부족: 예상({expected:.1f}°)의 {ccw_deg/expected*100:.0f}%만 회전')
                print('→ jupiter_driver의 angular 스케일이 너무 작음')
                under_ratio = expected / ccw_deg
                print(f'→ jupiter_driver.py 라인 175:')
                print(f'   turn_speed = angular_z * (100.0 / {0.5 / under_ratio:.2f})')
            else:
                print(f'\n✅ 정상: 로봇이 예상대로 회전 (오차 {abs(ccw_deg-expected)/expected*100:.1f}%)')
                print('→ 추가 미세 조정 가능')
        
        except ValueError:
            print('숫자 입력이 필요합니다.')
        
    except KeyboardInterrupt:
        print('\n중단됨')
    finally:
        twist = Twist()
        for _ in range(10):
            node.pub.publish(twist)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
