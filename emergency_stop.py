#!/usr/bin/env python3
"""긴급 정지 스크립트 - 로버를 즉시 정지시킵니다"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('🚨 긴급 정지 명령 전송 중...')
        
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # 3초간 20Hz로 정지 명령 전송
        for i in range(60):
            self.pub.publish(twist)
            self.get_logger().info(f'정지 명령 #{i+1}/60')
            time.sleep(0.05)
        
        self.get_logger().info('✅ 정지 명령 전송 완료!')

def main():
    rclpy.init()
    node = EmergencyStop()
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
