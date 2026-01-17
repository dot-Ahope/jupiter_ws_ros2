#!/usr/bin/env python3
# encoding: utf-8

# ROS 2 Python 클라이언트 라이브러리 rclpy 임포트
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# 기본 Python 라이브러리
import os
import time
import getpass
import threading
from time import sleep

# ROS 2 메시지 및 서비스 타입 임포트
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from jupiter_msgs.msg import Adjust


class JoyTeleop(Node):
    def __init__(self):
        # super().__init__을 호출하여 노드 이름을 'jupiter_joy'로 초기화
        super().__init__('jupiter_joy')
        
        # --- 상태 변수 ---
        self.user_name = getpass.getuser()
        self.Joy_active = True  # 자동으로 활성화
        self.linear_Gear = 1.0
        self.angular_Gear = 1.0
        
        # 서비스 호출과 다른 콜백이 겹치지 않도록 ReentrantCallbackGroup 사용
        self.callback_group = ReentrantCallbackGroup()

        # --- 파라미터 선언 ---
        self.declare_parameter('linear_speed_limit', 0.45)
        self.declare_parameter('angular_speed_limit', 2.0)
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        self.get_logger().info(f"Speed limits: linear={self.linear_speed_limit}, angular={self.angular_speed_limit}")

        # --- 퍼블리셔 생성 ---
        self.cmdVelPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_Adjust = self.create_publisher(Adjust, "/Adjust", 10)

        # --- 서브스크라이버 생성 ---
        self.sub_Joy = self.create_subscription(
            Joy,
            '/joy',
            self.buttonCallback,
            10,
            callback_group=self.callback_group)
        
        self.get_logger().info("Joy Teleop Node has been started.")

    def buttonCallback(self, joy_data):
        '''
        조이스틱 입력 처리
        '''
        if not isinstance(joy_data, Joy): 
            return
        
        # 디버그 로그 
        self.get_logger().info(f"Joy received - axes[1]={joy_data.axes[1]:.3f}, axes[2]={joy_data.axes[2]:.3f}")
        
        # 사용자 환경에 따라 적절한 처리
        if self.user_name == "pi": 
            self.user_pi(joy_data)
        else: 
            self.user_jetson(joy_data)

    def user_pi(self, joy_data):
        '''
        PI 환경용 - axes[3]이 회전
        '''
        linear_speed = joy_data.axes[1] * self.linear_speed_limit * self.linear_Gear
        angular_speed = joy_data.axes[3] * self.angular_speed_limit * self.angular_Gear
        
        self.publish_cmd_vel(linear_speed, angular_speed)

    def user_jetson(self, joy_data):
        '''
        Jetson 환경용 - axes[2]가 회전
        '''
        linear_speed = joy_data.axes[1] * self.linear_speed_limit * self.linear_Gear
        angular_speed = joy_data.axes[2] * self.angular_speed_limit * self.angular_Gear
        
        self.publish_cmd_vel(linear_speed, angular_speed)

    def publish_cmd_vel(self, linear_speed, angular_speed):
        '''
        cmd_vel 메시지 발행
        '''
        # 속도 제한 적용
        if linear_speed > self.linear_speed_limit: 
            linear_speed = self.linear_speed_limit
        elif linear_speed < -self.linear_speed_limit: 
            linear_speed = -self.linear_speed_limit
            
        if angular_speed > self.angular_speed_limit: 
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: 
            angular_speed = -self.angular_speed_limit

        # Twist 메시지 생성 및 발행
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmdVelPublisher.publish(twist)
        
        # Adjust 메시지 발행
        adjust_msg = Adjust()
        adjust_msg.adjust = (linear_speed != 0)
        self.pub_Adjust.publish(adjust_msg)
        
        self.get_logger().info(f"Published: linear={linear_speed:.3f}, angular={angular_speed:.3f}")


def main(args=None):
    rclpy.init(args=args)
    
    joy = JoyTeleop()
    
    executor = MultiThreadedExecutor()
    executor.add_node(joy)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        joy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
