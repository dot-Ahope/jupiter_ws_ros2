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
from std_msgs.msg import Bool


class JoyTeleop(Node):
    def __init__(self):
        # super().__init__을 호출하여 노드 이름을 'jupiter_joy'로 초기화
        super().__init__('jupiter_joy')
        
        # --- 상태 변수 ---
        self.user_name = getpass.getuser()
        self.Joy_active = False  # 자동으로 비활성화
        self.Buzzer_active = False
        self.cancel_time = time.time()
        self.linear_Gear = 1.0
        self.angular_Gear = 1.0
        self.deadzone = 0.1  # 10% 데드존(10% 미만의 입력값 무시)
        self.expo = 1.0  # 지수 커브 factor(1.0 = 선형, 2.0 = 제곱, 3.0 = 세제곱 등)
        
        # 서비스 호출과 다른 콜백이 겹치지 않도록 ReentrantCallbackGroup 사용
        self.callback_group = ReentrantCallbackGroup()

        # --- 파라미터 선언 ---
        self.declare_parameter('linear_speed_limit', 1.0) # 기존 : 0.45
        self.declare_parameter('angular_speed_limit', 2.00) # 기존 : 2.00
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        self.get_logger().info(f"Speed limits: linear={self.linear_speed_limit}, angular={self.angular_speed_limit}")

        # --- 퍼블리셔 생성 ---
        self.cmdVelPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_Adjust = self.create_publisher(Adjust, "/Adjust", 10)
        self.pub_Buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)

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
        #print(f"\rJoy received - axes[1]={joy_data.axes[1]:.3f}, axes[2]={joy_data.axes[2]:.3f}    ", end="", flush=True)
        # 사용자 환경에 따라 적절한 처리
        if self.user_name == "pi": 
            self.user_pi(joy_data)
        else: 
            self.user_jetson(joy_data)

    def apply_deadzone_and_expo(self, value):
        '''
        데드존과 지수 커브를 적용하여 조이스틱 값을 조정
        '''
        # 데드존 적용
        if abs(value) < self.deadzone:
            return 0.0
        
        # 데드존을 벗어난 값을 0-1 범위로 재조정
        sign = 1.0 if value > 0 else -1.0
        value = abs(value)
        normalized = (value - self.deadzone) / (1 - self.deadzone)
        
        # 지수 커브 적용
        adjusted = normalized ** self.expo
        
        return sign * adjusted

    def user_pi(self, joy_data):
        '''
        PI 환경용 - axes[3]이 회전
        '''
        # 데드존과 지수 커브 적용
        linear_input = self.apply_deadzone_and_expo(joy_data.axes[1])
        angular_input = self.apply_deadzone_and_expo(joy_data.axes[3])
        
        linear_speed = linear_input * self.linear_speed_limit * self.linear_Gear
        angular_speed = angular_input * self.angular_speed_limit * self.angular_Gear
        
        self.publish_cmd_vel(linear_speed, angular_speed)

    def user_jetson(self, joy_data):
        '''
        Jetson 환경용 - axes[2]가 회전
        '''
        #cancel nav
        if joy_data.buttons[9] == 1: self.cancel_nav()
        
        #Buzzer
        if joy_data.buttons[11] == 1:
            Buzzer_ctrl = Bool() 
            self.Buzzer_active=not self.Buzzer_active
            Buzzer_ctrl.data =self.Buzzer_active
            for i in range(3): self.pub_Buzzer.publish(Buzzer_ctrl)

        # 데드존과 지수 커브 적용
        linear_input = self.apply_deadzone_and_expo(joy_data.axes[1])
        angular_input = self.apply_deadzone_and_expo(joy_data.axes[2])
        
        linear_speed = linear_input * self.linear_speed_limit * self.linear_Gear
        angular_speed = angular_input * self.angular_speed_limit * self.angular_Gear
        
        self.publish_cmd_vel(linear_speed, angular_speed)

    def cancel_nav(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            Joy_ctrl = Bool()
            self.Joy_active = not self.Joy_active
            Joy_ctrl.data = self.Joy_active
            for i in range(3):
                self.pub_JoyState.publish(Joy_ctrl)
                #self.pub_goal.publish(GoalID())
                self.cmdVelPublisher.publish(Twist())
            self.cancel_time = now_time

    def publish_cmd_vel(self, linear_speed, angular_speed):
        '''
        cmd_vel 메시지 발행
        '''
        # 조이스틱 비활성 상태면 발행하지 않음 (cancel_nav로 토글)
        if not self.Joy_active:
             return

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
        #print(f"\rPublished: linear={linear_speed:.3f}, angular={angular_speed:.3f}    ", end="", flush=True)

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
