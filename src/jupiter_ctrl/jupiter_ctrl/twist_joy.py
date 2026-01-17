#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import getpass
import threading
import time

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('twist_joy')

        # --- 멤버 변수 초기화 ---
        self.user_name = getpass.getuser()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.joy_state = False
        self.joy_time = time.time()

        # --- 파라미터 선언 및 초기화 ---
        self.declare_parameter('linear_speed_limit', 2.0)
        self.declare_parameter('angular_speed_limit', 2.0)
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

        # --- 퍼블리셔 및 서브스크라이버 생성 ---
        self.pub_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.buttonCallback, 10)

        # --- 스레드 생성 ---
        # 별도의 스레드에서 주기적으로 속도 명령을 발행하는 로직
        # 스레드 타겟 함수는 그대로 사용하되, 내부의 ROS 1 API 호출을 수정합니다.
        self.vel_thread = threading.Thread(target=self.pub_vel)
        self.vel_thread.daemon = True
        self.vel_thread.start()
        
        self.get_logger().info('Twist Joy Teleop Node has been started.')

    def pub_vel(self):
        # rospy.is_shutdown() 대신 rclpy.ok()를 사용하여 ROS 2의 종료 상태를 확인합니다.
        while rclpy.ok():
            now_time = time.time()
            # 1초 이상 조이스틱 입력이 없으면 정지 메시지를 발행하는 로직
            if now_time - self.joy_time > 1.0:
                if self.joy_state:
                    self.pub_cmd_vel.publish(Twist())
                    self.joy_state = False
                self.joy_time = now_time

            # 현재 속도가 0이면 정지 메시지 발행, 아니면 현재 속도 발행
            if self.linear_speed == 0 and self.angular_speed == 0:
                if self.joy_state:
                    self.pub_cmd_vel.publish(Twist())
                    self.joy_state = False
            else:
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed
                self.pub_cmd_vel.publish(twist)
                self.joy_state = True
            
            # 루프 주기를 맞추기 위해 잠시 대기
            time.sleep(0.05) # 20Hz, rospy.Rate(20)와 유사한 효과

    def buttonCallback(self, joy_data: Joy):
        # 콜백에서는 속도 변수만 업데이트합니다. 실제 발행은 스레드에서 담당합니다.
        self.joy_time = time.time()
        if self.user_name == "jetson":
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[2] * self.angular_speed_limit
        else:
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[3] * self.angular_speed_limit

# --- 메인 실행 블록 (ROS 2 스타일) ---
def main(args=None):
    rclpy.init(args=args)
    joy_teleop_node = JoyTeleop()
    try:
        rclpy.spin(joy_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        joy_teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()