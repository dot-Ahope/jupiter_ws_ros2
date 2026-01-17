#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS 2 Python 클라이언트 라이브러리 rclpy와 필요한 메시지 타입을 임포트합니다.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import getpass

# ROS 1의 일반 클래스를 ROS 2의 'Node' 클래스를 상속받도록 변경합니다.
class JoyTeleop(Node):
    def __init__(self):
        # super().__init__으로 부모 클래스를 초기화하고 노드 이름을 'turtlebot_joy'로 지정합니다.
        super().__init__('turtlebot_joy')

        # --- 멤버 변수 초기화 (원본과 동일) ---
        self.user_name = getpass.getuser()

        # --- 파라미터 선언 및 초기화 ---
        # rospy.get_param 대신 self.declare_parameter로 파라미터를 선언하고 기본값을 설정합니다.
        self.declare_parameter('linear_speed_limit', 0.3)
        self.declare_parameter('angular_speed_limit', 1.0)

        # get_parameter로 선언된 파라미터 값을 가져옵니다.
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

        # --- 퍼블리셔 및 서브스크라이버 생성 ---
        # rospy.Publisher -> self.create_publisher
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # rospy.Subscriber -> self.create_subscription
        self.sub_joy = self.create_subscription(Joy, '/joy', self.buttonCallback, 10)
        
        # get_logger().info()를 사용하여 ROS 2 스타일로 로그를 출력합니다.
        self.get_logger().info('Turtlebot Joy Teleop Node has been started.')

    def buttonCallback(self, joy_data: Joy):
        # 콜백 함수 내의 로직은 대부분 동일합니다.
        # 사용자의 이름을 확인하여 다른 조이스틱 매핑을 처리합니다.
        if self.user_name == "jetson":
            linear_speed = joy_data.axes[1] * self.linear_speed_limit
            angular_speed = joy_data.axes[2] * self.angular_speed_limit
        else:
            linear_speed = joy_data.axes[1] * self.linear_speed_limit
            angular_speed = joy_data.axes[3] * self.angular_speed_limit
            
        # Twist 메시지를 생성하고 계산된 속도 값을 할당합니다.
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        
        # 수정된 퍼블리셔 객체를 사용하여 메시지를 발행합니다.
        self.pub_cmd_vel.publish(twist)


# --- 메인 실행 블록 (ROS 2 스타일) ---
def main(args=None):
    # rclpy를 초기화합니다.
    rclpy.init(args=args)
    # JoyTeleop 노드 클래스의 인스턴스를 생성합니다.
    joy_teleop_node = JoyTeleop()
    try:
        # rclpy.spin()으로 노드가 종료될 때까지 콜백 함수를 계속 호출합니다.
        rclpy.spin(joy_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드를 명시적으로 소멸시키고 rclpy를 종료합니다.
        joy_teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()