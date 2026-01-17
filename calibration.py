# 파일명: calibrate_linear.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# =======================================================
# ▼▼▼ 보정할 속도 레벨의 현재 값으로 이 부분을 수정하세요 ▼▼▼
# 레벨 1 보정 시: 0.1
# 레벨 3 보정 시: 0.4
TEST_LINEAR_SPEED = 0.1  # m/s
# =======================================================

# 테스트할 시간 (초)
TEST_DURATION = 5.0      # seconds

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        time.sleep(1)

    def run_test(self):
        twist = Twist()
        twist.linear.x = TEST_LINEAR_SPEED
        self.get_logger().info(f"Running test with speed: {TEST_LINEAR_SPEED} m/s for {TEST_DURATION}s")
        start_time = time.time()
        while time.time() - start_time < TEST_DURATION:
            self.publisher_.publish(twist)
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Test finished.")

def main(args=None):
    rclpy.init(args=args)
    node = Calibrator()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
