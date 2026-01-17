#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class LinearCalibrator(Node):
    def __init__(self):
        super().__init__('linear_calibrator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = None
        self.target_duration = 4.0  # 4 seconds
        self.target_speed = 0.25     # 0.25 m/s
        # Theoretical distance = 0.25 * 4.0 = 1.0 meter
        self.is_moving = False
        self.done = False

    def timer_callback(self):
        if self.done:
            return

        if not self.is_moving:
            print(f"Starting calibration run: Speed={self.target_speed}m/s, Duration={self.target_duration}s")
            print("Theoretical Distance: 1.0 meter")
            print("Please measure the ACTUAL distance traveled.")
            self.start_time = time.time()
            self.is_moving = True

        elapsed = time.time() - self.start_time
        msg = Twist()

        if elapsed < self.target_duration:
            msg.linear.x = self.target_speed
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.is_moving = False
            self.done = True
            print("Stop! Run complete.")
            print("------------------------------------------------")
            print("Calculation Formula:")
            print("New Scale = Current Scale * (1.0 / Measured Distance)")
            print("Current Scale is likely 1.2 (check launch file)")
            print("------------------------------------------------")
            # Force stop multiple times to ensure it stops
            for _ in range(5):
                self.publisher_.publish(msg)
                time.sleep(0.1)
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = LinearCalibrator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
