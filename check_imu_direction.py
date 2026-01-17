#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys

class ImuCheckNode(Node):
    def __init__(self):
        super().__init__('imu_check_node')
        self.subscription = self.create_subscription(
            Imu,
            '/jupiter/imu',  # Changed from /imu/data_calibrated to /jupiter/imu to check raw driver output
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.print_timer = 0

    def listener_callback(self, msg):
        self.print_timer += 1
        if self.print_timer % 5 == 0:  # Print frequently enough
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            wz = msg.angular_velocity.z
            
            print(f"Linear Acc [m/s^2]: x={ax:.2f}, y={ay:.2f}, z={az:.2f}")
            print(f"Angular Vel [rad/s]: z={wz:.2f}")
            
            if abs(wz) > 0.1:
                direction = "LEFT (CCW) +" if wz > 0 else "RIGHT (CW) -"
                print(f"--> Rotation Detected: {direction}")
            
            print("-" * 40)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCheckNode()
    print("=============================================================")
    print("IMU Direction Check Tool")
    print("=============================================================")
    print("Subscribing to: /imu/data_calibrated")
    print("\nINSTRUCTIONS:")
    print("1. ROTATION CHECK:")
    print("   - Rotate the robot Counter-Clockwise (Left).")
    print("   - Angular Vel Z should be POSITIVE (+).")
    print("   - If Negative (-), IMU is mounted upside down or inverted.")
    print("\n2. ACCELERATION CHECK:")
    print("   - Push the robot Forward abruptly.")
    print("   - Linear Acc X should spike POSITIVE (+).")
    print("   - If Negative (-), IMU X-axis is backward.")
    print("\nPress Ctrl+C to exit.")
    print("=============================================================")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
