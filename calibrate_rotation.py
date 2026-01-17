#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import sys
import time

class RotationCalibrator(Node):
    def __init__(self):
        super().__init__('rotation_calibrator')
        self.subscription = self.create_subscription(
            Imu,
            '/jupiter/imu',
            self.listener_callback,
            10)
        self.last_time = None
        self.total_yaw = 0.0
        self.running = False
        
        print("========================================================")
        print("Rotation Calibration Tool")
        print("========================================================")
        print("1. Align the robot to a clear mark on the floor.")
        print("2. Press ENTER to start integration.")
        print("3. Rotate the robot exactly 360 degrees (1 full turn).")
        print("4. Stop exactly at the mark.")
        print("5. Press Ctrl+C to see the result.")
        print("========================================================")

    def listener_callback(self, msg):
        if not self.running:
            return
            
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Integrate angular velocity (rad/s * s = rad)
        # Using Z axis
        self.total_yaw += msg.angular_velocity.z * dt
        
        degrees = math.degrees(self.total_yaw)
        print(f"\rCurrent Integrated Angle: {degrees:.2f} degrees", end="")

def main(args=None):
    rclpy.init(args=args)
    node = RotationCalibrator()
    
    input("Press Enter to start measuring...")
    print("Integration started! Rotate the robot...")
    node.running = True
    node.last_time = time.time() # Initial sync
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n========================================================")
        print("Calibration Result")
        print("========================================================")
        measured_angle = math.degrees(node.total_yaw)
        print(f"Measured Angle by IMU: {measured_angle:.2f} degrees")
        print(f"Target Angle (Physical): 360.00 degrees (Assumed)")
        
        if abs(measured_angle) > 1.0:
            ratio = 360.0 / abs(measured_angle)
            print(f"\nRecommended Correction Factor: {ratio:.4f}")
            print(f"Multiply your current sensitivity_gain by {ratio:.4f}")
        else:
            print("Error: Measured angle is too small.")
            
        print("========================================================")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
