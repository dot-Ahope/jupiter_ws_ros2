#!/usr/bin/env python3
"""
IMU Gyro Scale 진단 스크립트
============================
목적: IMU angular_velocity.z (gyro) 와 wheel encoder vyaw 를 실시간 비교하여
      gyro scale error 를 측정한다.

사용법:
  1. nav2_vslam_fused.launch.py 를 먼저 실행
  2. 이 스크립트를 실행:
     python3 ~/jupiter_ws_ros2/scripts/diagnose_imu_gyro_scale.py
  3. 로봇을 제자리에서 천천히 회전시킨다 (조이스틱이나 cmd_vel)
  4. Ctrl+C 로 종료 → 비율(ratio) 계산 결과 출력

측정 원리:
  - /jupiter/imu → angular_velocity.z  (IMU raw, before calib)
  - /imu/data_calibrated → angular_velocity.z  (IMU after bias subtraction)
  - /odom_adapted → twist.twist.angular.z  (wheel encoder vyaw)
  - EKF 는 포함하지 않음 (EKF 가 문제이므로)

  회전 중 IMU_vyaw / Wheel_vyaw 비율이 1.0 이면 정상.
  비율이 ~3.75 이면 ICM20948 gyro_ratio 가 잘못된 것 (should be 1/3754.9).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import time
import signal
import sys


class GyroScaleDiagnostic(Node):
    def __init__(self):
        super().__init__('gyro_scale_diagnostic')

        # Data storage
        self.imu_raw_samples = []       # (timestamp, gz)
        self.imu_calib_samples = []     # (timestamp, gz)
        self.wheel_samples = []         # (timestamp, vyaw)
        
        # Accumulators for real-time display
        self.imu_raw_integral = 0.0
        self.imu_calib_integral = 0.0
        self.wheel_integral = 0.0
        self.last_imu_raw_time = None
        self.last_imu_calib_time = None
        self.last_wheel_time = None
        
        # Rotation detection 
        self.rotation_started = False
        self.rotation_start_time = None
        self.rotation_segments = []  # list of (imu_integral, wheel_integral, duration)
        
        self.THRESHOLD = 0.05  # rad/s minimum to consider "rotating"

        # Subscribe to IMU raw (from driver, before calib)
        self.sub_imu_raw = self.create_subscription(
            Imu, '/jupiter/imu', self.imu_raw_cb, 10
        )
        
        # Subscribe to IMU calibrated (after bias subtraction)
        self.sub_imu_calib = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_calib_cb, 10
        )
        
        # Subscribe to wheel odom adapted
        self.sub_wheel = self.create_subscription(
            Odometry, '/odom_adapted', self.wheel_cb, 10
        )
        
        # Display timer (2Hz)
        self.create_timer(0.5, self.display_status)
        
        self.start_time = time.time()
        self.get_logger().info("=== IMU Gyro Scale Diagnostic Started ===")
        self.get_logger().info("Rotate the robot slowly. Press Ctrl+C to see results.")
        
    def imu_raw_cb(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z
        self.imu_raw_samples.append((now, gz))
        
        if self.last_imu_raw_time is not None:
            dt = now - self.last_imu_raw_time
            if 0.0 < dt < 0.5:  # sanity check
                self.imu_raw_integral += gz * dt
        self.last_imu_raw_time = now
        
    def imu_calib_cb(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z
        self.imu_calib_samples.append((now, gz))
        
        if self.last_imu_calib_time is not None:
            dt = now - self.last_imu_calib_time
            if 0.0 < dt < 0.5:
                self.imu_calib_integral += gz * dt
        self.last_imu_calib_time = now
        
    def wheel_cb(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vyaw = msg.twist.twist.angular.z
        self.wheel_samples.append((now, vyaw))
        
        if self.last_wheel_time is not None:
            dt = now - self.last_wheel_time
            if 0.0 < dt < 0.5:
                self.wheel_integral += vyaw * dt
        self.last_wheel_time = now
        
    def display_status(self):
        elapsed = time.time() - self.start_time
        
        # Get latest values
        imu_raw_gz = self.imu_raw_samples[-1][1] if self.imu_raw_samples else 0.0
        imu_calib_gz = self.imu_calib_samples[-1][1] if self.imu_calib_samples else 0.0
        wheel_vyaw = self.wheel_samples[-1][1] if self.wheel_samples else 0.0
        
        # Calculate running ratio
        ratio_str = "N/A"
        if abs(self.wheel_integral) > 0.1:  # need some rotation
            ratio = self.imu_calib_integral / self.wheel_integral
            ratio_str = f"{ratio:.3f}"
        
        raw_ratio_str = "N/A"
        if abs(self.wheel_integral) > 0.1:
            raw_ratio = self.imu_raw_integral / self.wheel_integral
            raw_ratio_str = f"{raw_ratio:.3f}"
        
        print(f"\r[{elapsed:6.1f}s] "
              f"IMU_raw_gz={imu_raw_gz:+7.4f} "
              f"IMU_cal_gz={imu_calib_gz:+7.4f} "
              f"Wheel_vyaw={wheel_vyaw:+7.4f} | "
              f"∫IMU_raw={math.degrees(self.imu_raw_integral):+7.1f}° "
              f"∫IMU_cal={math.degrees(self.imu_calib_integral):+7.1f}° "
              f"∫Wheel={math.degrees(self.wheel_integral):+7.1f}° | "
              f"ratio(cal/wheel)={ratio_str} "
              f"ratio(raw/wheel)={raw_ratio_str}",
              end='', flush=True)
        
    def print_summary(self):
        print("\n")
        print("=" * 80)
        print("IMU Gyro Scale Diagnostic - SUMMARY")
        print("=" * 80)
        
        print(f"\nSamples collected:")
        print(f"  IMU raw:        {len(self.imu_raw_samples)}")
        print(f"  IMU calibrated: {len(self.imu_calib_samples)}")
        print(f"  Wheel odom:     {len(self.wheel_samples)}")
        
        print(f"\nIntegrated yaw (total rotation):")
        print(f"  IMU raw:        {math.degrees(self.imu_raw_integral):+8.2f}°")
        print(f"  IMU calibrated: {math.degrees(self.imu_calib_integral):+8.2f}°")
        print(f"  Wheel encoder:  {math.degrees(self.wheel_integral):+8.2f}°")
        
        if abs(self.wheel_integral) > 0.1:  # ~5.7 degrees minimum
            ratio_raw = self.imu_raw_integral / self.wheel_integral
            ratio_cal = self.imu_calib_integral / self.wheel_integral
            
            print(f"\nScale Ratio:")
            print(f"  IMU_raw / Wheel   = {ratio_raw:.4f}")
            print(f"  IMU_calib / Wheel = {ratio_cal:.4f}")
            
            if abs(ratio_cal) > 1.5:
                correction = 1.0 / ratio_cal
                print(f"\n⚠ IMU gyro is OVER-READING by {ratio_cal:.2f}x")
                print(f"  Recommended imu_gyro_scale = {correction:.4f}")
                print(f"\n  To apply:")
                print(f"    Option A: Set parameter in jupiter_driver_params.yaml:")
                print(f"      imu_gyro_scale: {correction:.4f}")
                print(f"    Option B: Fix Rosmaster_Lib.py ICM20948 gyro_ratio:")
                print(f"      Current: gyro_ratio = 1/1000.0")
                print(f"      Fix:     gyro_ratio = 1/{1000.0/correction:.1f}")
            elif abs(ratio_cal) < 0.67:
                correction = 1.0 / ratio_cal
                print(f"\n⚠ IMU gyro is UNDER-READING by {1.0/ratio_cal:.2f}x")
                print(f"  Recommended imu_gyro_scale = {correction:.4f}")
            else:
                print(f"\n✓ IMU gyro scale looks CORRECT (ratio close to 1.0)")
                print(f"  The EKF yaw overshoot is NOT caused by gyro scale error.")
        else:
            print("\n⚠ Not enough rotation detected! Rotate at least ~10° and try again.")
        
        # Time-aligned segment analysis: find peak rotation period
        if self.imu_calib_samples and self.wheel_samples:
            print(f"\n--- Peak Rotation Period Analysis ---")
            # Find the 1-second window with highest wheel vyaw
            if len(self.wheel_samples) > 50:
                best_imu = 0.0
                best_wheel = 0.0
                best_t = 0.0
                
                # Sliding 1-second window
                for i in range(len(self.wheel_samples)):
                    t0 = self.wheel_samples[i][0]
                    t1 = t0 + 1.0
                    
                    # Sum wheel in window
                    w_sum = 0.0
                    w_count = 0
                    for j in range(i, len(self.wheel_samples)):
                        if self.wheel_samples[j][0] > t1:
                            break
                        w_sum += abs(self.wheel_samples[j][1])
                        w_count += 1
                    
                    if w_count > 0:
                        w_avg = w_sum / w_count
                        if w_avg > abs(best_wheel):
                            best_wheel = w_avg
                            best_t = t0
                            
                            # Find corresponding IMU calib avg
                            i_sum = 0.0
                            i_count = 0
                            for s in self.imu_calib_samples:
                                if t0 <= s[0] <= t1:
                                    i_sum += abs(s[1])
                                    i_count += 1
                            best_imu = i_sum / i_count if i_count > 0 else 0.0
                
                if best_wheel > 0.05:
                    peak_ratio = best_imu / best_wheel
                    print(f"  Peak 1s window at t={best_t:.1f}")
                    print(f"  IMU avg |gz|:   {best_imu:.4f} rad/s ({math.degrees(best_imu):.2f}°/s)")
                    print(f"  Wheel avg |vyaw|: {best_wheel:.4f} rad/s ({math.degrees(best_wheel):.2f}°/s)")
                    print(f"  Peak ratio: {peak_ratio:.4f}")
        
        print("=" * 80)


def main():
    rclpy.init()
    node = GyroScaleDiagnostic()
    
    def signal_handler(sig, frame):
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.print_summary()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
