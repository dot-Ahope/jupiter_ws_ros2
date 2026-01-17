#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from collections import deque

class SensorCalibrationAnalyzer(Node):
    def __init__(self):
        super().__init__('sensor_calibration_analyzer')
        
        # 데이터 저장
        self.imu_data = deque(maxlen=100)
        self.odom_data = deque(maxlen=100)
        self.laser_data = deque(maxlen=50)
        
        # Subscribe to sensor topics
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_filtered',
            self.imu_callback,
            10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
            
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        # 분석 타이머
        self.analysis_timer = self.create_timer(5.0, self.analyze_sensors)
        
        self.get_logger().info("=== Sensor Calibration Analyzer Started ===")
        self.get_logger().info("Collecting data for sensor alignment analysis...")

    def quaternion_to_euler(self, x, y, z, w):
        """쿼터니언을 오일러각으로 변환"""
        # Roll (x축)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y축)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(np.clip(sinp, -1.0, 1.0))
        
        # Yaw (z축)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def imu_callback(self, msg):
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, 
            msg.orientation.z, msg.orientation.w)
        
        self.imu_data.append({
            'timestamp': time.time(),
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'angular_vel': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        })

    def odom_callback(self, msg):
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        self.odom_data.append({
            'timestamp': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'linear_vel': [msg.twist.twist.linear.x, msg.twist.twist.linear.y],
            'angular_vel': msg.twist.twist.angular.z
        })

    def laser_callback(self, msg):
        # 라이다에서 전방 방향 감지 (0도 방향의 거리)
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # 전방 방향 (0도 근처) 거리
        front_idx = len(ranges) // 2
        front_distance = ranges[front_idx] if not np.isinf(ranges[front_idx]) else None
        
        # 좌/우 방향 거리
        left_idx = int(len(ranges) * 0.75)
        right_idx = int(len(ranges) * 0.25)
        left_distance = ranges[left_idx] if not np.isinf(ranges[left_idx]) else None
        right_distance = ranges[right_idx] if not np.isinf(ranges[right_idx]) else None
        
        self.laser_data.append({
            'timestamp': time.time(),
            'front_distance': front_distance,
            'left_distance': left_distance,
            'right_distance': right_distance,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max
        })

    def calculate_statistics(self, data_list, field):
        """데이터의 통계 계산"""
        if not data_list:
            return None
        
        values = []
        for item in data_list:
            if field in item and item[field] is not None:
                values.append(item[field])
        
        if not values:
            return None
            
        return {
            'mean': np.mean(values),
            'std': np.std(values),
            'min': np.min(values),
            'max': np.max(values),
            'range': np.max(values) - np.min(values)
        }

    def analyze_coordinate_alignment(self):
        """센서 간 좌표계 정렬 분석"""
        results = []
        
        if len(self.imu_data) < 10 or len(self.odom_data) < 10:
            return ["Insufficient data for analysis"]
        
        # IMU와 오도메트리의 Yaw 비교
        imu_yaw_stats = self.calculate_statistics(list(self.imu_data), 'yaw')
        odom_yaw_stats = self.calculate_statistics(list(self.odom_data), 'yaw')
        
        if imu_yaw_stats and odom_yaw_stats:
            yaw_offset = imu_yaw_stats['mean'] - odom_yaw_stats['mean']
            results.append(f"📐 Yaw Offset (IMU-Odom): {math.degrees(yaw_offset):.2f}°")
            
            if abs(yaw_offset) > 0.1:  # 5도 이상 차이
                results.append(f"⚠️  Significant yaw misalignment detected!")
            else:
                results.append("✅ Yaw alignment is good")
        
        # IMU 안정성 분석
        if imu_yaw_stats:
            results.append(f"🧭 IMU Yaw stability: ±{math.degrees(imu_yaw_stats['std']):.2f}°")
            if imu_yaw_stats['std'] > 0.05:  # 3도 이상 변동
                results.append("⚠️  IMU showing high variance (possible vibration)")
        
        # 오도메트리 안정성 분석
        if odom_yaw_stats:
            results.append(f"🎯 Odom Yaw stability: ±{math.degrees(odom_yaw_stats['std']):.2f}°")
            if odom_yaw_stats['std'] > 0.05:
                results.append("⚠️  Odometry showing high variance")
        
        return results

    def analyze_sensor_noise(self):
        """센서 노이즈 분석"""
        results = []
        
        # IMU 각속도 노이즈
        if self.imu_data:
            angular_vels = [data['angular_vel'][2] for data in list(self.imu_data)]  # Z축 각속도
            angular_vel_std = np.std(angular_vels)
            results.append(f"🔄 IMU Angular velocity noise: {angular_vel_std:.4f} rad/s")
            
            if angular_vel_std > 0.01:
                results.append("⚠️  High IMU angular velocity noise (causing vibration)")
            
        # 오도메트리 각속도 노이즈  
        if self.odom_data:
            odom_angular_vels = [data['angular_vel'] for data in list(self.odom_data)]
            odom_angular_std = np.std(odom_angular_vels)
            results.append(f"⚙️  Odom Angular velocity noise: {odom_angular_std:.4f} rad/s")
            
            if odom_angular_std > 0.01:
                results.append("⚠️  High odometry angular velocity noise")
        
        return results

    def detect_coordinate_system_direction(self):
        """좌표계 방향 감지"""
        results = []
        
        if len(self.imu_data) < 5:
            return ["Insufficient IMU data for direction analysis"]
        
        # IMU의 현재 방향 (정지 상태에서의 기준)
        recent_imu = list(self.imu_data)[-5:]
        avg_yaw = np.mean([data['yaw'] for data in recent_imu])
        avg_roll = np.mean([data['roll'] for data in recent_imu])
        avg_pitch = np.mean([data['pitch'] for data in recent_imu])
        
        results.append(f"🧭 Current IMU orientation:")
        results.append(f"   Roll: {math.degrees(avg_roll):.1f}°")
        results.append(f"   Pitch: {math.degrees(avg_pitch):.1f}°")
        results.append(f"   Yaw: {math.degrees(avg_yaw):.1f}°")
        
        # 좌표계 정렬 제안
        if abs(avg_yaw) > 0.1:  # 5도 이상
            correction_yaw = -avg_yaw  # 현재 yaw를 0으로 맞추기 위한 보정
            results.append(f"💡 Suggested coordinate correction:")
            results.append(f"   Add yaw rotation: {math.degrees(correction_yaw):.1f}°")
        
        return results

    def analyze_sensors(self):
        """종합 센서 분석"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📊 SENSOR CALIBRATION ANALYSIS REPORT")
        self.get_logger().info("="*60)
        
        # 데이터 수집 상태
        self.get_logger().info(f"📈 Data collected: IMU={len(self.imu_data)}, Odom={len(self.odom_data)}, Laser={len(self.laser_data)}")
        
        # 좌표계 정렬 분석
        self.get_logger().info("\n🔧 COORDINATE ALIGNMENT ANALYSIS:")
        alignment_results = self.analyze_coordinate_alignment()
        for result in alignment_results:
            self.get_logger().info(f"  {result}")
        
        # 센서 노이즈 분석
        self.get_logger().info("\n📳 SENSOR NOISE ANALYSIS:")
        noise_results = self.analyze_sensor_noise()
        for result in noise_results:
            self.get_logger().info(f"  {result}")
        
        # 좌표계 방향 감지
        self.get_logger().info("\n🎯 COORDINATE SYSTEM DIRECTION:")
        direction_results = self.detect_coordinate_system_direction()
        for result in direction_results:
            self.get_logger().info(f"  {result}")

def main():
    rclpy.init()
    analyzer = SensorCalibrationAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info("Analysis completed.")
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()