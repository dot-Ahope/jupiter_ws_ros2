#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import tf2_ros
import numpy as np
import time
from collections import deque

class LocalizationDiagnostic(Node):
    def __init__(self):
        super().__init__('localization_diagnostic')
        
        # TF2 Buffer 및 Listener 생성 (static transform 포함)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 데이터 저장용 변수들
        self.cmd_vel_data = None
        self.odom_raw_data = None
        self.odom_filtered_data = None
        self.imu_data = None
        self.tf_data = {}
        
        # 명령 히스토리 (최근 10개)
        self.cmd_history = deque(maxlen=10)
        self.odom_history = deque(maxlen=10)
        self.filtered_history = deque(maxlen=10)
        
        # 구독자들 생성
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_raw_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_raw_callback, 10)
        self.odom_filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_filtered_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_filtered', self.imu_callback, 10)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        # static transform도 모니터링 추가
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, 10)
        
        # 주기적 진단 타이머 (5초마다)
        self.diagnostic_timer = self.create_timer(5.0, self.run_diagnostics)
        
        # 성능 측정용
        self.start_time = time.time()
        self.cmd_count = 0
        self.odom_count = 0
        
        self.get_logger().info("=== Localization Diagnostic Node Started ===")
        self.get_logger().info("Monitoring: /cmd_vel, /odom_raw, /odometry/filtered, /imu/data_filtered, /tf, /tf_static")

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = msg
        self.cmd_history.append({
            'timestamp': time.time(),
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        })
        self.cmd_count += 1

    def odom_raw_callback(self, msg):
        self.odom_raw_data = msg
        self.odom_history.append({
            'timestamp': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        })
        self.odom_count += 1

    def odom_filtered_callback(self, msg):
        self.odom_filtered_data = msg
        self.filtered_history.append({
            'timestamp': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        })

    def imu_callback(self, msg):
        self.imu_data = msg

    def tf_callback(self, msg):
        for transform in msg.transforms:
            frame_pair = f"{transform.header.frame_id} -> {transform.child_frame_id}"
            self.tf_data[frame_pair] = {
                'timestamp': time.time(),
                'translation': transform.transform.translation,
                'rotation': transform.transform.rotation
            }

    def tf_static_callback(self, msg):
        """static transform 처리 - 한번만 업데이트되므로 별도 처리"""
        for transform in msg.transforms:
            frame_pair = f"{transform.header.frame_id} -> {transform.child_frame_id}"
            self.tf_data[frame_pair] = {
                'timestamp': time.time(),
                'translation': transform.transform.translation,
                'rotation': transform.transform.rotation,
                'static': True  # static transform 표시
            }

    def calculate_response_ratio(self):
        """cmd_vel 명령 대비 odom 응답 비율 계산"""
        if self.cmd_count == 0:
            return 0.0
        return (self.odom_count / self.cmd_count) * 100

    def analyze_movement_consistency(self):
        """명령과 실제 움직임의 일치성 분석"""
        if len(self.cmd_history) == 0 or len(self.odom_history) == 0:
            return "No data available"
        
        results = []
        
        # 최근 명령과 오도메트리 비교
        recent_cmd = self.cmd_history[-1] if self.cmd_history else None
        recent_odom = self.odom_history[-1] if self.odom_history else None
        recent_filtered = self.filtered_history[-1] if self.filtered_history else None
        
        if recent_cmd and recent_odom:
            cmd_linear = recent_cmd['linear_x']
            odom_linear = recent_odom['linear_x']
            cmd_angular = recent_cmd['angular_z']
            odom_angular = recent_odom['angular_z']
            
            # 선형 속도 비교
            if abs(cmd_linear) > 0.01:  # 명령이 있는 경우
                linear_ratio = odom_linear / cmd_linear if cmd_linear != 0 else 0
                if abs(linear_ratio - 1.0) > 0.5:  # 50% 이상 차이
                    results.append(f"⚠️  Linear velocity mismatch: cmd={cmd_linear:.3f}, odom={odom_linear:.3f}, ratio={linear_ratio:.2f}")
                else:
                    results.append(f"✅ Linear velocity OK: cmd={cmd_linear:.3f}, odom={odom_linear:.3f}")
            
            # 각속도 비교
            if abs(cmd_angular) > 0.01:  # 명령이 있는 경우
                angular_ratio = odom_angular / cmd_angular if cmd_angular != 0 else 0
                if abs(angular_ratio - 1.0) > 0.5:  # 50% 이상 차이
                    results.append(f"⚠️  Angular velocity mismatch: cmd={cmd_angular:.3f}, odom={odom_angular:.3f}, ratio={angular_ratio:.2f}")
                else:
                    results.append(f"✅ Angular velocity OK: cmd={cmd_angular:.3f}, odom={odom_angular:.3f}")
        
        # EKF 필터링 결과 비교
        if recent_odom and recent_filtered:
            raw_x, raw_y = recent_odom['x'], recent_odom['y']
            filtered_x, filtered_y = recent_filtered['x'], recent_filtered['y']
            position_diff = np.sqrt((raw_x - filtered_x)**2 + (raw_y - filtered_y)**2)
            
            if position_diff > 0.1:  # 10cm 이상 차이
                results.append(f"⚠️  Large EKF correction: raw=({raw_x:.3f}, {raw_y:.3f}), filtered=({filtered_x:.3f}, {filtered_y:.3f}), diff={position_diff:.3f}m")
            else:
                results.append(f"✅ EKF fusion OK: position_diff={position_diff:.3f}m")
        
        return results

    def check_tf_connectivity(self):
        """TF 연결성 확인 - tf2_ros.Buffer 사용"""
        expected_frames = [
            ("map", "odom"),
            ("odom", "base_footprint"), 
            ("base_footprint", "base_link"),
            ("base_link", "laser"),
            ("base_link", "imu_link")
        ]
        
        results = []
        
        for parent_frame, child_frame in expected_frames:
            try:
                # TF 변환을 조회해보기
                transform = self.tf_buffer.lookup_transform(
                    parent_frame, child_frame, rclpy.time.Time())
                
                # 성공적으로 조회된 경우
                results.append(f"✅ {parent_frame} -> {child_frame}: OK")
                
            except tf2_ros.LookupException:
                results.append(f"❌ {parent_frame} -> {child_frame}: MISSING")
            except tf2_ros.ConnectivityException:
                results.append(f"⚠️  {parent_frame} -> {child_frame}: NOT CONNECTED")
            except tf2_ros.ExtrapolationException:
                results.append(f"⚠️  {parent_frame} -> {child_frame}: EXTRAPOLATION ERROR")
            except Exception as e:
                results.append(f"❌ {parent_frame} -> {child_frame}: ERROR ({str(e)})")
        
        return results

    def run_diagnostics(self):
        """주기적 진단 실행"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("🔍 LOCALIZATION DIAGNOSTIC REPORT")
        self.get_logger().info("="*50)
        
        # 기본 상태 확인
        elapsed_time = time.time() - self.start_time
        response_ratio = self.calculate_response_ratio()
        
        self.get_logger().info(f"📊 Runtime: {elapsed_time:.1f}s | Commands: {self.cmd_count} | Odom messages: {self.odom_count}")
        self.get_logger().info(f"📈 Response ratio: {response_ratio:.1f}%")
        
        # 데이터 수신 상태
        self.get_logger().info("\n📡 DATA RECEPTION STATUS:")
        self.get_logger().info(f"  cmd_vel: {'✅' if self.cmd_vel_data else '❌'}")
        self.get_logger().info(f"  odom_raw: {'✅' if self.odom_raw_data else '❌'}")
        self.get_logger().info(f"  odom_filtered: {'✅' if self.odom_filtered_data else '❌'}")
        self.get_logger().info(f"  imu_filtered: {'✅' if self.imu_data else '❌'}")
        self.get_logger().info(f"  tf_buffer: {'✅' if self.tf_buffer else '❌'}")
        
        # 움직임 일치성 분석
        self.get_logger().info("\n🎯 MOVEMENT CONSISTENCY ANALYSIS:")
        consistency_results = self.analyze_movement_consistency()
        if isinstance(consistency_results, list):
            for result in consistency_results:
                self.get_logger().info(f"  {result}")
        else:
            self.get_logger().info(f"  {consistency_results}")
        
        # TF 연결성 확인
        self.get_logger().info("\n🔗 TF CONNECTIVITY CHECK:")
        tf_results = self.check_tf_connectivity()
        for result in tf_results:
            self.get_logger().info(f"  {result}")
        
        # 현재 위치 정보
        if self.odom_filtered_data:
            pos = self.odom_filtered_data.pose.pose.position
            orient = self.odom_filtered_data.pose.pose.orientation
            self.get_logger().info(f"\n📍 CURRENT POSITION:")
            self.get_logger().info(f"  Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            self.get_logger().info(f"  Orientation: ({orient.x:.3f}, {orient.y:.3f}, {orient.z:.3f}, {orient.w:.3f})")

def main():
    rclpy.init()
    node = LocalizationDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Diagnostic node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()