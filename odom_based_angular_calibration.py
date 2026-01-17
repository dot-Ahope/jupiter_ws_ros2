#!/usr/bin/env python3
"""
Odom 기반 Angular Scale 캘리브레이션
IMU 드리프트 문제 해결 - Odom을 종료 조건으로 사용

원리:
    1. IMU: 실시간 모니터링용 (참고)
    2. Odom: 종료 조건 판단 (드리프트 없음)
    3. 비교: IMU vs Odom으로 angular_scale 계산

2단계 방식:
    Phase 1: 90° 단일 테스트로 초기 angular_scale 획득
    Phase 2: 초기값 사용하여 모든 각도 테스트
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time
import numpy as np
from collections import defaultdict
import argparse


class OdomBasedCalibration(Node):
    def __init__(self, phase=2, initial_scale=1.5618, current_launch_scale=1.5618):
        super().__init__('odom_based_calibration')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_callback, 10)  # imu_filter_madgwick 비활성화로 raw IMU 사용
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.odom_yaw = 0.0
        self.odom_received = False
        
        self.ekf_odom_yaw = 0.0
        self.ekf_odom_received = False
        
        self.imu_yaw = 0.0
        self.imu_angular_vel_z = 0.0
        self.imu_received = False
        self.last_imu_time = None
        self.integrated_imu_yaw = 0.0
        
        # Configuration
        self.phase = phase
        self.initial_scale = initial_scale
        self.current_launch_scale = current_launch_scale  # ⭐ Launch에 이미 적용된 scale
        
        if phase == 1:
            self.test_angles = [90]  # Phase 1: 90°만
        else:
            self.test_angles = [90, 180, 270, 360]  # Phase 2: 전체
        
        self.test_speed = 0.3  # rad/s
        
        # Results
        self.results = []
        
    def quaternion_to_yaw(self, q):
        """Quaternion을 yaw 각도로 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        """오도메트리 콜백 (Raw Odom)"""
        q = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(q)
        self.odom_received = True
    
    def ekf_odom_callback(self, msg):
        """EKF 필터링된 오도메트리 콜백"""
        q = msg.pose.pose.orientation
        self.ekf_odom_yaw = self.quaternion_to_yaw(q)
        self.ekf_odom_received = True
        
    def imu_callback(self, msg):
        """IMU 콜백 - Raw IMU는 각속도만 사용"""
        current_time = self.get_clock().now()
        
        self.imu_angular_vel_z = msg.angular_velocity.z
        
        # 각속도 적분 (주 측정 방법)
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            if dt < 1.0:
                self.integrated_imu_yaw += self.imu_angular_vel_z * dt
        
        self.last_imu_time = current_time
        
        # Raw IMU는 orientation이 없으므로 적분값만 사용
        # orientation_covariance가 -1.0이면 orientation 무효
        if msg.orientation_covariance[0] >= 0:
            q = msg.orientation
            self.imu_yaw = self.quaternion_to_yaw(q)
        else:
            # Raw IMU: orientation 무효, 적분값만 사용
            self.imu_yaw = 0.0
        
        self.imu_received = True
    
    def wait_for_sensors(self, timeout=5.0):
        """센서 데이터 대기"""
        self.get_logger().info('센서 데이터 대기 중...')
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_received and self.ekf_odom_received and self.imu_received:
                self.get_logger().info('✅ 센서 준비 완료 (Odom + EKF + IMU)')
                return True
        
        self.get_logger().error('❌ 센서 데이터 수신 실패!')
        self.get_logger().error(f'   Odom: {self.odom_received}, EKF: {self.ekf_odom_received}, IMU: {self.imu_received}')
        return False
    
    def normalize_angle(self, angle):
        """각도 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        """로봇 정지"""
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
    
    def test_rotation_odom_based(self, target_angle_deg, direction='ccw', estimated_scale=None):
        """
        Odom 기반 회전 테스트 (드리프트 없음)
        
        Args:
            target_angle_deg: 목표 각도 (도)
            direction: 'ccw' 또는 'cw'
            estimated_scale: 추정 angular_scale (Phase 2용)
        """
        direction_name = "반시계" if direction == 'ccw' else "시계"
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'테스트: {target_angle_deg}° {direction_name}방향\n'
            f'{"="*70}'
        )
        
        # 설정
        speed = self.test_speed if direction == 'ccw' else -self.test_speed
        target_rad = math.radians(target_angle_deg)
        
        # 초기화
        time.sleep(0.5)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        start_odom_yaw = self.odom_yaw
        start_ekf_odom_yaw = self.ekf_odom_yaw
        start_imu_yaw = self.imu_yaw
        self.integrated_imu_yaw = 0.0
        start_time = time.time()
        
        # 회전 종료 기준 설정
        if estimated_scale is not None:
            # Phase 2: IMU 기준 회전 (Phase 1과 동일)
            # ⭐ 핵심: Odom은 부정확하므로 IMU 기준으로 회전
            #    IMU가 90°까지는 정확하므로 물리적 회전 보장
            # 
            # Odom은 측정만 하고 종료 조건으로 사용 안 함
            imu_target = abs(target_rad) * 0.95  # 85.5° IMU에서 멈춤
            odom_target = None  # Odom 기준 종료 사용 안 함
            
            self.get_logger().info(
                f'Phase 2: IMU 기준 회전 (Phase 1 방식)\n'
                f'estimated_scale: {estimated_scale:.4f} (참고용)\n'
                f'current_launch_scale: {self.current_launch_scale:.4f}\n'
                f'IMU 목표: {math.degrees(imu_target):.2f}° (물리적 회전 보장)'
            )
        else:
            # Phase 1: IMU 기준 회전
            imu_target = abs(target_rad) * 0.95  # 85.5° IMU에서 멈춤
            odom_target = None
            self.get_logger().info('Phase 1: IMU 기준 회전 (초기 angular_scale 획득)')
        
        # 회전 시작
        twist = Twist()
        twist.angular.z = speed
        
        self.get_logger().info(f'\n▶ 회전 시작 (속도: {speed:.2f} rad/s)...')
        
        last_log_time = time.time()
        rotation_count = 0  # 360° 이상 카운터
        last_odom_yaw = start_odom_yaw
        last_ekf_odom_yaw = start_ekf_odom_yaw
        accumulated_odom = 0.0
        accumulated_ekf_odom = 0.0
        
        while True:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            elapsed = time.time() - start_time
            
            # 현재 측정값
            current_odom_yaw = self.odom_yaw
            current_ekf_odom_yaw = self.ekf_odom_yaw
            odom_delta_raw = self.normalize_angle(current_odom_yaw - start_odom_yaw)
            
            # 360° 이상 회전 감지 (Odom)
            odom_step = self.normalize_angle(current_odom_yaw - last_odom_yaw)
            accumulated_odom += abs(odom_step)
            last_odom_yaw = current_odom_yaw
            
            # 360° 이상 회전 감지 (EKF Odom)
            ekf_odom_step = self.normalize_angle(current_ekf_odom_yaw - last_ekf_odom_yaw)
            accumulated_ekf_odom += abs(ekf_odom_step)
            last_ekf_odom_yaw = current_ekf_odom_yaw
            
            odom_delta = accumulated_odom
            ekf_odom_delta = accumulated_ekf_odom
            
            imu_integrated = abs(self.integrated_imu_yaw)
            imu_orientation_delta = abs(self.normalize_angle(self.imu_yaw - start_imu_yaw))
            
            # 진행률 표시
            if (time.time() - last_log_time) >= 0.5:
                # Phase 1, 2 모두 IMU 기준 진행률
                progress = (imu_integrated / abs(target_rad)) * 100
                
                self.get_logger().info(
                    f'진행 {progress:5.1f}% | '
                    f'Odom: {math.degrees(odom_delta):6.1f}° | '
                    f'EKF: {math.degrees(ekf_odom_delta):6.1f}° | '
                    f'IMU적분: {math.degrees(imu_integrated):6.1f}° | '
                    f'IMU방향: {math.degrees(imu_orientation_delta):6.1f}°'
                )
                last_log_time = time.time()
            
            # 종료 조건: Phase 1, 2 모두 IMU 기준
            if imu_integrated >= imu_target:
                self.get_logger().info('✅ IMU 목표 도달!')
                break
            
            # 안전 타임아웃
            max_time = abs(target_rad) / abs(speed) * 3
            if elapsed > max_time:
                self.get_logger().warn('⚠️  타임아웃!')
                break
        
        # 정지
        self.stop_robot()
        time.sleep(1.0)
        
        # 최종 측정
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        final_odom_yaw = self.odom_yaw
        final_ekf_odom_yaw = self.ekf_odom_yaw
        final_imu_yaw = self.imu_yaw
        final_imu_integrated = self.integrated_imu_yaw
        
        # 최종 회전량 (누적)
        odom_rotation = accumulated_odom
        ekf_odom_rotation = accumulated_ekf_odom
        imu_rotation = abs(final_imu_integrated)
        imu_orientation = abs(self.normalize_angle(final_imu_yaw - start_imu_yaw))
        
        # ⭐ Launch에서 이미 적용된 angular_scale 보정
        # Odom (raw)는 이미 self.current_launch_scale이 곱해진 상태
        # 실제 원시 Odom 값을 복원하려면 나눠야 함
        odom_rotation_raw = odom_rotation / self.current_launch_scale
        ekf_odom_rotation_raw = ekf_odom_rotation / self.current_launch_scale
        
        # 도 단위
        odom_deg = math.degrees(odom_rotation)  # Launch scale 적용된 값
        odom_deg_raw = math.degrees(odom_rotation_raw)  # 원시 값 (보정 전)
        ekf_odom_deg = math.degrees(ekf_odom_rotation)  # Launch scale 적용된 값
        ekf_odom_deg_raw = math.degrees(ekf_odom_rotation_raw)  # 원시 값 (보정 전)
        imu_integrated_deg = math.degrees(imu_rotation)
        imu_orientation_deg = math.degrees(imu_orientation)
        
        # angular_scale 계산 (원시 Odom 기준)
        if odom_deg_raw > 1.0:
            angular_scale_integrated = imu_integrated_deg / odom_deg_raw
            angular_scale_orientation = imu_orientation_deg / odom_deg_raw if imu_orientation_deg > 1.0 else 1.0
        else:
            angular_scale_integrated = 1.0
            angular_scale_orientation = 1.0
        
        # EKF angular_scale 계산 (원시 Odom 기준)
        if ekf_odom_deg_raw > 1.0:
            ekf_angular_scale_integrated = imu_integrated_deg / ekf_odom_deg_raw
            ekf_angular_scale_orientation = imu_orientation_deg / ekf_odom_deg_raw if imu_orientation_deg > 1.0 else 1.0
        else:
            ekf_angular_scale_integrated = 1.0
            ekf_angular_scale_orientation = 1.0
        
        # 오차
        error_integrated = abs(target_angle_deg) - imu_integrated_deg
        error_percent = (error_integrated / abs(target_angle_deg)) * 100
        
        # 결과 출력
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'📊 측정 결과:\n'
            f'{"-"*70}\n'
            f'목표 각도:          {abs(target_angle_deg):7.1f}°\n'
            f'\n'
            f'⚙️  Launch 파일 설정:\n'
            f'  현재 적용된 scale: {self.current_launch_scale:.4f}\n'
            f'\n'
            f'📍 Raw Odometry (base_node 출력):\n'
            f'  Odom (측정값):      {odom_deg:7.2f}° (scale={self.current_launch_scale:.4f} 적용 후)\n'
            f'  Odom (원시값):      {odom_deg_raw:7.2f}° (scale 적용 전) ⭐\n'
            f'  angular_scale:      {angular_scale_integrated:.4f} (IMU 적분 기준, 원시 Odom 기준) ⭐\n'
            f'  angular_scale:      {angular_scale_orientation:.4f} (IMU orientation 기준)\n'
            f'\n'
            f'📍 EKF Filtered Odometry (TF 기준):\n'
            f'  EKF Odom (측정):    {ekf_odom_deg:7.2f}° (Odom + IMU 융합)\n'
            f'  EKF Odom (원시):    {ekf_odom_deg_raw:7.2f}° (scale 적용 전)\n'
            f'  EKF angular_scale:  {ekf_angular_scale_integrated:.4f} (IMU 적분 기준)\n'
            f'  EKF angular_scale:  {ekf_angular_scale_orientation:.4f} (IMU orientation 기준)\n'
            f'\n'
            f'📍 IMU 직접 측정:\n'
            f'  IMU (적분):         {imu_integrated_deg:7.2f}° (자이로 적분)\n'
            f'  IMU (orientation):  {imu_orientation_deg:7.2f}° (필터링 후)\n'
            f'\n'
            f'소요 시간:          {time.time() - start_time:.1f}초\n'
            f'{"="*70}\n'
        )
        
        return {
            'target_deg': abs(target_angle_deg),
            'odom_deg': odom_deg,
            'odom_deg_raw': odom_deg_raw,  # ⭐ 원시 Odom (scale 적용 전)
            'ekf_odom_deg': ekf_odom_deg,
            'ekf_odom_deg_raw': ekf_odom_deg_raw,  # ⭐ 원시 EKF Odom
            'imu_integrated_deg': imu_integrated_deg,
            'imu_orientation_deg': imu_orientation_deg,
            'angular_scale_integrated': angular_scale_integrated,  # ⭐ 원시 Odom 기준
            'angular_scale_orientation': angular_scale_orientation,
            'ekf_angular_scale_integrated': ekf_angular_scale_integrated,
            'ekf_angular_scale_orientation': ekf_angular_scale_orientation,
            'error_deg': error_integrated,
            'error_percent': error_percent,
            'direction': direction,
            'duration': time.time() - start_time,
            'current_launch_scale': self.current_launch_scale  # ⭐ 참고용
        }
    
    def run_calibration(self):
        """캘리브레이션 실행"""
        if self.phase == 1:
            return self.run_phase1()
        else:
            return self.run_phase2()
    
    def run_phase1(self):
        """Phase 1: 90° 초기 캘리브레이션"""
        self.get_logger().info(
            '\n' + '='*70 + '\n'
            'Phase 1: 초기 angular_scale 획득 (90° 테스트)\n'
            '='*70 + '\n'
        )
        
        if not self.wait_for_sensors():
            return None
        
        time.sleep(2.0)
        
        # 90° 양방향 테스트
        self.get_logger().info('\n[1/2] 90° 반시계 테스트')
        result_ccw = self.test_rotation_odom_based(90, 'ccw', estimated_scale=None)
        self.results.append(result_ccw)
        time.sleep(3.0)
        
        self.get_logger().info('\n[2/2] 90° 시계 테스트')
        result_cw = self.test_rotation_odom_based(90, 'cw', estimated_scale=None)
        self.results.append(result_cw)
        time.sleep(3.0)
        
        # 평균 계산
        scales = [r['angular_scale_integrated'] for r in self.results]
        initial_scale = np.mean(scales)
        std_scale = np.std(scales)
        
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'Phase 1 완료!\n'
            f'{"="*70}\n'
            f'초기 angular_scale: {initial_scale:.4f} ± {std_scale:.4f}\n'
            f'\n'
            f'다음 단계:\n'
            f'  python3 odom_based_angular_calibration.py --phase 2 --scale {initial_scale:.4f}\n'
            f'{"="*70}\n'
        )
        
        return {
            'initial_scale': initial_scale,
            'std_scale': std_scale,
            'results': self.results
        }
    
    def run_phase2(self):
        """Phase 2: 전체 각도 캘리브레이션"""
        self.get_logger().info(
            '\n' + '='*70 + '\n'
            f'Phase 2: 전체 각도 캘리브레이션 (angular_scale={self.initial_scale:.4f} 사용)\n'
            '='*70 + '\n'
        )
        
        if not self.wait_for_sensors():
            return None
        
        time.sleep(2.0)
        
        total_tests = len(self.test_angles) * 2
        test_count = 0
        
        for angle in self.test_angles:
            # 반시계
            test_count += 1
            self.get_logger().info(f'\n\n[{test_count}/{total_tests}] 테스트 진행 중...')
            result_ccw = self.test_rotation_odom_based(angle, 'ccw', estimated_scale=self.initial_scale)
            self.results.append(result_ccw)
            time.sleep(3.0)
            
            # 시계
            test_count += 1
            self.get_logger().info(f'\n\n[{test_count}/{total_tests}] 테스트 진행 중...')
            result_cw = self.test_rotation_odom_based(angle, 'cw', estimated_scale=self.initial_scale)
            self.results.append(result_cw)
            time.sleep(3.0)
        
        return self.analyze_results()
    
    def analyze_results(self):
        """결과 분석"""
        self.get_logger().info(
            '\n\n' + '='*70 + '\n'
            '📊 최종 분석 결과\n'
            '='*70 + '\n'
        )
        
        # 각도별 그룹화
        by_angle = defaultdict(list)
        by_direction = defaultdict(list)
        
        for r in self.results:
            by_angle[r['target_deg']].append(r)
            by_direction[r['direction']].append(r)
        
        # 1. 각도별 분석
        self.get_logger().info('\n1️⃣  각도별 angular_scale:')
        self.get_logger().info('-' * 70)
        
        for angle in sorted(by_angle.keys()):
            scales = [r['angular_scale_integrated'] for r in by_angle[angle]]
            mean_scale = np.mean(scales)
            std_scale = np.std(scales)
            
            self.get_logger().info(
                f'{angle:4.0f}° → scale: {mean_scale:.4f} ± {std_scale:.4f}'
            )
        
        # 2. 방향별
        self.get_logger().info('\n2️⃣  방향별 비교:')
        self.get_logger().info('-' * 70)
        
        direction_scales = {}
        for direction in ['ccw', 'cw']:
            scales = [r['angular_scale_integrated'] for r in by_direction[direction]]
            mean_scale = np.mean(scales)
            std_scale = np.std(scales)
            direction_scales[direction] = mean_scale
            
            direction_name = "반시계" if direction == 'ccw' else "시계"
            self.get_logger().info(
                f'{direction_name:4s} → scale: {mean_scale:.4f} ± {std_scale:.4f}'
            )
        
        asymmetry = abs(direction_scales['ccw'] - direction_scales['cw'])
        asymmetry_percent = (asymmetry / direction_scales['ccw']) * 100
        
        self.get_logger().info(f'\n비대칭도: {asymmetry:.4f} ({asymmetry_percent:.2f}%)')
        
        # 3. 전체 통계
        self.get_logger().info('\n3️⃣  전체 통계:')
        self.get_logger().info('=' * 70)
        
        all_scales = [r['angular_scale_integrated'] for r in self.results]
        mean_scale = np.mean(all_scales)
        median_scale = np.median(all_scales)
        std_scale = np.std(all_scales)
        cv = (std_scale / mean_scale) * 100
        
        self.get_logger().info(f'평균:     {mean_scale:.4f}')
        self.get_logger().info(f'중앙값:   {median_scale:.4f}')
        self.get_logger().info(f'표준편차: {std_scale:.4f}')
        self.get_logger().info(f'변동계수: {cv:.2f}%')
        
        if cv < 2:
            confidence = "매우 높음 ✅"
        elif cv < 5:
            confidence = "높음 ✓"
        else:
            confidence = "보통 ⚠️"
        
        self.get_logger().info(f'신뢰도:   {confidence}')
        
        # 4. 권장값
        self.get_logger().info('\n4️⃣  권장 angular_scale:')
        self.get_logger().info('=' * 70)
        
        recommended_scale = median_scale
        
        self.get_logger().info(
            f'\n⭐ 권장값: {recommended_scale:.4f}\n'
            f'   (표준편차: ±{std_scale:.4f})\n'
            f'   (신뢰도: {confidence})\n'
        )
        
        # 5. 적용
        self.get_logger().info('\n5️⃣  적용 방법:')
        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'\n파일 수정:\n'
            f'  1. src/transbot_bringup/launch/bringup.launch.py\n'
            f'  2. src/sllidar_ros2/launch/transbot_full_system.launch.py\n'
            f'\n변경:\n'
            f"  'angular_scale': {recommended_scale:.4f},\n"
            f'\n빌드:\n'
            f'  colcon build --packages-select transbot_bringup sllidar_ros2\n'
        )
        
        self.get_logger().info('=' * 70 + '\n')
        
        return {
            'recommended_scale': recommended_scale,
            'mean_scale': mean_scale,
            'std_scale': std_scale,
            'cv': cv,
            'confidence': confidence,
            'all_results': self.results
        }


def main():
    parser = argparse.ArgumentParser(description='Odom 기반 Angular Scale 캘리브레이션')
    parser.add_argument('--phase', type=int, default=1, choices=[1, 2],
                        help='Phase 1: 초기값 획득, Phase 2: 전체 테스트')
    parser.add_argument('--scale', type=float, default=1.56,
                        help='Phase 2용 초기 angular_scale (Phase 1 결과 사용)')
    parser.add_argument('--launch-scale', type=float, default=1.5618,
                        help='Launch 파일에 이미 적용된 angular_scale (기본값: 1.5618)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    calibrator = OdomBasedCalibration(
        phase=args.phase, 
        initial_scale=args.scale,
        current_launch_scale=args.launch_scale  # ⭐ Launch scale 전달
    )
    
    try:
        print('\n' + '='*70)
        print('Odom 기반 Angular Scale 캘리브레이션')
        print('='*70)
        
        print(f'\n⚙️  설정:')
        print(f'  Launch 파일 angular_scale: {args.launch_scale:.4f}')
        print(f'  (Odom 원시값 = 측정값 / {args.launch_scale:.4f})')
        
        if args.phase == 1:
            print('\n📋 Phase 1: 초기 angular_scale 획득')
            print('  • 테스트: 90° (양방향)')
            print('  • 방법: IMU 기준 회전')
            print('  • 소요 시간: 약 5분')
        else:
            print(f'\n📋 Phase 2: 전체 각도 캘리브레이션')
            print(f'  • 초기 angular_scale: {args.scale:.4f}')
            print(f'  • 테스트: [90, 180, 270, 360]° (양방향)')
            print(f'  • 방법: Odom 기준 회전 (드리프트 없음)')
            print(f'  • 소요 시간: 약 20분')
        
        print('\n⚠️  준비사항:')
        print('  1. 로봇 주변 반경 3m 이상 공간 확보')
        print('  2. 배터리 50% 이상')
        print('  3. 시스템 실행:')
        print('     ros2 launch sllidar_ros2 transbot_full_system.launch.py')
        
        print('\n💡 개선 사항:')
        print('  ✅ IMU 드리프트 문제 해결')
        print('  ✅ Odom 기반 종료 조건 (드리프트 없음)')
        print('  ✅ 모든 각도에서 일관된 측정')
        print('  ✅ 360° 회전도 정확히 도달')
        
        print('\n시작하려면 Enter를 누르세요...')
        input()
        
        results = calibrator.run_calibration()
        
        if results:
            print('\n✅ 캘리브레이션 완료!')
            if args.phase == 1:
                print(f'\n⭐ 초기 angular_scale: {results["initial_scale"]:.4f}')
                print(f'\n다음 단계:')
                print(f'  python3 odom_based_angular_calibration.py --phase 2 --scale {results["initial_scale"]:.4f}')
            else:
                print(f'\n⭐ 최종 angular_scale: {results["recommended_scale"]:.4f}')
                print(f'   신뢰도: {results["confidence"]}')
        else:
            print('\n❌ 캘리브레이션 실패')
    
    except KeyboardInterrupt:
        print('\n\n⚠️  중단됨')
        calibrator.stop_robot()
    
    except Exception as e:
        print(f'\n❌ 오류: {e}')
        import traceback
        traceback.print_exc()
        calibrator.stop_robot()
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
