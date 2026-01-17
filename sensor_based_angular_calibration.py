#!/usr/bin/env python3
"""
센서 피드백 기반 Angular Scale 캘리브레이션
실제 IMU와 오도메트리 측정값을 비교하여 정확한 angular_scale 계산

사용법:
    python3 sensor_based_angular_calibration.py

원리:
    1. 명령: 일정 속도로 회전 시작
    2. IMU: 실제 회전 각도 측정 (절대값)
    3. Odom: 휠 인코더 측정값 (보정 필요)
    4. 비교: IMU 각도 / Odom 각도 = angular_scale
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


class SensorBasedCalibration(Node):
    def __init__(self):
        super().__init__('sensor_based_calibration')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.odom_yaw = 0.0
        self.odom_received = False
        
        self.imu_yaw = 0.0
        self.imu_received = False
        
        # IMU 각속도 누적 (더 정확한 방법)
        self.imu_angular_vel_z = 0.0
        self.last_imu_time = None
        self.integrated_imu_yaw = 0.0
        
        # Test configuration
        self.test_angles = [90, 180, 270, 360]
        self.test_speed = 0.3  # rad/s (일정 속도)
        
        # Results storage
        self.results = []
        
    def quaternion_to_yaw(self, q):
        """Quaternion을 yaw 각도로 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        """오도메트리 콜백 - 휠 인코더 기반 각도 (보정 전)"""
        q = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(q)
        self.odom_received = True
        
    def imu_callback(self, msg):
        """IMU 콜백 - 실제 각속도 측정"""
        current_time = self.get_clock().now()
        
        # 각속도 저장
        self.imu_angular_vel_z = msg.angular_velocity.z
        
        # 각속도 적분으로 각도 계산 (더 정확)
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            if dt < 1.0:  # 비정상적인 시간 차이 무시
                self.integrated_imu_yaw += self.imu_angular_vel_z * dt
        
        self.last_imu_time = current_time
        
        # Orientation도 저장 (참고용)
        q = msg.orientation
        self.imu_yaw = self.quaternion_to_yaw(q)
        self.imu_received = True
    
    def wait_for_sensors(self, timeout=5.0):
        """센서 데이터 대기"""
        self.get_logger().info('센서 데이터 대기 중...')
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_received and self.imu_received:
                self.get_logger().info('✅ 센서 준비 완료')
                return True
        
        self.get_logger().error('❌ 센서 데이터 수신 실패!')
        if not self.odom_received:
            self.get_logger().error('   - /odom_raw 토픽 없음')
        if not self.imu_received:
            self.get_logger().error('   - /imu/data_filtered 토픽 없음')
        return False
    
    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
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
    
    def test_rotation_with_sensors(self, target_angle_deg, direction='ccw'):
        """
        센서 피드백 기반 회전 테스트
        
        Args:
            target_angle_deg: 목표 각도 (도)
            direction: 'ccw' (반시계) 또는 'cw' (시계)
        
        Returns:
            dict: 측정 결과
        """
        direction_name = "반시계" if direction == 'ccw' else "시계"
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'테스트: {target_angle_deg}° {direction_name}방향\n'
            f'{"="*70}'
        )
        
        # 방향 설정
        speed = self.test_speed if direction == 'ccw' else -self.test_speed
        target_rad = math.radians(target_angle_deg)
        
        # 초기 상태 안정화
        time.sleep(0.5)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 초기값 기록
        start_odom_yaw = self.odom_yaw
        start_imu_yaw = self.imu_yaw
        self.integrated_imu_yaw = 0.0  # IMU 적분값 초기화
        start_time = time.time()
        
        self.get_logger().info(
            f'시작 상태:\n'
            f'  Odom yaw: {math.degrees(start_odom_yaw):.2f}°\n'
            f'  IMU yaw:  {math.degrees(start_imu_yaw):.2f}°'
        )
        
        # 회전 시작
        twist = Twist()
        twist.angular.z = speed
        
        self.get_logger().info(f'\n▶ 회전 시작 (명령 속도: {speed:.2f} rad/s)...')
        
        # IMU 기반 종료 조건: IMU 적분값이 목표의 95%에 도달
        target_threshold = abs(target_rad) * 0.95
        last_log_time = time.time()
        max_duration = abs(target_rad) / abs(speed) * 5  # 최대 5배 시간
        
        while True:
            # 명령 발행
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            elapsed = time.time() - start_time
            
            # 현재 측정값 계산
            odom_delta = self.normalize_angle(self.odom_yaw - start_odom_yaw)
            imu_delta = self.normalize_angle(self.imu_yaw - start_imu_yaw)
            imu_integrated = self.integrated_imu_yaw
            
            # 진행률 표시 (0.5초마다)
            if (time.time() - last_log_time) >= 0.5:
                progress = (abs(imu_integrated) / abs(target_rad)) * 100
                self.get_logger().info(
                    f'진행 {progress:5.1f}% | '
                    f'경과: {elapsed:4.1f}s | '
                    f'IMU적분: {math.degrees(imu_integrated):6.1f}° | '
                    f'Odom: {math.degrees(odom_delta):6.1f}°'
                )
                last_log_time = time.time()
            
            # 종료 조건: IMU 적분값 기준
            if abs(imu_integrated) >= target_threshold:
                self.get_logger().info('✅ 목표 각도 도달!')
                break
            
            # 안전 타임아웃
            if elapsed > max_duration:
                self.get_logger().warn('⚠️  타임아웃!')
                break
        
        # 정지
        self.stop_robot()
        time.sleep(1.0)
        
        # 최종 측정
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        final_odom_yaw = self.odom_yaw
        final_imu_yaw = self.imu_yaw
        final_imu_integrated = self.integrated_imu_yaw
        
        # 회전량 계산
        odom_rotation = self.normalize_angle(final_odom_yaw - start_odom_yaw)
        imu_rotation = self.normalize_angle(final_imu_yaw - start_imu_yaw)
        imu_integrated_rotation = final_imu_integrated
        
        # 여러 바퀴 회전 보정 (360° 이상)
        if abs(target_angle_deg) >= 360:
            full_rotations = int(abs(target_angle_deg) / 360)
            remainder = abs(target_angle_deg) % 360
            
            # Odom 보정
            if abs(odom_rotation) < math.radians(remainder + 90):
                if direction == 'ccw':
                    odom_rotation += full_rotations * 2 * math.pi
                else:
                    odom_rotation -= full_rotations * 2 * math.pi
            
            # IMU 보정 (orientation 기반)
            if abs(imu_rotation) < math.radians(remainder + 90):
                if direction == 'ccw':
                    imu_rotation += full_rotations * 2 * math.pi
                else:
                    imu_rotation -= full_rotations * 2 * math.pi
        
        # 도 단위 변환
        odom_deg = math.degrees(abs(odom_rotation))
        imu_deg = math.degrees(abs(imu_rotation))
        imu_integrated_deg = math.degrees(abs(imu_integrated_rotation))
        
        # Angular scale 계산 (IMU 적분값 기준 - 가장 정확)
        if odom_deg > 1.0:
            angular_scale_integrated = imu_integrated_deg / odom_deg
        else:
            angular_scale_integrated = 1.0
        
        # Angular scale 계산 (IMU orientation 기준 - 참고용)
        if odom_deg > 1.0:
            angular_scale_orientation = imu_deg / odom_deg
        else:
            angular_scale_orientation = 1.0
        
        # 오차 계산
        error_integrated = abs(target_angle_deg) - imu_integrated_deg
        error_percent = (error_integrated / abs(target_angle_deg)) * 100
        
        # 결과 출력
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'📊 측정 결과:\n'
            f'{"-"*70}\n'
            f'목표 각도:        {abs(target_angle_deg):7.1f}°\n'
            f'\n'
            f'IMU (적분):       {imu_integrated_deg:7.2f}° ⭐ (가장 정확)\n'
            f'IMU (orientation):{imu_deg:7.2f}°\n'
            f'Odom (raw):       {odom_deg:7.2f}° (보정 전)\n'
            f'\n'
            f'오차 (IMU적분):   {error_integrated:7.2f}° ({error_percent:.1f}%)\n'
            f'\n'
            f'angular_scale (적분 기준):  {angular_scale_integrated:.4f} ⭐\n'
            f'angular_scale (방향 기준):  {angular_scale_orientation:.4f}\n'
            f'\n'
            f'소요 시간:        {time.time() - start_time:.1f}초\n'
            f'{"="*70}\n'
        )
        
        return {
            'target_deg': abs(target_angle_deg),
            'imu_integrated_deg': imu_integrated_deg,
            'imu_orientation_deg': imu_deg,
            'odom_deg': odom_deg,
            'angular_scale_integrated': angular_scale_integrated,
            'angular_scale_orientation': angular_scale_orientation,
            'error_deg': error_integrated,
            'error_percent': error_percent,
            'direction': direction,
            'duration': time.time() - start_time
        }
    
    def run_calibration(self):
        """전체 캘리브레이션 실행"""
        self.get_logger().info(
            '\n' + '='*70 + '\n'
            '센서 피드백 기반 Angular Scale 캘리브레이션\n'
            '='*70 + '\n'
        )
        
        # 총 테스트 횟수
        total_tests = len(self.test_angles) * 2  # 양방향
        self.get_logger().info(
            f'📋 테스트 계획:\n'
            f'  • 각도: {self.test_angles}\n'
            f'  • 방향: 양방향 (시계 + 반시계)\n'
            f'  • 속도: {self.test_speed} rad/s (일정)\n'
            f'  • 총 테스트: {total_tests}회\n'
            f'  • 예상 시간: 약 {total_tests * 2}분\n'
        )
        
        if not self.wait_for_sensors():
            return None
        
        time.sleep(2.0)
        
        # 각 각도별로 양방향 테스트
        test_count = 0
        for angle in self.test_angles:
            # 반시계방향 (CCW)
            test_count += 1
            self.get_logger().info(f'\n\n📍 [{test_count}/{total_tests}] 테스트 진행 중...\n')
            result_ccw = self.test_rotation_with_sensors(angle, 'ccw')
            self.results.append(result_ccw)
            time.sleep(3.0)
            
            # 시계방향 (CW)
            test_count += 1
            self.get_logger().info(f'\n\n📍 [{test_count}/{total_tests}] 테스트 진행 중...\n')
            result_cw = self.test_rotation_with_sensors(angle, 'cw')
            self.results.append(result_cw)
            time.sleep(3.0)
        
        return self.analyze_results()
    
    def analyze_results(self):
        """결과 통계 분석"""
        self.get_logger().info(
            '\n\n' + '='*70 + '\n'
            '📊 분석 결과\n'
            '='*70 + '\n'
        )
        
        # 각도별 그룹화
        by_angle = defaultdict(list)
        by_direction = defaultdict(list)
        
        for r in self.results:
            by_angle[r['target_deg']].append(r)
            by_direction[r['direction']].append(r)
        
        # 1. 각도별 분석
        self.get_logger().info('\n1️⃣  각도별 angular_scale (IMU 적분 기준):')
        self.get_logger().info('-' * 70)
        
        for angle in sorted(by_angle.keys()):
            scales = [r['angular_scale_integrated'] for r in by_angle[angle]]
            mean_scale = np.mean(scales)
            std_scale = np.std(scales)
            
            self.get_logger().info(
                f'{angle:4.0f}° → scale: {mean_scale:.4f} ± {std_scale:.4f}'
            )
        
        # 2. 방향별 분석
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
        
        # 비대칭도
        asymmetry = abs(direction_scales['ccw'] - direction_scales['cw'])
        asymmetry_percent = (asymmetry / direction_scales['ccw']) * 100
        
        self.get_logger().info(f'\n비대칭도: {asymmetry:.4f} ({asymmetry_percent:.2f}%)')
        
        if asymmetry_percent > 3:
            self.get_logger().warn('⚠️  방향별 차이 3% 이상 - 기계적 문제 가능성')
        else:
            self.get_logger().info('✅ 방향별 차이 적음 - 정상')
        
        # 3. 전체 통계
        self.get_logger().info('\n3️⃣  전체 통계:')
        self.get_logger().info('=' * 70)
        
        all_scales = [r['angular_scale_integrated'] for r in self.results]
        mean_scale = np.mean(all_scales)
        median_scale = np.median(all_scales)
        std_scale = np.std(all_scales)
        min_scale = np.min(all_scales)
        max_scale = np.max(all_scales)
        
        self.get_logger().info(f'평균:     {mean_scale:.4f}')
        self.get_logger().info(f'중앙값:   {median_scale:.4f}')
        self.get_logger().info(f'표준편차: {std_scale:.4f}')
        self.get_logger().info(f'범위:     [{min_scale:.4f}, {max_scale:.4f}]')
        
        # 변동계수 (CV)
        cv = (std_scale / mean_scale) * 100
        
        if cv < 2:
            confidence = "매우 높음 ✅"
        elif cv < 5:
            confidence = "높음 ✓"
        elif cv < 10:
            confidence = "보통 ⚠️"
        else:
            confidence = "낮음 ❌"
        
        self.get_logger().info(f'변동계수: {cv:.2f}%')
        self.get_logger().info(f'신뢰도:   {confidence}')
        
        # 4. 권장값
        self.get_logger().info('\n4️⃣  권장 angular_scale:')
        self.get_logger().info('=' * 70)
        
        recommended_scale = median_scale  # 중앙값 사용 (이상치에 강인)
        
        self.get_logger().info(
            f'\n⭐ 권장값: {recommended_scale:.4f}\n'
            f'   (표준편차: ±{std_scale:.4f})\n'
            f'   (신뢰도: {confidence})\n'
        )
        
        # 5. 적용 방법
        self.get_logger().info('\n5️⃣  적용 방법:')
        self.get_logger().info('=' * 70)
        self.get_logger().info(
            '\n아래 파일들을 수정하세요:\n\n'
            '1️⃣  /home/jetson/transbot_ws_ros2/src/transbot_bringup/launch/bringup.launch.py\n'
            '2️⃣  /home/jetson/transbot_ws_ros2/src/sllidar_ros2/launch/transbot_full_system.launch.py\n'
            f'\n변경 내용:\n'
            f"  'angular_scale': {recommended_scale:.4f},\n"
            f'\n빌드:\n'
            f'  cd ~/transbot_ws_ros2\n'
            f'  colcon build --packages-select transbot_bringup sllidar_ros2\n'
        )
        
        self.get_logger().info('=' * 70 + '\n')
        
        return {
            'recommended_scale': recommended_scale,
            'mean_scale': mean_scale,
            'std_scale': std_scale,
            'confidence': confidence,
            'cv': cv,
            'asymmetry': asymmetry,
            'all_results': self.results
        }


def main():
    rclpy.init()
    
    calibrator = SensorBasedCalibration()
    
    try:
        print('\n' + '='*70)
        print('센서 피드백 기반 Angular Scale 캘리브레이션')
        print('='*70)
        print('\n📐 원리:')
        print('  1. IMU: 실제 회전 각도 측정 (각속도 적분)')
        print('  2. Odom: 휠 인코더 측정값 (보정 전)')
        print('  3. 비교: angular_scale = IMU 각도 / Odom 각도')
        print('\n📋 테스트 설정:')
        print(f'  • 각도: {calibrator.test_angles} (각 양방향)')
        print(f'  • 속도: {calibrator.test_speed} rad/s (일정)')
        print(f'  • 총 회전: {len(calibrator.test_angles)*2}회')
        print(f'  • 예상 시간: 약 {len(calibrator.test_angles)*2*2}분')
        print('\n⚠️  준비사항:')
        print('  1. 로봇 주변 반경 3m 이상 장애물 제거')
        print('  2. 배터리 50% 이상 충전 확인')
        print('  3. 아래 명령으로 시스템 먼저 실행:')
        print('     ros2 launch sllidar_ros2 transbot_full_system.launch.py')
        print('\n💡 장점:')
        print('  ✅ 시간 기반이 아닌 실제 센서 측정값 사용')
        print('  ✅ IMU 각속도 적분으로 정확한 회전량 계산')
        print('  ✅ 하드웨어 비선형성 자동 반영')
        print('  ✅ 과도 회전 방지')
        print('\n시작하려면 Enter를 누르세요...')
        input()
        
        results = calibrator.run_calibration()
        
        if results:
            print('\n✅ 캘리브레이션 완료!')
            print(f'\n⭐ 권장 angular_scale: {results["recommended_scale"]:.4f}')
            print(f'   신뢰도: {results["confidence"]}')
            print(f'   변동계수: {results["cv"]:.2f}%')
        else:
            print('\n❌ 캘리브레이션 실패')
    
    except KeyboardInterrupt:
        print('\n\n⚠️  사용자에 의해 중단됨')
        calibrator.stop_robot()
    
    except Exception as e:
        print(f'\n❌ 오류 발생: {e}')
        import traceback
        traceback.print_exc()
        calibrator.stop_robot()
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
