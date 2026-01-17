#!/usr/bin/env python3
"""
EKF 성능 비교 테스트
- IMU 기준 90° 회전 vs Odom 기준 90° 회전
- 각 센서의 측정값과 EKF 융합 결과 비교
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time
import sys

class EKFComparisonTest(Node):
    def __init__(self):
        super().__init__('ekf_comparison_test')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_callback, 10)
        
        # 데이터 저장
        self.odom_yaw = 0.0
        self.ekf_odom_yaw = 0.0
        self.imu_yaw = 0.0
        self.integrated_imu_yaw = 0.0
        self.last_imu_time = None
        
        # launch에서 설정된 angular_scale 파라미터 가져오기
        self.declare_parameter('angular_scale', 2.0251)
        self.angular_scale = self.get_parameter('angular_scale').value
        
        self.get_logger().info(f'EKF Comparison Test 초기화 완료')
        self.get_logger().info(f'Launch angular_scale: {self.angular_scale:.4f}')
        
    def odom_callback(self, msg):
        """Raw Odometry 콜백"""
        quat = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(quat)
        
    def ekf_odom_callback(self, msg):
        """EKF Filtered Odometry 콜백"""
        quat = msg.pose.pose.orientation
        self.ekf_odom_yaw = self.quaternion_to_yaw(quat)
        
    def imu_callback(self, msg):
        """IMU 콜백 - 자이로 적분"""
        current_time = time.time()
        
        if self.last_imu_time is not None:
            dt = current_time - self.last_imu_time
            if dt < 0.5:  # 정상적인 dt만 사용
                self.integrated_imu_yaw += msg.angular_velocity.z * dt
        
        self.last_imu_time = current_time
        
        # IMU orientation도 저장
        quat = msg.orientation
        self.imu_yaw = self.quaternion_to_yaw(quat)
    
    def quaternion_to_yaw(self, quat):
        """Quaternion을 yaw 각도로 변환"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
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
    
    def wait_for_sensors(self):
        """센서 데이터 대기"""
        self.get_logger().info('센서 데이터 대기 중...')
        
        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_imu_time is not None:
                self.get_logger().info('✅ 센서 준비 완료')
                time.sleep(1.0)
                return True
        
        self.get_logger().error('❌ 센서 타임아웃')
        return False
    
    def rotate_imu_based(self, target_degrees=90, speed=0.3):
        """IMU 적분 기준 회전"""
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'테스트 1: IMU 기준 {target_degrees}° 회전')
        self.get_logger().info(f'{"="*60}')
        
        # 방향 구분: 양수면 반시계(CCW), 음수면 시계(CW)
        direction = 1.0 if speed > 0 else -1.0
        target_rad = math.radians(target_degrees) * direction
        imu_target = target_rad * 0.95  # 목표의 95%에서 멈춤
        
        # 시작 측정
        start_odom = self.odom_yaw
        start_ekf = self.ekf_odom_yaw
        start_imu = self.imu_yaw
        self.integrated_imu_yaw = 0.0  # IMU 적분 초기화
        
        self.get_logger().info(f'IMU 목표: {math.degrees(imu_target):.2f}°')
        self.get_logger().info(f'회전 시작...\n')
        
        # 회전
        twist = Twist()
        twist.angular.z = speed
        
        start_time = time.time()
        last_log = start_time
        
        while True:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            # 현재 측정값 (방향 유지)
            imu_integrated = self.integrated_imu_yaw
            odom_delta = self.normalize_angle(self.odom_yaw - start_odom)
            ekf_delta = self.normalize_angle(self.ekf_odom_yaw - start_ekf)
            
            # 로그
            if time.time() - last_log > 0.5:
                progress = (imu_integrated / target_rad) * 100 if target_rad != 0 else 0
                self.get_logger().info(
                    f'진행 {progress:5.1f}% | '
                    f'IMU적분: {math.degrees(imu_integrated):+7.2f}° | '
                    f'Odom: {math.degrees(odom_delta):+7.2f}° | '
                    f'EKF: {math.degrees(ekf_delta):+7.2f}°'
                )
                last_log = time.time()
            
            # 종료 조건 (방향 고려)
            if direction > 0 and imu_integrated >= imu_target:
                self.get_logger().info('✅ IMU 목표 도달!')
                break
            elif direction < 0 and imu_integrated <= imu_target:
                self.get_logger().info('✅ IMU 목표 도달!')
                break
            
            # 타임아웃
            max_time = abs(target_rad) / abs(speed) * 3
            if time.time() - start_time > max_time:
                self.get_logger().warn('⚠️ 타임아웃!')
                break
        
        # 정지
        self.stop_robot()
        time.sleep(1.5)
        
        # 최종 측정
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        final_imu = self.integrated_imu_yaw
        final_odom = self.normalize_angle(self.odom_yaw - start_odom)
        final_ekf = self.normalize_angle(self.ekf_odom_yaw - start_ekf)
        
        duration = time.time() - start_time
        
        # 결과 출력
        self.print_results("IMU 기준", target_degrees, 
                          final_imu, final_odom, final_ekf, duration)
        
        return final_imu, final_odom, final_ekf
    
    def rotate_odom_based(self, target_degrees=90, speed=0.3):
        """Odom 측정값 기준 회전"""
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'테스트 2: Odom 기준 {target_degrees}° 회전')
        self.get_logger().info(f'{"="*60}')
        
        # 방향 구분: 양수면 반시계(CCW), 음수면 시계(CW)
        direction = 1.0 if target_degrees > 0 else -1.0
        target_rad = math.radians(abs(target_degrees)) * direction
        odom_target = target_rad * 0.95  # 목표의 95%에서 멈춤 (오버슈트 고려)
        
        # ⭐ 감속 구간 설정 (옵션 2: 감속 제어 - 최적화)
        # 60% 구간부터 감속 시작, 80% 구간부터 더 감속
        decel_start_1 = target_rad * 0.60  # 60% 지점
        decel_start_2 = target_rad * 0.80  # 80% 지점
        
        # 시작 측정
        start_odom = self.odom_yaw
        start_ekf = self.ekf_odom_yaw
        start_imu = self.imu_yaw
        self.integrated_imu_yaw = 0.0  # IMU 적분 초기화
        
        self.get_logger().info(f'Odom 목표: {math.degrees(odom_target):.2f}°')
        self.get_logger().info(f'⭐ 감속 구간: 60% ({math.degrees(decel_start_1):.1f}°) / 80% ({math.degrees(decel_start_2):.1f}°)')
        self.get_logger().info(f'회전 시작...\n')
        
        # 회전
        twist = Twist()
        twist.angular.z = speed
        
        start_time = time.time()
        last_log = start_time
        
        while True:
            # 현재 측정값 (방향 유지)
            imu_integrated = self.integrated_imu_yaw
            odom_delta = self.normalize_angle(self.odom_yaw - start_odom)
            ekf_delta = self.normalize_angle(self.ekf_odom_yaw - start_ekf)
            
            # ⭐⭐⭐ 감속 제어 (오버슈트 방지) ⭐⭐⭐
            # 절댓값 기준으로 진행률 계산
            abs_odom = abs(odom_delta)
            abs_target = abs(target_rad)
            abs_decel_1 = abs(decel_start_1)
            abs_decel_2 = abs(decel_start_2)
            
            if abs_odom >= abs_decel_2:
                twist.angular.z = speed * 0.4 * direction  # 80% 이후 40% 속도
            elif abs_odom >= abs_decel_1:
                twist.angular.z = speed * 0.7 * direction  # 60% 이후 70% 속도
            else:
                twist.angular.z = speed * direction  # 정상 속도
            
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            # 로그
            if time.time() - last_log > 0.5:
                progress = (odom_delta / target_rad) * 100 if target_rad != 0 else 0
                speed_percent = (twist.angular.z / speed) * 100
                self.get_logger().info(
                    f'진행 {progress:5.1f}% | 속도 {speed_percent:3.0f}% | '
                    f'Odom: {math.degrees(odom_delta):+7.2f}° | '
                    f'IMU적분: {math.degrees(imu_integrated):+7.2f}° | '
                    f'EKF: {math.degrees(ekf_delta):+7.2f}°'
                )
                last_log = time.time()
            
            # 종료 조건 (절댓값 기준)
            abs_odom_target = abs(odom_target)
            if abs_odom >= abs_odom_target:
                self.get_logger().info('✅ Odom 목표 도달!')
                break
            
            # 타임아웃
            max_time = abs(target_rad) / abs(speed) * 3
            if time.time() - start_time > max_time:
                self.get_logger().warn('⚠️ 타임아웃!')
                break
        
        # 정지
        self.stop_robot()
        time.sleep(1.5)
        
        # 최종 측정
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        final_imu = self.integrated_imu_yaw
        final_odom = self.normalize_angle(self.odom_yaw - start_odom)
        final_ekf = self.normalize_angle(self.ekf_odom_yaw - start_ekf)
        
        duration = time.time() - start_time
        
        # 결과 출력
        self.print_results("Odom 기준", target_degrees,
                          final_imu, final_odom, final_ekf, duration)
        
        return final_imu, final_odom, final_ekf
    
    def print_results(self, test_name, target, imu, odom, ekf, duration):
        """결과 출력"""
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'📊 {test_name} 회전 결과')
        self.get_logger().info(f'{"-"*60}')
        self.get_logger().info(f'목표:            {target:+.2f}°')
        self.get_logger().info(f'')
        self.get_logger().info(f'📍 측정값:')
        self.get_logger().info(f'  IMU 적분:      {math.degrees(imu):+7.2f}° (오차: {math.degrees(imu) - target:+.2f}°)')
        self.get_logger().info(f'  Odom (raw):    {math.degrees(odom):+7.2f}° (오차: {math.degrees(odom) - target:+.2f}°)')
        self.get_logger().info(f'  EKF (융합):    {math.degrees(ekf):+7.2f}° (오차: {math.degrees(ekf) - target:+.2f}°)')
        self.get_logger().info(f'')
        self.get_logger().info(f'⚙️  Odom 엔코더 원시값 (angular_scale 보정 전):')
        # Odom은 엔코더 원시값에 angular_scale을 곱해서 생성됨
        # 따라서 역계산은 나누기가 아니라 angular_scale로 나눠야 함
        odom_encoder_raw = odom / self.angular_scale
        self.get_logger().info(f'  {math.degrees(odom_encoder_raw):+7.2f}° (측정 {math.degrees(odom):+.2f}° ÷ {self.angular_scale:.4f})')
        self.get_logger().info(f'')
        self.get_logger().info(f'🔄 IMU vs Odom 비율:')
        if abs(odom) > 0.01:
            ratio = imu / odom
            self.get_logger().info(f'  {ratio:.4f} (IMU {math.degrees(imu):+.2f}° / Odom {math.degrees(odom):+.2f}°)')
        else:
            self.get_logger().info(f'  N/A (Odom 너무 작음)')
        self.get_logger().info(f'')
        self.get_logger().info(f'⏱️  소요 시간: {duration:.2f}초')
        self.get_logger().info(f'{"="*60}\n')


def main(args=None):
    rclpy.init(args=args)
    
    print('\n' + '='*60)
    print('EKF 성능 비교 테스트')
    print('='*60)
    print('\n⚠️  준비사항:')
    print('  1. 로봇 주변 2m 이상 공간 확보')
    print('  2. 시스템 실행:')
    print('     ros2 launch sllidar_ros2 jupiter_full_system.launch.py')
    print('\n📋 테스트 순서:')
    print('  1. IMU 기준 90° 회전 (반시계)')
    print('  2. IMU 기준 90° 회전 (시계)')
    print('  3. Odom 기준 90° 회전 (반시계)')
    print('  4. Odom 기준 90° 회전 (시계)')
    print('  5. 각 센서 측정값 및 EKF 융합 결과 비교')
    print('\n시작하려면 Enter를 누르세요...')
    input()
    
    node = EKFComparisonTest()
    
    if not node.wait_for_sensors():
        node.destroy_node()
        rclpy.shutdown()
        return
    
    time.sleep(1.0)
    
    try:
        # 테스트 1: IMU 기준 반시계
        print('\n' + '='*60)
        print('테스트 1/4: IMU 기준 반시계방향')
        print('='*60)
        imu_ccw = node.rotate_imu_based(target_degrees=90, speed=0.3)
        time.sleep(3.0)
        
        # 테스트 2: IMU 기준 시계
        print('\n' + '='*60)
        print('테스트 2/4: IMU 기준 시계방향')
        print('='*60)
        imu_cw = node.rotate_imu_based(target_degrees=90, speed=-0.3)
        time.sleep(3.0)
        
        # 테스트 3: Odom 기준 반시계
        print('\n' + '='*60)
        print('테스트 3/4: Odom 기준 반시계방향')
        print('='*60)
        odom_ccw = node.rotate_odom_based(target_degrees=90, speed=0.3)
        time.sleep(3.0)
        
        # 테스트 4: Odom 기준 시계
        print('\n' + '='*60)
        print('테스트 4/4: Odom 기준 시계방향')
        print('='*60)
        odom_cw = node.rotate_odom_based(target_degrees=90, speed=-0.3)
        
        # 최종 비교
        print('\n' + '='*60)
        print('📊 최종 비교 분석')
        print('='*60)
        
        print(f'\n1️⃣  IMU 기준 회전:')
        print(f'   반시계 (+90°):')
        print(f'     IMU 적분: {math.degrees(imu_ccw[0]):+7.2f}°')
        print(f'     Odom 측정: {math.degrees(imu_ccw[1]):+7.2f}°')
        print(f'     EKF 융합:  {math.degrees(imu_ccw[2]):+7.2f}°')
        print(f'   시계 (-90°):')
        print(f'     IMU 적분: {math.degrees(imu_cw[0]):+7.2f}°')
        print(f'     Odom 측정: {math.degrees(imu_cw[1]):+7.2f}°')
        print(f'     EKF 융합:  {math.degrees(imu_cw[2]):+7.2f}°')
        
        print(f'\n2️⃣  Odom 기준 회전:')
        print(f'   반시계 (+90°):')
        print(f'     IMU 적분: {math.degrees(odom_ccw[0]):+7.2f}°')
        print(f'     Odom 측정: {math.degrees(odom_ccw[1]):+7.2f}°')
        print(f'     EKF 융합:  {math.degrees(odom_ccw[2]):+7.2f}°')
        print(f'   시계 (-90°):')
        print(f'     IMU 적분: {math.degrees(odom_cw[0]):+7.2f}°')
        print(f'     Odom 측정: {math.degrees(odom_cw[1]):+7.2f}°')
        print(f'     EKF 융합:  {math.degrees(odom_cw[2]):+7.2f}°')
        
        print(f'\n3️⃣  EKF 성능 분석:')
        
        # IMU 기준에서의 EKF 차이
        ekf_imu_diff = abs(math.degrees(imu_ccw[2]) - math.degrees(imu_cw[2]))
        print(f'   IMU 기준 EKF 차이: {ekf_imu_diff:.2f}° (반시계 vs 시계)')
        
        # Odom 기준에서의 EKF 차이
        ekf_odom_diff = abs(math.degrees(odom_ccw[2]) - math.degrees(odom_cw[2]))
        print(f'   Odom 기준 EKF 차이: {ekf_odom_diff:.2f}° (반시계 vs 시계)')
        
        # 전체 EKF 일관성 (절대값으로 비교)
        ekf_all = [
            abs(math.degrees(imu_ccw[2])), abs(math.degrees(imu_cw[2])),
            abs(math.degrees(odom_ccw[2])), abs(math.degrees(odom_cw[2]))
        ]
        ekf_max = max(ekf_all)
        ekf_min = min(ekf_all)
        ekf_range = ekf_max - ekf_min
        
        print(f'   EKF 전체 범위: {ekf_range:.2f}° (최대 {ekf_max:+.2f}° - 최소 {ekf_min:+.2f}°)')
        
        if ekf_range < 10.0:
            print('   ✅ EKF가 일관되게 작동 (범위 < 10°)')
        elif ekf_range < 20.0:
            print('   ⚠️  EKF 일관성 보통 (범위 10-20°)')
        else:
            print('   ❌ EKF 일관성 낮음 (범위 > 20°)')
        
        print(f'\n4️⃣  센서 비대칭 분석:')
        
        # IMU 비대칭 (절대값 비교)
        imu_ccw_avg = abs(math.degrees(imu_ccw[0]))
        imu_cw_avg = abs(math.degrees(imu_cw[0]))
        imu_asymmetry = abs(imu_ccw_avg - imu_cw_avg)
        print(f'   IMU 적분 비대칭: {imu_asymmetry:.2f}° (반시계 {math.degrees(imu_ccw[0]):+.2f}° vs 시계 {math.degrees(imu_cw[0]):+.2f}°)')
        
        # Odom 비대칭 (IMU 기준 회전에서, 절대값 비교)
        odom_imu_ccw = abs(math.degrees(imu_ccw[1]))
        odom_imu_cw = abs(math.degrees(imu_cw[1]))
        odom_asymmetry = abs(odom_imu_ccw - odom_imu_cw)
        print(f'   Odom 측정 비대칭: {odom_asymmetry:.2f}° (반시계 {math.degrees(imu_ccw[1]):+.2f}° vs 시계 {math.degrees(imu_cw[1]):+.2f}°)')
        
        # 권장 angular_scale 계산
        print(f'\n5️⃣  angular_scale 분석:')
        print(f'   현재 설정: {node.angular_scale:.4f}')
        
        # IMU를 기준으로 Odom이 얼마나 측정했는지 계산
        imu_avg = (abs(math.degrees(imu_ccw[0])) + abs(math.degrees(imu_cw[0]))) / 2
        odom_avg = (abs(math.degrees(imu_ccw[1])) + abs(math.degrees(imu_cw[1]))) / 2
        
        if odom_avg > 1.0:
            actual_ratio = imu_avg / odom_avg
            recommended_scale = node.angular_scale * actual_ratio
            print(f'   IMU 평균: {imu_avg:.2f}°')
            print(f'   Odom 평균: {odom_avg:.2f}°')
            print(f'   실제 비율: {actual_ratio:.4f}')
            print(f'   권장 angular_scale: {recommended_scale:.4f}')
            
            if abs(actual_ratio - 1.0) < 0.05:
                print(f'   ✅ angular_scale이 잘 보정됨 (오차 < 5%)')
            elif abs(actual_ratio - 1.0) < 0.1:
                print(f'   ⚠️  angular_scale 미세 조정 권장 (오차 5-10%)')
            else:
                print(f'   ❌ angular_scale 재보정 필요 (오차 > 10%)')
        else:
            print(f'   ⚠️  Odom 측정값이 너무 작아 분석 불가')
        
        print('\n' + '='*60)
        
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
