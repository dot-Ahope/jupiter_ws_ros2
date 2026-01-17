#!/usr/bin/env python3
"""
90도 회전 테스트 - 오도메트리 데이터와 물리적 동작 일치 확인

테스트 시나리오:
1. 초기 위치/방향 기록
2. 90도 반시계방향(좌회전) 명령
3. 오도메트리 yaw 각도 변화 모니터링
4. 물리적 회전 방향과 데이터 일치 확인
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time
from collections import deque

class RotationTester(Node):
    def __init__(self):
        super().__init__('rotation_tester')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_raw_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_raw_callback, 10)
        self.odom_filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_filtered_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_calibrated', self.imu_callback, 10)  # ⭐ data_filtered → data_calibrated
        
        # Data storage
        self.odom_raw_yaw = None
        self.odom_filtered_yaw = None
        self.imu_angular_z = None
        
        self.initial_odom_raw_yaw = None
        self.initial_odom_filtered_yaw = None
        
        # History for analysis
        self.odom_raw_history = deque(maxlen=5000)
        self.odom_filtered_history = deque(maxlen=5000)
        self.imu_history = deque(maxlen=5000)
        
        self.test_running = False
        self.start_time = None
        
        self.get_logger().info('=== 90도 회전 테스트 초기화 완료 ===')
        self.get_logger().info('준비 중... 초기 데이터 수집 중...')
        
    def quaternion_to_yaw(self, orientation):
        """쿼터니언을 yaw 각도로 변환 (라디안)"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def odom_raw_callback(self, msg):
        """휠 오도메트리 콜백"""
        self.odom_raw_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        angular_z = msg.twist.twist.angular.z
        
        if self.test_running:
            self.odom_raw_history.append({
                'time': time.time() - self.start_time,
                'yaw': self.odom_raw_yaw,
                'angular_z': angular_z
            })
    
    def odom_filtered_callback(self, msg):
        """융합 오도메트리 콜백"""
        self.odom_filtered_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        angular_z = msg.twist.twist.angular.z
        
        if self.test_running:
            self.odom_filtered_history.append({
                'time': time.time() - self.start_time,
                'yaw': self.odom_filtered_yaw,
                'angular_z': angular_z
            })
    
    def imu_callback(self, msg):
        """IMU 콜백"""
        self.imu_angular_z = msg.angular_velocity.z
        
        if self.test_running:
            self.imu_history.append({
                'time': time.time() - self.start_time,
                'angular_z': self.imu_angular_z
            })
    
    def wait_for_data(self, timeout=10.0):
        """초기 데이터가 수신될 때까지 대기"""
        self.get_logger().info('센서 데이터 수신 대기 중...')
        rate = self.create_rate(10)
        start_time = time.time()
        last_status_time = start_time
        
        while rclpy.ok():
            elapsed = time.time() - start_time
            
            # 1초마다 상태 출력
            if time.time() - last_status_time >= 1.0:
                status = []
                status.append(f"  /odom_raw: {'✓' if self.odom_raw_yaw is not None else '✗'}")
                status.append(f"  /odometry/filtered: {'✓' if self.odom_filtered_yaw is not None else '✗'}")
                status.append(f"  /imu/data_calibrated: {'✓' if self.imu_angular_z is not None else '✗'}")  # ⭐ 표시 변경
                self.get_logger().info(f'[{elapsed:.1f}s] ' + ' | '.join(status))
                last_status_time = time.time()
            
            # 모든 데이터 수신 완료
            if (self.odom_raw_yaw is not None and 
                self.odom_filtered_yaw is not None and
                self.imu_angular_z is not None):
                self.get_logger().info('✓ 모든 센서 데이터 수신 완료')
                return True
            
            # 타임아웃 체크
            if elapsed > timeout:
                self.get_logger().error(f'타임아웃: {timeout}초 내에 센서 데이터를 수신하지 못했습니다.')
                missing = []
                if self.odom_raw_yaw is None:
                    missing.append('/odom_raw')
                if self.odom_filtered_yaw is None:
                    missing.append('/odometry/filtered')
                if self.imu_angular_z is None:
                    missing.append('/imu/data_calibrated')  # ⭐ 오류 메시지 변경
                self.get_logger().error(f'수신되지 않은 토픽: {", ".join(missing)}')
                return False
            
            # 콜백 처리를 위해 rclpy.spin_once 호출
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return False
    
    def ensure_stopped(self):
        """로봇이 완전히 정지할 때까지 정지 명령 전송"""
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        
        self.get_logger().info('정지 명령 전송 중...')
        
        # 3초간 20Hz로 정지 명령 전송
        for i in range(60):
            self.cmd_vel_pub.publish(stop_twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
        
        # 추가 안정화 대기
        time.sleep(1.0)
        self.get_logger().info('정지 완료')
    
    def print_status(self):
        """현재 상태 출력"""
        if self.odom_raw_yaw is None or self.odom_filtered_yaw is None:
            return
        
        # 회전량 계산
        if self.initial_odom_raw_yaw is not None:
            raw_delta = self.normalize_angle(self.odom_raw_yaw - self.initial_odom_raw_yaw)
            filtered_delta = self.normalize_angle(self.odom_filtered_yaw - self.initial_odom_filtered_yaw)
            
            self.get_logger().info(
                f'\n'
                f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
                f'[휠 오도메트리] /odom_raw\n'
                f'  현재 Yaw: {math.degrees(self.odom_raw_yaw):7.2f}°\n'
                f'  회전량:    {math.degrees(raw_delta):7.2f}° (목표: 90.0°)\n'
                f'  오차:      {math.degrees(raw_delta - math.pi/2):7.2f}°\n'
                f'\n'
                f'[융합 오도메트리] /odometry/filtered\n'
                f'  현재 Yaw: {math.degrees(self.odom_filtered_yaw):7.2f}°\n'
                f'  회전량:    {math.degrees(filtered_delta):7.2f}° (목표: 90.0°)\n'
                f'  오차:      {math.degrees(filtered_delta - math.pi/2):7.2f}°\n'
                f'\n'
                f'[IMU 각속도] /imu/data_calibrated\n'  # ⭐ 표시 변경
                f'  Angular.z: {self.imu_angular_z:7.4f} rad/s\n'
                f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
            )
    
    def run_rotation_test(self, target_angle_deg=90.0, angular_speed=0.2):
        """
        회전 테스트 실행
        
        Args:
            target_angle_deg: 목표 회전 각도 (도 단위, 양수=반시계방향)
            angular_speed: 각속도 (rad/s, 양수=반시계방향) - 기본값 0.2로 감소
        """
        self.get_logger().info('\n\n' + '='*50)
        self.get_logger().info(f'90도 회전 테스트 시작')
        self.get_logger().info(f'목표: {target_angle_deg}도 반시계방향 회전')
        self.get_logger().info(f'각속도: {angular_speed} rad/s ({math.degrees(angular_speed):.1f} deg/s)')
        self.get_logger().info('='*50 + '\n')
        
        # 1단계: 완전 정지 확인
        self.get_logger().info('━━━ 1단계: 로봇 정지 중 ━━━')
        self.ensure_stopped()
        self.get_logger().info('✓ 로봇 완전 정지 완료\n')
        
        # 2단계: 초기 데이터 대기
        self.get_logger().info('━━━ 2단계: 센서 데이터 수집 중 ━━━')
        if not self.wait_for_data(timeout=10.0):
            self.get_logger().error('센서 데이터 수신 실패. 테스트를 중단합니다.')
            self.get_logger().info('\n해결 방법:')
            self.get_logger().info('  1. transbot_full_system.launch.py가 실행 중인지 확인')
            self.get_logger().info('  2. 로봇에 작은 움직임을 주어 데이터 발행 시작:')
            self.get_logger().info('     ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"')
            self.get_logger().info('  3. 다시 테스트 실행')
            return
        
        time.sleep(1.0)  # 안정화 대기
        
        # 3단계: 초기 각도 기록
        self.get_logger().info('━━━ 3단계: 초기 각도 기록 ━━━')
        self.initial_odom_raw_yaw = self.odom_raw_yaw
        self.initial_odom_filtered_yaw = self.odom_filtered_yaw
        
        self.get_logger().info(
            f'  휠 오도메트리 Yaw:  {math.degrees(self.initial_odom_raw_yaw):7.2f}° ← 제어 기준\n'
            f'  융합 오도메트리 Yaw: {math.degrees(self.initial_odom_filtered_yaw):7.2f}° (참고용)\n'
        )
        
        # 테스트 시작
        self.test_running = True
        self.start_time = time.time()
        
        # 회전 시간 계산
        target_angle_rad = math.radians(target_angle_deg)
        estimated_time = abs(target_angle_rad / angular_speed)
        max_time = estimated_time * 2.0  # 안전을 위한 최대 시간 (2배)
        
        self.get_logger().info(f'\n예상 회전 시간: {estimated_time:.1f}초 (최대: {max_time:.1f}초)\n')
        self.get_logger().info('회전 시작...\n')
        
        # 회전 명령 발행
        twist = Twist()
        twist.angular.z = angular_speed  # 양수 = 반시계방향 (좌회전)
        
        # 목표 절대 각도 계산 (wrap 문제 해결)
        target_absolute_yaw = self.normalize_angle(
            self.initial_odom_raw_yaw + target_angle_rad
        )
        
        loop_rate = 20  # 20Hz로 제어
        elapsed = 0.0
        last_print_time = 0.0
        target_reached = False
        
        self.get_logger().info('━━━ 4단계: 회전 시작 ━━━')
        self.get_logger().info(
            f'  목표 절대 각도: {math.degrees(target_absolute_yaw):.1f}°\n'
            f'  회전 속도: {angular_speed} rad/s ({math.degrees(angular_speed):.1f} deg/s)\n'
        )
        
        while rclpy.ok():
            # 현재 각도와 목표 각도의 오차 계산 (최단 경로)
            if self.odom_raw_yaw is not None:
                error = self.normalize_angle(
                    target_absolute_yaw - self.odom_raw_yaw
                )
                remaining = abs(error)
                
                # 목표 각도 도달 확인 (5도 오차 허용)
                if remaining < math.radians(5.0):
                    target_reached = True
                    current_deg = math.degrees(self.odom_raw_yaw)
                    target_deg = math.degrees(target_absolute_yaw)
                    self.get_logger().info(
                        f'\n✓ 목표 각도 도달! '
                        f'(목표: {target_deg:.1f}°, 현재: {current_deg:.1f}°, 오차: {math.degrees(error):.1f}°) '
                        f'[odom_raw 기준]'
                    )
                    break
                
                # 감속 구간: 30도 남았을 때부터 속도 감소 (오버슈트 방지)
                if remaining < math.radians(30.0):
                    speed_scale = remaining / math.radians(30.0)
                    speed_scale = max(speed_scale, 0.2)  # 최소 20% 속도 유지
                    twist.angular.z = angular_speed * speed_scale * (1.0 if error > 0 else -1.0)
                else:
                    twist.angular.z = angular_speed * (1.0 if error > 0 else -1.0)
            
            # 회전 명령 전송 (위에서 계산된 twist 사용)
            self.cmd_vel_pub.publish(twist)
            elapsed = time.time() - self.start_time
            
            # 1초마다 상태 출력
            if elapsed - last_print_time >= 1.0:
                self.print_status()
                last_print_time = elapsed
            
            # 최대 시간 초과 확인 (안전장치)
            if elapsed > max_time:
                self.get_logger().warn(
                    f'\n⚠ 최대 시간({max_time:.1f}초) 초과! 회전 중단.'
                )
                break
            
            # 콜백 처리 및 대기 (20Hz)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(1.0 / loop_rate)
        
        # 정지 - 여러 번 전송하여 확실히 정지
        self.get_logger().info('\n\n━━━ 정지 명령 전송 중 ━━━')
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.linear.z = 0.0
        stop_twist.angular.x = 0.0
        stop_twist.angular.y = 0.0
        stop_twist.angular.z = 0.0
        
        # 2초간 20Hz로 정지 명령 전송 (확실한 정지)
        for i in range(40):
            self.cmd_vel_pub.publish(stop_twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
        
        self.get_logger().info('✓ 로버 정지 완료. 안정화 대기 중...\n')
        time.sleep(1.5)  # 안정화 대기
        
        # 최종 결과 분석
        self.analyze_results(target_angle_deg)
        
        self.test_running = False
    
    def analyze_results(self, target_angle_deg):
        """테스트 결과 분석 및 출력"""
        target_rad = math.radians(target_angle_deg)
        
        # 최종 회전량 계산
        raw_delta = self.normalize_angle(self.odom_raw_yaw - self.initial_odom_raw_yaw)
        filtered_delta = self.normalize_angle(self.odom_filtered_yaw - self.initial_odom_filtered_yaw)
        
        # 오차 계산
        raw_error = math.degrees(raw_delta - target_rad)
        filtered_error = math.degrees(filtered_delta - target_rad)
        
        # 물리적 회전 방향 확인
        expected_direction = "반시계방향 (좌회전)" if target_angle_deg > 0 else "시계방향 (우회전)"
        raw_direction = "반시계방향 (좌회전)" if raw_delta > 0 else "시계방향 (우회전)"
        filtered_direction = "반시계방향 (좌회전)" if filtered_delta > 0 else "시계방향 (우회전)"
        
        # 방향 일치 확인
        raw_direction_match = (raw_delta > 0) == (target_angle_deg > 0)
        filtered_direction_match = (filtered_delta > 0) == (target_angle_deg > 0)
        
        # 결과 출력
        self.get_logger().info(
            f'\n\n'
            f'{"="*70}\n'
            f'최종 결과 분석\n'
            f'{"="*70}\n'
            f'\n'
            f'[목표]\n'
            f'  회전 각도: {target_angle_deg:7.2f}°\n'
            f'  회전 방향: {expected_direction}\n'
            f'  ROS 규약:  양수 = 반시계방향(CCW, 좌회전), 음수 = 시계방향(CW, 우회전)\n'
            f'\n'
            f'{"─"*70}\n'
            f'\n'
            f'[휠 오도메트리] /odom_raw\n'
            f'  초기 Yaw:    {math.degrees(self.initial_odom_raw_yaw):7.2f}°\n'
            f'  최종 Yaw:    {math.degrees(self.odom_raw_yaw):7.2f}°\n'
            f'  실제 회전량: {math.degrees(raw_delta):7.2f}°\n'
            f'  오차:        {raw_error:7.2f}° ({abs(raw_error)/target_angle_deg*100:.1f}%)\n'
            f'  회전 방향:   {raw_direction}\n'
            f'  방향 일치:   {"✓ 일치" if raw_direction_match else "✗ 불일치 (심각한 문제!)"}\n'
            f'\n'
            f'{"─"*70}\n'
            f'\n'
            f'[융합 오도메트리] /odometry/filtered (EKF)\n'
            f'  초기 Yaw:    {math.degrees(self.initial_odom_filtered_yaw):7.2f}°\n'
            f'  최종 Yaw:    {math.degrees(self.odom_filtered_yaw):7.2f}°\n'
            f'  실제 회전량: {math.degrees(filtered_delta):7.2f}°\n'
            f'  오차:        {filtered_error:7.2f}° ({abs(filtered_error)/target_angle_deg*100:.1f}%)\n'
            f'  회전 방향:   {filtered_direction}\n'
            f'  방향 일치:   {"✓ 일치" if filtered_direction_match else "✗ 불일치 (심각한 문제!)"}\n'
            f'\n'
            f'{"─"*70}\n'
            f'\n'
            f'[데이터 샘플 수]\n'
            f'  휠 오도메트리: {len(self.odom_raw_history)}개\n'
            f'  융합 오도메트리: {len(self.odom_filtered_history)}개\n'
            f'  IMU:          {len(self.imu_history)}개\n'
            f'\n'
            f'{"="*70}\n'
        )
        
        # 평가
        self.get_logger().info('\n[종합 평가]\n')
        
        if raw_direction_match and filtered_direction_match:
            self.get_logger().info('✓ 회전 방향: 데이터와 물리적 동작이 일치합니다.')
        else:
            self.get_logger().error('✗ 회전 방향: 데이터와 물리적 동작이 불일치합니다! (심각)')
        
        if abs(raw_error) < 10.0:
            self.get_logger().info(f'✓ 휠 오도메트리 정확도: 우수 (오차 {abs(raw_error):.2f}°)')
        elif abs(raw_error) < 20.0:
            self.get_logger().warn(f'⚠ 휠 오도메트리 정확도: 보통 (오차 {abs(raw_error):.2f}°)')
        else:
            self.get_logger().error(f'✗ 휠 오도메트리 정확도: 불량 (오차 {abs(raw_error):.2f}°)')
        
        if abs(filtered_error) < 10.0:
            self.get_logger().info(f'✓ 융합 오도메트리 정확도: 우수 (오차 {abs(filtered_error):.2f}°)')
        elif abs(filtered_error) < 20.0:
            self.get_logger().warn(f'⚠ 융합 오도메트리 정확도: 보통 (오차 {abs(filtered_error):.2f}°)')
        else:
            self.get_logger().error(f'✗ 융합 오도메트리 정확도: 불량 (오차 {abs(filtered_error):.2f}°)')
        
        # 권장 사항
        self.get_logger().info('\n[권장 사항]\n')
        
        if not raw_direction_match or not filtered_direction_match:
            self.get_logger().error(
                '심각: 회전 방향이 반대입니다!\n'
                '  → 모터 배선 확인\n'
                '  → transbot_driver.py의 모터 제어 로직 확인\n'
                '  → cmd_vel_callback의 left_speed/right_speed 계산 확인'
            )
        
        if abs(raw_error) > 15.0:
            self.get_logger().warn(
                '휠 오도메트리 정확도 개선 필요:\n'
                '  → 휠 직경 캘리브레이션 수행\n'
                '  → 바닥 표면 확인 (슬립 방지)\n'
                '  → base.cpp의 linear_scale 파라미터 조정'
            )
        
        if abs(filtered_error) > 15.0:
            self.get_logger().warn(
                'EKF 융합 정확도 개선 필요:\n'
                '  → ekf_config.yaml의 센서 신뢰도 조정\n'
                '  → process_noise_covariance[5] (yaw) 조정\n'
                '  → IMU 캘리브레이션 확인'
            )
        
        self.get_logger().info('\n' + '='*70 + '\n')
        
        # CSV 파일로 저장 (추가 분석용)
        self.save_data_to_csv()
    
    def save_data_to_csv(self):
        """테스트 데이터를 CSV 파일로 저장"""
        import csv
        from datetime import datetime
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'/home/user/transbot_ws_ros2/rotation_test_{timestamp}.csv'
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # 헤더
                writer.writerow([
                    'time', 'source', 'yaw_rad', 'yaw_deg', 'angular_z_rad_s'
                ])
                
                # 휠 오도메트리 데이터
                for data in self.odom_raw_history:
                    writer.writerow([
                        f"{data['time']:.3f}",
                        'odom_raw',
                        f"{data['yaw']:.6f}",
                        f"{math.degrees(data['yaw']):.2f}",
                        f"{data['angular_z']:.6f}"
                    ])
                
                # 융합 오도메트리 데이터
                for data in self.odom_filtered_history:
                    writer.writerow([
                        f"{data['time']:.3f}",
                        'odom_filtered',
                        f"{data['yaw']:.6f}",
                        f"{math.degrees(data['yaw']):.2f}",
                        f"{data['angular_z']:.6f}"
                    ])
                
                # IMU 데이터
                for data in self.imu_history:
                    writer.writerow([
                        f"{data['time']:.3f}",
                        'imu',
                        '',
                        '',
                        f"{data['angular_z']:.6f}"
                    ])
            
            self.get_logger().info(f'데이터 저장 완료: {filename}')
            self.get_logger().info('Python/Excel로 추가 분석 가능')
            
        except Exception as e:
            self.get_logger().error(f'데이터 저장 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    tester = RotationTester()
    
    # 초기 콜백 처리를 위해 잠시 대기
    for _ in range(10):
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    try:
        # 90도 반시계방향 회전 테스트 (angular_scale=1.44 검증)
        tester.run_rotation_test(target_angle_deg=90.0, angular_speed=0.2)
        
        # 추가 테스트를 원하면 주석 해제
        # time.sleep(3.0)
        # tester.run_rotation_test(target_angle_deg=-90.0, angular_speed=-0.3)  # 시계방향
        
    except KeyboardInterrupt:
        tester.get_logger().info('테스트 중단됨')
    finally:
        # 정지 명령
        twist = Twist()
        twist.angular.z = 0.0
        tester.cmd_vel_pub.publish(twist)
        
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
