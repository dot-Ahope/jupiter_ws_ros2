#!/usr/bin/env python3
"""
Phase 1 진단: Odom vs IMU 일치점 찾기 + 물리적 회전 검증

목표:
  1. Odom과 IMU가 일치하는 angular_scale 찾기
  2. 실제 90° 회전 시 센서들이 어떻게 측정하는지 확인
  3. EKF가 왜 IMU를 outlier로 판단하는지 분석

전략:
  - 실제 로봇을 정확히 90° 회전 (바닥에 테이프로 표시)
  - 여러 angular_scale 값을 시도하며 측정
  - Odom, IMU, EKF 간 차이 분석
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time
import numpy as np


class Phase1Diagnosis(Node):
    def __init__(self):
        super().__init__('phase1_diagnosis')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/transbot/imu', self.imu_callback, 10)  # ⚠️ 임시 (시스템 재시작 후 /imu/data_raw로)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.odom_yaw = 0.0
        self.odom_received = False
        
        self.ekf_odom_yaw = 0.0
        self.ekf_odom_received = False
        
        self.imu_angular_vel_z = 0.0
        self.imu_received = False
        self.last_imu_time = None
        self.integrated_imu_yaw = 0.0
        
        # Diagnosis mode
        self.test_scales = [1.0, 1.2, 1.4, 1.56, 1.8, 2.0, 2.5, 3.0]
        self.results = []
        
    def quaternion_to_yaw(self, q):
        """Quaternion을 yaw 각도로 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        """오도메트리 콜백"""
        q = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(q)
        self.odom_received = True
    
    def ekf_odom_callback(self, msg):
        """EKF 필터링된 오도메트리 콜백"""
        q = msg.pose.pose.orientation
        self.ekf_odom_yaw = self.quaternion_to_yaw(q)
        self.ekf_odom_received = True
        
    def imu_callback(self, msg):
        """IMU 콜백"""
        current_time = self.get_clock().now()
        
        self.imu_angular_vel_z = msg.angular_velocity.z
        
        # 각속도 적분
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            if dt < 1.0:
                self.integrated_imu_yaw += self.imu_angular_vel_z * dt
        
        self.last_imu_time = current_time
        self.imu_received = True
    
    def normalize_angle(self, angle):
        """각도 정규화 [-π, π]"""
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
    
    def wait_for_sensors(self, timeout=5.0):
        """센서 데이터 대기"""
        self.get_logger().info('센서 데이터 대기 중...')
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_received and self.ekf_odom_received and self.imu_received:
                self.get_logger().info('✅ 센서 준비 완료')
                return True
        
        return False
    
    def manual_rotation_test(self):
        """
        수동 90° 회전 테스트
        
        사용자가 직접 로봇을 바닥 표시에 맞춰 정확히 90° 회전
        센서들이 어떻게 측정하는지 확인
        
        ⚠️ 모터는 전혀 사용하지 않음! 완전히 수동!
        """
        self.get_logger().info(
            '\n' + '='*80 + '\n'
            '📋 Test 1: 수동 90° 회전 (손으로만!)\n'
            '='*80 + '\n'
            '\n🎯 목적: 실제 90° 회전 시 센서들이 어떻게 측정하는가?\n'
            '\n📝 준비:\n'
            '  1. 바닥에 테이프로 시작 방향 (0°) 표시\n'
            '  2. 왼쪽 90° 방향에도 테이프 표시\n'
            '  3. 로봇을 시작 방향에 정렬\n'
            '\n⚠️  주의: 모터는 사용하지 않습니다. 손으로만 회전시킵니다!\n'
            '\n'
        )
        
        input('✋ 준비 완료 후 Enter를 누르세요 (센서 초기값 기록)...')
        
        # 초기값 기록
        self.get_logger().info('\n📸 센서 초기값 기록 중...')
        time.sleep(0.3)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        start_odom_yaw = self.odom_yaw
        start_ekf_odom_yaw = self.ekf_odom_yaw
        self.integrated_imu_yaw = 0.0
        
        self.get_logger().info(
            '\n✅ 초기값 기록 완료!\n'
            '\n👉 지금부터 로봇을 천천히 손으로 90° 회전시키세요\n'
            '   - 반시계 방향 (왼쪽)\n'
            '   - 90° 표시 테이프에 정확히 맞추기\n'
            '   - 천천히 회전 (5초 정도 소요)\n'
            '\n⏸️  회전 완료 후 Enter를 누르세요...\n'
            '\n💡 TIP: Enter를 누르기 전까지 계속 센서를 읽습니다.\n'
            '        천천히 회전시키고, 완전히 멈춘 후 Enter를 누르세요.\n'
        )
        
        # 백그라운드로 센서 읽기 (사용자가 Enter 누를 때까지)
        import select
        import sys
        
        self.get_logger().info('\n⏳ 센서 적분 중... (Enter를 누르면 종료)')
        last_log_time = time.time()
        
        while True:
            # 센서 데이터 읽기 (IMU 적분 계속됨)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            # 0.5초마다 진행 상황 표시
            if (time.time() - last_log_time) >= 0.5:
                imu_deg = math.degrees(abs(self.integrated_imu_yaw))
                odom_delta = abs(self.normalize_angle(self.odom_yaw - start_odom_yaw))
                odom_deg = math.degrees(odom_delta)
                
                self.get_logger().info(
                    f'   IMU: {imu_deg:6.1f}° | Odom: {odom_deg:6.1f}°'
                )
                last_log_time = time.time()
            
            # Enter 입력 확인 (non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = input()
                break
            
            time.sleep(0.01)
        
        # 최종값 측정
        self.get_logger().info('\n📸 센서 최종값 기록 중...')
        time.sleep(0.3)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        final_odom_yaw = self.odom_yaw
        final_ekf_odom_yaw = self.ekf_odom_yaw
        final_imu_integrated = self.integrated_imu_yaw
        
        # 회전량 계산
        odom_rotation = abs(self.normalize_angle(final_odom_yaw - start_odom_yaw))
        ekf_rotation = abs(self.normalize_angle(final_ekf_odom_yaw - start_ekf_odom_yaw))
        imu_rotation = abs(final_imu_integrated)
        
        # 도 단위
        odom_deg = math.degrees(odom_rotation)
        ekf_deg = math.degrees(ekf_rotation)
        imu_deg = math.degrees(imu_rotation)
        
        # angular_scale 계산
        target_deg = 90.0
        
        if odom_deg > 1.0:
            scale_needed_odom = imu_deg / odom_deg
            scale_needed_for_90_odom = target_deg / odom_deg
        else:
            scale_needed_odom = 0.0
            scale_needed_for_90_odom = 0.0
        
        if ekf_deg > 1.0:
            scale_needed_ekf = imu_deg / ekf_deg
            scale_needed_for_90_ekf = target_deg / ekf_deg
        else:
            scale_needed_ekf = 0.0
            scale_needed_for_90_ekf = 0.0
        
        # 결과 출력
        self.get_logger().info(
            '\n' + '='*80 + '\n'
            '📊 수동 회전 측정 결과 (실제 90° 회전)\n'
            '='*80 + '\n'
            f'\n'
            f'🎯 실제 물리적 회전:     90.00° (바닥 표시 기준)\n'
            f'\n'
            f'📍 센서 측정값:\n'
            f'  Odom (raw):           {odom_deg:7.2f}° (base_node 출력, angular_scale 적용 전)\n'
            f'  EKF Odom:             {ekf_deg:7.2f}° (Odom + IMU 융합)\n'
            f'  IMU (적분):           {imu_deg:7.2f}° (자이로 적분)\n'
            f'\n'
            f'📐 필요한 보정:\n'
            f'  Odom → 90°:           angular_scale = {scale_needed_for_90_odom:.4f}\n'
            f'  Odom → IMU 일치:      angular_scale = {scale_needed_odom:.4f}\n'
            f'  EKF → 90°:            angular_scale = {scale_needed_for_90_ekf:.4f}\n'
            f'  EKF → IMU 일치:       angular_scale = {scale_needed_ekf:.4f}\n'
            f'\n'
            f'🔍 센서 간 차이:\n'
            f'  IMU / Odom 비율:      {imu_deg/odom_deg if odom_deg > 0 else 0:.2f}x\n'
            f'  IMU / EKF 비율:       {imu_deg/ekf_deg if ekf_deg > 0 else 0:.2f}x\n'
            f'  Odom / 실제 비율:     {odom_deg/90.0:.2f}x (언더리포팅)\n'
            f'  IMU / 실제 비율:      {imu_deg/90.0:.2f}x\n'
            f'\n'
            f'💡 분석:\n'
        )
        
        # 진단
        if abs(odom_deg - 90.0) < 5.0:
            self.get_logger().info('  ✅ Odom이 정확합니다! angular_scale이 잘 설정됨')
        elif odom_deg < 90.0:
            self.get_logger().info(f'  ⚠️  Odom이 {90.0 - odom_deg:.1f}° 부족 (언더리포팅)')
            self.get_logger().info(f'     angular_scale을 {scale_needed_for_90_odom:.4f}로 증가 필요')
        else:
            self.get_logger().info(f'  ⚠️  Odom이 {odom_deg - 90.0:.1f}° 초과 (오버리포팅)')
        
        if abs(imu_deg - 90.0) < 5.0:
            self.get_logger().info('  ✅ IMU가 정확합니다!')
        elif imu_deg < 90.0:
            self.get_logger().info(f'  ⚠️  IMU가 {90.0 - imu_deg:.1f}° 부족')
        else:
            self.get_logger().info(f'  ⚠️  IMU가 {imu_deg - 90.0:.1f}° 초과')
        
        if abs(ekf_deg - 0.0) < 2.0:
            self.get_logger().info('  ❌ EKF가 IMU를 무시하고 있습니다! (Outlier 처리)')
            self.get_logger().info('     → Odom과 IMU 차이가 너무 커서 EKF가 IMU를 거부')
            self.get_logger().info(f'     → 해결: angular_scale을 {scale_needed_odom:.4f}로 조정')
        elif abs(ekf_deg - 90.0) < 10.0:
            self.get_logger().info('  ✅ EKF가 두 센서를 잘 융합하고 있습니다')
        else:
            self.get_logger().info('  ⚠️  EKF 융합에 문제가 있습니다')
        
        self.get_logger().info('='*80 + '\n')
        
        return {
            'odom_deg': odom_deg,
            'ekf_deg': ekf_deg,
            'imu_deg': imu_deg,
            'scale_for_90_odom': scale_needed_for_90_odom,
            'scale_for_imu_match': scale_needed_odom,
            'odom_underreporting': 90.0 / odom_deg if odom_deg > 0 else 0,
            'imu_accuracy': imu_deg / 90.0 if imu_deg > 0 else 0
        }
    
    def motor_driven_test(self, target_deg=90.0, speed=0.3):
        """
        모터 구동 90° 회전 테스트
        
        현재 angular_scale로 90° 명령 시 실제 얼마나 회전하는지 확인
        
        🤖 완전 자동! 사용자는 관찰만!
        """
        self.get_logger().info(
            '\n' + '='*80 + '\n'
            f'📋 Test 2: 모터 구동 {target_deg}° 회전 (자동!)\n'
            '='*80 + '\n'
            '\n🎯 목적: 현재 angular_scale로 90° 명령 시 실제 회전각은?\n'
            '\n📝 방법:\n'
            '  1. 로봇이 자동으로 Odom 90° 도달까지 회전\n'
            '  2. 당신은 바닥 표시와 비교하여 실제 회전각 관찰\n'
            '  3. 실제 90°인지 확인\n'
            '\n⚠️  주의:\n'
            '  - 로봇 주변 1m 이상 공간 확보!\n'
            '  - 자동으로 회전합니다. 손대지 마세요!\n'
            '  - 비상 시 Ctrl+C로 중단\n'
            '\n'
        )
        
        input('✋ 준비 완료 후 Enter를 누르세요 (자동 회전 시작)...')
        
        # 초기화
        self.get_logger().info('\n📸 센서 초기값 기록 중...')
        time.sleep(0.3)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        start_odom_yaw = self.odom_yaw
        start_ekf_odom_yaw = self.ekf_odom_yaw
        self.integrated_imu_yaw = 0.0
        start_time = time.time()
        
        # Odom 목표 (angular_scale이 이미 적용된 상태에서 도달할 값)
        target_rad = math.radians(target_deg)
        
        self.get_logger().info(
            f'\n✅ 초기값 기록 완료!\n'
            f'\n🤖 로봇이 자동으로 회전합니다! (목표: Odom {target_deg}°)\n'
            f'   👀 바닥 표시와 비교하여 실제 회전각을 관찰하세요...\n'
        )
        
        time.sleep(1.0)  # 사용자가 준비할 시간
        
        # 회전
        twist = Twist()
        twist.angular.z = speed
        
        last_odom_yaw = start_odom_yaw
        accumulated_odom = 0.0
        
        last_ekf_yaw = start_ekf_odom_yaw
        accumulated_ekf = 0.0
        
        last_log_time = time.time()
        
        while True:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            
            # Odom 누적
            current_odom_yaw = self.odom_yaw
            odom_step = self.normalize_angle(current_odom_yaw - last_odom_yaw)
            accumulated_odom += abs(odom_step)
            last_odom_yaw = current_odom_yaw
            
            # EKF 누적
            current_ekf_yaw = self.ekf_odom_yaw
            ekf_step = self.normalize_angle(current_ekf_yaw - last_ekf_yaw)
            accumulated_ekf += abs(ekf_step)
            last_ekf_yaw = current_ekf_yaw
            
            # IMU
            imu_deg = math.degrees(abs(self.integrated_imu_yaw))
            
            # 진행률
            if (time.time() - last_log_time) >= 0.5:
                odom_deg = math.degrees(accumulated_odom)
                ekf_deg = math.degrees(accumulated_ekf)
                progress = (odom_deg / target_deg) * 100
                
                self.get_logger().info(
                    f'진행 {progress:5.1f}% | '
                    f'Odom: {odom_deg:6.1f}° | '
                    f'EKF: {ekf_deg:6.1f}° | '
                    f'IMU: {imu_deg:6.1f}°'
                )
                last_log_time = time.time()
            
            # 종료 조건: Odom이 목표 도달
            if accumulated_odom >= target_rad * 0.98:
                break
            
            # 타임아웃
            if (time.time() - start_time) > 30.0:
                self.get_logger().warn('⚠️  타임아웃!')
                break
        
        # 정지
        self.get_logger().info('\n🛑 목표 도달! 로봇 정지 중...')
        self.stop_robot()
        
        self.get_logger().info(
            '\n👀 관찰 시간!\n'
            '   - 바닥 표시를 보고 실제 회전각을 확인하세요\n'
            '   - 정확히 90°인가요?\n'
            '   - 90°보다 적은가요? 많은가요?\n'
        )
        
        time.sleep(1.0)
        
        # 최종 측정
        self.get_logger().info('\n📸 센서 최종값 기록 중...')
        time.sleep(0.3)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        odom_deg = math.degrees(accumulated_odom)
        ekf_deg = math.degrees(accumulated_ekf)
        imu_deg = math.degrees(abs(self.integrated_imu_yaw))
        
        self.get_logger().info(
            '\n' + '='*80 + '\n'
            f'📊 Test 2 결과: 모터 구동 측정 (Odom {target_deg}° 명령)\n'
            '='*80 + '\n'
            f'\n'
            f'🎯 명령:                  {target_deg:.1f}° (Odom 기준)\n'
            f'\n'
            f'📍 센서 측정값:\n'
            f'  Odom (raw):             {odom_deg:7.2f}° ⭐\n'
            f'  EKF Odom:               {ekf_deg:7.2f}°\n'
            f'  IMU (적분):             {imu_deg:7.2f}°\n'
            f'\n'
            f'🔍 IMU/Odom 비율:         {imu_deg/odom_deg if odom_deg > 0 else 0:.2f}x\n'
            f'\n'
            f'💡 바닥 표시와 비교한 실제 회전:\n'
            f'   ✅ 정확히 90° 회전: angular_scale 잘 설정됨\n'
            f'   ⚠️  90°보다 적음: angular_scale 증가 필요\n'
            f'   ⚠️  90°보다 많음: angular_scale 감소 필요\n'
            f'\n'
            f'⚠️  EKF 진단:\n'
            f'   - EKF ≈ 0°: Odom과 IMU 차이가 커서 IMU를 무시 중\n'
            f'   - EKF ≈ 90°: 두 센서를 잘 융합하고 있음\n'
            f'='*80 + '\n'
        )
        
        return {
            'odom_deg': odom_deg,
            'ekf_deg': ekf_deg,
            'imu_deg': imu_deg,
            'imu_odom_ratio': imu_deg / odom_deg if odom_deg > 0 else 0
        }
    
    def run_diagnosis(self):
        """전체 진단 실행"""
        if not self.wait_for_sensors():
            self.get_logger().error('❌ 센서 데이터 수신 실패')
            return None
        
        self.get_logger().info(
            '\n' + '='*80 + '\n'
            'Phase 1 진단: Odom vs IMU 일치점 찾기\n'
            '='*80 + '\n'
            '\n🎯 목표:\n'
            '  1. 실제 90° 회전 시 각 센서가 어떻게 측정하는가?\n'
            '  2. Odom과 IMU가 일치하려면 angular_scale이 얼마여야 하는가?\n'
            '  3. EKF가 IMU를 outlier로 판단하는 이유는?\n'
            '\n📝 테스트 2개:\n'
            '  Test 1: 수동 회전 (손으로만!) - 물리적 정확도 확인\n'
            '  Test 2: 모터 구동 (자동!) - 현재 설정 확인\n'
            '\n'
        )
        
        input('✋ 시작하려면 Enter를 누르세요...')
        
        # Test 1: 수동 회전
        manual_result = self.manual_rotation_test()
        
        self.get_logger().info(
            '\n' + '='*80 + '\n'
            '✅ Test 1 완료!\n'
            '\n⏸️  잠시 휴식... (로봇을 다시 0° 위치로 이동시키세요)\n'
            '='*80
        )
        time.sleep(2.0)
        input('\n✋ Test 2를 시작하려면 Enter를 누르세요...')
        
        # Test 2: 모터 구동
        motor_result = self.motor_driven_test()
        
        # 종합 분석
        self.get_logger().info(
            '\n\n' + '='*80 + '\n'
            '🎯 종합 분석 및 권장사항\n'
            '='*80 + '\n'
        )
        
        # 1. IMU 정확도
        if manual_result:
            imu_accuracy = manual_result['imu_accuracy']
            self.get_logger().info(f'\n1️⃣  IMU 정확도: {imu_accuracy*100:.1f}%')
            
            if 0.95 <= imu_accuracy <= 1.05:
                self.get_logger().info('   ✅ IMU가 정확합니다!')
            elif imu_accuracy > 1.05:
                self.get_logger().info(f'   ⚠️  IMU가 {(imu_accuracy-1)*100:.1f}% 과대 측정')
                self.get_logger().info('      → IMU 캘리브레이션 필요')
            else:
                self.get_logger().info(f'   ⚠️  IMU가 {(1-imu_accuracy)*100:.1f}% 과소 측정')
        
        # 2. Odom 정확도
        if manual_result:
            odom_underreporting = manual_result['odom_underreporting']
            self.get_logger().info(f'\n2️⃣  Odom 언더리포팅: {odom_underreporting:.2f}x')
            self.get_logger().info(f'   → angular_scale = {odom_underreporting:.4f} 필요')
            
            if manual_result['scale_for_90_odom'] > 1.1:
                self.get_logger().info(
                    f'\n   ⚠️  Odom이 실제보다 {((odom_underreporting-1)*100):.1f}% 적게 측정'
                )
                self.get_logger().info('      원인: 휠 슬립, 펌웨어 under-counting 등')
        
        # 3. EKF 상태
        if motor_result:
            ekf_deg = motor_result['ekf_deg']
            self.get_logger().info(f'\n3️⃣  EKF 상태:')
            
            if ekf_deg < 5.0:
                self.get_logger().info(f'   ❌ EKF가 IMU를 무시하고 있습니다! ({ekf_deg:.1f}°)')
                self.get_logger().info('      원인: Odom과 IMU 차이가 너무 큼')
                self.get_logger().info(f'      해결: angular_scale을 {manual_result["scale_for_imu_match"]:.4f}로 조정')
            else:
                self.get_logger().info(f'   ✅ EKF가 센서를 융합하고 있습니다 ({ekf_deg:.1f}°)')
        
        # 4. 최종 권장
        if manual_result:
            self.get_logger().info(
                '\n4️⃣  최종 권장 angular_scale:\n'
                f'\n'
                f'   방법 A (IMU 기준): {manual_result["scale_for_imu_match"]:.4f}\n'
                f'     - Odom과 IMU를 일치시킴\n'
                f'     - EKF가 두 센서를 모두 사용\n'
                f'     - IMU가 정확하다고 가정\n'
                f'\n'
                f'   방법 B (물리적 기준): {manual_result["scale_for_90_odom"]:.4f}\n'
                f'     - Odom이 정확히 90° 측정하도록\n'
                f'     - 물리적 회전이 정확하다고 가정\n'
                f'     - IMU는 보조 역할\n'
                f'\n'
                f'   ⭐ 권장: 두 값의 평균 = {(manual_result["scale_for_imu_match"] + manual_result["scale_for_90_odom"])/2:.4f}\n'
            )
        
        self.get_logger().info('='*80 + '\n')
        
        return {
            'manual_result': manual_result,
            'motor_result': motor_result
        }


def main():
    rclpy.init()
    
    node = Phase1Diagnosis()
    
    try:
        print('\n' + '='*80)
        print('Phase 1 진단: Odom vs IMU 일치점 찾기')
        print('='*80)
        print('\n목표:')
        print('  1. 실제 90° 회전 시 센서 측정값 확인')
        print('  2. Odom과 IMU 일치를 위한 angular_scale 계산')
        print('  3. EKF가 IMU를 outlier로 처리하는 이유 분석')
        print('\n준비:')
        print('  1. 바닥에 테이프로 0°, 90° 방향 표시')
        print('  2. 로봇 배터리 50% 이상')
        print('  3. 시스템 실행 확인:')
        print('     ros2 launch sllidar_ros2 transbot_full_system.launch.py')
        print('\n시작하려면 Enter를 누르세요...')
        input()
        
        results = node.run_diagnosis()
        
        if results:
            print('\n✅ 진단 완료!')
            print('\n다음 단계:')
            print('  1. 권장 angular_scale 값을 launch 파일에 적용')
            print('  2. Phase 2 캘리브레이션 실행')
        else:
            print('\n❌ 진단 실패')
    
    except KeyboardInterrupt:
        print('\n\n⚠️  중단됨')
        node.stop_robot()
    
    except Exception as e:
        print(f'\n❌ 오류: {e}')
        import traceback
        traceback.print_exc()
        node.stop_robot()
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
