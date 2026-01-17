#!/usr/bin/env python3
# encoding: utf-8

"""
오도메트리 정보 분석 스크립트
드라이버 노드 실행 후 오도메트리 관련 토픽들을 모니터링하고 비교 분석
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import threading

class OdometryAnalyzer(Node):
    def __init__(self):
        super().__init__('odometry_analyzer')

        # 데이터 저장용 변수들
        self.vel_data = []
        self.odom_data = []
        self.imu_data = []
        self.cmd_vel_data = []
        self.start_time = time.time()

        # 이전 데이터 저장 (가속도 계산용)
        self.prev_vel_time = None
        self.prev_vel_linear_x = 0.0
        self.prev_odom_time = None
        self.prev_odom_linear_x = 0.0
        self.prev_cmd_vel_time = None
        self.prev_cmd_vel_linear_x = 0.0

        # 구독자 설정
        self.vel_sub = self.create_subscription(
            Twist,
            '/transbot/get_vel',
            self.vel_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 타이머로 분석 출력
        self.timer = self.create_timer(2.0, self.analyze_data)

        self.get_logger().info("오도메트리 분석기 시작됨")

    def vel_callback(self, msg):
        """속도 데이터 콜백"""
        timestamp = time.time() - self.start_time
        self.vel_data.append({
            'time': timestamp,
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z
        })
        # 최근 100개만 유지
        if len(self.vel_data) > 100:
            self.vel_data.pop(0)

    def odom_callback(self, msg):
        """오도메트리 데이터 콜백"""
        timestamp = time.time() - self.start_time
        self.odom_data.append({
            'time': timestamp,
            'pos_x': msg.pose.pose.position.x,
            'pos_y': msg.pose.pose.position.y,
            'vel_linear_x': msg.twist.twist.linear.x,
            'vel_linear_y': msg.twist.twist.linear.y,
            'vel_angular_z': msg.twist.twist.angular.z
        })
        # 최근 100개만 유지
        if len(self.odom_data) > 100:
            self.odom_data.pop(0)

    def imu_callback(self, msg):
        """IMU 데이터 콜백"""
        timestamp = time.time() - self.start_time
        self.imu_data.append({
            'time': timestamp,
            'accel_x': msg.linear_acceleration.x,
            'accel_y': msg.linear_acceleration.y,
            'accel_z': msg.linear_acceleration.z,
            'gyro_x': msg.angular_velocity.x,
            'gyro_y': msg.angular_velocity.y,
            'gyro_z': msg.angular_velocity.z
        })
        # 최근 100개만 유지
        if len(self.imu_data) > 100:
            self.imu_data.pop(0)

    def cmd_vel_callback(self, msg):
        """명령 속도 데이터 콜백"""
        timestamp = time.time() - self.start_time
        self.cmd_vel_data.append({
            'time': timestamp,
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z
        })
        # 최근 100개만 유지
        if len(self.cmd_vel_data) > 100:
            self.cmd_vel_data.pop(0)

    def analyze_data(self):
        """데이터 분석 및 출력 - 로버 정지 상태에서만 분석"""
        if not self.vel_data or not self.odom_data or not self.imu_data or not self.cmd_vel_data:
            self.get_logger().info("데이터 수집 중...")
            return

        # 최근 데이터 분석
        recent_vel = self.vel_data[-1] if self.vel_data else None
        recent_odom = self.odom_data[-1] if self.odom_data else None
        recent_imu = self.imu_data[-1] if self.imu_data else None
        recent_cmd_vel = self.cmd_vel_data[-1] if self.cmd_vel_data else None

        # 로버 정지 상태 확인
        is_robot_stationary = self.is_robot_stationary(recent_cmd_vel, recent_odom, recent_imu)
        
        if not is_robot_stationary:
            # 정지 상태가 아니면 분석 건너뜀
            return

        self.get_logger().info("="*50)
        self.get_logger().info("로버 정지 상태 분석 (시간: {:.2f}s)".format(recent_vel['time']))

        if recent_cmd_vel:
            self.get_logger().info("명령 속도 데이터 (/cmd_vel):")
            self.get_logger().info("  선속도: x={:.3f}, y={:.3f}".format(
                recent_cmd_vel['linear_x'], recent_cmd_vel['linear_y']))
            self.get_logger().info("  각속도: z={:.3f}".format(recent_cmd_vel['angular_z']))

        if recent_vel:
            self.get_logger().info("속도 데이터 (/transbot/get_vel):")
            self.get_logger().info("  선속도: x={:.3f}, y={:.3f}".format(
                recent_vel['linear_x'], recent_vel['linear_y']))
            self.get_logger().info("  각속도: z={:.3f}".format(recent_vel['angular_z']))

        if recent_odom:
            self.get_logger().info("오도메트리 데이터 (/odom_raw):")
            self.get_logger().info("  위치: x={:.3f}, y={:.3f}".format(
                recent_odom['pos_x'], recent_odom['pos_y']))
            self.get_logger().info("  선속도: x={:.3f}, y={:.3f}".format(
                recent_odom['vel_linear_x'], recent_odom['vel_linear_y']))
            self.get_logger().info("  각속도: z={:.3f}".format(recent_odom['vel_angular_z']))

        if recent_imu:
            self.get_logger().info("IMU 데이터 (/imu/data):")
            self.get_logger().info("  가속도: x={:.3f}, y={:.3f}, z={:.3f}".format(
                recent_imu['accel_x'], recent_imu['accel_y'], recent_imu['accel_z']))
            self.get_logger().info("  각속도: x={:.3f}, y={:.3f}, z={:.3f}".format(
                recent_imu['gyro_x'], recent_imu['gyro_y'], recent_imu['gyro_z']))

        # 데이터 일관성 검증
        self.check_data_consistency(recent_vel, recent_odom, recent_imu, recent_cmd_vel)

        # 통계 정보
        self.print_statistics()

    def is_robot_stationary(self, cmd_vel, odom, imu):
        """로버가 정지 상태인지 확인"""
        if not cmd_vel or not odom or not imu:
            return False
        
        # 정지 상태 판단 기준
        cmd_vel_threshold = 0.01  # 명령 속도 임계값 (m/s)
        odom_vel_threshold = 0.01  # 오도메트리 속도 임계값 (m/s)
        accel_threshold = 0.5     # 가속도 임계값 (m/s²) - 정지 시 노이즈 고려 (0.1 → 0.5로 증가)
        
        # 명령 속도가 0에 가까운지 확인
        cmd_vel_magnitude = (cmd_vel['linear_x']**2 + cmd_vel['linear_y']**2 + cmd_vel['angular_z']**2)**0.5
        if cmd_vel_magnitude > cmd_vel_threshold:
            return False
        
        # 오도메트리 속도가 0에 가까운지 확인
        odom_vel_magnitude = (odom['vel_linear_x']**2 + odom['vel_linear_y']**2 + odom['vel_angular_z']**2)**0.5
        if odom_vel_magnitude > odom_vel_threshold:
            return False
        
        # IMU 가속도가 정지 상태의 노이즈 범위 내인지 확인 (중력 제외)
        # z축이 아래를 향할 때 중력 제거된 가속도의 크기 계산
        if abs(imu['accel_z'] - 9.81) < 1.0:  # 중력이 제거됨 (z ≈ 0)
            accel_xy_magnitude = (imu['accel_x']**2 + imu['accel_y']**2)**0.5
        elif abs(imu['accel_z']) < 1.0:  # 중력이 제거됨 (z ≈ 0)
            accel_xy_magnitude = (imu['accel_x']**2 + imu['accel_y']**2)**0.5
        else:  # 중력이 포함됨
            accel_xy_magnitude = (imu['accel_x']**2 + imu['accel_y']**2)**0.5
        
        if accel_xy_magnitude > accel_threshold:
            return False
        
        return True

    def check_data_consistency(self, vel, odom, imu, cmd_vel):
        """데이터 일관성 검증"""
        self.get_logger().info("데이터 일관성 검증:")

        # 가속도 계산 및 비교
        self.calculate_and_compare_accelerations(cmd_vel, odom)

        if vel and odom:
            vel_diff_linear = abs(vel['linear_x'] - odom['vel_linear_x'])
            vel_diff_angular = abs(vel['angular_z'] - odom['vel_angular_z'])

            if vel_diff_linear > 0.01:
                self.get_logger().warn("선속도 불일치: vel={:.3f}, odom={:.3f}".format(
                    vel['linear_x'], odom['vel_linear_x']))
            else:
                self.get_logger().info("선속도 일치 ✓")

            if vel_diff_angular > 0.01:
                self.get_logger().warn("각속도 불일치: vel={:.3f}, odom={:.3f}".format(
                    vel['angular_z'], odom['vel_angular_z']))
            else:
                self.get_logger().info("각속도 일치 ✓")

        # 가속도 분석
        if imu:
            accel_magnitude = (imu['accel_x']**2 + imu['accel_y']**2 + imu['accel_z']**2)**0.5
            self.get_logger().info("가속도 크기: {:.3f} m/s² (중력 포함 시 ≈9.81)".format(accel_magnitude))

            # 중력 방향 검증 (z축이 아래를 향할 때)
            if abs(imu['accel_z'] + 9.81) < 1.0:  # 중력 제거됨
                self.get_logger().info("중력 벡터 제거됨 ✓ (z ≈ 0)")
            elif abs(imu['accel_z']) < 1.0:  # 중력 제거됨
                self.get_logger().info("중력 벡터 제거됨 ✓ (z ≈ 0)")
            else:
                self.get_logger().info("중력 벡터 포함됨 (z = {:.3f})".format(imu['accel_z']))

    def calculate_and_compare_accelerations(self, cmd_vel, odom):
        """명령 속도와 오도메트리 가속도 계산 및 비교"""
        if not cmd_vel or not odom:
            return

        current_time = time.time() - self.start_time

        # cmd_vel 가속도 계산
        if self.prev_cmd_vel_time is not None:
            dt_cmd = current_time - self.prev_cmd_vel_time
            if dt_cmd > 0.001:  # 너무 작은 시간 간격 제외
                cmd_accel = (cmd_vel['linear_x'] - self.prev_cmd_vel_linear_x) / dt_cmd
                self.get_logger().info("명령 가속도: {:.3f} m/s²".format(cmd_accel))
            else:
                cmd_accel = 0.0
        else:
            cmd_accel = 0.0

        # odom 가속도 계산
        if self.prev_odom_time is not None:
            dt_odom = current_time - self.prev_odom_time
            if dt_odom > 0.001:  # 너무 작은 시간 간격 제외
                odom_accel = (odom['vel_linear_x'] - self.prev_odom_linear_x) / dt_odom
                self.get_logger().info("오도메트리 가속도: {:.3f} m/s²".format(odom_accel))
            else:
                odom_accel = 0.0
        else:
            odom_accel = 0.0

        # 가속도 비교
        accel_diff = abs(cmd_accel - odom_accel)
        if accel_diff > 0.1:  # 0.1 m/s² 임계값
            self.get_logger().warn("가속도 불일치: cmd={:.3f}, odom={:.3f} (차이: {:.3f})".format(
                cmd_accel, odom_accel, accel_diff))
        else:
            self.get_logger().info("가속도 일치 ✓ (차이: {:.3f})".format(accel_diff))

        # 이전 값 업데이트
        self.prev_cmd_vel_time = current_time
        self.prev_cmd_vel_linear_x = cmd_vel['linear_x']
        self.prev_odom_time = current_time
        self.prev_odom_linear_x = odom['vel_linear_x']

    def print_statistics(self):
        """통계 정보 출력"""
        if len(self.vel_data) > 10:
            # 속도 데이터 통계
            vel_x_values = [d['linear_x'] for d in self.vel_data[-20:]]
            vel_x_avg = sum(vel_x_values) / len(vel_x_values)
            vel_x_max = max(vel_x_values)
            vel_x_min = min(vel_x_values)

            self.get_logger().info("통계 (최근 20개 샘플):")
            self.get_logger().info("  선속도 x: 평균={:.3f}, 최대={:.3f}, 최소={:.3f}".format(
                vel_x_avg, vel_x_max, vel_x_min))

def main(args=None):
    rclpy.init(args=args)

    analyzer = OdometryAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()