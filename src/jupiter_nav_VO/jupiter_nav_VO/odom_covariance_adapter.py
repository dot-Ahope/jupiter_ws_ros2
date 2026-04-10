#!/usr/bin/env python3
# encoding: utf-8
"""
Odometry Covariance Adapter Node (IMU 기반 휠 슬립 감지 포함)
==============================================================
base_node의 /odom_raw는 공분산이 0으로 발행됨.
robot_localization은 공분산이 0이면 해당 측정값을 무시하므로,
속도 기반 동적 공분산을 적용하여 /odom_adapted로 재발행한다.

동적 공분산 전략:
  1. 정지 상태 (|vx| < threshold, |vyaw| < threshold):
     - 낮은 공분산 → 높은 신뢰도
     
  2. 저속/고속 이동:
     - 속도에 비례하여 공분산 증가
     
  3. 회전 중:
     - 선형 속도의 공분산 증가 (회전 중 슬립 가능성)
     
  4. 가감속 감지:
     - 급격한 가속/감속 시 공분산 일시적 증가

  5. IMU 기반 휠 슬립 감지 (Method A):
     - IMU 가속도 vs 엔코더 유도 가속도 비교
     - 불일치가 클수록 공분산 동적 증폭 (비례적)
     - 적응형 바이어스 추정으로 IMU 오프셋 자동 보정
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math


class OdomCovarianceAdapter(Node):
    def __init__(self):
        super().__init__('odom_covariance_adapter')
        
        # ============================================================
        # 파라미터 선언
        # ============================================================
        # 기본 공분산 값 (정지 시)
        self.declare_parameter('base_linear_cov', 0.005)
        self.declare_parameter('base_angular_cov', 0.01)
        self.declare_parameter('base_pose_cov', 0.01)
        self.declare_parameter('base_orient_cov', 0.02)
        
        # 속도 임계값
        self.declare_parameter('stationary_linear_threshold', 0.01)
        self.declare_parameter('stationary_angular_threshold', 0.02)
        self.declare_parameter('high_speed_threshold', 0.3)
        
        # 동적 스케일 팩터
        self.declare_parameter('speed_cov_scale', 0.1)
        self.declare_parameter('rotation_cov_scale', 0.3)
        self.declare_parameter('accel_cov_scale', 0.5)
        
        # 정지 시 감쇠
        self.declare_parameter('stationary_cov_factor', 0.1)
        
        # 토픽 이름
        self.declare_parameter('input_odom_topic', '/odom_raw')
        self.declare_parameter('output_odom_topic', '/odom_adapted')
        self.declare_parameter('imu_topic', '/imu/data_calibrated')
        
        # 슬립 감지 파라미터
        self.declare_parameter('slip_mismatch_threshold', 0.5)   # m/s² — 이 이상 불일치 시 슬립
        self.declare_parameter('slip_max_multiplier', 50.0)       # 슬립 시 최대 공분산 배율
        self.declare_parameter('slip_scale', 10.0)                # 불일치 비율 → 배율 변환 계수
        self.declare_parameter('slip_decay', 0.92)                # 슬립 해제 시 배율 감쇠 (매 사이클)
        self.declare_parameter('slip_detect_count', 3)            # N회 연속 불일치 시 슬립 확정
        self.declare_parameter('slip_bias_alpha', 0.005)          # 바이어스 추정 EMA 계수
        
        # 파라미터 로드
        self.base_linear_cov = self.get_parameter('base_linear_cov').value
        self.base_angular_cov = self.get_parameter('base_angular_cov').value
        self.base_pose_cov = self.get_parameter('base_pose_cov').value
        self.base_orient_cov = self.get_parameter('base_orient_cov').value
        
        self.stationary_lin_thresh = self.get_parameter('stationary_linear_threshold').value
        self.stationary_ang_thresh = self.get_parameter('stationary_angular_threshold').value
        self.high_speed_thresh = self.get_parameter('high_speed_threshold').value
        
        self.speed_cov_scale = self.get_parameter('speed_cov_scale').value
        self.rotation_cov_scale = self.get_parameter('rotation_cov_scale').value
        self.accel_cov_scale = self.get_parameter('accel_cov_scale').value
        self.stationary_cov_factor = self.get_parameter('stationary_cov_factor').value
        
        input_topic = self.get_parameter('input_odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        
        self.slip_mismatch_thresh = self.get_parameter('slip_mismatch_threshold').value
        self.slip_max_mult = self.get_parameter('slip_max_multiplier').value
        self.slip_scale = self.get_parameter('slip_scale').value
        self.slip_decay = self.get_parameter('slip_decay').value
        self.slip_detect_count = self.get_parameter('slip_detect_count').value
        self.slip_bias_alpha = self.get_parameter('slip_bias_alpha').value
        
        # ============================================================
        # 상태 변수
        # ============================================================
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vyaw = 0.0
        self.prev_time = None
        self.accel_magnitude = 0.0
        
        # EMA 가속도
        self.ema_accel = 0.0
        self.ema_alpha = 0.3
        
        # IMU 슬립 감지 상태
        self.latest_imu_ax = 0.0          # IMU x축 선형 가속도
        self.imu_received = False          # IMU 데이터 수신 여부
        self.imu_encoder_bias = 0.0       # 적응형 바이어스 (encoder_ax - imu_ax 평균)
        self.slip_cov_multiplier = 1.0    # 현재 슬립 공분산 배율
        self.slip_counter = 0             # 연속 슬립 감지 카운터
        self.prev_slip_state = False      # 이전 슬립 상태 (로그용)
        
        # ============================================================
        # 퍼블리셔/서브스크라이버
        # ============================================================
        self.odom_sub = self.create_subscription(
            Odometry, input_topic, self.odom_callback, 50)
        
        self.odom_pub = self.create_publisher(
            Odometry, output_topic, 50)
        
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 50)
        
        self.get_logger().info(
            f'OdomCovarianceAdapter: {input_topic} → {output_topic} '
            f'(IMU slip detection: {imu_topic}, '
            f'thresh={self.slip_mismatch_thresh} m/s², '
            f'max_mult={self.slip_max_mult}x)')
    
    def imu_callback(self, msg: Imu):
        """IMU 데이터 수신 → x축 선형 가속도 저장"""
        self.latest_imu_ax = msg.linear_acceleration.x
        self.imu_received = True
    
    def odom_callback(self, msg: Odometry):
        """odom_raw 수신 → 동적 공분산 + 슬립 감지 적용 → odom_adapted 발행"""
        
        # 현재 속도 추출
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vyaw = msg.twist.twist.angular.z
        
        linear_speed = math.sqrt(vx * vx + vy * vy)
        angular_speed = abs(vyaw)
        
        # ============================================================
        # 가속도 추정 (엔코더 속도 변화량)
        # ============================================================
        current_time = self.get_clock().now()
        encoder_ax = 0.0
        dt = 0.0
        
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9
            if dt > 0.001:  # 최소 1ms
                ax = (vx - self.prev_vx) / dt
                ay = (vy - self.prev_vy) / dt
                encoder_ax = ax   # x축 가속도 (슬립 비교용)
                self.accel_magnitude = math.sqrt(ax * ax + ay * ay)
                self.ema_accel = self.ema_alpha * self.accel_magnitude + \
                                 (1.0 - self.ema_alpha) * self.ema_accel
        
        # ============================================================
        # IMU 기반 휠 슬립 감지
        # ============================================================
        if self.imu_received and self.prev_time is not None and dt > 0.001:
            imu_ax = self.latest_imu_ax
            
            # 엔코더 가속도와 IMU 가속도의 차이 (바이어스 보정)
            raw_diff = encoder_ax - imu_ax
            corrected_mismatch = abs(raw_diff - self.imu_encoder_bias)
            
            # 바이어스 적응 (슬립이 아닌 상태에서만 업데이트)
            if corrected_mismatch < self.slip_mismatch_thresh:
                self.imu_encoder_bias = (
                    (1.0 - self.slip_bias_alpha) * self.imu_encoder_bias +
                    self.slip_bias_alpha * raw_diff
                )
            
            # 슬립 판정
            if corrected_mismatch > self.slip_mismatch_thresh:
                self.slip_counter += 1
                if self.slip_counter >= self.slip_detect_count:
                    # 동적 슬립 배율: 불일치 크기에 비례
                    target_mult = 1.0 + (corrected_mismatch / self.slip_mismatch_thresh) * self.slip_scale
                    target_mult = min(target_mult, self.slip_max_mult)
                    # 즉시 적용 (슬립은 빠르게 반영)
                    self.slip_cov_multiplier = max(self.slip_cov_multiplier, target_mult)
                    
                    if not self.prev_slip_state:
                        self.get_logger().warn(
                            f'SLIP DETECTED: mismatch={corrected_mismatch:.2f} m/s² '
                            f'(enc_ax={encoder_ax:.2f}, imu_ax={imu_ax:.2f}, '
                            f'bias={self.imu_encoder_bias:.3f}), '
                            f'cov_mult={self.slip_cov_multiplier:.1f}x')
                        self.prev_slip_state = True
            else:
                self.slip_counter = 0
                # 점진적 감쇠
                self.slip_cov_multiplier = max(
                    1.0, self.slip_cov_multiplier * self.slip_decay)
                
                if self.prev_slip_state and self.slip_cov_multiplier < 1.1:
                    self.get_logger().info(
                        f'SLIP CLEARED: cov_mult={self.slip_cov_multiplier:.2f}x, '
                        f'bias={self.imu_encoder_bias:.3f}')
                    self.prev_slip_state = False
        
        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_vyaw = vyaw
        self.prev_time = current_time
        
        # ============================================================
        # 동적 공분산 계산
        # ============================================================
        is_stationary = (linear_speed < self.stationary_lin_thresh and 
                        angular_speed < self.stationary_ang_thresh)
        
        if is_stationary:
            cov_vx = self.base_linear_cov * self.stationary_cov_factor
            cov_vy = self.base_linear_cov * self.stationary_cov_factor
            cov_vyaw = self.base_angular_cov * self.stationary_cov_factor
            cov_x = self.base_pose_cov * self.stationary_cov_factor
            cov_y = self.base_pose_cov * self.stationary_cov_factor
            cov_yaw = self.base_orient_cov * self.stationary_cov_factor
        else:
            speed_factor = 1.0 + linear_speed * self.speed_cov_scale
            rotation_factor = 1.0 + angular_speed * self.rotation_cov_scale
            accel_factor = 1.0 + self.ema_accel * self.accel_cov_scale
            
            cov_vx = self.base_linear_cov * speed_factor * rotation_factor * accel_factor
            cov_vy = self.base_linear_cov * speed_factor * rotation_factor * accel_factor
            cov_vyaw = self.base_angular_cov * rotation_factor * accel_factor
            cov_x = self.base_pose_cov * speed_factor * rotation_factor
            cov_y = self.base_pose_cov * speed_factor * rotation_factor
            cov_yaw = self.base_orient_cov * rotation_factor
        
        # ============================================================
        # 슬립 배율 적용 (동적 공분산)
        # ============================================================
        slip_m = self.slip_cov_multiplier
        if slip_m > 1.0:
            cov_vx *= slip_m
            cov_vy *= slip_m
            cov_vyaw *= slip_m
            cov_x *= slip_m
            cov_y *= slip_m
            cov_yaw *= slip_m
        
        # ============================================================
        # 공분산 행렬 설정 (6x6 대각행렬)
        # ============================================================
        pose_cov = [0.0] * 36
        pose_cov[0]  = cov_x
        pose_cov[7]  = cov_y
        pose_cov[14] = 1e6
        pose_cov[21] = 1e6
        pose_cov[28] = 1e6
        pose_cov[35] = cov_yaw
        
        twist_cov = [0.0] * 36
        twist_cov[0]  = cov_vx
        twist_cov[7]  = cov_vy
        twist_cov[14] = 1e6
        twist_cov[21] = 1e6
        twist_cov[28] = 1e6
        twist_cov[35] = cov_vyaw
        
        # ============================================================
        # 메시지 발행
        # ============================================================
        adapted_msg = Odometry()
        adapted_msg.header = msg.header
        adapted_msg.child_frame_id = msg.child_frame_id
        adapted_msg.pose = msg.pose
        adapted_msg.twist = msg.twist
        adapted_msg.pose.covariance = pose_cov
        adapted_msg.twist.covariance = twist_cov
        
        self.odom_pub.publish(adapted_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
