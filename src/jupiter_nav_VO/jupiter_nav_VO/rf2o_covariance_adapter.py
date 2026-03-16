#!/usr/bin/env python3
"""
RF2O Rotation-Aware Dynamic Covariance Adapter
===============================================

RF2O LiDAR odometry의 공분산을 로봇 상태에 따라 동적으로 조절합니다.

문제:
  - RF2O는 /scan 스캔매칭 기반 → 제자리 회전 시 x,y 변위가 매우 부정확
  - 기존 vslam_covariance_adapter는 정적 공분산만 부여 (항상 0.5 m²)
  - 회전 중 RF2O의 noisy position이 EKF position을 오염시킴
  - 회전 후 직진 시 EKF heading 오차로 인한 좌측편향 발생

해결:
  - cmd_vel angular velocity를 모니터링하여 회전 상태 감지
  - 직진 시: 기본 공분산 (position_cov_base) → RF2O 정상 기여
  - 회전 시: 공분산 × rotation_multiplier → RF2O 영향력 대폭 감소
  - 회전 종료 후: exponential decay로 부드럽게 기본값 복귀

공분산 전략:
  - 직진 중: pose_cov = 0.5 m² → Wheel(0.001)의 500배 → RF2O 약한 기여
  - 회전 중: pose_cov = 0.5 × 20 = 10.0 m² → Wheel(~0.003)의 3333배 → RF2O 사실상 무시
  - 전환 시: 1초에 걸쳐 exponential decay → 급격한 EKF 점프 방지

2026-03-16 생성: Direction D — 180° 회전 후 좌편향 개선
"""

import rclpy
import time
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class RF2OCovarianceAdapter(Node):

    def __init__(self):
        super().__init__('rf2o_covariance_adapter')

        # ── Parameters ──
        self.declare_parameter('input_topic', '/odom_rf2o')
        self.declare_parameter('output_topic', '/odom_rf2o_adapted')

        # 기본 공분산 (직진 시)
        self.declare_parameter('position_cov_base', 0.5)        # m² - 직진 시 pose x,y 공분산
        self.declare_parameter('orientation_cov_base', 0.05)     # rad² - pose yaw 공분산
        self.declare_parameter('linear_vel_cov_base', 0.01)      # (m/s)² - twist vx,vy 공분산
        self.declare_parameter('angular_vel_cov_base', 0.01)     # (rad/s)² - twist wz 공분산

        # 회전 감지
        self.declare_parameter('rotation_threshold', 0.15)       # rad/s - 이 이상이면 회전 중으로 판단
        self.declare_parameter('rotation_multiplier', 20.0)      # 회전 중 공분산 배수 (position)
        self.declare_parameter('rotation_orient_multiplier', 5.0) # 회전 중 orientation 공분산 배수
        self.declare_parameter('transition_decay_time', 1.0)     # 초 - 회전→직진 전환 시간

        # cmd_vel 타임아웃
        self.declare_parameter('cmd_vel_timeout', 0.5)           # 초 - cmd_vel 수신 없으면 정지 판단

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.position_cov_base = self.get_parameter('position_cov_base').value
        self.orientation_cov_base = self.get_parameter('orientation_cov_base').value
        self.linear_vel_cov_base = self.get_parameter('linear_vel_cov_base').value
        self.angular_vel_cov_base = self.get_parameter('angular_vel_cov_base').value
        self.rotation_threshold = self.get_parameter('rotation_threshold').value
        self.rotation_multiplier = self.get_parameter('rotation_multiplier').value
        self.rotation_orient_multiplier = self.get_parameter('rotation_orient_multiplier').value
        self.transition_decay_time = self.get_parameter('transition_decay_time').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # ── State ──
        self.current_wz = 0.0            # 현재 cmd_vel angular.z
        self.last_cmd_time = 0.0          # 마지막 cmd_vel 수신 시각
        self.rotation_end_time = 0.0      # 회전 종료 시각 (decay 시작)
        self.is_rotating = False          # 현재 회전 중 여부
        self.current_multiplier = 1.0     # 현재 적용 중인 배수
        self.msg_count = 0
        self.last_log_time = 0.0

        # ── QoS ──
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Subscribers ──
        self.sub_odom = self.create_subscription(
            Odometry, input_topic, self.odom_callback, odom_qos)
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, cmd_qos)

        # ── Publisher ──
        self.pub = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(
            f'RF2O rotation-aware covariance adapter started\n'
            f'  {input_topic} → {output_topic}\n'
            f'  Base cov: pos={self.position_cov_base}, orient={self.orientation_cov_base}\n'
            f'  Rotation: threshold={self.rotation_threshold} rad/s, '
            f'multiplier={self.rotation_multiplier}x\n'
            f'  Transition decay: {self.transition_decay_time}s'
        )

    def cmd_vel_callback(self, msg: Twist):
        """cmd_vel 모니터링으로 회전 상태 감지"""
        self.current_wz = msg.angular.z
        self.last_cmd_time = time.time()

        was_rotating = self.is_rotating
        self.is_rotating = abs(self.current_wz) > self.rotation_threshold

        # 회전 → 직진 전환 시점 기록
        if was_rotating and not self.is_rotating:
            self.rotation_end_time = time.time()

    def compute_multiplier(self) -> float:
        """
        현재 상태에 따른 공분산 배수 계산

        Returns:
            1.0 (직진) ~ rotation_multiplier (회전 중)
            전환 시 exponential decay
        """
        now = time.time()

        # cmd_vel 타임아웃 → 정지 상태
        if now - self.last_cmd_time > self.cmd_vel_timeout:
            self.is_rotating = False

        if self.is_rotating:
            # 회전 강도에 비례하여 배수 증가
            # |wz|=0.15에서 1.0x → |wz|=1.0에서 rotation_multiplier
            wz_ratio = min(abs(self.current_wz) / max(self.rotation_threshold, 0.01), 5.0)
            target = 1.0 + (self.rotation_multiplier - 1.0) * min(wz_ratio, 1.0)
            # 빠른 상승 (즉시 적용)
            self.current_multiplier = max(self.current_multiplier, target)
            return self.current_multiplier

        elif self.rotation_end_time > 0 and self.transition_decay_time > 0:
            # 회전 종료 후 exponential decay
            elapsed = now - self.rotation_end_time
            if elapsed < self.transition_decay_time * 3:  # 3τ 후 완전 복귀
                decay = math.exp(-elapsed / self.transition_decay_time)
                self.current_multiplier = 1.0 + (self.rotation_multiplier - 1.0) * decay
                return self.current_multiplier
            else:
                self.current_multiplier = 1.0
                self.rotation_end_time = 0.0
                return 1.0
        else:
            self.current_multiplier = 1.0
            return 1.0

    def odom_callback(self, msg: Odometry):
        """RF2O odometry에 동적 공분산 적용"""
        multiplier = self.compute_multiplier()

        # 공분산 계산
        pos_cov = self.position_cov_base * multiplier
        orient_cov = self.orientation_cov_base * min(multiplier, self.rotation_orient_multiplier)
        lin_cov = self.linear_vel_cov_base * multiplier
        ang_cov = self.angular_vel_cov_base * min(multiplier, self.rotation_orient_multiplier)

        # Pose covariance (6×6, row-major)
        pose_cov = [0.0] * 36
        pose_cov[0]  = pos_cov      # x
        pose_cov[7]  = pos_cov      # y
        pose_cov[14] = 1e6          # z (2D, 무시)
        pose_cov[21] = 1e6          # roll (2D, 무시)
        pose_cov[28] = 1e6          # pitch (2D, 무시)
        pose_cov[35] = orient_cov   # yaw
        msg.pose.covariance = pose_cov

        # Twist covariance (6×6, row-major)
        twist_cov = [0.0] * 36
        twist_cov[0]  = lin_cov     # vx
        twist_cov[7]  = lin_cov     # vy
        twist_cov[14] = 1e6         # vz (2D, 무시)
        twist_cov[21] = 1e6         # vroll
        twist_cov[28] = 1e6         # vpitch
        twist_cov[35] = ang_cov     # vyaw
        msg.twist.covariance = twist_cov

        self.pub.publish(msg)
        self.msg_count += 1

        # 상태 로깅 (5초마다)
        now = time.time()
        if now - self.last_log_time > 5.0:
            state = "ROTATING" if self.is_rotating else ("TRANSITION" if self.current_multiplier > 1.1 else "STRAIGHT")
            self.get_logger().info(
                f'[{state}] mult={multiplier:.1f}x, pos_cov={pos_cov:.3f}, '
                f'wz={self.current_wz:.3f} rad/s, msgs={self.msg_count}'
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = RF2OCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
