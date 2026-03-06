#!/usr/bin/env python3
# encoding: utf-8
"""
VSLAM Covariance Adapter Node (with Anomaly-Based Gating)
===========================================================
Isaac ROS cuVSLAM은 pose/twist covariance를 0으로 발행하는 경우가 있다.
(특히 vo_state: 0 (tracking lost) 상태에서)

robot_localization은 covariance가 0이면 해당 측정을 무한 신뢰하므로,
드리프트된 VSLAM 데이터로 EKF가 발산하는 원인이 된다.

이 노드는:
  1. /visual_slam/tracking/odometry 를 구독
  2. 2단계 게이팅으로 잘못된 데이터 차단:
     a) vo_state 기반 게이팅 (VisualSlamStatus 메시지 가용 시)
     b) odom anomaly detection (항상 활성 — vo_state 불가 시 주력)
  3. covariance가 0인 경우 최소 공분산을 부여
  4. /visual_slam/tracking/odometry_adapted 로 재발행

Anomaly Detection 전략 (2026-02-27):
  - 연속 odom 메시지 간 yaw 변화량 감시
  - |Δyaw| > max_yaw_delta (기본 0.15 rad ≈ 8.6°) → anomaly
  - |Δposition| > max_pos_delta (기본 0.5m) → anomaly
  - anomaly 감지 시 즉시 차단 + lost 상태 진입
  - 연속 good_count_threshold (기본 30) 회 정상 → tracking 복구 판정
  - 복구 후 recovery_delay 대기 + 공분산 증폭으로 soft transition

이점:
  - isaac_ros_visual_slam_interfaces 의존성 없이 동작
  - vo_state=1이지만 데이터가 나쁜 경우도 감지 가능
  - Docker 내부 빌드된 메시지 타입 불필요
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time


def quaternion_to_yaw(qz, qw):
    """quaternion z, w → yaw (rad)"""
    return 2.0 * math.atan2(qz, qw)


def angle_diff(a, b):
    """최단 각도 차이 (rad), [-π, π]"""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class VslamCovarianceAdapter(Node):
    def __init__(self):
        super().__init__('vslam_covariance_adapter')

        # ============================================================
        # 파라미터 선언
        # ============================================================
        # 최소 pose 공분산 (0일 때 이 값으로 대체)
        self.declare_parameter('min_position_cov', 0.05)       # m²
        self.declare_parameter('min_orientation_cov', 0.02)    # rad²

        # 최소 twist 공분산
        self.declare_parameter('min_linear_vel_cov', 0.01)     # (m/s)²
        self.declare_parameter('min_angular_vel_cov', 0.005)   # (rad/s)²

        # 기존 공분산이 0보다 큰 경우 최소값으로 클램프
        self.declare_parameter('clamp_existing', True)

        # 토픽 이름
        self.declare_parameter('input_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('output_topic', '/visual_slam/tracking/odometry_adapted')

        # vo_state 게이팅 파라미터
        self.declare_parameter('status_topic', '/visual_slam/status')
        self.declare_parameter('enable_vo_state_gating', True)  # vo_state 구독 시도
        self.declare_parameter('good_vo_state', 1)

        # Anomaly detection 파라미터 (항상 활성)
        self.declare_parameter('max_yaw_delta', 0.15)           # rad (~8.6°) 스텝당 최대 yaw 변화
        self.declare_parameter('max_pos_delta', 0.5)            # m 스텝당 최대 위치 변화
        self.declare_parameter('good_count_threshold', 30)      # 연속 정상 횟수 → tracking 복구 판정

        # Frozen data detection (2026-03-04)
        # cuVSLAM이 tracking lost 시 마지막 pose를 계속 발행하는 경우 감지
        self.declare_parameter('frozen_threshold', 10)           # 연속 동일 pose 횟수 → frozen 판정
        self.declare_parameter('frozen_pos_tolerance', 1e-6)    # m - 위치 변화 없음 판정 기준
        self.declare_parameter('frozen_yaw_tolerance', 1e-6)    # rad - yaw 변화 없음 판정 기준

        # 공통 복구 파라미터
        self.declare_parameter('recovery_delay', 1.0)           # 복구 후 대기 시간 (초)
        self.declare_parameter('recovery_cov_multiplier', 5.0)  # 복구 직후 공분산 증폭
        self.declare_parameter('recovery_cov_decay_time', 3.0)  # 증폭 → 1.0x 수렴 시간

        # 파라미터 로드
        self.min_pos_cov = self.get_parameter('min_position_cov').value
        self.min_orient_cov = self.get_parameter('min_orientation_cov').value
        self.min_lin_vel_cov = self.get_parameter('min_linear_vel_cov').value
        self.min_ang_vel_cov = self.get_parameter('min_angular_vel_cov').value
        self.clamp_existing = self.get_parameter('clamp_existing').value

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.enable_vo_state_gating = self.get_parameter('enable_vo_state_gating').value
        self.good_vo_state = self.get_parameter('good_vo_state').value
        self.max_yaw_delta = self.get_parameter('max_yaw_delta').value
        self.max_pos_delta = self.get_parameter('max_pos_delta').value
        self.good_count_threshold = self.get_parameter('good_count_threshold').value
        self.recovery_delay = self.get_parameter('recovery_delay').value
        self.recovery_cov_multiplier = self.get_parameter('recovery_cov_multiplier').value
        self.recovery_cov_decay_time = self.get_parameter('recovery_cov_decay_time').value
        self.frozen_threshold = self.get_parameter('frozen_threshold').value
        self.frozen_pos_tolerance = self.get_parameter('frozen_pos_tolerance').value
        self.frozen_yaw_tolerance = self.get_parameter('frozen_yaw_tolerance').value

        # 6x6 공분산 행렬의 대각 인덱스
        self.pose_diag_indices = [0, 7, 14, 21, 28, 35]
        self.twist_diag_indices = [0, 7, 14, 21, 28, 35]

        self.min_pose_cov_diag = [
            self.min_pos_cov, self.min_pos_cov, self.min_pos_cov,
            self.min_orient_cov, self.min_orient_cov, self.min_orient_cov,
        ]
        self.min_twist_cov_diag = [
            self.min_lin_vel_cov, self.min_lin_vel_cov, self.min_lin_vel_cov,
            self.min_ang_vel_cov, self.min_ang_vel_cov, self.min_ang_vel_cov,
        ]

        # ============================================================
        # Anomaly detection 상태 (항상 활성)
        # ============================================================
        self.prev_odom = None           # 이전 odom 메시지 (pose)
        self.anomaly_detected = False   # 현재 anomaly 상태
        self.consecutive_good = 0       # 연속 정상 메시지 수
        self.tracking_ok = False        # 최종 판정: 발행 허용 여부
        self.tracking_recovered_time = None

        # ============================================================
        # Frozen data detection 상태 (2026-03-04)
        # ============================================================
        self.frozen_count = 0           # 연속 동일 pose 메시지 수
        self.frozen_gated_count = 0     # frozen으로 gate된 총 횟수

        # ============================================================
        # vo_state 상태 (선택적)
        # ============================================================
        self.vo_state = 0
        self.vo_state_available = False  # vo_state 구독 성공 여부

        # ============================================================
        # 통계
        # ============================================================
        self.msg_count = 0
        self.published_count = 0
        self.gated_count = 0
        self.anomaly_gated_count = 0
        self.vo_state_gated_count = 0
        self.zero_pose_cov_count = 0
        self.zero_twist_cov_count = 0

        # ============================================================
        # 퍼블리셔/서브스크라이버
        # ============================================================
        self.odom_sub = self.create_subscription(
            Odometry, input_topic, self.odom_callback, 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, output_topic, 10
        )

        # vo_state 구독 시도 (optional — 실패해도 anomaly detection으로 동작)
        if self.enable_vo_state_gating:
            self._setup_status_subscription(status_topic)

        self.create_timer(30.0, self.log_stats)

        vo_str = 'AVAILABLE' if self.vo_state_available else 'UNAVAILABLE (anomaly detection only)'
        self.get_logger().info(
            f'VslamCovarianceAdapter: {input_topic} → {output_topic}\n'
            f'  min_cov: pos={self.min_pos_cov}, orient={self.min_orient_cov}\n'
            f'  anomaly detection: max_yaw_delta={math.degrees(self.max_yaw_delta):.1f}°, '
            f'max_pos_delta={self.max_pos_delta}m, '
            f'good_count_threshold={self.good_count_threshold}\n'
            f'  frozen detection: threshold={self.frozen_threshold} msgs, '
            f'pos_tol={self.frozen_pos_tolerance}m, yaw_tol={self.frozen_yaw_tolerance}rad\n'
            f'  recovery: delay={self.recovery_delay}s, '
            f'cov_multiplier={self.recovery_cov_multiplier}x, '
            f'decay_time={self.recovery_cov_decay_time}s\n'
            f'  vo_state: {vo_str}'
        )

    # ================================================================
    # vo_state 구독 (선택적)
    # ================================================================
    def _setup_status_subscription(self, status_topic: str):
        """VisualSlamStatus 타입 구독 시도. 실패 시 anomaly detection만 사용."""
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.status_sub = self.create_subscription(
                VisualSlamStatus, status_topic,
                self._status_callback_typed, 10
            )
            self.vo_state_available = True
            self.get_logger().info(
                f'vo_state subscription OK on {status_topic}'
            )
        except (ImportError, Exception) as e:
            self.vo_state_available = False
            self.get_logger().warn(
                f'VisualSlamStatus import failed: {e}\n'
                f'  → Using anomaly detection as primary gating method.'
            )

    def _status_callback_typed(self, msg):
        """vo_state 콜백 — anomaly detection과 함께 사용"""
        prev = self.vo_state
        self.vo_state = msg.vo_state

        if prev == self.good_vo_state and self.vo_state != self.good_vo_state:
            self.get_logger().warn(
                f'VSLAM vo_state: tracking LOST ({self.vo_state})'
            )
        elif prev != self.good_vo_state and self.vo_state == self.good_vo_state:
            self.get_logger().info(
                f'VSLAM vo_state: tracking RECOVERED ({self.vo_state})'
            )

    # ================================================================
    # Anomaly Detection (항상 활성)
    # ================================================================
    def _check_frozen(self, msg: Odometry) -> bool:
        """
        Frozen data detection (2026-03-04).
        cuVSLAM이 tracking lost 시 마지막 pose를 계속 발행하는 경우 감지.
        연속 frozen_threshold 회 동일 pose → anomaly.

        Returns True if data is frozen (should gate).
        """
        if self.prev_odom is None:
            self.frozen_count = 0
            return False

        pose = msg.pose.pose

        # 위치 변화량
        dx = pose.position.x - self.prev_odom.position.x
        dy = pose.position.y - self.prev_odom.position.y
        pos_delta = math.sqrt(dx * dx + dy * dy)

        # Yaw 변화량
        prev_yaw = quaternion_to_yaw(
            self.prev_odom.orientation.z,
            self.prev_odom.orientation.w
        )
        curr_yaw = quaternion_to_yaw(
            pose.orientation.z,
            pose.orientation.w
        )
        yaw_delta = abs(angle_diff(curr_yaw, prev_yaw))

        if pos_delta < self.frozen_pos_tolerance and yaw_delta < self.frozen_yaw_tolerance:
            self.frozen_count += 1
            if self.frozen_count == self.frozen_threshold:
                self.get_logger().warn(
                    f'VSLAM FROZEN: {self.frozen_count} consecutive identical poses '
                    f'(pos_delta<{self.frozen_pos_tolerance}m, yaw_delta<{self.frozen_yaw_tolerance}rad). '
                    f'Gating stale data.'
                )
            return self.frozen_count >= self.frozen_threshold
        else:
            if self.frozen_count >= self.frozen_threshold:
                self.get_logger().info(
                    f'VSLAM unfrozen after {self.frozen_count} identical msgs. '
                    f'pos_delta={pos_delta:.6f}m, yaw_delta={math.degrees(yaw_delta):.3f}°'
                )
            self.frozen_count = 0
            return False

    def _check_anomaly(self, msg: Odometry) -> bool:
        """
        연속 odom 간 이상치 감지.
        Returns True if anomaly detected (should gate).
        """
        pose = msg.pose.pose

        # vo_state가 가용하고 tracking lost이면 즉시 anomaly
        if self.vo_state_available and self.vo_state != self.good_vo_state:
            self.prev_odom = None  # 이전 기준 무효화
            return True

        if self.prev_odom is None:
            # 첫 메시지 — 기준점 설정, anomaly 아님
            self.prev_odom = pose
            return False

        # Yaw 변화량 계산
        prev_yaw = quaternion_to_yaw(
            self.prev_odom.orientation.z,
            self.prev_odom.orientation.w
        )
        curr_yaw = quaternion_to_yaw(
            pose.orientation.z,
            pose.orientation.w
        )
        yaw_delta = abs(angle_diff(curr_yaw, prev_yaw))

        # 위치 변화량 계산
        dx = pose.position.x - self.prev_odom.position.x
        dy = pose.position.y - self.prev_odom.position.y
        pos_delta = math.sqrt(dx * dx + dy * dy)

        # 기준점 업데이트 (anomaly가 아닌 경우에만)
        is_anomaly = False

        if yaw_delta > self.max_yaw_delta:
            is_anomaly = True
            if not self.anomaly_detected:
                self.get_logger().warn(
                    f'Anomaly: yaw delta={math.degrees(yaw_delta):.1f}° '
                    f'> {math.degrees(self.max_yaw_delta):.1f}° threshold'
                )

        if pos_delta > self.max_pos_delta:
            is_anomaly = True
            if not self.anomaly_detected:
                self.get_logger().warn(
                    f'Anomaly: position delta={pos_delta:.3f}m '
                    f'> {self.max_pos_delta}m threshold'
                )

        if not is_anomaly:
            self.prev_odom = pose

        return is_anomaly

    def _update_tracking_state(self, is_anomaly: bool):
        """
        anomaly 감지 결과로 tracking 상태 관리.
        연속 good_count_threshold 횟수 정상 → 복구 판정.
        """
        now = time.monotonic()

        if is_anomaly:
            if self.tracking_ok or self.consecutive_good > 0:
                if self.tracking_ok:
                    self.get_logger().warn('VSLAM tracking LOST (anomaly detected). Gating.')
                self.tracking_ok = False
                self.tracking_recovered_time = None
            self.consecutive_good = 0
            self.anomaly_detected = True
        else:
            if self.anomaly_detected:
                self.consecutive_good += 1
                if self.consecutive_good >= self.good_count_threshold:
                    # 충분히 연속 정상 → 복구 시작
                    self.anomaly_detected = False
                    self.tracking_recovered_time = now
                    self.get_logger().info(
                        f'VSLAM tracking RECOVERED ({self.consecutive_good} '
                        f'consecutive good msgs). '
                        f'Waiting {self.recovery_delay}s before publishing.'
                    )
            else:
                # 정상 상태 유지 중
                if self.tracking_recovered_time is not None:
                    elapsed = now - self.tracking_recovered_time
                    if elapsed >= self.recovery_delay:
                        if not self.tracking_ok:
                            self.get_logger().info(
                                f'VSLAM recovery delay passed ({elapsed:.1f}s). '
                                f'Publishing resumed.'
                            )
                        self.tracking_ok = True
                else:
                    # anomaly가 한 번도 없었던 경우
                    self.tracking_ok = True

    # ================================================================
    # 공분산 증폭 배율
    # ================================================================
    def _get_recovery_cov_multiplier(self) -> float:
        """복구 직후 공분산 증폭 → 시간에 따라 1.0x로 감소"""
        if self.tracking_recovered_time is None:
            return 1.0

        elapsed = time.monotonic() - self.tracking_recovered_time
        if elapsed >= self.recovery_cov_decay_time + self.recovery_delay:
            return 1.0

        decay_elapsed = elapsed - self.recovery_delay
        if decay_elapsed <= 0:
            return self.recovery_cov_multiplier

        ratio = min(decay_elapsed / self.recovery_cov_decay_time, 1.0)
        mult = self.recovery_cov_multiplier + (1.0 - self.recovery_cov_multiplier) * ratio
        return max(mult, 1.0)

    # ================================================================
    # Odom 콜백
    # ================================================================
    def odom_callback(self, msg: Odometry):
        """VSLAM 오도메트리 수신 → frozen/anomaly 감지 → 공분산 보정 → 재발행"""
        self.msg_count += 1

        # 1) Frozen data detection (2026-03-04)
        is_frozen = self._check_frozen(msg)

        # 2) Anomaly detection (항상 수행)
        is_anomaly = self._check_anomaly(msg)

        # 3) Tracking 상태 관리 (anomaly만 반영, frozen은 별도 처리)
        #    frozen은 "정지 상태"와 "추적 실패 stale data"를 구분할 수 없으므로
        #    tracking 복구 판정(consecutive_good)을 방해하지 않도록 분리.
        #    → frozen일 때: 메시지는 발행하지 않되, tracking 상태는 anomaly만으로 판단
        self._update_tracking_state(is_anomaly)

        # 4) 게이팅 판정: frozen이면 무조건 gate (중복 데이터 EKF 투입 방지)
        if is_frozen:
            self.gated_count += 1
            self.frozen_gated_count += 1
            return

        if not self.tracking_ok:
            self.gated_count += 1
            if is_anomaly:
                self.anomaly_gated_count += 1
            if self.vo_state_available and self.vo_state != self.good_vo_state:
                self.vo_state_gated_count += 1
            return

        # 4) Pose Covariance 보정
        cov_multiplier = self._get_recovery_cov_multiplier()

        pose_cov = list(msg.pose.covariance)
        pose_had_zero = False
        for i, diag_idx in enumerate(self.pose_diag_indices):
            min_cov = self.min_pose_cov_diag[i] * cov_multiplier
            if abs(pose_cov[diag_idx]) < 1e-15:
                pose_cov[diag_idx] = min_cov
                pose_had_zero = True
            elif self.clamp_existing and pose_cov[diag_idx] < min_cov:
                pose_cov[diag_idx] = min_cov
        if pose_had_zero:
            self.zero_pose_cov_count += 1
        msg.pose.covariance = pose_cov

        # 5) Twist Covariance 보정
        twist_cov = list(msg.twist.covariance)
        twist_had_zero = False
        for i, diag_idx in enumerate(self.twist_diag_indices):
            min_cov = self.min_twist_cov_diag[i] * cov_multiplier
            if abs(twist_cov[diag_idx]) < 1e-15:
                twist_cov[diag_idx] = min_cov
                twist_had_zero = True
            elif self.clamp_existing and twist_cov[diag_idx] < min_cov:
                twist_cov[diag_idx] = min_cov
        if twist_had_zero:
            self.zero_twist_cov_count += 1
        msg.twist.covariance = twist_cov

        # 6) 재발행
        self.odom_pub.publish(msg)
        self.published_count += 1

    # ================================================================
    # 통계 로그
    # ================================================================
    def log_stats(self):
        if self.msg_count > 0:
            pub_pct = (self.published_count / self.msg_count) * 100
            gate_pct = (self.gated_count / self.msg_count) * 100
            cov_mult = self._get_recovery_cov_multiplier()

            self.get_logger().info(
                f'Stats: {self.msg_count} msgs | '
                f'pub={self.published_count} ({pub_pct:.0f}%) | '
                f'gated={self.gated_count} ({gate_pct:.0f}%) '
                f'[anomaly={self.anomaly_gated_count}, frozen={self.frozen_gated_count}, '
                f'vo_state={self.vo_state_gated_count}] | '
                f'zero_cov: pose={self.zero_pose_cov_count}, twist={self.zero_twist_cov_count} | '
                f'tracking_ok={self.tracking_ok} frozen_count={self.frozen_count} '
                f'vo_state={self.vo_state if self.vo_state_available else "N/A"} '
                f'cov_mult={cov_mult:.1f}x'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VslamCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
