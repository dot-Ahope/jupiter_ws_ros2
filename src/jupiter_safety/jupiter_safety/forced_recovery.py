#!/usr/bin/env python3
"""
Forced Recovery — Tier 2C (perception-independent recovery)
============================================================

When stuck_detector flags STUCK, this node publishes backward cmd_vel directly to
/cmd_vel_force, bypassing nav2's collision check.

Safety:
  - Pre-check rear obstacle via /scan_merged (angular sector around π=180°, robot rear).
    If anything within REAR_SAFE_DIST → skip recovery (would crash backward).
  - Cooldown after each recovery (RECOVERY_COOLDOWN) to prevent oscillation.
  - Hard timeout per recovery cycle.

State machine:
  IDLE → (stuck=True + rear safe) → BACKING_UP → STOP_BRIEF → COOLDOWN → IDLE
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


# Backup motion profile
BACKUP_VX = -0.10           # [m/s] backward speed (matches driver linear STARTUP kick)
BACKUP_DURATION = 1.0       # [s] backup time → 0.10m total
STOP_BRIEF_DURATION = 0.3   # [s] zero cmd_vel pause before releasing control to nav

# Safety
REAR_SAFE_DIST = 0.20       # [m] rear obstacle clearance required to backup
REAR_SECTOR_HALF_ANGLE = math.radians(30.0)   # ±30° around rear (π)

# Cooldown
RECOVERY_COOLDOWN = 5.0     # [s] after a recovery, ignore further stuck for this period

# Pub rate
PUB_RATE_HZ = 20.0


class ForcedRecovery(Node):

    def __init__(self):
        super().__init__('forced_recovery')

        self._is_stuck = False
        self._scan_ranges = []
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0
        self._scan_received = False

        self._state = 'IDLE'
        self._state_start = 0.0
        self._cooldown_until = 0.0

        self.create_subscription(Bool, '/stuck_status', self._cb_stuck, 10)
        # /scan_merged 는 sensor stream → BEST_EFFORT publisher (scan_merger). 기본 RELIABLE
        # 구독은 QoS incompatible 로 메시지 0 수신 → SensorDataQoS 사용.
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, '/scan_merged', self._cb_scan, scan_qos)

        self._pub_force = self.create_publisher(Twist, '/cmd_vel_force', 10)

        self.create_timer(1.0 / PUB_RATE_HZ, self._tick)

        self.get_logger().info(
            f"ForcedRecovery up — backup {BACKUP_VX} m/s × {BACKUP_DURATION}s, "
            f"rear safety {REAR_SAFE_DIST}m × ±{math.degrees(REAR_SECTOR_HALF_ANGLE):.0f}°, "
            f"cooldown {RECOVERY_COOLDOWN}s"
        )

    # -------- subscriptions --------
    def _cb_stuck(self, msg: Bool):
        self._is_stuck = msg.data

    def _cb_scan(self, msg: LaserScan):
        self._scan_ranges = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        self._scan_angle_inc = msg.angle_increment
        self._scan_received = True

    # -------- safety check --------
    def _is_rear_safe(self) -> bool:
        """후방 부채꼴 영역에 obstacle 이 REAR_SAFE_DIST 이내에 있는지 검사."""
        if not self._scan_received or len(self._scan_ranges) == 0:
            self.get_logger().warn("rear safety: no /scan_merged data → conservative SKIP backup")
            return False

        n = len(self._scan_ranges)
        # 후방 = pi rad (180°). scan 의 frame 이 base_link 라 0 rad 이 forward.
        # robot 의 forward 가 +x, angle 0 이면 +x 방향. 후방은 ±π (또는 -π).
        # ±30° 부채꼴 → angle ∈ [π-30°, π+30°] 또는 wrap-around 처리
        rear_target = math.pi
        for i, r in enumerate(self._scan_ranges):
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
            angle = self._scan_angle_min + i * self._scan_angle_inc
            # angle wrap to [-π, π]
            angle_wrapped = math.atan2(math.sin(angle), math.cos(angle))
            # rear sector: angle close to ±π
            angular_dist_to_rear = math.pi - abs(angle_wrapped)  # 0 = rear, π = forward
            if angular_dist_to_rear <= REAR_SECTOR_HALF_ANGLE:
                # 후방 부채꼴 안의 ray
                if r < REAR_SAFE_DIST:
                    return False
        return True

    # -------- state machine --------
    def _publish_zero(self):
        zero = Twist()
        self._pub_force.publish(zero)

    def _publish_backup(self):
        cmd = Twist()
        cmd.linear.x = BACKUP_VX
        self._pub_force.publish(cmd)

    def _tick(self):
        now = time.monotonic()

        # cooldown 동안엔 stuck 무시
        in_cooldown = now < self._cooldown_until

        if self._state == 'IDLE':
            if self._is_stuck and not in_cooldown:
                if self._is_rear_safe():
                    self._state = 'BACKING_UP'
                    self._state_start = now
                    self.get_logger().warn("STUCK → BACKING_UP (rear safe)")
                else:
                    self.get_logger().error(
                        "STUCK detected but rear NOT safe — skipping backup. "
                        "Operator intervention required."
                    )
                    # cooldown 줘서 spam 방지
                    self._cooldown_until = now + 2.0

        elif self._state == 'BACKING_UP':
            elapsed = now - self._state_start
            if elapsed < BACKUP_DURATION:
                self._publish_backup()
            else:
                self._state = 'STOP_BRIEF'
                self._state_start = now
                self.get_logger().info("BACKING_UP done → STOP_BRIEF")

        elif self._state == 'STOP_BRIEF':
            elapsed = now - self._state_start
            if elapsed < STOP_BRIEF_DURATION:
                self._publish_zero()
            else:
                self._state = 'IDLE'
                self._cooldown_until = now + RECOVERY_COOLDOWN
                self.get_logger().info(
                    f"recovery cycle complete → IDLE (cooldown {RECOVERY_COOLDOWN}s)"
                )


def main(args=None):
    rclpy.init(args=args)
    node = ForcedRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
