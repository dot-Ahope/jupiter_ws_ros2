#!/usr/bin/env python3
"""
Stuck Detector — Tier 2A + 2B (perception-independent)
========================================================

Two complementary detection mechanisms:

  2A. STALL DETECTION (motor encoder feedback vs commanded velocity)
      - subscribe: /cmd_vel (nav2 raw intent BEFORE collision_monitor gating),
                   /jupiter/get_vel (MCU encoder readout)
      - logic: |cmd.linear.x| > STALL_CMD_MIN AND
               |actual.linear.x| < STALL_ACTUAL_MAX AND
               sustained > STALL_DURATION → stalled = True
      - rationale: motor commanded but wheels not turning = wall / glass / heavy obstacle,
        OR collision_monitor blocking nav2 stream (rover wedged in lethal zone).
        We watch /cmd_vel (raw intent) instead of /cmd_vel_safe so that "nav2 wants to
        move but is being gated" is also detected → forced_recovery 가 후진으로 escape.
        Independent of perception. "Sensors can lie, motors can't."

  2B. COLLISION SPIKE (IMU acceleration)
      - subscribe: /jupiter/imu
      - logic: linear_acceleration.x sudden negative spike < COLLISION_ACCEL_THRESH
               AND robot was commanded forward → collision detected
      - rationale: instant deceleration peak = physical impact. Catches faster than 2A.

Output:
  /stuck_status (std_msgs/Bool) — True for STUCK_LATCH_DURATION seconds, then auto-reset.
  Subscribers (forced_recovery, alert_publisher) react.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


# Stall detection thresholds
STALL_CMD_MIN = 0.05         # |cmd.linear.x| 가 이 값 이상이어야 stall 후보
STALL_ACTUAL_MAX = 0.02      # |actual.linear.x| 가 이 값 이하면 모터 안 도는 것
STALL_DURATION = 1.5         # [s] 위 조건이 이 시간 동안 지속되면 stall

# Collision spike thresholds
COLLISION_ACCEL_THRESH = -2.0   # [m/s²] linear_acceleration.x 가 이 값 이하 (negative spike)
COLLISION_CMD_FWD_MIN = 0.05    # 충돌 직전 forward cmd 가 있었어야 (정지 중 IMU spike 무시)

# Stuck latch
STUCK_LATCH_DURATION = 1.0   # [s] stuck=True 발행 후 이 시간 동안 유지 (subscriber 가 잡을 시간)
TICK_RATE_HZ = 10.0          # 검출 루프 주기


class StuckDetector(Node):

    def __init__(self):
        super().__init__('stuck_detector')

        # 최신 신호 저장
        self._cmd_x = 0.0
        self._cmd_z = 0.0
        self._actual_x = 0.0
        self._actual_z = 0.0
        self._imu_accel_x = 0.0

        # stall 누적 시간
        self._stall_accum = 0.0
        self._last_tick = time.monotonic()

        # latch
        self._stuck_until = 0.0

        # /cmd_vel = nav2 raw intent (controller_server + behavior_server 의 BackUp/Spin).
        # /cmd_vel_safe (twist_mux 출력) 가 아니라 /cmd_vel 을 봐야 collision_monitor 가 차단한
        # 상태도 stall 로 검출됨. forced_recovery 가 자체 후방 LiDAR safety 로 backup 시도.
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd, 10)
        self.create_subscription(Twist, '/jupiter/get_vel', self._cb_actual, 10)
        self.create_subscription(Imu, '/jupiter/imu', self._cb_imu, 50)

        self._pub_stuck = self.create_publisher(Bool, '/stuck_status', 10)

        self.create_timer(1.0 / TICK_RATE_HZ, self._tick)

        self.get_logger().info(
            f"StuckDetector up — stall: |cmd|>{STALL_CMD_MIN} & |actual|<{STALL_ACTUAL_MAX} for {STALL_DURATION}s; "
            f"collision: accel.x<{COLLISION_ACCEL_THRESH}m/s² with forward cmd"
        )

    def _cb_cmd(self, msg: Twist):
        self._cmd_x = msg.linear.x
        self._cmd_z = msg.angular.z

    def _cb_actual(self, msg: Twist):
        self._actual_x = msg.linear.x
        self._actual_z = msg.angular.z

    def _cb_imu(self, msg: Imu):
        # raw imu, x = forward direction
        self._imu_accel_x = msg.linear_acceleration.x

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        # 2A — Stall detection
        cmd_abs = abs(self._cmd_x)
        actual_abs = abs(self._actual_x)
        stall_condition = (cmd_abs > STALL_CMD_MIN) and (actual_abs < STALL_ACTUAL_MAX)

        if stall_condition:
            self._stall_accum += dt
        else:
            self._stall_accum = 0.0

        stalled = self._stall_accum >= STALL_DURATION

        # 2B — Collision spike
        # IMU x 축 accel 의 갑작스런 음 spike (forward 중에)
        # 정지 상태에선 accel.x ≈ 0 또는 중력 영향만 (mounting 따라). raw IMU 라
        # gravity 보상 없으므로 base level 변동 큼 → 보수적 threshold 사용.
        collision_spike = (
            self._cmd_x > COLLISION_CMD_FWD_MIN and  # forward 명령 중
            self._imu_accel_x < COLLISION_ACCEL_THRESH   # 강한 음 spike
        )

        # latch: 둘 중 하나 trigger 되면 stuck=True 를 STUCK_LATCH_DURATION 동안 유지
        if stalled or collision_spike:
            self._stuck_until = now + STUCK_LATCH_DURATION
            if stalled and self._stall_accum < STALL_DURATION + dt:  # 첫 trigger 만 로그
                self.get_logger().warn(
                    f"STUCK detected (stall): cmd_x={self._cmd_x:+.3f}, actual_x={self._actual_x:+.3f}, "
                    f"accum={self._stall_accum:.2f}s"
                )
            if collision_spike:
                self.get_logger().warn(
                    f"STUCK detected (collision spike): cmd_x={self._cmd_x:+.3f}, "
                    f"imu_accel_x={self._imu_accel_x:+.2f} m/s²"
                )

        is_stuck = now < self._stuck_until

        msg = Bool()
        msg.data = is_stuck
        self._pub_stuck.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StuckDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
