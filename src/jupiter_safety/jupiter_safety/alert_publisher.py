#!/usr/bin/env python3
"""
Alert Publisher — Tier 2D
==========================

When stuck_status flips True, publish buzzer pattern to alert operator.

  Pattern: short beeps (0.2s ON, 0.3s OFF) for ALERT_DURATION seconds.
  After alert window, automatic silence (let recovery do its work).
  If stuck persists past recovery cooldown, alert again.

Subscribers required:
  - /Buzzer (std_msgs/Bool) — Jupiter base buzzer

Future extensions:
  - publish to OLED-display topic (currently OLED service is closed-source binary)
  - LED pattern via GPIO
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


ALERT_DURATION = 3.0      # [s] beep pattern continues this long after stuck=True
BEEP_ON = 0.2             # [s]
BEEP_OFF = 0.3            # [s]
TICK_RATE_HZ = 20.0


class AlertPublisher(Node):

    def __init__(self):
        super().__init__('alert_publisher')

        self._is_stuck = False
        self._alert_until = 0.0
        self._beep_phase_start = 0.0
        self._beep_on = False

        self.create_subscription(Bool, '/stuck_status', self._cb_stuck, 10)
        self._pub_buzzer = self.create_publisher(Bool, '/Buzzer', 10)

        self.create_timer(1.0 / TICK_RATE_HZ, self._tick)

        self.get_logger().info(
            f"AlertPublisher up — beep ({BEEP_ON}/{BEEP_OFF}s) for {ALERT_DURATION}s on stuck"
        )

    def _cb_stuck(self, msg: Bool):
        if msg.data and not self._is_stuck:
            # rising edge — alert window 시작
            now = time.monotonic()
            self._alert_until = now + ALERT_DURATION
            self._beep_phase_start = now
            self._beep_on = True
            self.get_logger().warn("STUCK alert started — beeping operator")
        self._is_stuck = msg.data

    def _tick(self):
        now = time.monotonic()
        if now < self._alert_until:
            phase_elapsed = now - self._beep_phase_start
            if self._beep_on and phase_elapsed >= BEEP_ON:
                # ON 끝, OFF phase 시작
                self._beep_on = False
                self._beep_phase_start = now
            elif not self._beep_on and phase_elapsed >= BEEP_OFF:
                # OFF 끝, 새 ON phase
                self._beep_on = True
                self._beep_phase_start = now

            msg = Bool()
            msg.data = self._beep_on
            self._pub_buzzer.publish(msg)
        else:
            # alert window 종료 — buzzer off
            if self._beep_on:
                self._beep_on = False
                msg = Bool()
                msg.data = False
                self._pub_buzzer.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AlertPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
