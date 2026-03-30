#!/usr/bin/env python3
"""
VSLAM 오도메트리 테스트 스크립트
- /visual_slam/tracking/odometry 토픽을 구독하여 데이터 기록
- 지정 시간 동안 수집 후 통계 출력
- 용도: cuVSLAM 설정 변경 후 추적 성능 비교

사용법:
  python3 test_vslam_odom.py [duration_sec] [label]
  예) python3 test_vslam_odom.py 30 "test_A_emitter_off_60hz"
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys
import time


class VslamOdomTester(Node):
    def __init__(self, duration=30.0, label="test"):
        super().__init__('vslam_odom_tester')
        self.duration = duration
        self.label = label
        self.data = []
        self.start_time = None
        self.first_msg_time = None

        self.sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.callback,
            10
        )
        self.timer = self.create_timer(1.0, self.status_tick)
        self.wall_start = time.monotonic()
        self.get_logger().info(
            f'[{label}] Waiting for /visual_slam/tracking/odometry... '
            f'(timeout: {duration}s)'
        )

    def callback(self, msg):
        now = time.monotonic()
        if self.first_msg_time is None:
            self.first_msg_time = now
            delay = now - self.wall_start
            self.get_logger().info(
                f'[{self.label}] First message received! '
                f'(init delay: {delay:.1f}s)'
            )

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        yaw = 2.0 * math.atan2(q.z, q.w)

        self.data.append({
            'wall_t': now,
            'x': p.x, 'y': p.y, 'z': p.z,
            'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w,
            'yaw': yaw,
            'vx': t.linear.x, 'vy': t.linear.y,
            'wz': t.angular.z,
        })

    def status_tick(self):
        elapsed = time.monotonic() - self.wall_start
        n = len(self.data)
        if n > 0:
            d = self.data[-1]
            self.get_logger().info(
                f'[{self.label}] t={elapsed:.0f}s msgs={n} '
                f'x={d["x"]:.4f} y={d["y"]:.4f} yaw={math.degrees(d["yaw"]):.1f}°'
            )
        else:
            self.get_logger().warn(
                f'[{self.label}] t={elapsed:.0f}s — NO DATA yet'
            )

        if elapsed >= self.duration:
            self.print_summary()
            raise SystemExit(0)

    def print_summary(self):
        n = len(self.data)
        print(f'\n{"="*60}')
        print(f'VSLAM Odometry Test: {self.label}')
        print(f'{"="*60}')

        if n == 0:
            print(f'  ❌ NO DATA received in {self.duration}s')
            print(f'  cuVSLAM did not publish any odometry.')
            return

        init_delay = self.first_msg_time - self.wall_start
        data_duration = self.data[-1]['wall_t'] - self.data[0]['wall_t']
        rate = (n - 1) / data_duration if data_duration > 0 else 0

        xs = [d['x'] for d in self.data]
        ys = [d['y'] for d in self.data]
        yaws = [d['yaw'] for d in self.data]

        # Frozen detection
        frozen_count = 0
        max_frozen = 0
        cur_frozen = 0
        for i in range(1, n):
            dx = abs(self.data[i]['x'] - self.data[i-1]['x'])
            dy = abs(self.data[i]['y'] - self.data[i-1]['y'])
            if dx < 1e-7 and dy < 1e-7:
                cur_frozen += 1
                max_frozen = max(max_frozen, cur_frozen)
            else:
                if cur_frozen > 5:
                    frozen_count += 1
                cur_frozen = 0

        # Jump detection
        jumps = []
        for i in range(1, n):
            dx = self.data[i]['x'] - self.data[i-1]['x']
            dy = self.data[i]['y'] - self.data[i-1]['y']
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0.05:
                jumps.append(dist)

        print(f'  Duration: {self.duration}s (data: {data_duration:.1f}s)')
        print(f'  Init delay: {init_delay:.1f}s')
        print(f'  Messages: {n}')
        print(f'  Rate: {rate:.1f} Hz')
        print(f'  Position X: {min(xs):.4f} ~ {max(xs):.4f} (range={max(xs)-min(xs):.4f})')
        print(f'  Position Y: {min(ys):.4f} ~ {max(ys):.4f} (range={max(ys)-min(ys):.4f})')
        print(f'  Yaw: {math.degrees(min(yaws)):.1f}° ~ {math.degrees(max(yaws)):.1f}° (range={math.degrees(max(yaws)-min(yaws)):.1f}°)')
        print(f'  First: x={xs[0]:.4f} y={ys[0]:.4f} yaw={math.degrees(yaws[0]):.1f}°')
        print(f'  Last:  x={xs[-1]:.4f} y={ys[-1]:.4f} yaw={math.degrees(yaws[-1]):.1f}°')
        displacement = math.sqrt((xs[-1]-xs[0])**2 + (ys[-1]-ys[0])**2)
        print(f'  Net displacement: {displacement:.4f}m')
        print(f'  Net yaw change: {math.degrees(yaws[-1]-yaws[0]):.1f}°')
        print(f'  Frozen periods (>5 identical): {frozen_count} (max streak: {max_frozen})')
        print(f'  Large jumps (>5cm): {len(jumps)}' + (f' (max: {max(jumps):.3f}m)' if jumps else ''))
        print(f'{"="*60}\n')


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    label = sys.argv[2] if len(sys.argv) > 2 else "test"

    rclpy.init()
    node = VslamOdomTester(duration=duration, label=label)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
