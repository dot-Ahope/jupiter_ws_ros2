#!/usr/bin/env python3
"""
Nav2 2m 전진+복귀 테스트 (Nav2 파라미터 수정 검증용)
- Goal 1: 현재 heading 방향으로 2.0m 전진
- Goal 2: 시작 위치로 복귀
- 5개 토픽 동시 모니터링 + /rosout adapter 로그 캡처
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import Log

import math
import time
import numpy as np
from collections import defaultdict


class Nav2TwoMeterTest(Node):
    def __init__(self):
        super().__init__('nav2_2m_test')
        self.cb_group = ReentrantCallbackGroup()

        # Action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group)

        # Data storage
        self.ekf_msgs = []
        self.raw_msgs = []
        self.adapted_msgs = []
        self.cov_msgs = []
        self.rosout_msgs = []
        self.cmd_vel_count = 0

        # Subscriptions
        self.create_subscription(
            Odometry, '/odom', self._ekf_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self._raw_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry_adapted', self._adapted_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            PoseWithCovarianceStamped, '/visual_slam/tracking/vo_pose_covariance',
            self._cov_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            Log, '/rosout', self._rosout_cb, 50,
            callback_group=self.cb_group)

        from geometry_msgs.msg import Twist
        self.create_subscription(
            Twist, '/cmd_vel', self._cmdvel_cb, 10,
            callback_group=self.cb_group)

        # State
        self.start_pose = None
        self.start_time = time.time()
        self.goal_results = []

    def _ekf_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.ekf_msgs.append({
            't': time.time() - self.start_time,
            'x': p.x, 'y': p.y, 'yaw': yaw
        })
        if self.start_pose is None:
            self.start_pose = {'x': p.x, 'y': p.y, 'yaw': yaw}

    def _raw_cb(self, msg):
        p = msg.pose.pose.position
        self.raw_msgs.append({
            't': time.time() - self.start_time,
            'x': p.x, 'y': p.y, 'z': p.z
        })

    def _adapted_cb(self, msg):
        p = msg.pose.pose.position
        self.adapted_msgs.append({
            't': time.time() - self.start_time,
            'x': p.x, 'y': p.y
        })

    def _cov_cb(self, msg):
        cov = msg.pose.covariance
        self.cov_msgs.append({
            't': time.time() - self.start_time,
            'cov_xx': cov[0], 'cov_yy': cov[7], 'cov_zz': cov[14]
        })

    def _rosout_cb(self, msg):
        if 'vslam' in msg.name.lower() or 'covariance' in msg.name.lower() or \
           'VSLAM' in msg.msg or 'anomaly' in msg.msg.lower() or \
           'recovery' in msg.msg.lower() or 'frozen' in msg.msg.upper() or \
           'tracking' in msg.msg.lower():
            self.rosout_msgs.append({
                't': time.time() - self.start_time,
                'level': msg.level,
                'name': msg.name,
                'msg': msg.msg
            })

    def _cmdvel_cb(self, msg):
        self.cmd_vel_count += 1

    def send_goal(self, x, y, yaw, label="Goal"):
        """Send navigation goal and wait for result"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return {'label': label, 'result': 'SERVER_UNAVAILABLE', 'duration': 0}

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f'{label}: Sending goal ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')
        t0 = time.time()

        future = self.nav_client.send_goal_async(goal_msg)
        # Wait for goal acceptance
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return {'label': label, 'result': 'REJECTED', 'duration': time.time() - t0}

        self.get_logger().info(f'{label}: Goal ACCEPTED, waiting for result...')

        result_future = goal_handle.get_result_async()
        timeout = 60.0  # 60s max per goal
        while not result_future.done() and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        duration = time.time() - t0

        if not result_future.done():
            # Timeout — cancel
            self.get_logger().warn(f'{label}: TIMEOUT after {duration:.1f}s, canceling...')
            cancel_future = goal_handle.cancel_goal_async()
            t_cancel = time.time()
            while not cancel_future.done() and (time.time() - t_cancel) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            result_status = 'TIMEOUT'
        else:
            result = result_future.result()
            status = result.status
            # action_msgs/GoalStatus: 2=ACCEPTED, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            status_map = {2: 'ACCEPTED', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            result_status = status_map.get(status, f'UNKNOWN({status})')

        # Get current EKF position for distance to goal
        if self.ekf_msgs:
            last = self.ekf_msgs[-1]
            dist_to_goal = math.sqrt((last['x'] - x)**2 + (last['y'] - y)**2)
        else:
            dist_to_goal = -1

        self.get_logger().info(f'{label}: {result_status} in {duration:.1f}s, dist_to_goal={dist_to_goal:.3f}m')

        result_info = {
            'label': label,
            'result': result_status,
            'duration': duration,
            'dist_to_goal': dist_to_goal,
            'target': (x, y, yaw)
        }
        self.goal_results.append(result_info)
        return result_info

    def print_results(self):
        """Print comprehensive test results"""
        total_time = time.time() - self.start_time
        print("\n" + "="*70)
        print("Nav2 2m 전진+복귀 테스트 결과 (파라미터 수정 후)")
        print("="*70)

        # Goal results
        print(f"\n--- Goal Results ---")
        for r in self.goal_results:
            tx, ty, tyaw = r['target']
            print(f"  {r['label']}: {r['result']} | {r['duration']:.1f}s | "
                  f"dist_to_goal={r['dist_to_goal']:.3f}m | "
                  f"target=({tx:.3f}, {ty:.3f})")

        # Message counts
        print(f"\n--- Message Statistics ({total_time:.1f}s) ---")
        print(f"  EKF /odom:    {len(self.ekf_msgs)} msgs ({len(self.ekf_msgs)/total_time:.1f} Hz)")
        print(f"  VSLAM raw:    {len(self.raw_msgs)} msgs ({len(self.raw_msgs)/total_time:.1f} Hz)")
        print(f"  VSLAM adapted:{len(self.adapted_msgs)} msgs ({len(self.adapted_msgs)/total_time:.1f} Hz)")
        print(f"  vo_pose_cov:  {len(self.cov_msgs)} msgs ({len(self.cov_msgs)/total_time:.1f} Hz)")
        print(f"  cmd_vel:      {self.cmd_vel_count} commands")
        if self.raw_msgs:
            passthrough = len(self.adapted_msgs) / len(self.raw_msgs) * 100
            print(f"  Pass-through: {len(self.adapted_msgs)}/{len(self.raw_msgs)} ({passthrough:.1f}%)")

        # EKF travel
        if len(self.ekf_msgs) > 1:
            total_travel = 0
            for i in range(1, len(self.ekf_msgs)):
                dx = self.ekf_msgs[i]['x'] - self.ekf_msgs[i-1]['x']
                dy = self.ekf_msgs[i]['y'] - self.ekf_msgs[i-1]['y']
                total_travel += math.sqrt(dx*dx + dy*dy)
            start = self.ekf_msgs[0]
            end = self.ekf_msgs[-1]
            print(f"\n--- EKF Travel ---")
            print(f"  Start:  ({start['x']:.3f}, {start['y']:.3f}, yaw={math.degrees(start['yaw']):.1f}°)")
            print(f"  End:    ({end['x']:.3f}, {end['y']:.3f}, yaw={math.degrees(end['yaw']):.1f}°)")
            print(f"  Total travel: {total_travel:.3f}m")
            displacement = math.sqrt((end['x']-start['x'])**2 + (end['y']-start['y'])**2)
            print(f"  Net displacement: {displacement:.3f}m (should be ~0 for round-trip)")

        # VSLAM raw jumps
        if len(self.raw_msgs) > 1:
            deltas = []
            for i in range(1, len(self.raw_msgs)):
                dx = self.raw_msgs[i]['x'] - self.raw_msgs[i-1]['x']
                dy = self.raw_msgs[i]['y'] - self.raw_msgs[i-1]['y']
                deltas.append(math.sqrt(dx*dx + dy*dy))
            jumps_03 = sum(1 for d in deltas if d > 0.3)
            jumps_05 = sum(1 for d in deltas if d > 0.5)
            max_delta = max(deltas)
            mean_delta = np.mean(deltas) * 1000  # mm
            print(f"\n--- VSLAM Raw Inter-frame Deltas ---")
            print(f"  Max delta: {max_delta:.4f}m")
            print(f"  Mean delta: {mean_delta:.2f}mm")
            print(f"  Jumps >0.3m: {jumps_03}")
            print(f"  Jumps >0.5m: {jumps_05}")
            print(f"  x range: [{min(m['x'] for m in self.raw_msgs):.3f}, {max(m['x'] for m in self.raw_msgs):.3f}]")

        # Covariance
        if self.cov_msgs:
            cov_xx = [m['cov_xx'] for m in self.cov_msgs]
            print(f"\n--- vo_pose_covariance ---")
            print(f"  cov_xx: mean={np.mean(cov_xx):.6f}, max={max(cov_xx):.6f}, min={min(cov_xx):.6f}")

        # Adapter rosout logs
        print(f"\n--- Adapter Logs ({len(self.rosout_msgs)} events) ---")
        # Categorize
        categories = defaultdict(int)
        for m in self.rosout_msgs:
            msg = m['msg']
            if 'anomaly' in msg.lower() or 'LOST' in msg:
                categories['anomaly'] += 1
            elif 'recovery' in msg.lower():
                categories['recovery'] += 1
            elif 'FROZEN' in msg or 'frozen' in msg:
                categories['frozen'] += 1
            elif 'Stats' in msg:
                categories['stats'] += 1
            else:
                categories['other'] += 1

        for cat, cnt in sorted(categories.items()):
            print(f"  {cat}: {cnt}")

        # Print anomaly/recovery logs in detail
        important = [m for m in self.rosout_msgs
                     if any(k in m['msg'].lower() for k in ['anomaly', 'lost', 'recovery', 'auto-recovery', 'resumed'])]
        if important:
            print(f"\n  [Important Events]")
            for m in important:
                print(f"    t={m['t']:.1f}s: {m['msg'][:120]}")

        # Print last stats log
        stats_logs = [m for m in self.rosout_msgs if 'Stats' in m['msg']]
        if stats_logs:
            print(f"\n  [Last Stats Log]")
            last_stat = stats_logs[-1]
            print(f"    t={last_stat['t']:.1f}s: {last_stat['msg']}")

        # Comparison with previous test (Section 11)
        print(f"\n--- Comparison with Previous Test (Section 11, before param fix) ---")
        print(f"  {'Metric':<25} {'Before':>12} {'After':>12} {'Change':>12}")
        print(f"  {'-'*25} {'-'*12} {'-'*12} {'-'*12}")

        prev_g1 = {'result': 'ABORTED', 'duration': 17.7, 'dist': 1.131}
        prev_g2 = {'result': 'ABORTED', 'duration': 15.9, 'dist': 0.109}

        if len(self.goal_results) >= 1:
            g1 = self.goal_results[0]
            print(f"  {'Goal1 result':<25} {'ABORTED':>12} {g1['result']:>12}")
            print(f"  {'Goal1 duration':<25} {'17.7s':>12} {g1['duration']:.1f}s{' ':>5}")
            print(f"  {'Goal1 dist_to_goal':<25} {'1.131m':>12} {g1['dist_to_goal']:.3f}m{' ':>4}")
        if len(self.goal_results) >= 2:
            g2 = self.goal_results[1]
            print(f"  {'Goal2 result':<25} {'ABORTED':>12} {g2['result']:>12}")
            print(f"  {'Goal2 duration':<25} {'15.9s':>12} {g2['duration']:.1f}s{' ':>5}")
            print(f"  {'Goal2 dist_to_goal':<25} {'0.109m':>12} {g2['dist_to_goal']:.3f}m{' ':>4}")

        print("\n" + "="*70)
        if all(r['result'] == 'SUCCEEDED' for r in self.goal_results):
            print("✅ ABORTED 해소 확인 — Nav2 파라미터 수정 효과 검증됨")
        elif any(r['result'] == 'SUCCEEDED' for r in self.goal_results):
            print("⚠️ 부분 성공 — 일부 goal SUCCEEDED, 추가 분석 필요")
        else:
            print("❌ 여전히 ABORTED — 추가 파라미터 조정 필요")
        print("="*70)


def main():
    rclpy.init()
    node = Nav2TwoMeterTest()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Wait for initial EKF pose
    print("Waiting for EKF pose...")
    t0 = time.time()
    while node.start_pose is None and (time.time() - t0) < 10:
        executor.spin_once(timeout_sec=0.1)

    if node.start_pose is None:
        print("ERROR: No EKF pose received")
        return

    sp = node.start_pose
    print(f"Start pose: ({sp['x']:.3f}, {sp['y']:.3f}, yaw={math.degrees(sp['yaw']):.1f}°)")

    # Goal 1: 2m forward in current heading direction
    g1_x = sp['x'] + 2.0 * math.cos(sp['yaw'])
    g1_y = sp['y'] + 2.0 * math.sin(sp['yaw'])
    g1_yaw = sp['yaw']

    print(f"\n{'='*50}")
    print(f"Goal 1: 2m forward → ({g1_x:.3f}, {g1_y:.3f})")
    print(f"{'='*50}")

    r1 = node.send_goal(g1_x, g1_y, g1_yaw, "Goal1 (2m forward)")

    # Brief pause between goals
    time.sleep(2.0)
    for _ in range(20):
        executor.spin_once(timeout_sec=0.1)

    # Goal 2: return to start
    print(f"\n{'='*50}")
    print(f"Goal 2: Return → ({sp['x']:.3f}, {sp['y']:.3f})")
    print(f"{'='*50}")

    r2 = node.send_goal(sp['x'], sp['y'], sp['yaw'], "Goal2 (return)")

    # Post-test monitoring (5s)
    print("\nPost-test monitoring (5s)...")
    t_post = time.time()
    while time.time() - t_post < 5:
        executor.spin_once(timeout_sec=0.1)

    # Print results
    node.print_results()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
