#!/usr/bin/env python3
"""
Direction D 테스트: RF2O 회전인식 동적 공분산 + IMU gyro_scale 1.025
=================================================================

변경사항 (vs Direction C):
  1. rf2o_covariance_adapter: 정적 0.5 → 회전 인식 동적 (직진 0.5, 회전 10.0)
  2. imu_gyro_scale: 1.0 → 1.025 (자이로 2.3% 과소측정 보정)

테스트 항목:
  A. 0.5m 전진 + 복귀 (기존 Direction C 비교)
  B. 180° 회전 후 yaw 정확도 (센서별)
  C. 회전 후 0.5m 직진의 좌측편향 측정

기대효과:
  - 180° 회전 후 EKF yaw 오차: -3.91° → < -2°
  - 회전 후 직진 좌측편향: 개선
  - Goal1 시간: 6.3s 유지 (직진 성능 보존)
  - Goal2 시간: 11.8s → 개선 (회전 후 수렴 빨라짐)

사용법:
  1. nav2_vslam_fused.launch.py 재시작 (방향 D 설정 적용)
  2. 로봇을 직선으로 0.5m 이상 여유 있는 곳에 배치
  3. python3 ~/jupiter_ws_ros2/scripts/test_direction_d.py

Direction C 기준선 (비교용):
  Goal1: SUCCEEDED 6.3s, lat_err=-0.003m (0.5m 직진)
  Goal2: SUCCEEDED 11.8s, EKF yaw oscillation +48°
  180° 후 yaw 오차: -3.91° (보정 전)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

import math
import time
import numpy as np


class DirectionDTest(Node):
    def __init__(self):
        super().__init__('direction_d_test')
        self.cb = ReentrantCallbackGroup()

        # QoS
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # Action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)

        # Data storage
        self.ekf = []       # /odom (EKF output)
        self.wheel = []     # /odom_raw (wheel encoder)
        self.rf2o = []      # /odom_rf2o (RF2O raw)
        self.rf2o_ada = []  # /odom_rf2o_adapted (RF2O dynamic cov)
        self.vslam = []     # /visual_slam/tracking/odometry
        self.cmds = []      # /cmd_vel

        self.t0 = time.time()
        self.start_pose = None
        self.goal_results = []

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self._ekf_cb, 10, callback_group=self.cb)
        self.create_subscription(Odometry, '/odom_raw', self._wheel_cb, odom_qos, callback_group=self.cb)
        self.create_subscription(Odometry, '/odom_rf2o', self._rf2o_cb, odom_qos, callback_group=self.cb)
        self.create_subscription(Odometry, '/odom_rf2o_adapted', self._rf2o_ada_cb, 10, callback_group=self.cb)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                                 self._vslam_cb, odom_qos, callback_group=self.cb)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10, callback_group=self.cb)

        # cmd_vel publisher for rotation test
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _yaw(self, msg):
        q = msg.pose.pose.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def _odom_data(self, msg):
        p = msg.pose.pose.position
        return {'t': time.time()-self.t0, 'x': p.x, 'y': p.y, 'yaw': self._yaw(msg),
                'wz': msg.twist.twist.angular.z, 'vx': msg.twist.twist.linear.x}

    def _cov_data(self, msg):
        d = self._odom_data(msg)
        d['cov_x'] = msg.pose.covariance[0]
        d['cov_y'] = msg.pose.covariance[7]
        d['cov_yaw'] = msg.pose.covariance[35]
        return d

    def _ekf_cb(self, msg):
        d = self._odom_data(msg)
        self.ekf.append(d)
        if self.start_pose is None:
            self.start_pose = d.copy()

    def _wheel_cb(self, msg): self.wheel.append(self._odom_data(msg))
    def _rf2o_cb(self, msg): self.rf2o.append(self._odom_data(msg))
    def _rf2o_ada_cb(self, msg): self.rf2o_ada.append(self._cov_data(msg))
    def _vslam_cb(self, msg): self.vslam.append(self._odom_data(msg))
    def _cmd_cb(self, msg): self.cmds.append({'t': time.time()-self.t0, 'vx': msg.linear.x, 'wz': msg.angular.z})

    # ── Navigation ──
    def send_goal(self, x, y, yaw, label, timeout=60.0):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server not available')
            return {'label': label, 'result': 'NO_SERVER', 'duration': 0, 'dist': -1}

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f'{label}: goal ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')
        t0 = time.time()

        future = self.nav_client.send_goal_async(goal)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        handle = future.result()
        if not handle.accepted:
            return {'label': label, 'result': 'REJECTED', 'duration': 0, 'dist': -1}

        res_future = handle.get_result_async()
        while not res_future.done() and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        dur = time.time() - t0
        if not res_future.done():
            handle.cancel_goal_async()
            status = 'TIMEOUT'
        else:
            s = res_future.result().status
            status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(s, f'UNKNOWN({s})')

        dist = -1
        if self.ekf:
            last = self.ekf[-1]
            dist = math.sqrt((last['x']-x)**2 + (last['y']-y)**2)

        r = {'label': label, 'result': status, 'duration': dur, 'dist': dist}
        self.goal_results.append(r)
        self.get_logger().info(f'{label}: {status} in {dur:.1f}s, dist={dist:.3f}m')
        return r

    # ── 180° Rotation Test ──
    def rotation_test(self, executor):
        """순수 회전 테스트: cmd_vel로 180° 회전, 센서별 yaw 정확도 측정"""
        print("\n" + "="*60)
        print("테스트 B: 180° 회전 yaw 정확도 측정")
        print("="*60)

        # 현재 상태 기록
        for _ in range(20):
            executor.spin_once(timeout_sec=0.05)

        if not self.ekf:
            print("ERROR: No EKF data")
            return {}

        e0 = self.ekf[-1]
        w0 = self.wheel[-1] if self.wheel else None
        r0 = self.rf2o[-1] if self.rf2o else None
        v0 = self.vslam[-1] if self.vslam else None
        ra0 = self.rf2o_ada[-1] if self.rf2o_ada else None

        target_yaw = e0['yaw'] + math.pi
        if target_yaw > math.pi: target_yaw -= 2 * math.pi

        print(f"  시작 EKF yaw: {math.degrees(e0['yaw']):.2f}°")
        print(f"  목표 yaw: {math.degrees(target_yaw):.2f}° (+180°)")

        # 회전 실행
        rot_start = time.time()
        rotation_wz = 0.5  # rad/s CCW

        while time.time() - rot_start < 10.0:
            executor.spin_once(timeout_sec=0.02)
            if self.ekf:
                cur = self.ekf[-1]['yaw']
                diff = abs(cur - target_yaw)
                if diff > math.pi: diff = 2*math.pi - diff
                if diff < 0.10 and time.time()-rot_start > 2.0:  # within ~5.7°
                    break

            tw = Twist()
            tw.angular.z = rotation_wz
            self.cmd_pub.publish(tw)

        # 정지
        tw = Twist()
        self.cmd_pub.publish(tw)
        time.sleep(0.5)
        for _ in range(30):
            executor.spin_once(timeout_sec=0.05)

        rot_dur = time.time() - rot_start
        e1 = self.ekf[-1]

        # 2초 후 안정화
        ts = time.time()
        while time.time()-ts < 2.0:
            executor.spin_once(timeout_sec=0.05)
        e2 = self.ekf[-1]

        # 센서별 yaw 변화 측정
        sensors = {}

        # EKF
        ekf_change = math.degrees(e2['yaw'] - e0['yaw'])
        ekf_error = math.degrees(e2['yaw'] - target_yaw)
        if abs(ekf_error) > 180: ekf_error -= 360 * (1 if ekf_error > 0 else -1)
        sensors['EKF'] = {'change': ekf_change, 'error': ekf_error, 'final': math.degrees(e2['yaw'])}

        # Other sensors
        for name, data, d0 in [('Wheel', self.wheel, w0), ('RF2O', self.rf2o, r0),
                                ('VSLAM', self.vslam, v0)]:
            if d0 and data:
                d1 = data[-1]
                change = math.degrees(d1['yaw'] - d0['yaw'])
                sensors[name] = {'change': change, 'error': change - 180.0,
                                 'final': math.degrees(d1['yaw'])}

        # RF2O adapted covariance during rotation
        rf2o_cov_during_rot = []
        if ra0 and self.rf2o_ada:
            t_rot_start = rot_start - (time.time() - self.t0 - (time.time() - rot_start - self.t0))
            # Simple: just get recent rf2o_ada covs
            for d in self.rf2o_ada[-50:]:
                rf2o_cov_during_rot.append(d.get('cov_x', 0))

        # 결과 출력
        print(f"\n  회전 소요: {rot_dur:.1f}s")
        print(f"  EKF yaw 즉시: {math.degrees(e1['yaw']):.2f}°, 2초 후: {math.degrees(e2['yaw']):.2f}°")
        print(f"  안정화 drift: {math.degrees(e2['yaw']-e1['yaw']):+.2f}° (2s)")
        print(f"\n  {'센서':<10} {'yaw 변화':>12} {'오차(180° 대비)':>16} {'최종 yaw':>12}")
        print(f"  {'-'*10} {'-'*12} {'-'*16} {'-'*12}")
        for name in ['EKF', 'Wheel', 'RF2O', 'VSLAM']:
            if name in sensors:
                s = sensors[name]
                print(f"  {name:<10} {s['change']:+12.2f}° {s['error']:+16.2f}° {s['final']:12.2f}°")

        # RF2O adapted cov
        if rf2o_cov_during_rot:
            print(f"\n  RF2O adapted pose_cov_x (최근 50): "
                  f"min={min(rf2o_cov_during_rot):.3f}, max={max(rf2o_cov_during_rot):.3f}, "
                  f"mean={np.mean(rf2o_cov_during_rot):.3f}")

        return sensors

    # ── Analysis ──
    def analyze_straight_segments(self):
        """직진 구간의 좌측편향 분석"""
        if not self.ekf or not self.start_pose:
            return

        sp = self.start_pose
        heading = sp['yaw']

        print(f"\n--- 직진 구간 좌측편향 분석 ---")

        # Goal1 방향 단위벡터
        fwd = np.array([math.cos(heading), math.sin(heading)])
        lat = np.array([-math.sin(heading), math.cos(heading)])  # left-positive

        # 전체 경로에서 전진 구간 추출 (vx > 0.1)
        for ekf_d in self.ekf:
            if abs(ekf_d.get('vx', 0)) > 0.05:
                pos = np.array([ekf_d['x'] - sp['x'], ekf_d['y'] - sp['y']])
                fwd_dist = np.dot(pos, fwd)
                lat_dist = np.dot(pos, lat)

                # 0.5m 지점 ± 0.05m
                if abs(fwd_dist - 0.5) < 0.05:
                    print(f"  [0.5m 지점] lateral={lat_dist*1000:+.1f}mm "
                          f"(+:좌측, -:우측)")
                    break

    def analyze_cmd_vel(self):
        """cmd_vel 분석: 회전 중 wz 패턴"""
        if not self.cmds:
            return

        # 회전 구간 (|wz| > 0.3)
        rot_cmds = [c for c in self.cmds if abs(c['wz']) > 0.3]
        straight_cmds = [c for c in self.cmds if abs(c['vx']) > 0.05 and abs(c['wz']) < 0.1]

        print(f"\n--- cmd_vel 분석 ---")
        if rot_cmds:
            wz_vals = [c['wz'] for c in rot_cmds]
            print(f"  회전 구간: {len(rot_cmds)} cmds, "
                  f"wz mean={np.mean(wz_vals)*1000:.1f} mrad/s, "
                  f"max={max(wz_vals)*1000:.1f} mrad/s")
        if straight_cmds:
            wz_vals = [c['wz'] for c in straight_cmds]
            print(f"  직진 구간: {len(straight_cmds)} cmds, "
                  f"wz mean={np.mean(wz_vals)*1000:.1f} mrad/s (편향 지표)")

    def print_comparison(self):
        """Direction C vs D 비교표"""
        print("\n" + "="*70)
        print("Direction D 결과: RF2O 회전인식 동적공분산 + IMU 보정")
        print("="*70)

        print(f"\n{'metric':<30} {'Direction C':>15} {'Direction D':>15} {'변화':>10}")
        print(f"{'-'*30} {'-'*15} {'-'*15} {'-'*10}")

        # Goal results
        c_baseline = [
            ('Goal1 result', 'SUCCEEDED', None),
            ('Goal1 duration', '6.3s', None),
            ('Goal2 result', 'SUCCEEDED', None),
            ('Goal2 duration', '11.8s', None),
            ('180° yaw error', '-3.91°', None),
        ]

        for i, (metric, c_val, _) in enumerate(c_baseline):
            d_val = ''
            delta = ''
            if i < 2 and len(self.goal_results) >= 1:
                g = self.goal_results[0]
                if i == 0:
                    d_val = g['result']
                elif i == 1:
                    d_val = f"{g['duration']:.1f}s"
                    try:
                        delta = f"{g['duration'] - 6.3:+.1f}s"
                    except:
                        pass
            elif i < 4 and len(self.goal_results) >= 2:
                g = self.goal_results[1]
                if i == 2:
                    d_val = g['result']
                elif i == 3:
                    d_val = f"{g['duration']:.1f}s"
                    try:
                        delta = f"{g['duration'] - 11.8:+.1f}s"
                    except:
                        pass
            print(f"  {metric:<28} {c_val:>15} {d_val:>15} {delta:>10}")

        # Data counts
        total = time.time() - self.t0
        print(f"\n--- 데이터 수신 ({total:.1f}s) ---")
        for name, data in [('EKF /odom', self.ekf), ('Wheel /odom_raw', self.wheel),
                           ('RF2O /odom_rf2o', self.rf2o), ('RF2O adapted', self.rf2o_ada),
                           ('VSLAM raw', self.vslam), ('cmd_vel', self.cmds)]:
            hz = len(data)/total if total > 0 else 0
            print(f"  {name:<20}: {len(data):>5} msgs ({hz:.1f} Hz)")

        # RF2O adapted covariance distribution
        if self.rf2o_ada:
            covs = [d['cov_x'] for d in self.rf2o_ada if 'cov_x' in d]
            if covs:
                print(f"\n--- RF2O Adapted 공분산 분포 ---")
                bins = [(0, 0.6, 'base(≤0.6)'), (0.6, 2, 'low(0.6-2)'),
                        (2, 5, 'mid(2-5)'), (5, 100, 'high(5+)')]
                for lo, hi, label in bins:
                    cnt = sum(1 for c in covs if lo <= c < hi)
                    print(f"  {label:<15}: {cnt:>5} ({cnt/len(covs)*100:.1f}%)")
                print(f"  Range: [{min(covs):.3f}, {max(covs):.3f}]")

        self.analyze_straight_segments()
        self.analyze_cmd_vel()

        # EKF travel
        if len(self.ekf) > 1:
            s = self.ekf[0]
            e = self.ekf[-1]
            disp = math.sqrt((e['x']-s['x'])**2 + (e['y']-s['y'])**2)
            print(f"\n--- EKF 경로 ---")
            print(f"  시작: ({s['x']:.3f}, {s['y']:.3f}, yaw={math.degrees(s['yaw']):.1f}°)")
            print(f"  종료: ({e['x']:.3f}, {e['y']:.3f}, yaw={math.degrees(e['yaw']):.1f}°)")
            print(f"  순변위: {disp:.3f}m (왕복이면 ~0)")

            # yaw 전체 범위
            yaws = [math.degrees(d['yaw']) for d in self.ekf]
            print(f"  yaw 범위: [{min(yaws):.1f}°, {max(yaws):.1f}°] "
                  f"(span={max(yaws)-min(yaws):.1f}°)")

        print("\n" + "="*70)
        if all(r['result'] == 'SUCCEEDED' for r in self.goal_results):
            print("✅ Direction D: 모든 Goal SUCCEEDED")
        elif any(r['result'] == 'SUCCEEDED' for r in self.goal_results):
            print("⚠️ Direction D: 부분 성공")
        else:
            print("❌ Direction D: 실패 — 추가 조정 필요")
        print("="*70)


def main():
    rclpy.init()
    node = DirectionDTest()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # EKF 대기
    print("EKF pose 대기 중...")
    t0 = time.time()
    while node.start_pose is None and time.time()-t0 < 10:
        executor.spin_once(timeout_sec=0.1)

    if not node.start_pose:
        print("ERROR: EKF pose 없음")
        rclpy.shutdown()
        return

    sp = node.start_pose
    print(f"시작: ({sp['x']:.3f}, {sp['y']:.3f}, yaw={math.degrees(sp['yaw']):.1f}°)\n")

    # =============================================
    # 테스트 A: Nav2 0.5m 전진 + 복귀 (Direction C 비교)
    # =============================================
    print("="*60)
    print("테스트 A: Nav2 0.5m 전진 + 복귀")
    print("="*60)

    g1_x = sp['x'] + 0.5 * math.cos(sp['yaw'])
    g1_y = sp['y'] + 0.5 * math.sin(sp['yaw'])
    node.send_goal(g1_x, g1_y, sp['yaw'], "Goal1 (0.5m 전진)")

    time.sleep(2.0)
    for _ in range(20):
        executor.spin_once(timeout_sec=0.1)

    node.send_goal(sp['x'], sp['y'], sp['yaw'], "Goal2 (복귀)")

    time.sleep(2.0)
    for _ in range(20):
        executor.spin_once(timeout_sec=0.1)

    # =============================================
    # 테스트 B: 180° 회전 yaw 정확도
    # =============================================
    rotation_result = node.rotation_test(executor)

    time.sleep(2.0)
    for _ in range(20):
        executor.spin_once(timeout_sec=0.1)

    # =============================================
    # 결과 출력
    # =============================================
    node.print_comparison()

    # 180° rotation 결과 별도 출력
    if rotation_result:
        print(f"\n--- 180° 회전 비교 (Direction C→D) ---")
        print(f"  {'센서':<10} {'C: yaw 오차':>15} {'D: yaw 오차':>15}")
        print(f"  {'-'*10} {'-'*15} {'-'*15}")
        c_errors = {'EKF': -3.91, 'Wheel': -3.63, 'RF2O': -4.55, 'VSLAM': -7.35}
        for name in ['EKF', 'Wheel', 'RF2O', 'VSLAM']:
            c_err = f"{c_errors.get(name, 'N/A'):+.2f}°" if name in c_errors else 'N/A'
            d_err = f"{rotation_result[name]['error']:+.2f}°" if name in rotation_result else 'N/A'
            print(f"  {name:<10} {c_err:>15} {d_err:>15}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
