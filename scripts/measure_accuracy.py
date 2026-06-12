#!/usr/bin/env python3
"""
Jupiter 측정 자동화 도구
============================

목적:
  - 직진 시 좌/우 wheel 비대칭 / lateral drift / IMU drift 의 진짜 원인 식별
  - PID gain 조정으로 해결 가능한지 vs mechanical (wheel diameter 등) 측면인지 판별

Mode:
  1. static_bias   — 정지 시 IMU yaw bias + noise (30초)
  2. straight      — 직진 vx m/s × T초 → lateral / yaw drift 측정
                     (사용자가 줄자로 측정 + 자동 collect 결합)
  3. wheel_dist    — 직진 vx m/s × T초 → 좌/우 wheel cumulative 거리 측정
                     (encoder 적분 vs 실측 비교 — wheel diameter 산출)
  4. straight_multi — straight test N회 반복 + 통계

사용 예:
  # 정지 IMU bias (Robot 정지시키고 실행)
  python3 measure_accuracy.py static_bias

  # 직진 1m 후 lateral drift 측정 (vx=0.15, 7s, ~1m)
  python3 measure_accuracy.py straight --vx 0.15 --duration 7

  # 직진 5회 반복
  python3 measure_accuracy.py straight_multi --vx 0.15 --duration 7 --repeats 5

  # Wheel 거리 측정 (저속 + 측정)
  python3 measure_accuracy.py wheel_dist --vx 0.10 --duration 10

전제:
  - jupiter_bringup driver 실행 중 (cmd_vel 송신, /jupiter/imu, /jupiter/wheel_speeds, /odom publish)
  - nav2 OFF 권장 (cmd_vel 경합 방지) — collision_monitor 는 통과
  - 빈 공간 (회전 반경 + 1.5m 직진 여유)
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """wrap-aware (b - a) 각도 차이, [-pi, pi]"""
    d = b - a
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class MeasureNode(Node):

    def __init__(self, odom_topic='/odom'):
        super().__init__('measure_accuracy')

        # 측정 버퍼 (recording=True 일 때만 쌓임)
        self.imu_data = []        # (t, gx, gy, gz, ax, ay, az)
        self.wheel_data = []      # (t, left, right)
        self.odom_data = []       # (t, x, y, yaw, vx, wz)

        # 최신 메시지 (recording 무관, 항상 갱신) — initial pose 캡처용
        self.latest_imu = None
        self.latest_wheel = None
        self.latest_odom = None

        self.recording = False
        self.start_time = 0.0

        # 직진/회전 추적용 초기 odom pose
        self.odom_initial = None  # (x, y, yaw)

        # Subscribers
        self._odom_topic = odom_topic
        self.create_subscription(Imu, '/jupiter/imu', self.imu_cb, 100)
        self.create_subscription(Float32MultiArray, '/jupiter/wheel_speeds', self.wheel_cb, 50)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # IMU bias (static_bias 모드에서 측정 후 저장)
        self.imu_bias_z = 0.0

    def imu_cb(self, msg):
        # 항상 최신 저장
        self.latest_imu = msg
        if not self.recording:
            return
        t = self.now_rel()
        self.imu_data.append((
            t,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ))

    def wheel_cb(self, msg):
        self.latest_wheel = msg
        if not self.recording:
            return
        if len(msg.data) < 2:
            return
        t = self.now_rel()
        self.wheel_data.append((t, msg.data[0], msg.data[1]))

    def odom_cb(self, msg):
        self.latest_odom = msg
        if not self.recording:
            return
        t = self.now_rel()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.odom_data.append((
            t, x, y, yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ))

    def wait_for_initial_data(self, max_wait=5.0):
        """모든 핵심 topic 의 첫 메시지 받을 때까지 wait"""
        start = time.time()
        while time.time() - start < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_imu is not None and self.latest_odom is not None:
                return True
        return False

    def now_rel(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.start_time

    def reset_buffers(self):
        self.imu_data.clear()
        self.wheel_data.clear()
        self.odom_data.clear()
        self.odom_initial = None

    def start_recording(self):
        self.reset_buffers()
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.recording = True

    def stop_recording(self):
        self.recording = False

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.05)

    def spin_for(self, duration_s):
        """duration_s 동안 callback 처리하면서 wait"""
        end = time.time() + duration_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


# =============================================================
# Mode 1: static_bias
# =============================================================
def run_static_bias(node, duration=30.0):
    print("=" * 60)
    print(f"=== STATIC BIAS — Robot 정지 + IMU/wheel 측정 ({duration:.0f}s) ===")
    print("=" * 60)
    print("Robot 을 정지된 상태로 두고 Enter 누르세요...")
    input()

    # 초기 데이터 받기까지 대기 (subscriber 안정화)
    print("Topic 수신 대기...")
    if not node.wait_for_initial_data(max_wait=5.0):
        print("⚠️ Topic 수신 실패 — topic 확인:")
        print("   ros2 topic hz /jupiter/imu")
        print("   ros2 topic hz /jupiter/wheel_speeds")

    print(f"측정 시작 ({duration:.0f}초)...")
    node.start_recording()
    elapsed_log = 0
    while node.now_rel() < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = node.now_rel()
        if int(elapsed) >= elapsed_log + 5:
            elapsed_log = int(elapsed)
            print(f"  {elapsed_log}s / {duration:.0f}s — IMU samples: {len(node.imu_data)}")
    node.stop_recording()

    print("\n--- 결과 ---")
    if node.imu_data:
        gx_list = [d[1] for d in node.imu_data]
        gy_list = [d[2] for d in node.imu_data]
        gz_list = [d[3] for d in node.imu_data]
        ax_list = [d[4] for d in node.imu_data]
        ay_list = [d[5] for d in node.imu_data]
        az_list = [d[6] for d in node.imu_data]
        n = len(gz_list)

        def stats(arr):
            avg = sum(arr) / n
            rms = math.sqrt(sum((a - avg) ** 2 for a in arr) / n)
            return avg, rms

        avg_gz, rms_gz = stats(gz_list)
        avg_gx, rms_gx = stats(gx_list)
        avg_gy, rms_gy = stats(gy_list)
        avg_az, rms_az = stats(az_list)

        print(f"IMU samples: n={n}")
        print(f"  Angular velocity (정지 시):")
        print(f"    gx avg={math.degrees(avg_gx):+.3f}°/s, RMS={math.degrees(rms_gx):.3f}°/s")
        print(f"    gy avg={math.degrees(avg_gy):+.3f}°/s, RMS={math.degrees(rms_gy):.3f}°/s")
        print(f"    gz avg={math.degrees(avg_gz):+.3f}°/s, RMS={math.degrees(rms_gz):.3f}°/s ← yaw bias")
        print(f"  Linear acceleration:")
        print(f"    az avg={avg_az:+.3f} m/s² (gravity ≈ 9.81)")

        if abs(math.degrees(avg_gz)) < 0.5:
            print("\n  ✓ IMU yaw bias 작음 (<0.5°/s) — sensor 자체는 정상")
        elif abs(math.degrees(avg_gz)) < 2.0:
            print("\n  △ IMU yaw bias 중간 (0.5~2°/s) — 일부 보정 필요할 수 있음")
        else:
            print("\n  ⚠️ IMU yaw bias 큼 (>2°/s) — sensor calibration 또는 IMU mount 문제")

        # 저장 (다음 모드에서 빼기 위해)
        node.imu_bias_z = avg_gz
    else:
        print("IMU data 수신 안 됨!")

    if node.wheel_data:
        wl_list = [d[1] for d in node.wheel_data]
        wr_list = [d[2] for d in node.wheel_data]
        n = len(wl_list)
        avg_l = sum(wl_list) / n
        avg_r = sum(wr_list) / n
        print(f"\nWheel speeds (정지 시): n={n}")
        print(f"  좌 평균: {avg_l:+.4f} m/s")
        print(f"  우 평균: {avg_r:+.4f} m/s")
        if abs(avg_l) > 0.01 or abs(avg_r) > 0.01:
            print("  ⚠️ 정지 시인데 wheel 회전 잔류 — encoder bias 또는 freewheel")


# =============================================================
# Mode 2: straight (직진 1회)
# =============================================================
def run_straight(node, vx=0.15, duration=7.0, interactive=True, prefix=""):
    """
    직진 명령 후 odom + IMU 적분 측정
    사용자가 줄자로 실측한 lateral drift 입력 가능
    """
    if interactive:
        print("=" * 60)
        print(f"=== STRAIGHT DRIVE TEST (vx={vx}, {duration}s) ===")
        print("=" * 60)
        print("준비:")
        print("  1. Robot 시작 위치를 분필/테이프로 마킹")
        print("  2. 직진 경로 전방 정리 (장애물 없음)")
        print("  3. 끝난 후 도착 위치 마킹 + 줄자 측정 준비")
        print("Enter 누르면 시작...")
        input()

    # 초기 odom/imu 받기까지 대기 (subscriber 안정화)
    print(f"{prefix}초기 odom/IMU 수신 대기...")
    if not node.wait_for_initial_data(max_wait=5.0):
        print(f"{prefix}⚠️ 초기 데이터 수신 실패 — topic 확인:")
        print(f"{prefix}   ros2 topic hz {node._odom_topic}")
        print(f"{prefix}   ros2 topic hz /jupiter/imu")
        return None

    # 시작 pose 캡처 (latest_odom 사용)
    om = node.latest_odom
    x0 = om.pose.pose.position.x
    y0 = om.pose.pose.position.y
    yaw0 = quat_to_yaw(om.pose.pose.orientation)
    node.odom_initial = (x0, y0, yaw0)
    print(f"{prefix}시작 odom: x={x0:+.3f}, y={y0:+.3f}, yaw={math.degrees(yaw0):+.2f}°")

    # 측정 시작
    node.start_recording()

    print(f"{prefix}직진 cmd 발사 (vx={vx} m/s, {duration}s)...")
    cmd_rate = 20.0  # Hz
    cmd_period = 1.0 / cmd_rate
    end_time = time.time() + duration
    while time.time() < end_time:
        node.publish_cmd(vx, 0.0)
        rclpy.spin_once(node, timeout_sec=cmd_period)

    # 정지
    print(f"{prefix}정지 명령...")
    for _ in range(10):
        node.publish_cmd(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
    # 잔류 motion 잡기 위해 추가 spin
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.5)

    node.stop_recording()

    # 결과 분석
    print(f"\n{prefix}--- 결과 분석 ---")

    # 1. Odom 상의 lateral drift — recording 중 받은 odom, 부족하면 latest_odom 사용
    if node.odom_data:
        x_end, y_end, yaw_end = node.odom_data[-1][1], node.odom_data[-1][2], node.odom_data[-1][3]
    elif node.latest_odom is not None:
        om = node.latest_odom
        x_end = om.pose.pose.position.x
        y_end = om.pose.pose.position.y
        yaw_end = quat_to_yaw(om.pose.pose.orientation)
        print(f"{prefix}(note: recording 중 odom 미수집, latest_odom 사용)")
    else:
        print(f"{prefix}⚠️ odom 데이터 없음!")
        return None
    dx_odom = x_end - x0
    dy_odom = y_end - y0
    dyaw_odom_deg = math.degrees(angle_diff(yaw0, yaw_end))
    dist_odom = math.hypot(dx_odom, dy_odom)
    # 시작 yaw 기준 frame 으로 변환 (X = 직진 방향, Y = lateral)
    cs = math.cos(yaw0)
    sn = math.sin(yaw0)
    x_local = cs * dx_odom + sn * dy_odom        # forward (시작 heading 기준)
    y_local = -sn * dx_odom + cs * dy_odom       # lateral (왼쪽 양수)
    print(f"{prefix}Odom (EKF) 측정:")
    print(f"{prefix}  이동 거리 X (forward): {x_local:+.3f} m")
    print(f"{prefix}  이동 거리 Y (lateral): {y_local:+.3f} m  ({y_local*100:+.1f} cm)")
    print(f"{prefix}  yaw 변화: {dyaw_odom_deg:+.2f}°")
    if abs(x_local) > 0.01:
        slope = math.degrees(math.atan2(y_local, x_local))
        print(f"{prefix}  진행 각도 (slope = atan(Y/X)): {slope:+.2f}°")

    # 2. IMU 적분 yaw
    if node.imu_data:
        # bias 보정
        bias = node.imu_bias_z
        imu_yaw = 0.0
        imu_t_prev = node.imu_data[0][0]
        for d in node.imu_data[1:]:
            t = d[0]
            gz = d[3] - bias
            dt = t - imu_t_prev
            imu_yaw += gz * dt
            imu_t_prev = t
        imu_yaw_deg = math.degrees(imu_yaw)
        print(f"{prefix}IMU 적분 yaw (bias 보정 후): {imu_yaw_deg:+.2f}° (bias={math.degrees(bias):+.3f}°/s)")
    else:
        imu_yaw_deg = None

    # 3. Wheel 적분 (좌/우 거리)
    wl_dist = 0.0
    wr_dist = 0.0
    if node.wheel_data:
        wt_prev = node.wheel_data[0][0]
        for d in node.wheel_data[1:]:
            t = d[0]
            wl = d[1]
            wr = d[2]
            dt = t - wt_prev
            wl_dist += abs(wl) * dt
            wr_dist += abs(wr) * dt
            wt_prev = t
        print(f"{prefix}Wheel 적분 거리:")
        print(f"{prefix}  좌 wheel cumulative: {wl_dist:.3f} m")
        print(f"{prefix}  우 wheel cumulative: {wr_dist:.3f} m")
        diff_pct = 100 * (wl_dist - wr_dist) / max(wl_dist, wr_dist, 0.001)
        print(f"{prefix}  좌-우 차: {wl_dist - wr_dist:+.4f} m ({diff_pct:+.2f}%)")

    # 4. 사용자 실측 입력
    real_dx = None
    real_dy = None
    real_yaw = None
    if interactive:
        print(f"\n{prefix}--- 실측 입력 (생략 시 Enter) ---")
        try:
            s = input(f"{prefix}실제 X 이동거리 (m, 줄자 측정): ").strip()
            real_dx = float(s) if s else None
        except ValueError:
            real_dx = None
        try:
            s = input(f"{prefix}실제 Y lateral drift (m, 좌+/우-): ").strip()
            real_dy = float(s) if s else None
        except ValueError:
            real_dy = None
        try:
            s = input(f"{prefix}실제 yaw 변화 (°, 각도기 측정): ").strip()
            real_yaw = float(s) if s else None
        except ValueError:
            real_yaw = None

        if real_dx is not None:
            print(f"\n{prefix}--- 실측 vs 측정 비교 ---")
            err_x = x_local - real_dx
            print(f"{prefix}  X 차이 (odom - 실측): {err_x:+.3f} m ({100*err_x/real_dx:+.1f}%)")
            if real_dy is not None:
                err_y = y_local - real_dy
                print(f"{prefix}  Y 차이 (odom - 실측): {err_y:+.3f} m")
            if real_yaw is not None and imu_yaw_deg is not None:
                err_yaw_imu = imu_yaw_deg - real_yaw
                err_yaw_odom = dyaw_odom_deg - real_yaw
                print(f"{prefix}  yaw 차이 (IMU - 실측): {err_yaw_imu:+.2f}°")
                print(f"{prefix}  yaw 차이 (odom - 실측): {err_yaw_odom:+.2f}°")

    return {
        'x_local': x_local,
        'y_local': y_local,
        'dyaw_odom_deg': dyaw_odom_deg,
        'imu_yaw_deg': imu_yaw_deg,
        'wl_dist': wl_dist,
        'wr_dist': wr_dist,
        'wheel_diff_pct': 100 * (wl_dist - wr_dist) / max(wl_dist, wr_dist, 0.001) if wl_dist and wr_dist else None,
        'real_dx': real_dx,
        'real_dy': real_dy,
        'real_yaw': real_yaw,
    }


# =============================================================
# Mode 3: wheel_dist (저속 직진 + wheel cumulative)
# =============================================================
def run_wheel_dist(node, vx=0.10, duration=10.0):
    print("=" * 60)
    print(f"=== WHEEL DISTANCE CALIBRATION ===")
    print("=" * 60)
    print("목적: 좌/우 wheel 의 encoder 적분 거리 vs 실측 거리 비교")
    print(f"  cmd vx={vx} m/s × {duration}s = 예상 {vx*duration:.2f} m 이동")
    print("준비:")
    print("  1. Robot 시작 위치 마킹")
    print("  2. 끝난 후 X 방향 실제 이동거리 + Y 변위 측정")
    print("Enter 누르면 시작...")
    input()

    result = run_straight(node, vx=vx, duration=duration, interactive=True,
                          prefix="  ")

    if result and result.get('real_dx'):
        print(f"\n=== Wheel diameter correction 계산 ===")
        # encoder 가 보고하는 좌측 누적 거리 vs 실측
        # 만약 encoder 가 wheel 회전수 기반이고 wheel_diameter 가 고정이면:
        #   reported_distance = pulses × (2π × wheel_radius / pulses_per_rev)
        # 실측 / reported 비율 = correction factor
        if result['wl_dist'] > 0 and result['wr_dist'] > 0:
            # 평균 wheel cumulative (좌/우)
            avg_wheel = (result['wl_dist'] + result['wr_dist']) / 2
            ratio = result['real_dx'] / avg_wheel
            print(f"  Wheel encoder 적분 평균: {avg_wheel:.3f} m")
            print(f"  실측 X 거리: {result['real_dx']:.3f} m")
            print(f"  실측/encoder ratio: {ratio:.4f}")
            if abs(ratio - 1.0) < 0.05:
                print(f"  ✓ Wheel calibration 양호 (오차 <5%)")
            else:
                print(f"  ⚠️ Wheel calibration 오차 큼 — circumference 또는 pulse/rev 조정 필요")
                print(f"     현재 firmware: 1320 pulse/rev, 204mm circle")
                new_circle = 204 * ratio
                print(f"     제안: circle 204mm → {new_circle:.1f}mm")


# =============================================================
# Mode 4: straight_multi (N 회 반복 + 통계)
# =============================================================
def run_straight_multi(node, vx=0.15, duration=7.0, repeats=5):
    print("=" * 60)
    print(f"=== STRAIGHT DRIVE × {repeats} RUNS (vx={vx}, {duration}s each) ===")
    print("=" * 60)
    print("Tips:")
    print("  - 매 run 마다 robot 을 시작 위치로 복귀시키기 (수동 또는 직접)")
    print("  - 일관된 환경 / 시작 yaw 유지")
    print("Enter 누르면 시작...")
    input()

    results = []
    for i in range(repeats):
        print(f"\n\n{'='*40}")
        print(f"Run {i+1}/{repeats}")
        print('=' * 40)
        if i > 0:
            print("Robot 을 시작 위치로 복귀시키고 Enter...")
            input()
        r = run_straight(node, vx=vx, duration=duration, interactive=False,
                         prefix=f"  [Run {i+1}] ")
        if r:
            results.append(r)

    # 통계
    print(f"\n\n{'='*60}")
    print("=== 종합 통계 ===")
    print('=' * 60)
    if not results:
        print("결과 없음")
        return

    n = len(results)
    print(f"\nRun 별 결과:")
    print(f"{'#':>3s} {'X(m)':>7s} {'Y(m)':>7s} {'Y(cm)':>7s} {'odom_yaw':>9s} {'imu_yaw':>9s} {'WL-WR%':>8s}")
    print("-" * 60)
    for i, r in enumerate(results):
        imu_yaw = f"{r['imu_yaw_deg']:+.2f}" if r.get('imu_yaw_deg') is not None else "  n/a"
        wd = f"{r['wheel_diff_pct']:+.2f}" if r.get('wheel_diff_pct') is not None else "  n/a"
        print(f"{i+1:>3d} {r['x_local']:+7.3f} {r['y_local']:+7.3f} {r['y_local']*100:+7.1f} "
              f"{r['dyaw_odom_deg']:+9.2f} {imu_yaw:>9s} {wd:>8s}")

    print(f"\n평균 / RMS:")
    for key, name in [('x_local', 'X 이동 (m)'),
                       ('y_local', 'Y lateral (m)'),
                       ('dyaw_odom_deg', 'odom yaw (°)'),
                       ('imu_yaw_deg', 'IMU yaw (°)'),
                       ('wheel_diff_pct', 'WL-WR (%)')]:
        vals = [r[key] for r in results if r.get(key) is not None]
        if not vals:
            continue
        avg = sum(vals) / len(vals)
        rms = math.sqrt(sum((v - avg) ** 2 for v in vals) / len(vals))
        print(f"  {name:<20s}: avg={avg:+.4f}, RMS={rms:.4f}, n={len(vals)}")

    # 진단
    ys = [r['y_local'] for r in results]
    avg_y = sum(ys) / len(ys)
    rms_y = math.sqrt(sum((y - avg_y) ** 2 for y in ys) / len(ys))
    print(f"\n--- 진단 ---")
    if abs(avg_y) > 0.05:
        print(f"  Y lateral drift 평균 {avg_y*100:+.1f} cm — **systematic bias** (mechanical/calibration 원인)")
    elif rms_y > 0.05:
        print(f"  Y lateral RMS {rms_y*100:.1f} cm 큼 — measurement noise, 안정성 부족")
    else:
        print(f"  Y lateral 안정 (avg={avg_y*100:+.1f}cm, RMS={rms_y*100:.1f}cm) — drift 없음")


# =============================================================
# Main
# =============================================================
def main():
    parser = argparse.ArgumentParser(description='Jupiter 정확도 측정 자동화 도구')
    parser.add_argument('mode', choices=['static_bias', 'straight', 'straight_multi', 'wheel_dist'],
                        help='측정 모드')
    parser.add_argument('--vx', type=float, default=0.15, help='직진 속도 (m/s)')
    parser.add_argument('--duration', type=float, default=7.0, help='직진 시간 (s)')
    parser.add_argument('--repeats', type=int, default=5, help='반복 횟수 (multi mode)')
    parser.add_argument('--static-duration', type=float, default=30.0, help='정지 측정 시간 (s)')
    parser.add_argument('--odom-topic', type=str, default='/odom', help='odom topic (default /odom)')
    args = parser.parse_args()

    rclpy.init()
    node = MeasureNode(odom_topic=args.odom_topic)
    try:
        if args.mode == 'static_bias':
            run_static_bias(node, duration=args.static_duration)
        elif args.mode == 'straight':
            run_straight(node, vx=args.vx, duration=args.duration, interactive=True)
        elif args.mode == 'straight_multi':
            run_straight_multi(node, vx=args.vx, duration=args.duration,
                               repeats=args.repeats)
        elif args.mode == 'wheel_dist':
            run_wheel_dist(node, vx=args.vx, duration=args.duration)
    except KeyboardInterrupt:
        print("\n인터럽트 — 정지...")
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
