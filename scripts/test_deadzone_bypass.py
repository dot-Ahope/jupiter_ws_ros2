#!/usr/bin/env python3
"""방안A-4 ESC 데드존 스킵 검증 + Nav2 Goal 테스트

토픽 구조:
  Part 1,2: /cmd_vel_nav → velocity_smoother → /cmd_vel → driver (Nav2 경로 사용)
  Part 3:   Nav2 action → controller_server → /cmd_vel_nav → smoother → driver
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math, time, threading, sys

class DeadzoneAndGoalTest(Node):
    def __init__(self):
        super().__init__('deadzone_goal_test')
        # velocity_smoother 입력 토픽으로 publish → smoother → /cmd_vel → driver
        # /cmd_vel 직접 publish하면 smoother(20Hz)와 경합하여 명령 50%+ 손실
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.yaw_vals = []
        self.odom_msgs = []
        self.vslam_count = 0
        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry_adapted',
                                 self.vslam_cb, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.yaw_vals.append((time.time(), math.degrees(yaw)))
        self.odom_msgs.append(msg)
        
    def vslam_cb(self, msg):
        self.vslam_count += 1

    def test_deadzone(self):
        """Part 1: 작은 회전 명령 → velocity_smoother deadband 확인
        
        deadband_velocity[2] = 0.05이므로 0.1 rad/s는 smoother 통과.
        MCU 방안A-4 ESC 데드존 스킵이 작동하면 로봇이 회전해야 함.
        """
        print("\n" + "="*60)
        print("Part 1: ESC 데드존 스킵 테스트 (angular.z = 0.1 rad/s, 3초)")
        print("="*60)
        
        time.sleep(1.0)
        yaw_start = self.yaw_vals[-1][1] if self.yaw_vals else 0.0
        
        twist = Twist()
        twist.angular.z = 0.1  # > deadband(0.05) → smoother 통과 → driver → MCU
        
        print(f"Initial yaw: {yaw_start:.2f} deg")
        print(f"Sending angular.z = 0.1 rad/s (> deadband 0.05) ...")
        expected_deg = math.degrees(0.1 * 3.0)  # 17.2°
        
        t0 = time.time()
        while time.time() - t0 < 3.0:
            self.pub.publish(twist)
            time.sleep(0.05)
        
        # 정지
        twist.angular.z = 0.0
        for _ in range(20):
            self.pub.publish(twist)
            time.sleep(0.05)
        time.sleep(1.0)
        
        yaw_end = self.yaw_vals[-1][1] if self.yaw_vals else 0.0
        yaw_delta = yaw_end - yaw_start
        if yaw_delta > 180: yaw_delta -= 360
        if yaw_delta < -180: yaw_delta += 360
        
        print(f"Final yaw: {yaw_end:.2f} deg")
        print(f"Yaw change: {yaw_delta:.2f} deg (expected: {expected_deg:.1f}°)")
        
        if abs(yaw_delta) > 5.0:
            print(">>> PASS: 방안A-4 ESC 데드존 스킵 작동 — 로봇 회전됨")
            return True
        else:
            print(">>> FAIL: 로봇 미움직임 — ESC 데드존 문제 지속")
            return False

    def test_medium_rotation(self):
        """Part 2: 중간 회전 명령 — ESC 데드존 스킵 + ANGULAR_SCALE_FACTOR 검증
        
        /cmd_vel_nav로 publish → velocity_smoother → /cmd_vel → driver
        expected: 0.5 rad/s × 3s = 85.9°
        """
        print("\n" + "="*60)
        print("Part 2: 중간 회전 테스트 (angular.z = 0.5 rad/s, 3초)")
        print("="*60)
        
        time.sleep(1.0)
        yaw_start = self.yaw_vals[-1][1] if self.yaw_vals else 0.0
        
        twist = Twist()
        twist.angular.z = 0.5
        expected_deg = math.degrees(0.5 * 3.0)  # 85.9°
        
        print(f"Initial yaw: {yaw_start:.2f} deg")
        print(f"Sending angular.z = 0.5 rad/s ... (expected: {expected_deg:.1f}°)")
        
        t0 = time.time()
        while time.time() - t0 < 3.0:
            self.pub.publish(twist)
            time.sleep(0.05)
        
        twist.angular.z = 0.0
        for _ in range(20):
            self.pub.publish(twist)
            time.sleep(0.05)
        time.sleep(1.0)
        
        yaw_end = self.yaw_vals[-1][1] if self.yaw_vals else 0.0
        yaw_delta = yaw_end - yaw_start
        if yaw_delta > 180: yaw_delta -= 360
        if yaw_delta < -180: yaw_delta += 360
        
        accuracy = abs(yaw_delta) / expected_deg * 100 if expected_deg > 0 else 0
        
        print(f"Final yaw: {yaw_end:.2f} deg")
        print(f"Yaw change: {yaw_delta:.2f} deg (accuracy: {accuracy:.0f}%)")
        
        if abs(yaw_delta) < 5.0:
            print(">>> FAIL: 로봇 미움직임")
        elif accuracy > 85:
            print(f">>> PASS: 회전 정확도 {accuracy:.0f}% — ANGULAR_SCALE_FACTOR 적합")
        else:
            suggested = 0.401 * (expected_deg / abs(yaw_delta)) if abs(yaw_delta) > 1 else 0.401
            print(f">>> WARN: 회전 정확도 {accuracy:.0f}% — ANGULAR_SCALE_FACTOR 조정 필요")
            print(f"   현재: 0.401, 제안: {suggested:.3f}")
        return yaw_delta

    def send_nav_goal(self, x, y, yaw_deg, timeout=60.0):
        """Nav2 goal 전송 및 대기"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            print("Nav2 action server not available!")
            return None, 0.0
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        t0 = time.time()
        future = self.nav_client.send_goal_async(goal)
        
        # 배경 spin에서 처리하므로 future.done() 폴링
        while not future.done():
            time.sleep(0.05)
            if time.time() - t0 > 10.0:
                print("Goal send timeout!")
                return 'SEND_TIMEOUT', time.time() - t0
        
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            print("Goal rejected!")
            return 'REJECTED', 0.0
        
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            time.sleep(0.1)
            if time.time() - t0 > timeout:
                goal_handle.cancel_goal_async()
                time.sleep(1.0)
                return 'TIMEOUT', time.time() - t0
        
        result = result_future.result()
        elapsed = time.time() - t0
        
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 'SUCCEEDED', elapsed
        elif status == GoalStatus.STATUS_ABORTED:
            return 'ABORTED', elapsed
        elif status == GoalStatus.STATUS_CANCELED:
            return 'CANCELED', elapsed
        else:
            return f'STATUS_{status}', elapsed

    def test_nav_goals(self):
        """Part 3: Nav2 Goal1 (0.5m 전진) + Goal2 (복귀)"""
        print("\n" + "="*60)
        print("Part 3: Nav2 Goal 테스트")
        print("="*60)
        
        # 현재 위치 기록
        time.sleep(0.5)
        if not self.odom_msgs:
            print("No odom data!")
            return
        
        cur = self.odom_msgs[-1].pose.pose
        cur_x = cur.position.x
        cur_y = cur.position.y
        q = cur.orientation
        cur_yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        
        print(f"Current pose: x={cur_x:.3f}, y={cur_y:.3f}, yaw={cur_yaw:.1f} deg")
        
        # Goal1: 0.5m 전진
        goal1_x = cur_x + 0.5 * math.cos(math.radians(cur_yaw))
        goal1_y = cur_y + 0.5 * math.sin(math.radians(cur_yaw))
        
        print(f"\n--- Goal1: 0.5m forward ({goal1_x:.3f}, {goal1_y:.3f}, {cur_yaw:.1f}) ---")
        
        vslam_before = self.vslam_count
        yaw_before_g1 = [v[1] for v in self.yaw_vals[-1:]] if self.yaw_vals else [0]
        
        status1, time1 = self.send_nav_goal(goal1_x, goal1_y, cur_yaw, timeout=30.0)
        vslam_g1 = self.vslam_count - vslam_before
        
        print(f"Goal1: {status1} in {time1:.1f}s (VSLAM adapted: {vslam_g1})")
        
        time.sleep(2.0)
        
        # Goal2: 원래 위치 복귀
        print(f"\n--- Goal2: Return to ({cur_x:.3f}, {cur_y:.3f}, {cur_yaw:.1f}) ---")
        
        vslam_before = self.vslam_count
        yaw_vals_g2_start = len(self.yaw_vals)
        
        status2, time2 = self.send_nav_goal(cur_x, cur_y, cur_yaw, timeout=60.0)
        
        vslam_g2 = self.vslam_count - vslam_before
        
        # Goal2 중 yaw 분석
        yaw_during_g2 = [v[1] for v in self.yaw_vals[yaw_vals_g2_start:]]
        if yaw_during_g2:
            yaw_min = min(yaw_during_g2)
            yaw_max = max(yaw_during_g2)
            yaw_span = yaw_max - yaw_min
        else:
            yaw_span = 0
        
        # 최종 위치
        if self.odom_msgs:
            final = self.odom_msgs[-1].pose.pose
            dist = math.sqrt((final.position.x - cur_x)**2 + (final.position.y - cur_y)**2)
        else:
            dist = -1
        
        print(f"Goal2: {status2} in {time2:.1f}s (VSLAM adapted: {vslam_g2})")
        print(f"  Yaw span during Goal2: {yaw_span:.1f} deg")
        print(f"  Final distance to start: {dist:.3f}m")
        
        # 결과 요약
        print("\n" + "="*60)
        print("결과 요약")
        print("="*60)
        print(f"Goal1: {status1} {time1:.1f}s | VSLAM: {vslam_g1}")
        print(f"Goal2: {status2} {time2:.1f}s | VSLAM: {vslam_g2} | yaw_span: {yaw_span:.1f}°")
        
        if status2 == 'SUCCEEDED':
            print(">>> Goal2 SUCCESS — 데드존 우회 효과 확인!")
        elif yaw_span < 90:
            print(">>> Goal2 실패했지만 yaw_span < 90° — 회전 통제됨 (이전 359°)")
        else:
            print(f">>> Goal2 실패, yaw_span={yaw_span:.1f}° — 여전히 회전 문제")


def main():
    rclpy.init()
    node = DeadzoneAndGoalTest()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Part 1: 데드존 우회 확인
        dz_ok = node.test_deadzone()
        
        # Part 2: 중간 회전
        node.test_medium_rotation()
        
        if '--skip-nav' not in sys.argv:
            # Part 3: Nav2 Goal
            node.test_nav_goals()
        else:
            print("\n[Nav2 test skipped (--skip-nav)]")
            
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        # 정지 명령
        twist = Twist()
        for _ in range(10):
            node.pub.publish(twist)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
