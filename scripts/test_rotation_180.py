#!/usr/bin/env python3
"""
180° 제자리 회전 테스트 + 데이터 수집 스크립트
현재 위치에서 heading만 180° 뒤로 회전하는 goal 전송 후,
cmd_vel, odom, cmd_vel_nav 데이터를 실시간 수집하여 CSV로 저장
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math, time, csv, os, signal, sys
from collections import deque

OUTPUT_DIR = "/home/jetson/jupiter_ws_ros2/test_results"
TIMEOUT = 60.0  # seconds

def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def yaw_to_quat(yaw):
    """Returns (x, y, z, w)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class RotationTestCollector(Node):
    def __init__(self):
        super().__init__('rotation_test_collector')
        
        # Data storage
        self.data_rows = []
        self.start_time = None
        self.initial_yaw = None
        self.target_yaw = None
        self.goal_done = False
        self.goal_result = None
        
        # Latest values
        self.latest_odom = None
        self.latest_cmd_vel = None
        self.latest_cmd_vel_nav = None
        self.latest_vel_feedback = None
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_cb, 10)
        self.create_subscription(Twist, '/jupiter/get_vel', self.vel_feedback_cb, 10)
        
        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer for data collection (50Hz)
        self.collect_timer = self.create_timer(0.02, self.collect_data)
        
        self.get_logger().info("Waiting for odom...")
        
    def odom_cb(self, msg):
        self.latest_odom = msg
        if self.initial_yaw is None:
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            self.initial_yaw = yaw
            self.get_logger().info(f"Initial yaw: {math.degrees(yaw):.1f} deg")
            
    def cmd_vel_cb(self, msg):
        self.latest_cmd_vel = msg
        
    def cmd_vel_nav_cb(self, msg):
        self.latest_cmd_vel_nav = msg
        
    def vel_feedback_cb(self, msg):
        self.latest_vel_feedback = msg
        
    def collect_data(self):
        if self.start_time is None or self.latest_odom is None:
            return
            
        t = time.time() - self.start_time
        odom = self.latest_odom
        yaw = quat_to_yaw(odom.pose.pose.orientation)
        
        row = {
            'time': round(t, 3),
            'odom_x': round(odom.pose.pose.position.x, 4),
            'odom_y': round(odom.pose.pose.position.y, 4),
            'odom_yaw_deg': round(math.degrees(yaw), 2),
            'odom_vx': round(odom.twist.twist.linear.x, 4),
            'odom_wz': round(odom.twist.twist.angular.z, 4),
            'cmd_vel_vx': 0.0,
            'cmd_vel_wz': 0.0,
            'cmd_vel_nav_vx': 0.0,
            'cmd_vel_nav_wz': 0.0,
            'vel_fb_vx': 0.0,
            'vel_fb_wz': 0.0,
        }
        
        if self.latest_cmd_vel:
            row['cmd_vel_vx'] = round(self.latest_cmd_vel.linear.x, 4)
            row['cmd_vel_wz'] = round(self.latest_cmd_vel.angular.z, 4)
        if self.latest_cmd_vel_nav:
            row['cmd_vel_nav_vx'] = round(self.latest_cmd_vel_nav.linear.x, 4)
            row['cmd_vel_nav_wz'] = round(self.latest_cmd_vel_nav.angular.z, 4)
        if self.latest_vel_feedback:
            row['vel_fb_vx'] = round(self.latest_vel_feedback.linear.x, 4)
            row['vel_fb_wz'] = round(self.latest_vel_feedback.angular.z, 4)
            
        # Compute heading error to target
        if self.target_yaw is not None:
            err = self.target_yaw - yaw
            # normalize to [-pi, pi]
            while err > math.pi: err -= 2*math.pi
            while err < -math.pi: err += 2*math.pi
            row['heading_error_deg'] = round(math.degrees(err), 2)
        else:
            row['heading_error_deg'] = 0.0
            
        self.data_rows.append(row)
        
    def send_goal(self):
        if self.initial_yaw is None:
            self.get_logger().error("No initial yaw!")
            return
            
        # Compute 180° rotated target
        self.target_yaw = self.initial_yaw + math.pi
        if self.target_yaw > math.pi:
            self.target_yaw -= 2 * math.pi
            
        self.get_logger().info(f"Sending 180° rotation goal:")
        self.get_logger().info(f"  Current yaw: {math.degrees(self.initial_yaw):.1f} deg")
        self.get_logger().info(f"  Target yaw:  {math.degrees(self.target_yaw):.1f} deg")
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available")
            return
            
        # Create goal at current position with 180° rotated heading
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Use current position from odom (in map frame this might differ slightly)
        goal.pose.pose.position.x = self.latest_odom.pose.pose.position.x
        goal.pose.pose.position.y = self.latest_odom.pose.pose.position.y
        goal.pose.pose.position.z = 0.0
        
        qx, qy, qz, qw = yaw_to_quat(self.target_yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        
        self.start_time = time.time()
        
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)
        
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self.goal_done = True
            self.goal_result = "REJECTED"
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)
        
    def feedback_cb(self, feedback_msg):
        pass  # Data collected via topic subscribers
        
    def result_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.goal_result = "SUCCEEDED"
        elif status == 6:  # ABORTED
            self.goal_result = "ABORTED"
        elif status == 5:  # CANCELED
            self.goal_result = "CANCELED"
        else:
            self.goal_result = f"STATUS_{status}"
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Goal result: {self.goal_result} ({elapsed:.1f}s)")
        self.goal_done = True
        
    def save_data(self):
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(OUTPUT_DIR, f"rotation_180_test_{ts}.csv")
        
        if not self.data_rows:
            self.get_logger().warn("No data collected")
            return None
            
        keys = self.data_rows[0].keys()
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.data_rows)
            
        self.get_logger().info(f"Saved {len(self.data_rows)} rows to {csv_path}")
        return csv_path
        
    def print_summary(self):
        if not self.data_rows:
            return
            
        print("\n" + "="*80)
        print("180° ROTATION TEST SUMMARY")
        print("="*80)
        
        total_time = self.data_rows[-1]['time']
        print(f"Total time: {total_time:.1f}s")
        print(f"Result: {self.goal_result}")
        print(f"Data points: {len(self.data_rows)}")
        
        # Yaw trajectory
        yaws = [r['odom_yaw_deg'] for r in self.data_rows]
        errors = [r['heading_error_deg'] for r in self.data_rows]
        cmd_nav_wz = [r['cmd_vel_nav_wz'] for r in self.data_rows]
        cmd_wz = [r['cmd_vel_wz'] for r in self.data_rows]
        odom_wz = [r['odom_wz'] for r in self.data_rows]
        
        print(f"\nInitial yaw: {yaws[0]:.1f} deg")
        print(f"Final yaw:   {yaws[-1]:.1f} deg")
        print(f"Target yaw:  {math.degrees(self.target_yaw):.1f} deg")
        print(f"Final error: {errors[-1]:.1f} deg")
        
        # Phase analysis: find when RotationShim ends and DWB takes over
        # RotationShim outputs fixed angular vel, DWB varies
        print(f"\n--- cmd_vel_nav (DWB/RotationShim output) ---")
        nav_wz_vals = sorted(set(round(w, 2) for w in cmd_nav_wz if abs(w) > 0.01))
        print(f"Unique angular values: {nav_wz_vals}")
        
        # Detect zero-command periods (DWB dead zone)
        zero_periods = []
        in_zero = False
        zero_start = 0
        for r in self.data_rows:
            t = r['time']
            if abs(r['cmd_vel_nav_wz']) < 0.01 and abs(r['heading_error_deg']) > 5.0:
                if not in_zero:
                    in_zero = True
                    zero_start = t
            else:
                if in_zero:
                    zero_dur = t - zero_start
                    if zero_dur > 0.05:
                        zero_periods.append((zero_start, t, zero_dur))
                    in_zero = False
        if in_zero:
            zero_periods.append((zero_start, self.data_rows[-1]['time'], 
                               self.data_rows[-1]['time'] - zero_start))
        
        if zero_periods:
            print(f"\n--- DEAD ZONE DETECTIONS (cmd_nav_wz=0 while error>5°) ---")
            for start, end, dur in zero_periods:
                err_at_start = None
                for r in self.data_rows:
                    if r['time'] >= start:
                        err_at_start = r['heading_error_deg']
                        break
                print(f"  t={start:.2f}~{end:.2f} ({dur:.2f}s) error≈{err_at_start:.1f}°")
        else:
            print(f"\nNo dead zone detected (good!)")
        
        # Hunting detection: count direction changes in cmd_vel_nav angular
        sign_changes = 0
        last_sign = 0
        for w in cmd_nav_wz:
            if abs(w) > 0.02:
                s = 1 if w > 0 else -1
                if last_sign != 0 and s != last_sign:
                    sign_changes += 1
                last_sign = s
        print(f"\ncmd_vel_nav angular sign changes: {sign_changes}")
        
        # Overshoot detection
        # Find if error crosses zero and goes negative (overshoot)
        overshoots = 0
        min_err = errors[0]
        max_abs_err_after_approach = 0
        passed_zero = False
        for i, e in enumerate(errors):
            if not passed_zero and abs(e) < 5.0:
                passed_zero = True
            if passed_zero and abs(e) > max_abs_err_after_approach:
                max_abs_err_after_approach = abs(e)
            if i > 0 and errors[i-1] * e < 0:  # sign change
                overshoots += 1
                
        print(f"Heading error zero-crossings (overshoots): {overshoots}")
        if passed_zero:
            print(f"Max error after first approach: {max_abs_err_after_approach:.1f}°")
        
        # Time breakdown
        print(f"\n--- TIME BREAKDOWN ---")
        # Find when error first gets below 20deg (RotationShim→DWB handoff)
        rotation_shim_end = None
        for r in self.data_rows:
            if abs(r['heading_error_deg']) < 20.0:
                rotation_shim_end = r['time']
                break
        if rotation_shim_end:
            print(f"RotationShim phase (error >20°): ~{rotation_shim_end:.1f}s")
            print(f"DWB RotateToGoal phase: ~{total_time - rotation_shim_end:.1f}s")
        
        # cmd_vel vs odom lag
        print(f"\n--- VELOCITY TRACKING ---")
        max_cmd = max(abs(w) for w in cmd_wz)
        max_odom = max(abs(w) for w in odom_wz)
        print(f"Max cmd_vel angular: {max_cmd:.3f} rad/s")
        print(f"Max odom angular:    {max_odom:.3f} rad/s")
        
        # Velocity feedback
        fb_wz = [r['vel_fb_wz'] for r in self.data_rows]
        if any(abs(w) > 0.01 for w in fb_wz):
            max_fb = max(abs(w) for w in fb_wz)
            print(f"Max vel_feedback angular: {max_fb:.3f} rad/s")

        print("\n" + "="*80)


def main():
    rclpy.init()
    node = RotationTestCollector()
    
    # Wait for initial odom
    timeout = time.time() + 5.0
    while node.initial_yaw is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if node.initial_yaw is None:
        print("ERROR: No odom received within 5s")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Send the 180° rotation goal
    node.send_goal()
    
    # Spin until goal completes or timeout
    deadline = time.time() + TIMEOUT
    while not node.goal_done and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    
    if not node.goal_done:
        print(f"TIMEOUT after {TIMEOUT}s")
        node.goal_result = "TIMEOUT"
    
    # Collect a bit more data after completion
    post_end = time.time() + 2.0
    while time.time() < post_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    
    # Save and summarize
    csv_path = node.save_data()
    node.print_summary()
    
    if csv_path:
        print(f"\nCSV: {csv_path}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
