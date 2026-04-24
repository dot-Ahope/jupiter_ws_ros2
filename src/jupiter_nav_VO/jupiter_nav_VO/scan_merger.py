#!/usr/bin/env python3
"""
Jupiter Scan Merger (2026-04-24)
================================

RPLidar `/scan` (laser frame) 과 D455 depth-derived `/scan_from_depth` (base_link frame)
를 병합하여 `/scan_merged` 로 발행.

전략:
  1. 두 scan 을 각각 target_frame(base_link) 기준 점군 (angle, range) 으로 변환.
  2. 공통 angle grid (0.5° 간격, -π ~ π) 에 투영.
  3. 같은 bin 에 두 센서가 모두 hit 하면 **최소 range** 채택 (더 가까운 장애물 우선).
  4. LaserScan 으로 발행.

Subscribes:
  /scan            (sensor_msgs/LaserScan, RPLidar)
  /scan_from_depth (sensor_msgs/LaserScan, D455 depth z-slice)

Publishes:
  /scan_merged     (sensor_msgs/LaserScan, target_frame=base_link)

Parameters:
  target_frame       : default "base_link"
  merged_angle_min   : default -pi
  merged_angle_max   : default +pi
  merged_angle_increment : default 0.00873 (0.5°)
  merged_range_min   : default 0.1
  merged_range_max   : default 5.0
  publish_rate_hz    : default 10.0
  scan_timeout_sec   : default 0.5  (scan 한 개라도 이 이상 stale 이면 skip)
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

import tf2_ros
from sensor_msgs.msg import LaserScan


def _quat_to_yaw(q) -> float:
    """Return yaw from geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _laserscan_to_base_points(scan: LaserScan,
                              tf_buffer: tf2_ros.Buffer,
                              target_frame: str):
    """
    LaserScan → list of (angle_target, range_target) in target_frame.
    Returns None on TF lookup failure.
    """
    try:
        tf = tf_buffer.lookup_transform(
            target_frame,
            scan.header.frame_id,
            Time(),  # latest available
            timeout=Duration(seconds=0.1),
        )
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        return None

    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    yaw = _quat_to_yaw(tf.transform.rotation)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    pts = []
    angle = scan.angle_min
    range_min = scan.range_min
    range_max = scan.range_max
    for r in scan.ranges:
        if not math.isnan(r) and not math.isinf(r) and range_min <= r <= range_max:
            x_src = r * math.cos(angle)
            y_src = r * math.sin(angle)
            x_tgt = tx + x_src * cos_y - y_src * sin_y
            y_tgt = ty + x_src * sin_y + y_src * cos_y
            a = math.atan2(y_tgt, x_tgt)
            d = math.hypot(x_tgt, y_tgt)
            pts.append((a, d))
        angle += scan.angle_increment
    return pts


class ScanMerger(Node):

    def __init__(self):
        super().__init__('scan_merger')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('merged_angle_min', -math.pi)
        self.declare_parameter('merged_angle_max', math.pi)
        self.declare_parameter('merged_angle_increment', 0.00873)  # 0.5°
        self.declare_parameter('merged_range_min', 0.1)
        self.declare_parameter('merged_range_max', 5.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('scan_timeout_sec', 0.5)

        self.target_frame = self.get_parameter('target_frame').value
        self.a_min = float(self.get_parameter('merged_angle_min').value)
        self.a_max = float(self.get_parameter('merged_angle_max').value)
        self.a_inc = float(self.get_parameter('merged_angle_increment').value)
        self.r_min = float(self.get_parameter('merged_range_min').value)
        self.r_max = float(self.get_parameter('merged_range_max').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timeout = float(self.get_parameter('scan_timeout_sec').value)

        self.n_bins = int(math.ceil((self.a_max - self.a_min) / self.a_inc))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_lidar = None
        self.scan_depth = None

        self.create_subscription(LaserScan, '/scan', self._cb_lidar, sensor_qos)
        self.create_subscription(LaserScan, '/scan_from_depth', self._cb_depth, sensor_qos)

        self.pub = self.create_publisher(LaserScan, '/scan_merged', sensor_qos)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_merged)

        self.get_logger().info(
            f'ScanMerger up. target_frame={self.target_frame}, '
            f'bins={self.n_bins}, rate={rate_hz} Hz'
        )

    def _cb_lidar(self, msg: LaserScan):
        self.scan_lidar = msg

    def _cb_depth(self, msg: LaserScan):
        self.scan_depth = msg

    def _is_fresh(self, msg: LaserScan) -> bool:
        if msg is None:
            return False
        now = self.get_clock().now().nanoseconds
        stamp = int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        # Jetson 이 깨어난 뒤 얼마 지나지 않은 경우 등 stamp 가 너무 작으면 skip.
        if stamp <= 0:
            return False
        age = (now - stamp) * 1e-9
        return abs(age) < self.timeout

    def _publish_merged(self):
        lidar_fresh = self._is_fresh(self.scan_lidar)
        depth_fresh = self._is_fresh(self.scan_depth)

        if not lidar_fresh and not depth_fresh:
            return

        ranges = [float('inf')] * self.n_bins

        def project(scan):
            if scan is None:
                return
            pts = _laserscan_to_base_points(scan, self.tf_buffer, self.target_frame)
            if pts is None:
                return
            for (a, d) in pts:
                if d < self.r_min or d > self.r_max:
                    continue
                if a < self.a_min or a >= self.a_max:
                    continue
                idx = int((a - self.a_min) / self.a_inc)
                if 0 <= idx < self.n_bins and d < ranges[idx]:
                    ranges[idx] = d

        if lidar_fresh:
            project(self.scan_lidar)
        if depth_fresh:
            project(self.scan_depth)

        merged = LaserScan()
        # Pick the newer stamp between the two inputs
        now = self.get_clock().now().to_msg()
        merged.header.stamp = now
        merged.header.frame_id = self.target_frame
        merged.angle_min = self.a_min
        merged.angle_max = self.a_max
        merged.angle_increment = self.a_inc
        merged.time_increment = 0.0
        merged.scan_time = 1.0 / max(1.0, float(self.get_parameter('publish_rate_hz').value))
        merged.range_min = self.r_min
        merged.range_max = self.r_max
        merged.ranges = ranges
        merged.intensities = []

        self.pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
