#!/usr/bin/env python3
"""
Goal Pose Re-stamper Node
=========================
PC(Foxglove)에서 발행한 goal_pose의 header.stamp을 Time(0,0)으로 교체하여
Nav2 planner가 TF 조회 시 **항상 최신 TF**를 사용하도록 함.

문제 배경:
  - PC↔Jetson 시계 차이와 관계없이, 장시간 navigation(>10s)에서
    BT recovery 후 planner가 원래 goal stamp으로 TF 조회 시
    TF buffer(기본 10s)에서 해당 시간이 evicted되어 "past extrapolation" 에러 발생.
  - stamp=Time(0,0)은 ROS2 TF2에서 "latest available transform" 의미.
  - 어떤 recovery 지연이 발생해도 TF 조회가 항상 성공.

이 노드는:
  1. /goal_pose 토픽을 /goal_pose_raw로 remap하여 수신
  2. header.stamp을 Time(0,0)으로 교체 (= TF2 latest available)
  3. /goal_pose로 재발행

Nav2 bt_navigator는 /goal_pose를 구독하므로 정상 동작.
Foxglove는 /goal_pose_raw에 발행 (launch에서 remap).

Usage:
  # Launch에서 자동 실행 (nav2_fused_navigation.launch.py)
  # 또는 수동:
  ros2 run jupiter_nav_VO goal_pose_restamper
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


class GoalPoseRestamper(Node):
    def __init__(self):
        super().__init__('goal_pose_restamper')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.sub = self.create_subscription(
            PoseStamped,
            'goal_pose_in',   # launch에서 remap: /goal_pose_raw → goal_pose_in
            self.goal_callback,
            qos,
        )
        self.pub = self.create_publisher(
            PoseStamped,
            'goal_pose_out',  # launch에서 remap: goal_pose_out → /goal_pose
            qos,
        )

        self.get_logger().info(
            'Goal pose re-stamper active: '
            'listening on goal_pose_in, publishing to goal_pose_out'
        )

    def goal_callback(self, msg: PoseStamped):
        original_stamp = msg.header.stamp
        now = self.get_clock().now().to_msg()

        # 원래 stamp과 현재 시간의 차이 계산 (초) — 디버깅용
        orig_sec = original_stamp.sec + original_stamp.nanosec * 1e-9
        now_sec = now.sec + now.nanosec * 1e-9
        diff = orig_sec - now_sec

        self.get_logger().info(
            f'Goal received: position=({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}), '
            f'stamp_offset={diff:+.2f}s → re-stamped to Time(0) [latest TF]'
        )

        # stamp=Time(0,0): TF2가 항상 최신 transform 사용
        # → Nav2 recovery 지연과 무관하게 TF 조회 성공
        msg.header.stamp = Time(sec=0, nanosec=0)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
