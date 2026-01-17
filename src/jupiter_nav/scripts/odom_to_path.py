#!/usr/bin/env python3
"""
Odometry to Path Converter Node
오도메트리 메시지를 Path 메시지로 변환하여 RViz에서 경로 시각화
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('max_path_length', 1000)  # 최대 경로 길이
        
        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_path_length = self.get_parameter('max_path_length').value
        
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # Publisher
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        # Path message
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        
        self.get_logger().info(f'Odometry to Path node started')
        self.get_logger().info(f'  Input: {odom_topic}')
        self.get_logger().info(f'  Output: {path_topic}')
        self.get_logger().info(f'  Frame: {self.frame_id}')
    
    def odom_callback(self, msg: Odometry):
        """오도메트리를 받아서 Path에 추가"""
        # PoseStamped 생성
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        
        # Path에 추가
        self.path.poses.append(pose)
        
        # 최대 길이 제한
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # Path 발행
        self.path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
