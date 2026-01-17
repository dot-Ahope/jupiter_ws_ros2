#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random

class DummyLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('dummy_laser_scan_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10Hz
        self.get_logger().info('Dummy Laser Scan Publisher started')

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'camera_depth_frame'

        # Scan parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0

        # Generate dummy ranges (simulate some obstacles)
        num_ranges = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = []

        for i in range(num_ranges):
            angle = scan.angle_min + i * scan.angle_increment

            # Create some fake obstacles
            if abs(angle) < 0.1:  # Front
                distance = random.uniform(1.0, 3.0)
            elif abs(angle - math.pi/2) < 0.1:  # Right
                distance = random.uniform(0.5, 2.0)
            elif abs(angle + math.pi/2) < 0.1:  # Left
                distance = random.uniform(0.5, 2.0)
            else:
                distance = random.uniform(3.0, 8.0)  # Clear areas

            # Add some noise
            distance += random.uniform(-0.1, 0.1)
            distance = max(scan.range_min, min(scan.range_max, distance))

            scan.ranges.append(distance)

        scan.intensities = [100.0] * len(scan.ranges)  # Dummy intensities

        self.publisher.publish(scan)
        self.get_logger().debug('Published dummy laser scan')

def main(args=None):
    rclpy.init(args=args)
    node = DummyLaserScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
