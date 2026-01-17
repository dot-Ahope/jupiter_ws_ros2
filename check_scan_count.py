import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanChecker(Node):
    def __init__(self):
        super().__init__('scan_checker')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.count = 0

    def listener_callback(self, msg):
        self.count += 1
        if self.count > 1:
            return # Only check once
        
        total_ranges = len(msg.ranges)
        
        # Count potential valid readings (not inf, not nan, and within range)
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        num_valid = len(valid_ranges)
        
        print(f"Angle Min: {msg.angle_min}")
        print(f"Angle Max: {msg.angle_max}")
        print(f"Angle Increment: {msg.angle_increment}")
        print(f"Total Array Size: {total_ranges}")
        print(f"Valid Readings: {num_valid}")
        print(f"Percentage Valid: {num_valid/total_ranges*100:.1f}%")
        
        # Check distribution
        # Split into 4 quadrants
        q_size = total_ranges // 4
        q1 = valid_ranges_in_slice(msg.ranges, 0, q_size, msg.range_min, msg.range_max)
        q2 = valid_ranges_in_slice(msg.ranges, q_size, q1[1], msg.range_min, msg.range_max) # logic error here in simplified logging
        
        # Better: simple histogram
        chunk = total_ranges // 8
        print("Valid points spatial distribution (8 sectors):")
        for i in range(8):
            start = i * chunk
            end = (i+1) * chunk
            sector_data = msg.ranges[start:end]
            valid_in_sector = len([x for x in sector_data if msg.range_min <= x <= msg.range_max])
            print(f"Sector {i+1}: {valid_in_sector}/{len(sector_data)}")

        rclpy.shutdown()

def valid_ranges_in_slice(ranges, start, end, rmin, rmax):
    return 0, end # dummy

def main(args=None):
    rclpy.init(args=args)
    checker = ScanChecker()
    rclpy.spin(checker)
    checker.destroy_node()

if __name__ == '__main__':
    main()
