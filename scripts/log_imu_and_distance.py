#!/usr/bin/env python3
#python3 /home/jetson/transbot_ws_ros2/scripts/log_imu_and_distance.py --help
"""
간단한 ROS2 노드: IMU 데이터와 이동 거리(odometry/velocity 기반)를 기록합니다.
사용법:
  source install/setup.bash
  python3 scripts/log_imu_and_distance.py

출력: ~/imu_distance_log.csv

구독 토픽:
 - /transbot/imu (sensor_msgs/Imu)
 - /transbot/get_vel (geometry_msgs/Twist)  # 드라이버가 발행하는 현재 속도(선속도 m/s)
 - /odom_raw (nav_msgs/Odometry) optional (pose 기반 거리 계산)

CSV 칼럼: timestamp,dist_vel_m,dist_odom_m,imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time
from os.path import expanduser
import csv


class ImuDistanceLogger(Node):
    def __init__(self, csv_path=None):
        super().__init__('imu_distance_logger')
        self.csv_path = csv_path or expanduser('~/imu_distance_log.csv')

        # 상태 변수
        self.last_vel_time = None
        self.last_odom_time = None
        self.distance_from_vel = 0.0
        self.distance_from_odom = 0.0
        self.last_odom_x = None
        self.last_odom_y = None

        # IMU latest
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_az = 0.0
        self.imu_gx = 0.0
        self.imu_gy = 0.0
        self.imu_gz = 0.0

        # 구독자
        self.create_subscription(Imu, '/transbot/imu', self.imu_cb, 10)
        self.create_subscription(Twist, '/transbot/get_vel', self.vel_cb, 10)
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)

        # 타이머로 주기적 로그 (1Hz)
        self.create_timer(1.0, self.timer_cb)

        # CSV 헤더 준비
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'dist_vel_m', 'dist_odom_m',
                                 'imu_ax', 'imu_ay', 'imu_az', 'imu_gx', 'imu_gy', 'imu_gz'])
            self.get_logger().info(f'Logging to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to open CSV: {e}')

    def imu_cb(self, msg: Imu):
        # 가속도와 각속도 저장 (raw body-frame values)
        try:
            self.imu_ax = msg.linear_acceleration.x
            self.imu_ay = msg.linear_acceleration.y
            self.imu_az = msg.linear_acceleration.z
            self.imu_gx = msg.angular_velocity.x
            self.imu_gy = msg.angular_velocity.y
            self.imu_gz = msg.angular_velocity.z
        except Exception as e:
            self.get_logger().warn(f'IMU cb error: {e}')

    def vel_cb(self, msg: Twist):
        # velocity 기반 거리 적분
        now = self.get_clock().now().to_msg()
        t = now.sec + now.nanosec * 1e-9
        if self.last_vel_time is None:
            self.last_vel_time = t
            return
        dt = t - self.last_vel_time
        self.last_vel_time = t
        # forward velocity assumed in msg.linear.x (m/s)
        v = msg.linear.x
        self.distance_from_vel += v * dt

    def odom_cb(self, msg: Odometry):
        # pose 기반 거리 계산
        try:
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if self.last_odom_x is None:
                self.last_odom_x = x
                self.last_odom_y = y
                self.last_odom_time = now
                return
            dx = x - self.last_odom_x
            dy = y - self.last_odom_y
            d = (dx**2 + dy**2) ** 0.5
            self.distance_from_odom += d
            self.last_odom_x = x
            self.last_odom_y = y
            self.last_odom_time = now
        except Exception as e:
            self.get_logger().warn(f'Odom cb error: {e}')

    def timer_cb(self):
        ts = time()
        # 콘솔 출력
        self.get_logger().info(f'Distance (vel)={self.distance_from_vel:.3f} m, (odom)={self.distance_from_odom:.3f} m | IMU a=({self.imu_ax:.3f},{self.imu_ay:.3f},{self.imu_az:.3f}) g=({self.imu_gx:.3f},{self.imu_gy:.3f},{self.imu_gz:.3f})')
        # CSV에 기록
        try:
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([ts, f'{self.distance_from_vel:.6f}', f'{self.distance_from_odom:.6f}',
                                 f'{self.imu_ax:.6f}', f'{self.imu_ay:.6f}', f'{self.imu_az:.6f}',
                                 f'{self.imu_gx:.6f}', f'{self.imu_gy:.6f}', f'{self.imu_gz:.6f}'])
        except Exception as e:
            self.get_logger().error(f'CSV write error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuDistanceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
