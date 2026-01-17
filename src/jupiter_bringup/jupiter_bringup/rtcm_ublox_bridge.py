#!/usr/bin/env python3
# filepath: /home/jetson/jupiter_ws_ros2/src/jupiter_bringup/jupiter_bringup/rtcm_ublox_bridge.py

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message
import serial
import time

class RTCMUbloxBridge(Node):
    def __init__(self):
        super().__init__('rtcm_ublox_bridge')

        # RTCM 메시지 구독
        self.subscription = self.create_subscription(
            Message,
            '/rtcm',
            self.rtcm_callback,
            10)

        # 시리얼 포트 설정 (u-blox GPS 연결)
        self.serial_port = '/dev/ttyUSB0'  # 실제 포트로 변경
        self.baudrate = 38400

        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.get_logger().info(f"Connected to u-blox GPS on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPS: {e}")
            self.ser = None

    def rtcm_callback(self, msg):
        if self.ser is None:
            return

        try:
            # RTCM 메시지를 시리얼 포트로 전송
            self.ser.write(msg.message)
            self.get_logger().debug(f"Sent RTCM message of length {len(msg.message)}")
        except Exception as e:
            self.get_logger().error(f"Failed to send RTCM message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RTCMUbloxBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
