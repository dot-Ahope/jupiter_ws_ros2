#!/usr/bin/env python3
# 국토지리원 CORS NTRIP 데이터를 UBLOX 수신기로 전달하는 브릿지 노드

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message
from std_msgs.msg import String
import serial
import time
import threading

class RTCMUbloxBridge(Node):
    def __init__(self):
        super().__init__('rtcm_ublox_bridge')

        # 매개변수 선언
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('rtcm_topic', '/rtcm')
        
        # 매개변수 가져오기
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.rtcm_topic = self.get_parameter('rtcm_topic').get_parameter_value().string_value

        # 시리얼 포트 설정 (u-blox GPS 연결)
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.get_logger().info(f"연결됨: u-blox GPS on {self.serial_port} @ {self.baudrate}bps")
        except Exception as e:
            self.get_logger().error(f"GPS 연결 실패: {e}")
            self.ser = None
        
        # RTCM 메시지 구독
        self.subscription = self.create_subscription(
            Message,
            self.rtcm_topic,
            self.rtcm_callback,
            10)
        
        # 통계 발행
        self.stats_publisher = self.create_publisher(String, '/rtcm_bridge/stats', 10)
        self.rtcm_count = 0
        self.last_rtcm_time = time.time()
        self.create_timer(1.0, self.publish_stats)
        
        # 연결 모니터링 타이머
        self.create_timer(5.0, self.check_connection)

    def rtcm_callback(self, msg):
        if self.ser is None:
            return

        try:
            # RTCM 메시지를 시리얼 포트로 전송
            self.ser.write(msg.message)
            self.rtcm_count += 1
            self.last_rtcm_time = time.time()
            self.get_logger().debug(f"RTCM 메시지 전송 완료: {len(msg.message)} bytes")
        except Exception as e:
            self.get_logger().error(f"RTCM 메시지 전송 실패: {e}")
            self.try_reconnect()

    def publish_stats(self):
        now = time.time()
        elapsed = now - self.last_rtcm_time
        stats_msg = String()
        stats_msg.data = f"RTCM 메시지: {self.rtcm_count} 수신됨, 마지막 메시지: {elapsed:.1f}초 전"
        self.stats_publisher.publish(stats_msg)
        
        # 로그에 통계 출력
        if elapsed > 10.0:
            self.get_logger().warning(f"RTCM 메시지 수신 지연됨: {elapsed:.1f}초 동안 없음")
        else:
            self.get_logger().info(f"RTCM 상태: {stats_msg.data}")

    def check_connection(self):
        if self.ser is None or not self.ser.is_open:
            self.try_reconnect()
    
    def try_reconnect(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except:
                pass
        
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.get_logger().info(f"GPS에 다시 연결됨: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"GPS 재연결 실패: {e}")
            self.ser = None

    def __del__(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = RTCMUbloxBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
