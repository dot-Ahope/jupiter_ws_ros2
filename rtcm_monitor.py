#!/usr/bin/env python3
"""
RTCM 데이터 모니터링 노드
/rtcm 토픽에서 RTCM 메시지를 구독하여 데이터 수신 상태를 모니터링합니다.
"""

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message
from std_msgs.msg import String
import time

class RTCMMonitor(Node):
    def __init__(self):
        super().__init__('rtcm_monitor')

        # RTCM 메시지 구독
        self.subscription = self.create_subscription(
            Message,
            '/rtcm',
            self.rtcm_callback,
            10)

        # 통계 퍼블리셔
        self.stats_publisher = self.create_publisher(String, '/rtcm_monitor/stats', 10)

        # 통계 변수
        self.rtcm_count = 0
        self.last_rtcm_time = time.time()
        self.total_bytes = 0

        # 통계 타이머 (1초마다)
        self.create_timer(1.0, self.publish_stats)

        self.get_logger().info("RTCM 모니터링 노드가 시작되었습니다.")

    def rtcm_callback(self, msg):
        """RTCM 메시지 수신 콜백"""
        current_time = time.time()

        self.rtcm_count += 1
        self.total_bytes += len(msg.message)
        self.last_rtcm_time = current_time

        # 메시지 정보 로깅
        self.get_logger().info(
            f"RTCM 메시지 수신: #{self.rtcm_count}, "
            f"크기: {len(msg.message)} bytes, "
            f"타임스탬프: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

        # 첫 50바이트의 헥사 덤프 (디버깅용)
        if len(msg.message) > 0:
            hex_data = msg.message[:50].hex()
            self.get_logger().debug(f"RTCM 데이터 샘플: {hex_data}")

    def publish_stats(self):
        """통계 정보 퍼블리시"""
        now = time.time()
        elapsed = now - self.last_rtcm_time

        # 통계 계산
        avg_msg_size = self.total_bytes / self.rtcm_count if self.rtcm_count > 0 else 0
        msg_rate = self.rtcm_count / max(now - self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9, 1.0)

        stats_msg = String()
        stats_msg.data = (
            f"RTCM 통계 - 수신 메시지: {self.rtcm_count}, "
            f"평균 크기: {avg_msg_size:.1f} bytes, "
            f"총 바이트: {self.total_bytes}, "
            f"마지막 메시지: {elapsed:.1f}초 전, "
            f"메시지율: {msg_rate:.2f} Hz"
        )

        self.stats_publisher.publish(stats_msg)

        # 상태 로깅
        if elapsed > 10.0:
            self.get_logger().warning(f"RTCM 메시지 수신 지연: {elapsed:.1f}초 동안 없음")
        elif self.rtcm_count > 0:
            self.get_logger().info(f"RTCM 상태 정상: {stats_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RTCMMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()