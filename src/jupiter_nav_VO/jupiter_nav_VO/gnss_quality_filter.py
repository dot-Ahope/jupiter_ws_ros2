#!/usr/bin/env python3
# =============================================================================
# GNSS Quality Filter Node (간소화 버전)
# =============================================================================
#
# zed_f9p_rtk_node가 올바른 NavSatStatus + 공분산을 설정하므로
# 이 필터는 단순 게이트 역할만 수행한다:
#   - STATUS_NO_FIX (-1) → 게이트아웃
#   - position_covariance > max_covariance → 게이트아웃
#   - 그 외 → 원본 공분산 그대로 통과 (소스 노드의 정확한 값을 신뢰)
#
# 품질 등급 (zed_f9p_rtk_node의 NavSatStatus 매핑):
# NavSatStatus에는 -1, 0, 1, 2 네 값만 존재 (3은 없음)
#   RTK Fixed : STATUS_GBAS_FIX (2), cov=0.0004  (cm급)
#   RTK Float : STATUS_SBAS_FIX (1), cov=0.25    (dm급)
#   3D Fix    : STATUS_FIX (0),      cov=6.25    (m급)
#   No Fix    : STATUS_NO_FIX (-1)                 → 게이트아웃
# 참고: GBAS/SBAS는 원래 DGPS/WAAS 의미이나, RTK 전용 상수가 없어 재활용
#
# 입력: /fix (NavSatFix from zed_f9p_rtk_node)
# 출력: /fix_filtered (NavSatFix, 품질 통과 시에만 발행)
# =============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# NavSatStatus → 사람이 읽을 수 있는 품질 문자열
_STATUS_NAME = {
    NavSatStatus.STATUS_NO_FIX: 'NO_FIX',
    NavSatStatus.STATUS_FIX: 'GPS',
    NavSatStatus.STATUS_SBAS_FIX: 'RTK_FLOAT',
    NavSatStatus.STATUS_GBAS_FIX: 'RTK_FIXED',
}


class GnssQualityFilter(Node):
    def __init__(self):
        super().__init__('gnss_quality_filter')

        # Parameters
        self.declare_parameter('input_topic', '/fix')
        self.declare_parameter('output_topic', '/fix_filtered')
        self.declare_parameter('max_covariance', 50.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_cov = self.get_parameter('max_covariance').value

        # Subscriptions / Publishers
        self.fix_sub = self.create_subscription(
            NavSatFix, input_topic, self._fix_cb, 10)
        self.fix_pub = self.create_publisher(NavSatFix, output_topic, 10)

        # State
        self.prev_quality = ''

        self.get_logger().info(
            f'GNSS Quality Filter (simple): {input_topic} → {output_topic}, '
            f'max_cov={self.max_cov}')

    def _fix_cb(self, msg: NavSatFix):
        status = msg.status.status
        raw_cov = msg.position_covariance[0] if msg.position_covariance else 9999.0

        # ── 품질 판정 (NavSatStatus 기반, 간단) ──
        if status == NavSatStatus.STATUS_NO_FIX:
            quality = 'NO_FIX'
        elif raw_cov > self.max_cov:
            quality = 'POOR'
        else:
            quality = _STATUS_NAME.get(status, 'GPS')

        # 품질 변경 시 로그
        if quality != self.prev_quality:
            self.get_logger().info(
                f'GNSS quality: {self.prev_quality or "INIT"} → {quality} '
                f'(status={status}, cov={raw_cov:.4f})')
            self.prev_quality = quality

        # ── 게이트아웃 ──
        if quality in ('NO_FIX', 'POOR'):
            return

        # ── 원본 메시지 그대로 통과 (소스 노드의 공분산을 신뢰) ──
        self.fix_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GnssQualityFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
