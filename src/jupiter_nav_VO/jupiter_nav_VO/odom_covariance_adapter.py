#!/usr/bin/env python3
# encoding: utf-8
"""
Odometry Covariance Adapter Node
=================================
base_node의 /odom_raw는 공분산이 0으로 발행됨.
robot_localization은 공분산이 0이면 해당 측정값을 무시하므로,
속도 기반 동적 공분산을 적용하여 /odom_adapted로 재발행한다.

동적 공분산 전략:
  1. 정지 상태 (|vx| < threshold, |vyaw| < threshold):
     - 낮은 공분산 → 높은 신뢰도
     - 이동하지 않으므로 드리프트가 거의 없음
     
  2. 저속 이동 (|vx| < 0.3 m/s):
     - 기본 공분산
     - 엔코더 정확도가 가장 높은 구간
     
  3. 고속 이동 (|vx| > 0.3 m/s):
     - 속도에 비례하여 공분산 증가
     - 슬립, 미끄러짐 가능성 반영
     
  4. 회전 중 (|vyaw| > threshold):
     - 선형 속도의 공분산 증가 (회전 중 슬립)
     - 각속도의 공분산은 상대적으로 유지

  5. 가감속 감지 (가속도 변화량 모니터링):
     - 급격한 가속/감속 시 공분산 일시적 증가
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time


class OdomCovarianceAdapter(Node):
    def __init__(self):
        super().__init__('odom_covariance_adapter')
        
        # ============================================================
        # 파라미터 선언
        # ============================================================
        # 기본 공분산 값 (정지 시)
        self.declare_parameter('base_linear_cov', 0.005)      # (m/s)² - 정지 시 선형 속도 분산
        self.declare_parameter('base_angular_cov', 0.01)       # (rad/s)² - 정지 시 각속도 분산
        self.declare_parameter('base_pose_cov', 0.01)          # m² - 정지 시 위치 분산
        self.declare_parameter('base_orient_cov', 0.02)        # rad² - 정지 시 방향 분산
        
        # 속도 임계값
        self.declare_parameter('stationary_linear_threshold', 0.01)   # m/s
        self.declare_parameter('stationary_angular_threshold', 0.02)  # rad/s
        self.declare_parameter('high_speed_threshold', 0.3)           # m/s
        
        # 동적 스케일 팩터
        self.declare_parameter('speed_cov_scale', 0.1)         # 속도 × 이 값 = 추가 공분산
        self.declare_parameter('rotation_cov_scale', 0.3)      # 회전 시 선형 공분산 증가 배율
        self.declare_parameter('accel_cov_scale', 0.5)         # 가속도 변화 시 공분산 증가 배율
        
        # 정지 시 감쇠 (drift 방지)
        self.declare_parameter('stationary_cov_factor', 0.1)   # 정지 시 공분산 축소 비율
        
        # 토픽 이름
        self.declare_parameter('input_odom_topic', '/odom_raw')
        self.declare_parameter('output_odom_topic', '/odom_adapted')
        
        # 파라미터 로드
        self.base_linear_cov = self.get_parameter('base_linear_cov').value
        self.base_angular_cov = self.get_parameter('base_angular_cov').value
        self.base_pose_cov = self.get_parameter('base_pose_cov').value
        self.base_orient_cov = self.get_parameter('base_orient_cov').value
        
        self.stationary_lin_thresh = self.get_parameter('stationary_linear_threshold').value
        self.stationary_ang_thresh = self.get_parameter('stationary_angular_threshold').value
        self.high_speed_thresh = self.get_parameter('high_speed_threshold').value
        
        self.speed_cov_scale = self.get_parameter('speed_cov_scale').value
        self.rotation_cov_scale = self.get_parameter('rotation_cov_scale').value
        self.accel_cov_scale = self.get_parameter('accel_cov_scale').value
        self.stationary_cov_factor = self.get_parameter('stationary_cov_factor').value
        
        input_topic = self.get_parameter('input_odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        
        # ============================================================
        # 상태 변수
        # ============================================================
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vyaw = 0.0
        self.prev_time = None
        self.accel_magnitude = 0.0     # 현재 가속도 크기
        
        # 지수 이동 평균 (Exponential Moving Average) 상태
        self.ema_accel = 0.0
        self.ema_alpha = 0.3          # EMA 감쇠 계수 (0~1, 클수록 반응 빠름)
        
        # ============================================================
        # 퍼블리셔/서브스크라이버
        # ============================================================
        self.odom_sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            50
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            output_topic,
            50
        )
        
        self.get_logger().info(
            f'OdomCovarianceAdapter: {input_topic} → {output_topic} '
            f'(base_lin_cov={self.base_linear_cov}, base_ang_cov={self.base_angular_cov})'
        )
    
    def odom_callback(self, msg: Odometry):
        """odom_raw 수신 → 동적 공분산 적용 → odom_adapted 발행"""
        
        # 현재 속도 추출
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vyaw = msg.twist.twist.angular.z
        
        linear_speed = math.sqrt(vx * vx + vy * vy)
        angular_speed = abs(vyaw)
        
        # ============================================================
        # 가속도 추정 (속도 변화량)
        # ============================================================
        current_time = self.get_clock().now()
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9
            if dt > 0.001:  # 최소 1ms
                ax = (vx - self.prev_vx) / dt
                ay = (vy - self.prev_vy) / dt
                self.accel_magnitude = math.sqrt(ax * ax + ay * ay)
                # EMA로 가속도 평활화 (급격한 스파이크 방지)
                self.ema_accel = self.ema_alpha * self.accel_magnitude + \
                                 (1.0 - self.ema_alpha) * self.ema_accel
        
        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_vyaw = vyaw
        self.prev_time = current_time
        
        # ============================================================
        # 동적 공분산 계산
        # ============================================================
        is_stationary = (linear_speed < self.stationary_lin_thresh and 
                        angular_speed < self.stationary_ang_thresh)
        
        if is_stationary:
            # 정지 상태: 매우 낮은 공분산 (높은 신뢰도)
            # → "로봇이 움직이지 않는다"는 강한 신호
            cov_vx = self.base_linear_cov * self.stationary_cov_factor
            cov_vy = self.base_linear_cov * self.stationary_cov_factor
            cov_vyaw = self.base_angular_cov * self.stationary_cov_factor
            cov_x = self.base_pose_cov * self.stationary_cov_factor
            cov_y = self.base_pose_cov * self.stationary_cov_factor
            cov_yaw = self.base_orient_cov * self.stationary_cov_factor
        else:
            # 이동 상태: 속도에 비례하여 공분산 증가
            speed_factor = 1.0 + linear_speed * self.speed_cov_scale
            rotation_factor = 1.0 + angular_speed * self.rotation_cov_scale
            accel_factor = 1.0 + self.ema_accel * self.accel_cov_scale
            
            # 선형 속도 공분산
            cov_vx = self.base_linear_cov * speed_factor * rotation_factor * accel_factor
            cov_vy = self.base_linear_cov * speed_factor * rotation_factor * accel_factor
            
            # 각속도 공분산
            cov_vyaw = self.base_angular_cov * rotation_factor * accel_factor
            
            # 위치 공분산 (적분 오차 누적)
            cov_x = self.base_pose_cov * speed_factor * rotation_factor
            cov_y = self.base_pose_cov * speed_factor * rotation_factor
            cov_yaw = self.base_orient_cov * rotation_factor
        
        # ============================================================
        # 공분산 행렬 설정 (6x6 대각행렬)
        # [x, y, z, roll, pitch, yaw]
        # ============================================================
        # Pose Covariance
        pose_cov = [0.0] * 36
        pose_cov[0]  = cov_x       # x
        pose_cov[7]  = cov_y       # y
        pose_cov[14] = 1e6         # z (2D이므로 무시)
        pose_cov[21] = 1e6         # roll (2D이므로 무시)
        pose_cov[28] = 1e6         # pitch (2D이므로 무시)
        pose_cov[35] = cov_yaw     # yaw
        
        # Twist Covariance
        twist_cov = [0.0] * 36
        twist_cov[0]  = cov_vx     # vx
        twist_cov[7]  = cov_vy     # vy
        twist_cov[14] = 1e6        # vz (2D이므로 무시)
        twist_cov[21] = 1e6        # vroll (2D이므로 무시)
        twist_cov[28] = 1e6        # vpitch (2D이므로 무시)
        twist_cov[35] = cov_vyaw   # vyaw
        
        # ============================================================
        # 메시지 발행
        # ============================================================
        adapted_msg = Odometry()
        adapted_msg.header = msg.header
        adapted_msg.child_frame_id = msg.child_frame_id
        adapted_msg.pose = msg.pose
        adapted_msg.twist = msg.twist
        
        # 공분산 적용
        adapted_msg.pose.covariance = pose_cov
        adapted_msg.twist.covariance = twist_cov
        
        self.odom_pub.publish(adapted_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
