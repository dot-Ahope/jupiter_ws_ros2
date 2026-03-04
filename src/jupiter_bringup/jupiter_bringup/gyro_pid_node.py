import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from rcl_interfaces.msg import ParameterDescriptor
import math

class GyroPidNode(Node):
    def __init__(self):
        super().__init__('gyro_pid_node')

        # --- Parameters ---
        self.declare_parameter('enable_pid', True)
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('dt', 0.05) # 20Hz
        self.declare_parameter('max_integral', 0.5)
        self.declare_parameter('base_frame', 'base_link')
        
        # --- Variables ---
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_angular_z = 0.0
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.imu_frame_id = None

        # --- Subscriptions ---
        # 원본 명령 (Nav2 등)
        self.sub_cmd_vel = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # RealSense IMU
        self.sub_imu = self.create_subscription(
            Imu, '/camera/imu', self.imu_callback, 10)

        # --- Publisher ---
        # 보정된 명령 (Motor Driver로 전송)
        self.pub_cmd_vel = self.create_publisher(
            Twist, 'cmd_vel_filtered', 10)

        # --- TF for Coordinate Transformation ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Control Loop ---
        dt = self.get_parameter('dt').value
        self.timer = self.create_timer(dt, self.control_loop)
        
        self.get_logger().info("Gyro PID Node Started in jupiter_bringup. Waiting for IMU data...")

    def cmd_vel_callback(self, msg):
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def imu_callback(self, msg):
        # IMU 데이터의 프레임 ID 저장 (TF 변환에 사용)
        self.imu_frame_id = msg.header.frame_id
        
        # 1. IMU 각속도 벡터 생성
        angular_vel_vector = tf2_geometry_msgs.Vector3Stamped()
        angular_vel_vector.header = msg.header
        angular_vel_vector.vector = msg.angular_velocity

        # 2. base_link 좌표계로 변환 (Robot Frame)
        base_frame = self.get_parameter('base_frame').value
        try:
            # 타임아웃 0.05초
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05)
            )
            
            # 벡터 회전 변환 수행
            vel_in_base = tf2_geometry_msgs.do_transform_vector3(angular_vel_vector, transform)
            
            # 로봇의 회전 속도 (Yaw Rate)는 Z축 성분
            self.current_angular_z = vel_in_base.vector.z
            
        except TransformException as ex:
            self.current_angular_z = msg.angular_velocity.z

    def control_loop(self):
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        dt = self.get_parameter('dt').value
        max_int = self.get_parameter('max_integral').value
        enable_pid = self.get_parameter('enable_pid').value

        # 1. Timeout Check (Safety)
        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_diff > 0.5:
            self.publish_cmd(0.0, 0.0)
            self.reset_pid()
            return

        # 2. Stop Command Check
        if abs(self.target_linear_x) < 0.001 and abs(self.target_angular_z) < 0.001:
             self.publish_cmd(0.0, 0.0)
             self.reset_pid()
             return

        # 3. PID Calculation
        if enable_pid:
            error = self.target_angular_z - self.current_angular_z
            
            # P
            p_out = kp * error
            
            # I
            self.integral += error * dt
            self.integral = max(min(self.integral, max_int), -max_int)
            i_out = ki * self.integral
            
            # D
            derivative = (error - self.prev_error) / dt
            d_out = kd * derivative
            
            self.prev_error = error
            
            pid_output = p_out + i_out + d_out
        else:
            pid_output = 0.0

        # 4. Feed-Forward Control (Target + Adjustment)
        final_angular_z = self.target_angular_z + pid_output
        
        self.publish_cmd(self.target_linear_x, final_angular_z)

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd_vel.publish(msg)

    def reset_pid(self):
        self.prev_error = 0.0
        self.integral = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = GyroPidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
