#!/usr/bin/env python3
# encoding: utf-8
import sys
import os
# Jupiter 라이브러리 경로 - 필요시 설치 또는 경로 수정 필요
# sys.path.append("/home/jetson/Jupiter/jupiter")

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
from cv_bridge import CvBridge
import subprocess

class DeviceService(Node):
    def __init__(self):
        super().__init__('device_service')
        
        # QoS 설정
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 파라미터 선언
        self.declare_parameter('camera_device', 'astra')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_fps', 30)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', camera_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', camera_qos)
        
        # OpenCV 브릿지
        self.bridge = CvBridge()
        
        # 카메라 설정
        self.setup_camera()
        
        # 타이머 생성
        self.create_timer(1.0/self.get_parameter('camera_fps').value, self.timer_callback)
        
        self.get_logger().info('Device Service has been initialized')
        
    def setup_camera(self):
        """카메라 장치 설정"""
        camera_device = self.get_parameter('camera_device').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        
        self.cap = None
        
        try:
            if camera_device == 'astra':
                # Astra 카메라는 ros2_astra_camera 패키지에서 처리하므로
                # device_srv는 카메라를 열지 않음
                self.get_logger().info('Astra mode: camera handled by astra_camera_node')
                return
            else:
                # USB 2.0 카메라 사용 (/dev/video0 또는 /dev/video1)
                # /dev/video2가 없으므로 /dev/video0부터 시도
                for device_id in [0, 1]:
                    self.cap = cv2.VideoCapture(device_id)
                    if self.cap.isOpened():
                        self.get_logger().info(f'USB camera opened: /dev/video{device_id}')
                        break
                    self.cap = None
                
                if self.cap is None:
                    self.get_logger().error('Failed to open camera device')
                    return
                
                # 카메라 설정
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                
                self.get_logger().info(f'Camera initialized: USB 2.0 Camera ({width}x{height})')
            
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {str(e)}')
            self.cap = None
            
    def timer_callback(self):
        """카메라 프레임 발행"""
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                try:
                    # OpenCV 이미지를 ROS 메시지로 변환
                    img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_rgb_optical_frame"
                    
                    # 이미지 발행
                    self.image_pub.publish(img_msg)
                    
                    # 카메라 정보 발행
                    camera_info = CameraInfo()
                    camera_info.header = img_msg.header
                    camera_info.height = frame.shape[0]
                    camera_info.width = frame.shape[1]
                    self.camera_info_pub.publish(camera_info)
                    
                except Exception as e:
                    self.get_logger().error(f'Failed to publish image: {str(e)}')
                    
    def destroy_node(self):
        """노드 종료 시 정리"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = DeviceService()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
