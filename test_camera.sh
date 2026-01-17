#!/bin/bash

# Astra 카메라 테스트 스크립트

echo "=== Astra 카메라 테스트 ==="
echo ""

# 1. USB 디바이스 확인
echo "1. USB 디바이스 확인:"
lsusb | grep -i "orbbec\|astra\|2bc5"
echo ""

# 2. 비디오 디바이스 확인
echo "2. 비디오 디바이스:"
ls -l /dev/video* 2>/dev/null
echo ""

# 3. ROS2 환경 설정
source ~/transbot_ws_ros2/install/setup.bash

# 4. 카메라 노드 실행
echo "3. 카메라 노드 실행 중..."
echo "   (Ctrl+C로 종료)"
echo ""

ros2 run astra_camera astra_camera_node \
  --ros-args \
  -p camera_name:=camera \
  -p enable_color:=true \
  -p enable_depth:=true \
  -p enable_ir:=true \
  -p color_width:=640 \
  -p color_height:=480 \
  -p color_fps:=15 \
  -p depth_width:=640 \
  -p depth_height:=480 \
  -p depth_fps:=15 \
  -p ir_width:=640 \
  -p ir_height:=480 \
  -p ir_fps:=15 \
  -p enable_point_cloud:=true

echo ""
echo "=== 다른 터미널에서 테스트 ==="
echo ""
echo "# 모든 카메라 토픽 확인:"
echo "ros2 topic list | grep camera"
echo ""
echo "# RGB 토픽 Hz:"
echo "ros2 topic hz /camera/color/image_raw"
echo ""
echo "# Depth 토픽 Hz:"
echo "ros2 topic hz /camera/depth/image_raw"
echo ""
echo "# IR 토픽 Hz:"
echo "ros2 topic hz /camera/ir/image_raw"
echo ""
