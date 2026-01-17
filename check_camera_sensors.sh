#!/bin/bash

echo "=========================================="
echo "  Astra 카메라 센서 상태 확인"
echo "=========================================="
echo ""

# ROS2 환경 설정
source ~/transbot_ws_ros2/install/setup.bash

echo "1. 카메라 토픽 목록:"
echo "---"
ros2 topic list | grep camera
echo ""

echo "2. RGB 센서 (컬러 카메라):"
echo "---"
timeout 3 ros2 topic hz /camera/color/image_raw 2>&1 | head -5 || echo "❌ RGB 데이터 없음"
echo ""

echo "3. Depth 센서 (깊이):"
echo "---"
timeout 3 ros2 topic hz /camera/depth/image_raw 2>&1 | head -5 || echo "❌ Depth 데이터 없음"
echo ""

echo "4. IR 센서 (적외선):"
echo "---"
timeout 3 ros2 topic hz /camera/ir/image_raw 2>&1 | head -5 || echo "❌ IR 데이터 없음"
echo ""

echo "=========================================="
echo "  예상 출력: ~15 Hz (각 센서)"
echo "=========================================="
echo ""

echo "📊 센서 정보:"
echo "  - RGB: 컬러 카메라 (독립 센서)"
echo "  - IR:  적외선 센서 (Depth와 동일 센서)"
echo "  - Depth: IR 센서로부터 계산된 깊이"
echo ""
