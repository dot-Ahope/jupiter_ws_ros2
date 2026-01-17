#!/bin/bash
# Astra Pro 카메라 재시작 스크립트 (RGB UVC 모드)

echo "=== Astra Pro Camera Restart (RGB+Depth+IR) ==="
echo "1. Killing existing camera nodes..."
pkill -9 -f astra_camera_node
sleep 2

echo "2. Cleaning up mutex files..."
rm -f /tmp/XnCore.Mutex.*
sleep 1

echo "3. Checking USB devices..."
lsusb | grep -i "2bc5\|orbbec"

echo ""
echo "4. Launching camera with UVC support..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source install/setup.bash
ros2 launch transbot_nav test_camera.launch.py

