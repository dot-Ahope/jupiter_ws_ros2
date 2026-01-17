#!/bin/bash
# RTAB-Map RGB-D SLAM 실행 스크립트
# RGB + Depth + IR 모든 센서 활성화

echo "==================================="
echo "RTAB-Map RGB-D SLAM 시작"
echo "==================================="
echo ""
echo "✅ 카메라: RGB (UVC) + Depth + IR"
echo "✅ LiDAR: RPLidar A1"
echo "✅ IMU: MPU6050"
echo "✅ EKF: robot_localization"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source install/setup.bash

# RTAB-Map 실행 (RViz 포함)
echo "Starting RTAB-Map with RViz..."
ros2 launch transbot_nav transbot_rtabmap.launch.py use_rgbd:=true use_rviz:=true

