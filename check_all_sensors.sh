#!/bin/bash
# 모든 센서 상태 확인 스크립트

echo "========================================"
echo "Transbot 센서 상태 확인"
echo "========================================"
echo ""

# 1. USB 카메라 장치
echo "1. USB 카메라 장치:"
lsusb | grep -i "2bc5\|orbbec"
echo ""

# 2. ROS2 노드
echo "2. 실행 중인 ROS2 노드:"
ros2 node list 2>/dev/null | grep -E "camera|rtabmap" || echo "  (카메라/RTAB-Map 노드 없음)"
echo ""

# 3. 카메라 토픽
echo "3. 카메라 토픽:"
ros2 topic list 2>/dev/null | grep camera || echo "  (카메라 토픽 없음)"
echo ""

# 4. RTAB-Map 토픽
echo "4. RTAB-Map 토픽:"
ros2 topic list 2>/dev/null | grep rtabmap || echo "  (RTAB-Map 토픽 없음)"
echo ""

# 5. TF 프레임
echo "5. TF 프레임:"
timeout 2 ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | head -n 5 || echo "  (TF 연결 없음)"
echo ""

# 6. 센서 Hz
if ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
    echo "6. 센서 Hz 측정:"
    echo "  RGB:"
    timeout 3 ros2 topic hz /camera/color/image_raw 2>/dev/null | tail -1
    echo "  Depth:"
    timeout 3 ros2 topic hz /camera/depth/image_raw 2>/dev/null | tail -1
    echo "  IR:"
    timeout 3 ros2 topic hz /camera/ir/image_raw 2>/dev/null | tail -1
else
    echo "6. 센서 Hz: (카메라가 실행되지 않음)"
fi

echo ""
echo "========================================"
echo "확인 완료"
echo "========================================"

