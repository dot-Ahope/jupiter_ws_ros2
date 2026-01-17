#!/bin/bash

echo "=========================================="
echo "  Astra 카메라 Launch 테스트"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source install/setup.bash

echo "📷 카메라 노드 실행 중..."
echo ""
echo "✅ 활성화된 센서:"
echo "   - RGB (컬러): /camera/color/image_raw"
echo "   - Depth (깊이): /camera/depth/image_raw"
echo "   - IR (적외선): /camera/ir/image_raw"
echo ""
echo "🔍 다른 터미널에서 테스트:"
echo "   cd ~/transbot_ws_ros2"
echo "   ./check_camera_sensors.sh"
echo ""
echo "🛑 종료: Ctrl+C"
echo ""
echo "=========================================="
echo ""

ros2 launch transbot_nav test_camera.launch.py
