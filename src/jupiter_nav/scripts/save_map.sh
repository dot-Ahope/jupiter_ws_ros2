#!/bin/bash

# SLAM Toolbox 지도 저장 스크립트
# 사용법: ./save_map.sh <지도_이름>

if [ -z "$1" ]; then
    echo "사용법: $0 <지도_이름>"
    echo "예제: $0 my_office_map"
    exit 1
fi

MAP_NAME=$1
MAP_DIR="/home/user/transbot_ws_ros2/src/transbot_nav/maps"

# maps 디렉토리 생성
mkdir -p $MAP_DIR

echo "지도 저장 중: $MAP_NAME"

# 1. SLAM Toolbox posegraph 저장 (.posegraph)
echo "1단계: SLAM Toolbox posegraph 저장..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_DIR/$MAP_NAME'}"

# 2초 대기 (저장 완료 대기)
sleep 2

# 2. 표준 ROS2 맵 저장 (.pgm + .yaml)
echo "2단계: ROS2 표준 맵 포맷 저장..."
ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/$MAP_NAME

echo ""
echo "✅ 지도 저장 완료!"
echo "저장 위치: $MAP_DIR/"
echo "파일들:"
echo "  - ${MAP_NAME}.posegraph (SLAM Toolbox 재사용 가능)"
echo "  - ${MAP_NAME}.pgm (이미지 파일)"
echo "  - ${MAP_NAME}.yaml (메타데이터)"
echo ""
echo "지도 재사용 방법:"
echo "  ros2 launch transbot_nav localization.launch.py map:=$MAP_DIR/${MAP_NAME}.yaml"
