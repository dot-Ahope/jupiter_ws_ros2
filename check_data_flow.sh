#!/bin/bash

echo "데이터 흐름 확인 스크립트"
echo "=========================="
echo ""
echo "1. 시스템 실행 확인..."
ros2 node list | grep -E "transbot_driver|base_node" || echo "❌ 시스템이 실행되지 않았습니다!"
echo ""

echo "2. /transbot/get_vel 토픽 확인 (3초)..."
timeout 3 ros2 topic echo /transbot/get_vel --field angular.z | head -5
echo ""

echo "3. /odom_raw 토픽 확인 (3초)..."  
timeout 3 ros2 topic echo /odom_raw --field twist.twist.angular.z | head -5
echo ""

echo "4. 회전 테스트 (0.3 rad/s, 3초)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}' &
sleep 0.5

echo "   /transbot/get_vel 모니터링:"
timeout 2 ros2 topic echo /transbot/get_vel --field angular.z | head -3

echo ""
echo "   /odom_raw 모니터링:"
timeout 2 ros2 topic echo /odom_raw --field twist.twist.angular.z | head -3

echo ""
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.0}}'
echo ""
echo "완료!"
