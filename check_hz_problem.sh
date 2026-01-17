#!/bin/bash

echo "=========================================="
echo "Hz 문제 진단 스크립트"
echo "=========================================="
echo ""

# 시스템 확인
echo "1. 시스템 실행 확인..."
ros2 node list | grep -E "transbot_driver|base_node" > /dev/null
if [ $? -ne 0 ]; then
    echo "❌ 시스템이 실행되지 않았습니다!"
    echo "먼저 실행하세요: ros2 launch sllidar_ros2 transbot_full_system.launch.py"
    exit 1
fi
echo "✅ 시스템 실행 중"
echo ""

# 토픽 Hz 확인
echo "2. 토픽 발행 주기 확인..."
echo ""

echo "📊 /imu/data_calibrated (IMU - 정상 작동):"
timeout 5 ros2 topic hz /imu/data_calibrated 2>&1 | grep "average rate" | tail -1
echo ""

echo "📊 /transbot/get_vel (하드웨어 피드백):"
timeout 5 ros2 topic hz /transbot/get_vel 2>&1 | grep "average rate" | tail -1
echo ""

echo "📊 /odom_raw (base_node 발행):"
timeout 5 ros2 topic hz /odom_raw 2>&1 | grep "average rate" | tail -1
echo ""

# 회전 중 데이터 수집
echo "3. 회전 테스트 중 데이터 수집..."
echo ""
echo "⏳ 3초간 0.3 rad/s 회전 명령..."

# 백그라운드에서 데이터 수집
timeout 4 ros2 topic echo /odom_raw --field twist.twist.angular.z > /tmp/odom_data.txt 2>/dev/null &
ODOM_PID=$!

timeout 4 ros2 topic echo /imu/data_calibrated --field angular_velocity.z > /tmp/imu_data.txt 2>/dev/null &
IMU_PID=$!

sleep 0.5

# 회전 명령
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}' > /dev/null 2>&1

sleep 3

# 정지 명령
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.0}}' > /dev/null 2>&1

sleep 0.5

# 데이터 분석
echo ""
echo "4. 수집된 데이터 분석..."
echo ""

ODOM_LINES=$(wc -l < /tmp/odom_data.txt 2>/dev/null || echo 0)
IMU_LINES=$(wc -l < /tmp/imu_data.txt 2>/dev/null || echo 0)

# 0.0이 아닌 값만 카운트
ODOM_NONZERO=$(grep -v "^0.0$\|^---$\|^$" /tmp/odom_data.txt 2>/dev/null | wc -l || echo 0)
IMU_NONZERO=$(grep -v "^0.0$\|^---$\|^$" /tmp/imu_data.txt 2>/dev/null | wc -l || echo 0)

echo "📈 /odom_raw:"
echo "   총 메시지: $ODOM_LINES개"
echo "   0이 아닌 값: $ODOM_NONZERO개"
echo "   예상 (50Hz × 3초): 150개"

if [ $ODOM_NONZERO -lt 30 ]; then
    echo "   ❌ 심각한 데이터 손실! (80% 이상)"
elif [ $ODOM_NONZERO -lt 100 ]; then
    echo "   ⚠️  데이터 손실 있음 (33% 이상)"
else
    echo "   ✅ 정상"
fi

echo ""
echo "📈 /imu/data_calibrated:"
echo "   총 메시지: $IMU_LINES개"
echo "   0이 아닌 값: $IMU_NONZERO개"
echo "   예상 (100Hz × 3초): 300개"

if [ $IMU_NONZERO -lt 100 ]; then
    echo "   ⚠️  데이터 손실"
else
    echo "   ✅ 정상"
fi

echo ""
echo "5. 결론..."
echo ""

if [ $ODOM_NONZERO -lt 30 ]; then
    echo "❌ 문제 확인: /odom_raw 발행 주기가 너무 느림"
    echo ""
    echo "원인:"
    echo "  - /transbot/get_vel이 10Hz로만 발행"
    echo "  - base_node가 /transbot/get_vel을 그대로 전달"
    echo "  - 결과: /odom_raw도 10Hz"
    echo ""
    echo "해결책:"
    echo "  1. transbot_driver의 get_vel 발행 주기 증가 (10Hz → 50Hz)"
    echo "  2. 또는 base_node에서 보간(interpolation) 추가"
    echo ""
elif [ $ODOM_NONZERO -lt 100 ]; then
    echo "⚠️  /odom_raw 발행 주기 개선 필요"
    echo "현재 약 $(($ODOM_NONZERO / 3))Hz → 목표 50Hz"
else
    echo "✅ /odom_raw 발행 주기 정상"
    echo "다른 문제를 확인해야 합니다."
fi

# 정리
rm -f /tmp/odom_data.txt /tmp/imu_data.txt 2>/dev/null

echo ""
echo "=========================================="
