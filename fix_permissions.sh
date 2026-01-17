#!/bin/bash
# 실행 전 문제 해결 가이드

echo "=========================================="
echo "  Transbot 실행 전 문제 해결"
echo "=========================================="
echo ""

# 1. 시리얼 포트 권한 확인
echo "1. 시리얼 포트 권한 확인..."
if groups | grep -q dialout; then
    echo "   ✓ dialout 그룹에 속해있음"
else
    echo "   ✗ dialout 그룹에 없음"
    echo "   해결: sudo usermod -a -G dialout \$USER"
    echo "   주의: 로그아웃 후 재로그인 필요!"
fi

# 임시 권한 부여 (현재 세션만)
echo ""
echo "2. 임시 시리얼 포트 권한 부여 (현재 세션)..."
if [ -c /dev/ttyTHS1 ]; then
    sudo chmod 666 /dev/ttyTHS1
    echo "   ✓ /dev/ttyTHS1 권한 설정됨 (임시)"
else
    echo "   ✗ /dev/ttyTHS1 없음"
fi

if [ -c /dev/ttyUSB0 ]; then
    sudo chmod 666 /dev/ttyUSB0
    echo "   ✓ /dev/ttyUSB0 권한 설정됨 (임시)"
fi

if [ -c /dev/ttyUSB1 ]; then
    sudo chmod 666 /dev/ttyUSB1
    echo "   ✓ /dev/ttyUSB1 권한 설정됨 (임시)"
fi

# 3. LiDAR 연결 확인
echo ""
echo "3. LiDAR 연결 확인..."
if lsusb | grep -i -E "cp210|silicon|slamtec" > /dev/null; then
    echo "   ✓ LiDAR USB 장치 감지됨"
    lsusb | grep -i -E "cp210|silicon|slamtec"
else
    echo "   ⚠ LiDAR USB 장치 미감지"
    echo "   확인: LiDAR가 USB에 연결되어 있는지 확인"
fi

# 4. IMU 캘리브레이션 파일 확인
echo ""
echo "4. IMU 캘리브레이션 파일 확인..."
if [ -f /home/jetson/transbot_ws_ros2/imu_calib.yaml ]; then
    echo "   ✓ imu_calib.yaml 파일 존재"
else
    echo "   ⚠ imu_calib.yaml 파일 없음"
    echo "   필요시 IMU 캘리브레이션 실행 필요"
fi

# 5. 환경 설정
echo ""
echo "5. ROS2 환경 설정..."
if [ -f /home/jetson/transbot_ws_ros2/install/setup.bash ]; then
    echo "   ✓ setup.bash 존재"
    echo "   실행: source /home/jetson/transbot_ws_ros2/install/setup.bash"
else
    echo "   ✗ setup.bash 없음 (빌드 필요)"
fi

echo ""
echo "=========================================="
echo "  준비 완료!"
echo "=========================================="
echo ""
echo "다음 명령으로 실행:"
echo ""
echo "  cd /home/jetson/transbot_ws_ros2"
echo "  source install/setup.bash"
echo "  ros2 launch transbot_nav transbot_full_system.launch.py use_rviz:=true"
echo ""
echo "또는 간단하게:"
echo "  newgrp dialout"
echo "  (그룹 적용 후 위 명령 실행)"
echo ""
