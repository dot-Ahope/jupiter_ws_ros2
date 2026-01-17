#!/bin/bash

echo "=== Astra 카메라 USB 권한 설정 ==="
echo ""

# USB 규칙 파일 복사
echo "1. USB 규칙 파일 복사..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo cp "$SCRIPT_DIR/99-astra-camera.rules" /etc/udev/rules.d/

# udev 규칙 리로드
echo "2. udev 규칙 리로드..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 사용자를 plugdev 그룹에 추가
echo "3. 사용자를 plugdev 그룹에 추가..."
sudo usermod -a -G plugdev $USER

echo ""
echo "✅ 완료!"
echo ""
echo "⚠️  변경사항 적용을 위해 다음 중 하나를 수행하세요:"
echo "   1. 로그아웃 후 다시 로그인"
echo "   2. 또는 카메라 USB 재연결"
echo "   3. 또는 시스템 재부팅"
echo ""
