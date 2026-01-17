# u-blox F9P RTK GPS 센서 설정 가이드

## 개요
이 문서는 u-blox F9P RTK GPS 센서를 transbot 로봇에 통합하기 위한 설정 가이드입니다.

## udev 규칙 설정
u-blox F9P RTK GPS 센서를 항상 같은 장치 이름으로 인식하기 위해 udev 규칙을 설정합니다:

```bash
sudo cp /home/user/transbot_ws_ros2/src/transbot_bringup/udev/99-ublox-f9p.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 장치 확인
udev 규칙 설정 후, 다음 명령어로 장치가 `/dev/ttyACM_UBLOX`로 생성되었는지 확인합니다:
```bash
ls -l /dev/ttyACM*
```

만약 장치 이름이 다르다면 `f9p_rover_params.yaml` 파일에서 `device` 경로를 수정하세요.

## 빌드 및 실행
1. 워크스페이스 빌드:
```bash
cd /home/user/transbot_ws_ros2
colcon build --symlink-install --packages-select transbot_bringup
```

2. 센서 실행:
```bash
source install/setup.bash
ros2 launch transbot_bringup bringup.launch.py
```

## 토픽 확인
GPS가 올바르게 동작하는지 확인하려면 다음 명령어를 사용합니다:
```bash
ros2 topic echo /ublox_gps/fix
ros2 topic echo /ublox_gps/fix_velocity
```

## 문제 해결
1. 장치 권한 문제:
```bash
sudo chmod 666 /dev/ttyACM0
```

2. GPS 신호 확인:
```bash
ros2 topic hz /ublox_gps/fix
```

3. 디버깅 모드 활성화:
`f9p_rover_params.yaml` 파일에서 `debug` 값을 1 또는 2로 변경하여 상세 로그를 확인할 수 있습니다.
