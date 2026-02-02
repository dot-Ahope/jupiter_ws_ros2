# 작업 요약 (2026-01-28)

## 1. Nav2 파라미터 최적화
**파일:** `src/jupiter_nav_VO/config/nav2_params_vslam.yaml`

로버의 물리적 특성과 안정적인 주행을 위해 파라미터를 수정했습니다.

*   **속도 제한 상향:**
    *   `max_vel_x`: 0.26 m/s → **0.45 m/s**
    *   `max_vel_theta`: 1.0 rad/s → **2.0 rad/s**
*   **가속도 제한 완화 (부드러운 주행):**
    *   `acc_lim_x`: 2.5 m/s² → **0.6 m/s²**
    *   `acc_lim_theta`: 3.2 rad/s² → **2.0 rad/s²**
    *   급출발/급정지를 방지하여 하드웨어 부하 감소.
*   **Footprint 적용:**
    *   로버 크기(L360 x W245 mm) 반영.
    *   `footprint: "[ [0.18, 0.1225], [0.18, -0.1225], [-0.18, -0.1225], [-0.18, 0.1225] ]"`

## 2. 모터 드라이버 (Jupiter Driver) 설정
**파일:** `src/jupiter_bringup/param/jupiter_driver_params.yaml`

Nav2와의 동기화 및 PWM 맵핑(Calibration)을 위한 설정을 변경했습니다.

*   **속도 한계 동기화:**
    *   Nav2의 최대 속도(0.45 m/s, 2.0 rad/s)와 일치시킴.
    *   (추후 제어 여유분을 위해 드라이버 한계를 더 높일 수 있음)
*   **PID 게인 초기화 (맵핑용):**
    *   `Kp`, `Ki`, `Kd`를 모두 **0.0**으로 설정.
    *   순수 PWM vs 속도 관계를 측정하기 위함. (맵핑 완료 후 복구 필요)

## 3. 조이스틱 제어 노드 (Jupiter Joy) 기능 추가
**파일:** `src/jupiter_ctrl/jupiter_ctrl/jupiter_joy.py`

기존 Yahboom 예제의 유용한 기능을 Jupiter 패키지로 이식하고 로직을 개선했습니다.

*   **기능 이식:**
    *   **Cancel Nav:** 버튼 9번(Start/Back)을 누르면 자율주행 취소 및 로봇 정지, 조이스틱 모드 토글.
    *   **Buzzer:** 버튼 11번(L3/Click)을 누르면 부저 울림.
*   **안전 로직:**
    *   조이스틱 모드가 비활성 상태(`Joy_active = False`)일 때 `cmd_vel` 명령을 보내지 않도록 수정.
*   **속도 파라미터:**
    *   Nav2와 동일하게 선형 0.45 m/s, 각속도 2.0 rad/s로 제한 설정.

## 4. 시스템 서비스 디버깅 및 해결
**파일:** `/home/jetson/service/joystick/start_jupiter_joystick.sh`

서비스 실행 시 의도한 파이썬 스크립트가 실행되지 않는 문제를 해결했습니다.

*   **원인:** `colcon build` 미수행으로 인한 변경 사항 미반영 및 쉘 스크립트 내 실행 경로 오설정.
*   **해결:**
    *   워크스페이스 빌드 (`colcon build --symlink-install`).
    *   시작 스크립트(`start_jupiter_joystick.sh`) 내 `source` 경로 및 `ros2 launch` 파일 경로 수정.

## 5. 유용한 명령어
*   **서비스 로그 실시간 확인:**
    ```bash
    sudo journalctl -u jupiter_joystick.service -f
    ```
*   **서비스 재시작:**
    ```bash
    sudo systemctl restart jupiter_joystick.service
    ```
