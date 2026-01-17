# 작업 요약 보고서 (Work Summary Report)

## 1. 문제 상황 (Issue)
* **초기 상황**: 휠 인코더를 물리적으로 제거했으나 EKF 설정에는 남아있어 오도메트리 오류 발생.
* **주요 증상**: 
  * IMU 데이터가 모두 `0.0`으로 출력됨 (하드웨어 통신 불량).
  * SLAM 맵은 생성되나 로봇 위치가 업데이트되지 않음 (Localization Frozen).
  * 드라이버 노드 재부팅 시 간헐적으로만 작동.

## 2. 해결 조치 (Resolution)

### A. EKF 설정 변경 (`jupiter_nav/config/ekf_config.yaml`)
* **Odom 비활성화**: 휠 인코더(`odom0`) 항목을 주석 처리하여 EKF가 IMU 데이터에만 의존하도록 변경.
* **IMU 신뢰도 조정**: 
  * `imu0`의 각속도(Angular Velocity) 공분산을 `0.000025`로 설정하여 회전 데이터 신뢰도 대폭 상향.
  * 프로세스 노이즈(Process Noise) 조정으로 예측 반응성 개선.

### B. 드라이버 초기화 로직 개선 (`jupiter_bringup/jupiter_driver.py`)
* **재시도 메커니즘 도입**: 하드웨어 초기화 실패 시 3회 재시도 로직 추가.
* **Watchdog(감시) 기능 추가**: 
  * `publish_imu` 루프 내에서 데이터가 1초 이상 모두 `0.0`일 경우 통신 끊김으로 간주.
  * 자동으로 `set_auto_report_state(True)`를 호출하여 데이터 스트리밍 재활성화.

### C. 시리얼 통신 라이브러리 안정화 (`Rosmaster_Lib.py`)
* **스레드 보호**: `__receive_data` 스레드 내부에 `try-except` 블록 추가.
* **효과**: 일시적인 시리얼 읽기 오류가 발생해도 스레드가 죽지 않고 복구되어 데이터 수신 유지.

## 3. 현재 상태 (Current Status)
* **IMU 데이터**: 정상 수신 확인 (Z축 가속도 약 -9.8, 각속도 반응).
* **시스템**: `jupiter_full_system.launch.py` 실행 시 드라이버, EKF, SLAM 노드 정상 구동.

## 4. 남은 과제 (Remaining Tasks)
* **방향 반전 문제**: Rviz 상에서 로봇 거동이 실제와 반대로 나타나는 현상 (URDF 또는 IMU 축 설정 확인 필요).
* **TF/Time Sync 경고**: 로그에 나타나는 "Message Filter dropping message" 경고 분석 및 해결 필요.
