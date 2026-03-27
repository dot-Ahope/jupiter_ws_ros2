# Daily Work Log

## 2026-03-24
- Docker 단독 테스트 실행 → nvblox TF lookup 실패 (`camera_link` 미발견) 발견
- 근본 원인: Docker에서 odom→base_link→camera_link TF 체인 미발행
- 수정: `standalone` launch arg 추가 (cuVSLAM publish_odom_to_base_tf 제어)
- 수정: `base_link→camera_link` 정적 TF 퍼블리셔 추가 (URDF 기반)
- nvblox 복셀 토픽 설정 확인 — 설정 자체는 정상, TF가 원인
- 상세: 진행과정_nvblox_integration.md, 진단명령어_nvblox_integration.md

## 2026-03-23
- Astra 카메라 레거시 완전 제거 (bashrc, bringup, device_srv, calibrate_imu, udev rules)
- nvblox Nav2 costmap 통합: nav2_params_fused.yaml에 nvblox_costmap_layer 추가
- Docker VSLAM+nvblox 통합 launch 작성 (isaac_ros_vslam_nvblox_realsense.launch.py)
- Docker 내부에서 nvblox_nav2 + realsense_splitter 빌드 완료
- bashrc에 nvblox plugin 환경변수 추가
- 상세: TIL_2026_03_23_Nvblox_Integration_Astra_Cleanup.md

## 2026-01-07
- Initialized Git repository
- Created .gitignore and daily backup script
