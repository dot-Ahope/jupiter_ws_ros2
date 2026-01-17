# 🗺️ SLAM 맵 품질 개선 가이드 (코드 수정 없이)

## 🔍 문제 증상

### 1. 벽이 두껍게 그려짐
```
정상:  |    (단일 선)
문제:  |||   (여러 선이 겹침)
```

### 2. 같은 벽이 각도 차이로 인식
```
정상:  |
       |
문제:  |
        \   (각도 벌어짐)
```

### 3. 루프 클로저 실패
- 시작점으로 돌아왔는데 맵이 맞지 않음
- 복도가 겹쳐 보임

---

## 🎯 원인 분석

### 1. 오도메트리 드리프트 (주요 원인)
```
현재 문제:
- angular_scale: 1.8819 (12.5% 오차)
- 회전 중 데이터 손실 32%
- 90° 회전 시 실제 135° 이동

결과:
- 로봇이 회전할 때 각도 오차 누적
- SLAM이 잘못된 위치에 스캔 투영
```

### 2. 회전 속도 과다
```
현재 설정:
- cmd_vel 최대 각속도: 1.0 rad/s (57°/s)
- 너무 빠른 회전 → 스캔 매칭 실패
```

### 3. SLAM 파라미터 미튜닝
```
기본값 사용 시:
- 루프 클로저 감도 낮음
- 스캔 매칭 임계값 높음
```

---

## ✅ 해결 방법 (코드 수정 없이)

## 방법 1: angular_scale 보정 (가장 중요! ⭐⭐⭐)

### 1-1. 현재 값 확인
```bash
ros2 param get /base_node angular_scale
# 결과: 1.8819
```

### 1-2. 테스트로 정확한 값 계산
```bash
cd ~/transbot_ws_ros2/src/transbot_nav/scripts
python3 ekf_comparison_test.py

# 결과에서 "권장 angular_scale" 확인
# 예: 2.1164 (테스트 결과)
```

### 1-3. Launch 파일 수정
```bash
nano ~/transbot_ws_ros2/src/transbot_nav/launch/transbot_full_system.launch.py

# Line 154 찾기:
'angular_scale': 1.8819,

# 변경:
'angular_scale': 2.1164,  # 또는 테스트 결과값
```

### 1-4. 빌드 및 재시작
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_nav --symlink-install
source install/setup.bash

# 시스템 재시작
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

**기대 효과**:
- 회전 정확도 12.5% 향상
- 벽 두께 감소
- 각도 벌어짐 현상 감소

---

## 방법 2: 회전 속도 감소 (중요! ⭐⭐)

### 2-1. 텔레옵 최대 속도 제한
```bash
# 키보드 조작 시
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.3 -p turn:=0.5

# speed: 직진 속도 (m/s)
# turn: 회전 속도 (rad/s) - 1.0 → 0.5로 감소
```

### 2-2. 조이스틱 사용 시 스케일 조정
```bash
# 만약 joy_teleop 사용 중이라면
nano ~/transbot_ws_ros2/src/transbot_bringup/config/joy_teleop.yaml

# angular 스케일 감소:
angular: 0.5  # 1.0 → 0.5
```

### 2-3. 수동 주행 시 주의사항
- **천천히 회전**: 급회전 금지
- **부드러운 조작**: 급가속/급정지 금지
- **일정한 속도 유지**: 속도 변화 최소화

**기대 효과**:
- 스캔 매칭 성공률 향상
- 맵 왜곡 감소
- 루프 클로저 성공률 증가

---

## 방법 3: SLAM 파라미터 튜닝 (중요! ⭐⭐)

### 3-1. SLAM Toolbox 설정 파일 수정
```bash
nano ~/transbot_ws_ros2/src/sllidar_ros2/config/mapper_params_online_async.yaml
```

### 3-2. 권장 파라미터 변경
```yaml
slam_toolbox:
  ros__parameters:
    # 스캔 매칭 품질 향상
    minimum_travel_distance: 0.2    # 기본: 0.5 → 0.2 (더 자주 업데이트)
    minimum_travel_heading: 0.3     # 기본: 0.5 → 0.3 (더 민감하게)
    
    # 루프 클로저 활성화 (⭐ 가장 중요)
    mode: mapping                    # 확인
    
    # 루프 클로저 검색 반경 증가
    loop_search_maximum_distance: 4.0  # 기본: 3.0 → 4.0
    
    # 스캔 매칭 해상도 향상
    resolution: 0.05                 # 기본값 확인 (작을수록 정밀)
    
    # 코릴레이션 검색 공간 증가 (정확도 향상)
    correlation_search_space_dimension: 0.5  # 기본: 0.3 → 0.5
    correlation_search_space_resolution: 0.01  # 기본값 유지
    correlation_search_space_smear_deviation: 0.1  # 기본값 유지
    
    # 루프 클로저 임계값 완화 (더 쉽게 매칭)
    loop_match_minimum_chain_size: 10    # 기본: 10 (유지)
    loop_match_maximum_variance_coarse: 3.0  # 기본: 3.0 (유지)
    loop_match_minimum_response_coarse: 0.7   # 기본: 0.7 (유지)
    
    # 링크 매칭 임계값 완화 (스캔 매칭 용이)
    link_match_minimum_response_fine: 0.5   # 기본: 0.7 → 0.5
    link_scan_maximum_distance: 1.5         # 기본: 1.5 (유지)
    
    # 최적화 빈도 증가 (지도 품질 향상)
    optimization_step_size: 10     # 몇 스캔마다 최적화 (작을수록 자주)
    do_loop_closing: true          # 루프 클로저 활성화 확인
```

### 3-3. 빌드 및 재시작
```bash
cd ~/transbot_ws_ros2
colcon build --packages-select sllidar_ros2 --symlink-install
source install/setup.bash

ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

**기대 효과**:
- 루프 클로저 성공 시 맵 자동 보정
- 스캔 매칭 정확도 향상
- 누적 오차 감소

---

## 방법 4: 맵핑 주행 기법 개선 (⭐⭐⭐)

### 4-1. 최적의 주행 패턴

#### ✅ 좋은 예
```
1. 천천히 직진 (0.2 m/s)
2. 멈춤 (1초)
3. 천천히 회전 (0.3 rad/s)
4. 멈춤 (1초)
5. 반복
```

#### ❌ 나쁜 예
```
1. 빠른 직진 (0.5 m/s)
2. 급회전 (1.0 rad/s)
3. 급정지
```

### 4-2. 주행 순서
```
1단계: 방 외곽 천천히 순회
   - 벽을 따라 시계/반시계 방향
   - 속도: 0.2 m/s 이하
   
2단계: 중앙 통과
   - 시작점 근처 통과 (루프 클로저 유도)
   
3단계: 같은 경로 역방향
   - 양방향 데이터 수집
   
4단계: 시작점 복귀
   - 루프 클로저 성공 확인
```

### 4-3. 주의사항
- 🚫 **급회전 금지**: 90° 회전 시 2-3초 소요
- 🚫 **급정지 금지**: 서서히 감속
- ✅ **특징점 지나기**: 코너, 문, 기둥 천천히
- ✅ **균일한 속도**: 일정하게 유지
- ✅ **좋은 조명**: 어두우면 LiDAR는 괜찮지만 IMU 드리프트 증가 가능

---

## 방법 5: 루프 클로저 유도 (⭐)

### 5-1. 시작점 반복 통과
```
시작점 → 목적지 → 시작점 → 목적지 → 시작점

효과:
- SLAM이 "같은 장소"임을 인식
- 자동으로 맵 보정
```

### 5-2. 루프 클로저 성공 확인
```bash
# RViz에서 확인:
# - 맵이 갑자기 보정됨 (벽이 얇아짐)
# - "Loop closure detected" 같은 메시지

# 터미널에서 확인:
ros2 topic echo /slam_toolbox/graph_visualization
# 루프 클로저 엣지 추가 확인
```

### 5-3. 강제 루프 클로저 (서비스 호출)
```bash
# 현재 위치를 시작점으로 강제 루프 클로저
ros2 service call /slam_toolbox/manual_loop_closure \
  slam_toolbox/srv/LoopClosure \
  "{}"
```

---

## 방법 6: 맵 저장 후 수동 보정 (최후 수단)

### 6-1. 맵 저장
```bash
ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/user/my_map'}}"
```

### 6-2. 이미지 편집 도구로 수정
```bash
# GIMP 설치 (없으면)
sudo apt install gimp

# 맵 열기
gimp /home/user/my_map.pgm

# 수동 편집:
# 1. 두꺼운 벽 → 얇게 (지우개 도구)
# 2. 중복 벽 제거
# 3. 저장 (같은 파일명)
```

### 6-3. 맵 재사용
```bash
# 수정한 맵으로 로컬라이제이션
ros2 launch transbot_nav transbot_localization.launch.py \
  map:=/home/user/my_map.yaml
```

---

## 방법 7: 환경 개선 (⭐)

### 7-1. 바닥 조건
- ✅ **평평한 바닥**: 카펫, 요철 최소화
- ✅ **미끄럽지 않은 바닥**: 바퀴 슬립 방지
- 🚫 **경사 금지**: 바퀴가 공전하면 엔코더 오차

### 7-2. 환경 특징
- ✅ **벽/장애물 많이**: LiDAR 특징점 증가
- ✅ **코너 명확**: 스캔 매칭 기준점
- 🚫 **긴 복도 금지**: 특징점 부족
- 🚫 **유리/거울 금지**: LiDAR 반사 오류

### 7-3. 조명
- ✅ **균일한 조명**: IMU 드리프트 감소 (간접 효과)
- 🚫 **깜빡임 금지**: 센서 안정성

---

## 📊 단계별 적용 순서

### 1단계: 즉시 적용 (15분)
```bash
# 1. angular_scale 보정
nano ~/transbot_ws_ros2/src/transbot_nav/launch/transbot_full_system.launch.py
# 'angular_scale': 2.1164 설정

# 2. 빌드
cd ~/transbot_ws_ros2
colcon build --packages-select transbot_nav --symlink-install
```

### 2단계: 파라미터 튜닝 (10분)
```bash
# SLAM 파라미터 수정
nano ~/transbot_ws_ros2/src/sllidar_ros2/config/mapper_params_online_async.yaml
# 위의 권장값 적용

# 빌드
colcon build --packages-select sllidar_ros2 --symlink-install
```

### 3단계: 재매핑 (10분)
```bash
# 시스템 시작
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 천천히 주행 (0.2 m/s, 0.3 rad/s)
# 시작점 반복 통과
# 루프 클로저 확인
```

### 4단계: 맵 저장 및 확인
```bash
# 맵 저장
ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/user/improved_map'}}"

# RViz에서 확인
# - 벽 두께
# - 각도 일치도
```

---

## 🔍 문제별 해결 우선순위

### 문제 1: 벽이 두껍게 그려짐
**원인**: 회전 시 각도 오차
**해결**:
1. ⭐⭐⭐ angular_scale 보정 (방법 1)
2. ⭐⭐ 회전 속도 감소 (방법 2)
3. ⭐ 천천히 주행 (방법 4)

### 문제 2: 각도가 벌어져 인식
**원인**: 누적 오차
**해결**:
1. ⭐⭐⭐ angular_scale 보정 (방법 1)
2. ⭐⭐ 루프 클로저 활성화 (방법 3)
3. ⭐ 시작점 반복 통과 (방법 5)

### 문제 3: 루프 클로저 실패
**원인**: SLAM 파라미터 부적합
**해결**:
1. ⭐⭐⭐ SLAM 파라미터 튜닝 (방법 3)
2. ⭐⭐ 천천히 주행 (방법 4)
3. ⭐ 환경 개선 (방법 7)

---

## 📈 기대 결과

### Before (현재)
```
벽 두께:     |||  (3-5픽셀)
각도 오차:   10-15°
루프 클로저: 실패
맵핑 시간:   10분
```

### After (개선 후)
```
벽 두께:     |   (1-2픽셀)
각도 오차:   2-5°
루프 클로저: 성공
맵핑 시간:   15분 (천천히 주행)
```

---

## 🎓 추가 팁

### Tip 1: RViz에서 실시간 확인
```
Views → Displays → Map
- Topic: /map
- Color Scheme: map
- Alpha: 0.7 (투명도 조절하여 중복 확인)
```

### Tip 2: 맵 품질 평가
```bash
# 맵 이미지 보기
eog /home/user/my_map.pgm

# 확인 사항:
# - 벽 두께 1-2픽셀
# - 직선이 직선으로
# - 사각형이 사각형으로
# - 시작점과 끝점 일치
```

### Tip 3: 로그 확인
```bash
# SLAM 경고 메시지 확인
ros2 topic echo /rosout | grep -i "slam\|loop"

# 자주 나오는 경고:
# - "Scan matching failed" → 너무 빠름
# - "Loop closure rejected" → 파라미터 조정 필요
```

---

## 🔧 트러블슈팅

### Q1: angular_scale을 바꿔도 개선 안 됨
**A**: 
1. 빌드 확인: `colcon build --packages-select transbot_nav`
2. 재시작 확인: 이전 프로세스 종료 후 재실행
3. 파라미터 확인: `ros2 param get /base_node angular_scale`

### Q2: 루프 클로저가 안 됨
**A**:
1. 시작점을 **천천히** 다시 지나가기
2. `loop_search_maximum_distance` 증가 (4.0 → 6.0)
3. 수동 루프 클로저 서비스 호출

### Q3: 맵이 계속 흔들림
**A**:
1. IMU 캘리브레이션 재실행
2. 바닥 상태 확인 (평평한지)
3. 속도 더 낮추기 (0.1 m/s)

---

## 📝 체크리스트

맵핑 시작 전 확인:

- [ ] angular_scale 보정 완료 (ekf_comparison_test 실행)
- [ ] SLAM 파라미터 튜닝 완료
- [ ] 바닥 평평하고 미끄럽지 않음
- [ ] 주변에 특징점 충분 (벽, 코너, 장애물)
- [ ] 배터리 충분 (50% 이상)
- [ ] RViz에서 /map 토픽 시각화 확인

맵핑 중 주의:

- [ ] 천천히 주행 (0.2 m/s 이하)
- [ ] 부드럽게 회전 (0.3 rad/s 이하)
- [ ] 특징점 천천히 통과
- [ ] 시작점 2회 이상 통과
- [ ] 루프 클로저 확인

맵핑 완료 후:

- [ ] 맵 저장 완료
- [ ] 벽 두께 확인 (1-2픽셀)
- [ ] 각도 일치 확인
- [ ] 시작점=끝점 확인
- [ ] 필요 시 GIMP로 수동 보정

---

## 🎯 결론

**가장 효과적인 방법** (우선순위):

1. ⭐⭐⭐ **angular_scale 보정** (12.5% 정확도 향상)
2. ⭐⭐⭐ **천천히 주행** (0.2 m/s, 0.3 rad/s)
3. ⭐⭐ **SLAM 파라미터 튜닝** (루프 클로저 활성화)
4. ⭐⭐ **시작점 반복 통과** (루프 클로저 유도)
5. ⭐ **환경 최적화** (평평한 바닥, 특징점 많이)

**기대 효과**:
- 벽 두께: 3-5픽셀 → 1-2픽셀
- 각도 오차: 10-15° → 2-5°
- 맵 품질: 중간 → 우수

**시간 투자**:
- 설정 변경: 30분
- 재매핑: 15분
- 총: 45분

코드 수정 없이도 **맵 품질을 크게 개선**할 수 있습니다! 🎉
