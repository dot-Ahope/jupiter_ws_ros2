# 🔍 데이터 흐름 및 보정값 적용 분석

**작성일**: 2025-10-17  
**문제**: SLAM 상 각도와 실제 로버 회전 각도 불일치

---

## 📊 현재 데이터 흐름

### 1. Raw 데이터 수집
```
transbot_driver.py → /transbot/get_vel (휠 인코더)
                   → /transbot/imu (IMU 원시 데이터)
```

### 2. 오도메트리 계산 (base_node)
```
/transbot/get_vel → base_node (angular_scale=1.56 적용) → /odom_raw
```
**✅ angular_scale = 1.56 보정 적용됨**

### 3. IMU 보정 체인
```
/transbot/imu → imu_calib (imu_calib.yaml) → /imu/data_calibrated
              → imu_filter_madgwick → /imu/data_filtered
```
**⚠️ imu_calib.yaml: 언제 캘리브레이션 했는지 불명확**

### 4. EKF 융합
```
/odom_raw (angular_scale=1.56 적용됨) ─┐
                                       ├─→ EKF → /odometry/filtered
/imu/data_filtered (보정+필터링됨) ────┘
```

### 5. SLAM 입력
```
SLAM Toolbox 설정:
  - odom_frame: odom
  - base_frame: base_footprint
  - scan_topic: /scan
```

**❌ 문제: SLAM이 어떤 오도메트리를 사용하는지 불명확!**

---

## 🎯 핵심 문제점

### 문제 1: SLAM이 사용하는 오도메트리가 불명확

**SLAM Toolbox는 TF를 통해 오도메트리 정보를 받습니다:**

```yaml
# slam_params.yaml
odom_frame: odom
base_frame: base_footprint
```

**TF 트리**:
- EKF가 `odom → base_footprint` TF 발행
- base_node가 `odom → base_link` TF 발행 (raw)

**충돌 가능성**: 두 노드가 같은 `odom` 프레임에서 서로 다른 자식 프레임으로 TF를 발행!

### 문제 2: base_node가 발행하는 TF가 SLAM에 영향

**현재 설정**:
```python
# transbot_full_system.launch.py
transbot_base_node = Node(
    parameters=[{
        'angular_scale': 1.56,  # 90° 회전 테스트용 보정
    }]
)
```

**base_node는 두 가지를 발행**:
1. `/odom_raw` 토픽 (데이터)
2. `odom → base_link` TF 변환 (좌표계)

**문제**: SLAM이 `base_footprint`를 사용하는데, base_node는 `base_link`로 TF 발행!

### 문제 3: IMU 캘리브레이션 데이터 오래됨

```yaml
# ekf_config.yaml
imu0: /imu/data_filtered
imu0_config: [false, false, false,
              false, false, false,  # yaw 각도 비활성화
              false, false, false,
              false, false, true,   # yaw 각속도만 활성화
              false, false, false]
```

**IMU는 yaw 각속도만 제공**하는데:
- `imu_calib.yaml`이 오래된 캘리브레이션이면
- 각속도 데이터가 부정확
- EKF 융합 결과가 왜곡됨

---

## 💡 해결 방안

### Option A: base_node의 TF 발행 비활성화 (추천) ⭐

**base_node가 TF를 발행하지 않도록 설정:**

```cpp
// base.cpp에서 publish_tf 파라미터 추가
this->declare_parameter("publish_tf", false);
publish_tf_ = this->get_parameter("publish_tf").as_bool();

if (publish_tf_) {
    tf_broadcaster_->sendTransform(odom_trans);
}
```

**이유**:
- EKF가 `odom → base_footprint` TF를 발행
- SLAM이 `base_footprint` 사용
- base_node는 `/odom_raw` 토픽만 발행하면 충분

### Option B: base_frame 통일

**SLAM 설정을 `base_link`로 변경:**

```yaml
# slam_params.yaml
base_frame: base_link  # base_footprint → base_link
```

**문제점**: EKF가 `base_footprint` 사용 → 불일치

### Option C: IMU 재캘리브레이션 ⭐

**IMU 캘리브레이션 다시 수행:**

```bash
# 1. IMU 캘리브레이션 데이터 수집
ros2 launch imu_calib calibrate.launch.py

# 2. imu_calib.yaml 업데이트
# 3. 시스템 재시작
```

---

## 🔧 권장 조치 순서

### 1단계: TF 트리 확인 (즉시)

```bash
# 시스템 실행 중
ros2 run tf2_tools view_frames
# frames_YYYY-MM-DD_HH.MM.SS.gv 생성됨
evince frames_*.pdf
```

**확인 사항**:
- `odom → base_link` 누가 발행? (base_node)
- `odom → base_footprint` 누가 발행? (EKF)
- 충돌 여부 확인

### 2단계: base_node TF 발행 비활성화

**이유**: EKF가 융합된 오도메트리로 TF 발행해야 함

```python
# transbot_full_system.launch.py
transbot_base_node = Node(
    parameters=[{
        'linear_scale': 1.2,
        'angular_scale': 1.56,
        'publish_tf': False,  # TF 발행 비활성화 ⭐
        'is_multi_robot': False
    }]
)
```

### 3단계: IMU 재캘리브레이션 (선택)

**현재 IMU 캘리브레이션 확인:**

```bash
cat /home/user/transbot_ws_ros2/imu_calib.yaml
```

**오래되었다면 재캘리브레이션 수행**

### 4단계: SLAM 테스트

```bash
# 1. 시스템 시작
sudo systemctl restart yahboomcar_bringup

# 2. SLAM 확인
ros2 topic echo /odometry/filtered --once

# 3. 회전 테스트
# - RViz에서 SLAM 맵 관찰
# - 90° 회전 명령
# - SLAM 상 회전각과 실제 회전각 비교
```

---

## 📋 체크리스트

- [ ] TF 트리 확인 (`view_frames`)
- [ ] base_node TF 발행 확인 (충돌 여부)
- [ ] EKF가 사용하는 오도메트리 확인 (`/odom_raw`)
- [ ] SLAM이 사용하는 base_frame 확인 (`base_footprint`)
- [ ] IMU 캘리브레이션 날짜 확인
- [ ] base_node에 `publish_tf` 파라미터 추가
- [ ] IMU 재캘리브레이션 필요 시 수행

---

## 🎯 예상 결과

### 수정 전
```
base_node (angular_scale=1.56) → odom→base_link TF (부정확)
EKF → odom→base_footprint TF (융합됨)
SLAM → base_footprint 사용 (EKF 데이터)

문제: base_node의 1.56 보정이 SLAM에 직접 영향 안 함!
```

### 수정 후
```
base_node → /odom_raw 토픽만 발행 (angular_scale=1.56)
EKF → /odom_raw 융합 → odom→base_footprint TF
SLAM → base_footprint 사용 (보정된 데이터)

해결: 보정값이 EKF를 거쳐 SLAM에 전달됨 ✅
```

---

## 📊 데이터 흐름 다이어그램

```
[휠 인코더] → /transbot/get_vel
                ↓
         [base_node]
    (angular_scale=1.56)
                ↓
          /odom_raw 토픽 ────────┐
          odom→base_link TF ❌    │
                                 │
[IMU] → /transbot/imu            │
          ↓                      │
    [imu_calib]                  │
    (imu_calib.yaml ⚠️)          │
          ↓                      │
    /imu/data_calibrated         │
          ↓                      ↓
    [imu_filter]            [EKF 융합]
          ↓                      ↓
    /imu/data_filtered → /odometry/filtered
                         odom→base_footprint TF ✅
                                 ↓
                          [SLAM Toolbox]
                         (base_footprint 사용)
```

---

**결론**: `angular_scale=1.56` 보정은 `/odom_raw` 토픽에 적용되지만,  
SLAM이 사용하는 TF는 **EKF가 발행**하므로 보정값이 EKF를 거쳐야 합니다!

현재 문제는 **base_node가 TF도 발행**해서 충돌 가능성이 있습니다.
