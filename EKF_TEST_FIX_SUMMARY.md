# 🔧 ekf_comparison_test.py 수정 내역

## 발견된 문제점

### 1. ❌ angular_scale 기본값 오류
```python
# 잘못된 코드 (Line 30)
self.declare_parameter('angular_scale', 3.776)

# 수정된 코드
self.declare_parameter('angular_scale', 1.8819)
```
- **문제**: 파라미터 기본값이 3.776으로 되어 있어 launch 파일에서 값을 전달하지 않으면 잘못된 값 사용
- **영향**: 측정값 계산에 잘못된 scale 적용

### 2. ❌ 절대값 사용으로 방향 정보 손실
```python
# 잘못된 코드
final_imu = abs(self.integrated_imu_yaw)
final_odom = abs(self.normalize_angle(self.odom_yaw - start_odom))
final_ekf = abs(self.normalize_angle(self.ekf_odom_yaw - start_ekf))

# 수정된 코드
final_imu = self.integrated_imu_yaw
final_odom = self.normalize_angle(self.odom_yaw - start_odom)
final_ekf = self.normalize_angle(self.ekf_odom_yaw - start_ekf)
```
- **문제**: 시계(CW) / 반시계(CCW) 방향 구분 불가
- **영향**: 
  - +90° 회전과 -90° 회전이 모두 +90°로 표시됨
  - 비대칭 분석 불가능
  - IMU vs Odom 비교 왜곡

### 3. ❌ 종료 조건 로직 오류
```python
# 잘못된 코드
if imu_integrated >= imu_target:  # 음수 회전 시 영원히 멈추지 않음
    break

# 수정된 코드
if direction > 0 and imu_integrated >= imu_target:  # 반시계
    break
elif direction < 0 and imu_integrated <= imu_target:  # 시계
    break
```
- **문제**: 시계 방향(-90°) 회전 시 종료 조건이 성립하지 않아 타임아웃까지 회전
- **영향**: 테스트 시간 3배 증가, 부정확한 측정

### 4. ❌ 엔코더 원시값 역계산 오류
```python
# 잘못된 코드
odom_raw = odom / self.angular_scale

# 수정된 코드
odom_encoder_raw = odom / self.angular_scale  # 실제로는 이게 맞음
```
- **문제**: 주석 설명이 부족하여 혼란 야기
- **설명**: 
  - `base_node`에서 엔코더 원시값에 `angular_scale`을 **곱해서** `/odom_raw` 생성
  - 따라서 역계산은 **나누기**가 맞음
  - 이전 생각(곱하기)이 틀렸음

## 수정 내역

### ✅ 1. 파라미터 기본값 수정
- Line 30: `3.776` → `1.8819`

### ✅ 2. 방향 정보 유지
- 모든 `abs()` 제거하여 +/- 방향 보존
- 출력 포맷 변경: `{value:6.2f}` → `{value:+7.2f}` (부호 표시)

### ✅ 3. 종료 조건 로직 개선
- 방향 변수 추가: `direction = 1.0 if speed > 0 else -1.0`
- 방향별 종료 조건 분기 처리

### ✅ 4. 로그 메시지 개선
- 진행률 계산에 0으로 나누기 방지
- 부호 표시로 방향 명확화

### ✅ 5. 비대칭 분석 개선
- 절대값 비교로 크기 차이 계산
- 원본 값도 함께 표시하여 방향 확인 가능

### ✅ 6. angular_scale 자동 분석 추가
```python
# 새로 추가된 기능
print(f'\n5️⃣  angular_scale 분석:')
print(f'   현재 설정: {node.angular_scale:.4f}')

imu_avg = (abs(math.degrees(imu_ccw[0])) + abs(math.degrees(imu_cw[0]))) / 2
odom_avg = (abs(math.degrees(imu_ccw[1])) + abs(math.degrees(imu_cw[1]))) / 2

if odom_avg > 1.0:
    actual_ratio = imu_avg / odom_avg
    recommended_scale = node.angular_scale * actual_ratio
    print(f'   권장 angular_scale: {recommended_scale:.4f}')
```
- IMU와 Odom 평균값으로 실제 비율 계산
- 권장 angular_scale 자동 계산
- 오차 범위에 따른 상태 표시 (✅/⚠️/❌)

## 기대 효과

### 이전 출력 예시 (문제 있음)
```
📊 IMU 기준 회전 결과
목표:            90.00°
📍 측정값:
  IMU 적분:      87.23° (오차: -2.77°)
  Odom (raw):   155.48° (오차: +65.48°)  ← 잘못됨
  EKF (융합):    92.15° (오차: +2.15°)
```

### 수정 후 예상 출력
```
📊 IMU 기준 회전 결과
목표:            +90.00°
📍 측정값:
  IMU 적분:      +87.23° (오차: -2.77°)
  Odom (raw):     +89.45° (오차: -0.55°)  ← 정상
  EKF (융합):     +88.67° (오차: -1.33°)

⚙️  Odom 엔코더 원시값 (angular_scale 보정 전):
  +47.53° (측정 +89.45° ÷ 1.8819)

🔄 IMU vs Odom 비율:
  0.9752 (IMU +87.23° / Odom +89.45°)
```

### 최종 비교 분석 개선
```
5️⃣  angular_scale 분석:
   현재 설정: 1.8819
   IMU 평균: 87.12°
   Odom 평균: 89.23°
   실제 비율: 0.9764
   권장 angular_scale: 1.8374
   ⚠️  angular_scale 미세 조정 권장 (오차 5-10%)
```

## 사용 방법

```bash
cd ~/transbot_ws_ros2/src/transbot_nav/scripts
python3 ekf_comparison_test.py
```

**주의**: 시스템이 실행 중이어야 함
```bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py
```

## 검증 포인트

1. ✅ **방향 정보**: +90° 와 -90°가 올바르게 표시되는지
2. ✅ **측정값 일치**: IMU와 Odom이 비슷한 값(±10° 이내)인지
3. ✅ **EKF 융합**: 두 센서의 중간값으로 융합되는지
4. ✅ **비대칭 분석**: 반시계/시계 회전이 대칭적인지
5. ✅ **권장값**: angular_scale 권장값이 현재 설정과 ±10% 이내인지
