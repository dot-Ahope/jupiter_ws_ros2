# RPLidar 후방 설치 - 올바른 이해와 설정

## 🎯 **핵심 깨달음**

**라이다가 후방을 향하는 것은 정상적인 물리 현상이며, 데이터를 인위적으로 반전할 필요가 없습니다!**

## 📐 **물리적 설치 상황**

```
     로봇 정면 방향 (+X)
            ↑
            |
        [로봇 본체]
            |
            ↓
     라이다 설치 (후방을 향함)
```

**실제 설치:**
- 라이다 센서가 로봇 후방(-X 방향)을 향함
- URDF: `<origin xyz="-0.03 0 0.13" rpy="0 0 3.14159"/>`
- yaw = 180° (π rad) = 후방을 향함

## 🔄 **좌표 변환의 이해**

### 1. LaserScan 데이터 (laser 프레임)

```
LaserScan 메시지:
- frame_id: "laser"
- angle = 0: laser 프레임의 +X 방향 (라이다 정면)
- angle = π/2: laser 프레임의 +Y 방향 (라이다 왼쪽)
- angle = -π/2: laser 프레임의 -Y 방향 (라이다 오른쪽)
```

### 2. TF 변환 (laser → base_link)

```
URDF에서 정의:
laser_joint: rpy="0 0 3.14159" (yaw=180°)

변환 효과:
- laser +X (angle=0) → base_link -X (로봇 후방)
- laser +Y (angle=π/2) → base_link -Y (로봇 오른쪽)
- laser -Y (angle=-π/2) → base_link +Y (로봇 왼쪽)
```

### 3. SLAM/Navigation에서의 해석

```
SLAM은 자동으로 TF를 사용:
1. LaserScan을 laser 프레임에서 읽음
2. TF를 통해 base_link 프레임으로 변환
3. 맵 좌표계(map)로 최종 변환
4. 모든 포인트가 올바른 위치에 배치됨 ✓
```

## ❌ **잘못된 시도들**

### 시도 1: `inverted=True`
```python
'inverted': True  # ❌ 잘못된 접근
```
**문제:**
- ranges 배열 순서 반전 → X축 대칭 발생
- 실제 물체 위치가 좌우 반전됨
- TF가 이미 처리하는 것을 중복 처리

### 시도 2: scan_inverter 노드
```python
# ❌ 불필요한 복잡성
inverted_msg.ranges = list(reversed(msg.ranges))
```
**문제:**
- ranges 배열 반전 → X축 대칭
- angle_increment 음수 → SLAM 실패
- TF 변환과 충돌

### 시도 3: URDF에서 rpy="0 0 0"
```xml
<origin xyz="-0.03 0 0.13" rpy="0 0 0"/>  <!-- ❌ -->
```
**문제:**
- 라이다가 정면을 향한다고 거짓 정보 제공
- TF 변환이 물리적 설치와 불일치
- 모든 포인트가 잘못된 위치에 매핑됨

## ✅ **올바른 설정 (최종)**

### 1. URDF (TF 정의)

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser"/>
  <!-- 물리적 설치 그대로 표현 -->
  <origin xyz="-0.03 0 0.13" rpy="0 0 3.14159"/>
</joint>
```

**의미:**
- 라이다가 로봇 후방(-X)을 향함 (사실)
- TF 시스템이 자동으로 좌표 변환 처리

### 2. Launch 파일 (드라이버 설정)

```python
sllidar_node = Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    parameters=[{
        'frame_id': 'laser',
        'inverted': False,      # ✓ 데이터 그대로 사용
        'angle_compensate': True,
        # ... 기타 파라미터
    }]
)
```

**원칙:**
- **데이터 조작 없음**
- TF가 모든 변환 처리
- 단순하고 명확한 구조

### 3. 추가 노드 불필요

```python
# ❌ 불필요 - 삭제
scan_inverter_node = Node(...)  
```

## 🧪 **검증 방법**

### 1. TF 확인

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# laser → base_link 변환 확인
ros2 run tf2_ros tf2_echo base_link laser

# 출력 예시:
# Translation: [-0.030, 0.000, 0.130]
# Rotation: [0.000, 0.000, 1.000] (180°)
```

### 2. RViz 시각화

```bash
ros2 launch sllidar_ros2 transbot_full_system.launch.py use_rviz:=true
```

**확인 사항:**
1. Fixed Frame: `base_link` 또는 `map`
2. LaserScan 토픽 추가: `/scan`
3. TF 표시 활성화

**예상 결과:**
- 로봇 **앞**에 물체 → 라이다 스캔에 **보이지 않음** (라이다가 후방을 향함)
- 로봇 **뒤**에 물체 → 라이다 스캔에 **보임** ✓
- 로봇 **왼쪽** 뒤에 물체 → 라이다 **왼쪽**에 표시 ✓
- 로봇 **오른쪽** 뒤에 물체 → 라이다 **오른쪽**에 표시 ✓

### 3. SLAM 테스트

```bash
# SLAM 실행
ros2 launch sllidar_ros2 transbot_full_system.launch.py

# 로봇 이동
# - 후진하면서 벽을 스캔
# - 회전하면서 주변 환경 매핑
```

**예상 결과:**
- 지도가 정상적으로 생성됨 ✓
- 로봇의 이동과 지도 업데이트가 일치 ✓
- 루프 클로저가 정상 작동 ✓

## 📊 **회전 방향 이해**

**물리적 현상:**
```
로봇이 시계방향 회전 (CW):
→ 라이다가 보는 물체는 반시계방향(CCW)으로 회전하는 것처럼 보임
→ 이것은 정상! (카메라를 돌리면 배경이 반대로 움직이는 것과 동일)
```

**SLAM에서의 처리:**
```
SLAM은:
1. 이전 스캔과 현재 스캔을 비교
2. TF를 통해 로봇의 실제 회전 파악
3. 스캔 매칭으로 정확한 위치 추정
4. 모든 것이 일관되게 작동 ✓
```

## 🎯 **핵심 원칙**

### 1. **URDF는 물리적 사실을 표현**
- 라이다가 후방을 향함 → `yaw=180°`
- 거짓 정보를 넣지 말 것

### 2. **TF가 모든 변환을 처리**
- laser 프레임 → base_link 프레임
- base_link 프레임 → odom 프레임
- odom 프레임 → map 프레임

### 3. **데이터는 조작하지 않음**
- LaserScan 데이터는 그대로
- `inverted=False`
- 별도 변환 노드 불필요

### 4. **단순함이 최고**
- 복잡한 변환 체인 피하기
- TF 시스템 신뢰하기
- 표준 규칙 따르기

## 📝 **최종 구성**

```
시스템 구성:
1. RPLidar 드라이버 → /scan (laser 프레임)
2. robot_state_publisher → TF (laser → base_link)
3. SLAM Toolbox → /map (자동으로 TF 사용)
4. Navigation2 → 경로 계획 (자동으로 TF 사용)

데이터 흐름:
RPLidar → LaserScan(laser) → [TF] → base_link → [TF] → map
                               ↑
                            URDF 정의
                         yaw=180°
```

## 🎉 **결론**

**라이다가 후방을 향하는 것은 문제가 아닙니다!**

- ✅ URDF에 물리적 설치 정확히 표현
- ✅ LaserScan 데이터 그대로 사용
- ✅ TF가 자동으로 모든 변환 처리
- ✅ SLAM/Navigation 정상 작동
- ✅ 단순하고 유지보수 쉬움

**불필요한 것:**
- ❌ `inverted=True`
- ❌ scan_inverter 노드
- ❌ 수동 각도 변환
- ❌ ranges 배열 조작

**핵심:**
> **TF를 믿어라. 데이터를 조작하지 마라.**
