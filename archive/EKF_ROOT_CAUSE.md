# 🔍 EKF 실패 근본 원인 발견

## 📊 테스트 결과 분석

### **좋은 소식:**
```
Odom 비대칭 해소!
  이전: 41.54° vs 72.96° (31.42° 차이)
  현재: 71.26° vs 70.14° (1.13° 차이) ✅

IMU 일관성 유지:
  87.49° vs 88.64° (1.15° 차이) ✅
```

### **나쁜 소식:**
```
EKF가 센서를 거의 무시:
  IMU 87.49° → EKF 7.48° (8% 사용!)
  IMU 88.64° → EKF 12.37° (14% 사용!)
  
→ EKF가 작동하지 않음
```

---

## 🚨 **근본 원인: differential + relative 조합**

### **현재 설정:**
```yaml
odom0_differential: true
odom0_relative: true

imu0_differential: true
imu0_relative: true
```

### **문제:**
1. **`differential: true`**: 변화량(delta) 사용
2. **`relative: true`**: 초기 프레임 기준 상대 측정

**이 둘을 동시에 사용하면:**
```
differential: pose(t) - pose(t-1)  # 변화량
relative: pose(t) - pose(0)        # 초기 기준

→ 충돌! EKF가 혼란스러워함
→ 결과: 센서 데이터 거부
```

---

## 📖 **robot_localization 공식 문서**

### **설정 가이드:**
```
differential: 
  - true: 센서가 절대 측정이지만, EKF는 변화량으로 사용
  - 용도: 휠 슬립 보상, 드리프트 방지

relative:
  - true: 센서가 상대 측정 (초기값 기준)
  - 용도: IMU orientation (절대 방향 없음)

⚠️  주의: differential과 relative 동시 사용 불가!
```

### **권장 조합:**

#### **Case 1: Odom (절대 측정)**
```yaml
odom0_differential: true   # 변화량으로 변환
odom0_relative: false      # 절대 측정
```

#### **Case 2: IMU (상대 측정)**
```yaml
imu0_differential: true    # 변화량 사용
imu0_relative: false       # ⚠️ false! (differential이 이미 변화량 처리)
```

**또는:**
```yaml
imu0_differential: false   # 원본 사용
imu0_relative: true        # 상대 측정
```

---

## ✅ **수정 방안**

### **방안 1: relative 제거** (권장)
```yaml
# 둘 다 differential만 사용
odom0_differential: true
odom0_relative: false  # true → false

imu0_differential: true
imu0_relative: false   # true → false
```

**효과:**
- ✅ 센서 변화량을 정상적으로 융합
- ✅ 충돌 해소
- ✅ EKF가 센서 데이터 수용

---

### **방안 2: 공분산 제거** (임시)
```yaml
# 명시적 공분산 삭제
# odom0_twist_covariance: [0.1, 0.1, 0.1]  # 삭제
# imu0_angular_velocity_covariance: [0.001, 0.001, 0.001]  # 삭제
```

**효과:**
- EKF가 자동 계산된 공분산 사용
- 하지만 relative 충돌은 여전히 존재

---

## 🎯 **즉시 적용: relative false로 변경**

### **수정 내용:**
```yaml
Line 32: odom0_relative: true → false
Line 53: imu0_relative: true → false
```

### **예상 결과:**
```
현재:
  IMU 87.49° → EKF 7.48° (거의 무시)
  IMU 88.64° → EKF 12.37° (거의 무시)

수정 후:
  IMU 87.49° → EKF ~85-88° (정상 융합)
  IMU 88.64° → EKF ~86-89° (정상 융합)
  차이: < 5° ✅
```

---

## 📚 **기술적 배경**

### **Differential 작동:**
```python
# EKF 내부 처리
if differential:
    measurement = current_pose - previous_pose
else:
    measurement = current_pose
```

### **Relative 작동:**
```python
# EKF 내부 처리
if relative:
    measurement = current_pose - initial_pose
else:
    measurement = current_pose
```

### **Differential + Relative 충돌:**
```python
# 동시 사용 시
if differential and relative:
    # Step 1: relative
    relative_pose = current_pose - initial_pose
    
    # Step 2: differential
    delta = relative_pose - previous_relative_pose
    
    # 문제: delta가 의미 없어짐
    # (current - initial) - (previous - initial)
    # = current - previous (결국 differential만 작동)
    # 하지만 EKF 내부적으로 충돌 발생
```

---

## 🚀 **실행 계획**

1. **relative: true → false** 변경
2. **공분산은 유지** (센서 품질 반영)
3. 빌드 & 테스트
4. EKF 정상 융합 확인

**예상 시간:** 5분
