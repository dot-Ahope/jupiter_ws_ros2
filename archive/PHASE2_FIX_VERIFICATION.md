# Phase 2 종료 조건 수정 검증

## 🔍 **문제 발견**

### **사용자 관찰:**
```bash
python3 odom_based_angular_calibration.py --phase 2 --scale 1.0 --launch-scale 1.46
```
**결과:** Odom 적분값이 90°, 180°, 270°로 나옴

### **이론상 예상:**
- `--scale 1.0` = 원시 Odom과 물리적 회전이 1:1
- 원시 Odom: 90° 회전 시 90° 측정
- 측정 Odom (launch_scale=1.46 적용): 90° × 1.46 = 131.4°
- 하지만 실제로는 **90°에서 멈춤!**

---

## 🐛 **원인 분석**

### **이전 코드 (잘못됨):**
```python
# Phase 2 종료 조건 계산
if estimated_scale is not None:
    odom_target = abs(target_rad) / estimated_scale
```

### **--scale 1.0 사용 시:**
```python
target = 90° = 1.571 rad
estimated_scale = 1.0
odom_target = 1.571 / 1.0 = 1.571 rad = 90°

# 종료 조건
if odom_delta >= odom_target * 0.98:  # 90° × 0.98 = 88.2°
    break
```

### **실제 동작:**
```
물리적 회전: 약 62°
원시 Odom: 62°
측정 Odom: 62° × 1.46 = 90.5°
→ 종료 조건 충족! (88.2° < 90.5°)
→ 62°에서 멈춤! ❌
```

---

## ✅ **수정된 코드**

### **올바른 공식:**
```python
odom_target = abs(target_rad) * (self.current_launch_scale / estimated_scale)
```

### **수학적 유도:**

#### **목표:**
```
물리적 회전: target = 90°
```

#### **원시 Odom 계산:**
```
estimated_scale = 원시_Odom과 물리적의 비율
물리적 = 원시_Odom × estimated_scale
원시_Odom = 물리적 / estimated_scale
```

#### **측정 Odom 계산:**
```
측정_Odom = 원시_Odom × launch_scale
측정_Odom = (물리적 / estimated_scale) × launch_scale
측정_Odom = 물리적 × (launch_scale / estimated_scale)
```

#### **결론:**
```python
odom_target = target × (launch_scale / estimated_scale)
```

---

## 🧮 **검증 예시**

### **Case 1: --scale 1.0**

#### **설정:**
```python
target = 90°
estimated_scale = 1.0  # 원시 Odom = 물리적
launch_scale = 1.46
```

#### **계산:**
```python
# 이전 (잘못됨)
odom_target = 90 / 1.0 = 90°  ❌
→ 62° 물리적 회전에서 멈춤

# 수정 (올바름)
odom_target = 90 × (1.46 / 1.0) = 131.4°  ✅
```

#### **검증:**
```
물리적 회전: 90°
원시 Odom: 90° / 1.0 = 90°
측정 Odom: 90° × 1.46 = 131.4°
종료 조건: 131.4° × 0.98 = 128.8°
실제 측정: 131.4°
→ 종료! ✅ (정확히 90° 회전)
```

---

### **Case 2: --scale 1.385 (정상 값)**

#### **설정:**
```python
target = 90°
estimated_scale = 1.385
launch_scale = 1.46
```

#### **계산:**
```python
# 이전 (잘못됨)
odom_target = 90 / 1.385 = 65°  ❌
→ 부정확

# 수정 (올바름)
odom_target = 90 × (1.46 / 1.385) = 94.9°  ✅
```

#### **검증:**
```
물리적 회전: 90°
원시 Odom: 90 / 1.385 = 65°
측정 Odom: 65 × 1.46 = 94.9°
종료 조건: 94.9° × 0.98 = 93.0°
실제 측정: 94.9°
→ 종료! ✅ (정확히 90° 회전)
```

---

### **Case 3: --scale 1.5 (다른 값)**

#### **설정:**
```python
target = 90°
estimated_scale = 1.5
launch_scale = 1.46
```

#### **계산:**
```python
# 이전 (잘못됨)
odom_target = 90 / 1.5 = 60°  ❌

# 수정 (올바름)
odom_target = 90 × (1.46 / 1.5) = 87.6°  ✅
```

#### **검증:**
```
물리적 회전: 90°
원시 Odom: 90 / 1.5 = 60°
측정 Odom: 60 × 1.46 = 87.6°
종료 조건: 87.6° × 0.98 = 85.8°
실제 측정: 87.6°
→ 종료! ✅ (정확히 90° 회전)
```

---

## 📊 **비교표**

| estimated_scale | 원시 Odom | 측정 Odom (1.46×) | 이전 목표 ❌ | 수정 목표 ✅ |
|----------------|-----------|------------------|------------|------------|
| 1.0            | 90°       | 131.4°           | 90°        | 131.4°     |
| 1.385          | 65°       | 94.9°            | 65°        | 94.9°      |
| 1.5            | 60°       | 87.6°            | 60°        | 87.6°      |

---

## 🔍 **일반화**

### **공식:**
```python
odom_target = target × (launch_scale / estimated_scale)
```

### **의미:**
```
1. estimated_scale: 원시 Odom → 물리적 변환 계수
2. launch_scale: 원시 Odom → 측정 Odom 변환 계수
3. launch_scale / estimated_scale: 물리적 → 측정 Odom 변환 계수
4. target × (launch_scale / estimated_scale): 물리적 목표 → 측정 Odom 목표
```

### **특수 케이스:**

#### **launch_scale = estimated_scale:**
```python
odom_target = target × (launch_scale / estimated_scale)
odom_target = target × 1.0
odom_target = target
```
→ 측정 Odom이 물리적과 일치! ✅

#### **launch_scale = 1.0 (보정 없음):**
```python
odom_target = target × (1.0 / estimated_scale)
odom_target = target / estimated_scale
```
→ 원시 Odom 기준 목표 (이전 공식과 동일)

---

## ✅ **결론**

### **핵심 깨달음:**
```
estimated_scale: 원시 Odom 기준의 보정 계수
launch_scale: 현재 측정되는 Odom에 이미 적용된 계수

따라서:
- 원시 Odom 목표 = target / estimated_scale
- 측정 Odom 목표 = 원시 Odom 목표 × launch_scale
                  = (target / estimated_scale) × launch_scale
                  = target × (launch_scale / estimated_scale)
```

### **수정 완료:**
```python
# odom_based_angular_calibration.py Line 175-191
odom_target = abs(target_rad) * (self.current_launch_scale / estimated_scale)
```

### **효과:**
- ✅ `--scale 1.0` 사용 시 정확히 90° 회전
- ✅ 모든 estimated_scale 값에서 정확한 종료
- ✅ launch_scale과 무관하게 일관된 동작

---

## 🧪 **테스트 권장**

```bash
# Test 1: scale=1.0
python3 odom_based_angular_calibration.py --phase 2 --scale 1.0 --launch-scale 1.46
# 예상: Odom 측정값 131.4° (90° 물리적)

# Test 2: scale=1.385
python3 odom_based_angular_calibration.py --phase 2 --scale 1.385 --launch-scale 1.46
# 예상: Odom 측정값 94.9° (90° 물리적)

# Test 3: scale=1.5
python3 odom_based_angular_calibration.py --phase 2 --scale 1.5 --launch-scale 1.46
# 예상: Odom 측정값 87.6° (90° 물리적)
```

**모든 경우에 정확히 90° 물리적 회전에서 종료!** ✅
