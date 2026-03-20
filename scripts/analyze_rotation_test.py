#!/usr/bin/env python3
"""180° 회전 테스트 데이터 종합 분석"""
import csv, math, sys

CSV_PATH = "/home/jetson/jupiter_ws_ros2/test_results/rotation_180_test_20260320_124132.csv"
MAP_ODOM_YAW_OFFSET = 18.6  # degrees (measured via tf lookup)

with open(CSV_PATH) as f:
    rows = list(csv.DictReader(f))
for r in rows:
    for k in r:
        r[k] = float(r[k])

print("=" * 80)
print("COMPREHENSIVE 180° ROTATION TEST ANALYSIS")
print("=" * 80)

# 1. Phase identification
print("\n[1] PHASE IDENTIFICATION")
print(f"  Initial odom yaw: {rows[0]['odom_yaw_deg']:.1f}°")
print(f"  Final odom yaw:   {rows[-1]['odom_yaw_deg']:.1f}°")
odom_rotation = rows[-1]['odom_yaw_deg'] - rows[0]['odom_yaw_deg']
print(f"  Odom rotation:    {odom_rotation:.1f}°")
print(f"  map→odom yaw offset: {MAP_ODOM_YAW_OFFSET:.1f}°")

target_odom = rows[0]['odom_yaw_deg'] + 180
if target_odom > 180:
    target_odom -= 360
map_target = target_odom + MAP_ODOM_YAW_OFFSET
if map_target > 180:
    map_target -= 360
print(f"  Goal yaw (set in map frame): {map_target:.1f}°")
map_initial = rows[0]['odom_yaw_deg'] + MAP_ODOM_YAW_OFFSET
map_final = rows[-1]['odom_yaw_deg'] + MAP_ODOM_YAW_OFFSET
actual_needed_rotation = map_target - map_initial
if actual_needed_rotation > 180: actual_needed_rotation -= 360
if actual_needed_rotation < -180: actual_needed_rotation += 360
print(f"  Needed rotation (map frame): {actual_needed_rotation:.1f}°")
final_map_err = map_target - map_final
if final_map_err > 180: final_map_err -= 360
if final_map_err < -180: final_map_err += 360
print(f"  Final map-frame error (est): {final_map_err:.1f}°")
print(f"  Final odom-frame error:      {rows[-1]['heading_error_deg']:.1f}°")
print(f"  → Discrepancy = map-odom yaw offset ({MAP_ODOM_YAW_OFFSET:.1f}°)")

# 2. Controller identification
print("\n[2] CONTROLLER IDENTIFICATION")
phase1_end = None
for r in rows:
    if r['cmd_vel_nav_wz'] > 0.01 and r['cmd_vel_nav_wz'] < 0.99:
        phase1_end = r['time']
        break
first_cmd = None
for r in rows:
    if r['cmd_vel_nav_wz'] > 0.01:
        first_cmd = r
        break
print(f"  First nav cmd: t={first_cmd['time']:.3f}, wz={first_cmd['cmd_vel_nav_wz']:.4f}")
print(f"  Full-speed phase (nav_wz=1.0): t=0.1 ~ t={phase1_end:.3f}")
print(f"  rotate_to_heading_angular_vel=0.5 (param)")
print(f"  Actual output=1.0 (max_vel_theta)")
print(f"  ★ RotationShim NOT ACTIVE")
print(f"    Reason: same-position goal → 0-length path → no forward_sampling_distance point")
print(f"    DWB RotateToGoal controls directly")

# 3. Velocity tracking
print("\n[3] VELOCITY TRACKING (cmd_vel_wz vs odom_wz)")
max_lag = 0
max_lag_time = 0
total_lag_integral = 0
prev_t = None
for r in rows:
    if r['time'] < 0.5:
        prev_t = r['time']
        continue
    lag = abs(r['odom_wz'] - r['cmd_vel_wz'])
    if lag > max_lag:
        max_lag = lag
        max_lag_time = r['time']
    if prev_t:
        dt = r['time'] - prev_t
        total_lag_integral += lag * dt
    prev_t = r['time']

print(f"  Max tracking lag: {max_lag:.3f} rad/s at t={max_lag_time:.3f}s")
print(f"  Integral |odom_wz - cmd_vel_wz| dt: {total_lag_integral:.3f} rad·s")
print(f"  Extra rotation from tracking error: {math.degrees(total_lag_integral):.1f}°")

max_odom_wz = max(r['odom_wz'] for r in rows)
max_odom_t = next(r['time'] for r in rows if r['odom_wz'] == max_odom_wz)
overshoot_pct = ((max_odom_wz - 1.0) / 1.0) * 100
print(f"  Peak odom_wz: {max_odom_wz:.3f} rad/s at t={max_odom_t:.3f}s ({overshoot_pct:+.1f}% vs cmd 1.0)")

# 4. Deceleration analysis
print("\n[4] DECELERATION PHASE (Nav2 cmd drops from 1.0)")
decel_start_t = None
for r in rows:
    if r['cmd_vel_nav_wz'] > 0.01 and r['cmd_vel_nav_wz'] < 0.99:
        decel_start_t = r['time']
        break
nav_zero_t = None
for r in rows:
    if r['time'] > 2.5 and r['cmd_vel_nav_wz'] == 0:
        nav_zero_t = r['time']
        break

print(f"  Decel start (nav<1.0): t={decel_start_t:.3f}")
print(f"  Nav2 stops (nav=0):    t={nav_zero_t:.3f}")
print(f"  Decel duration: {nav_zero_t - decel_start_t:.2f}s")

err_decel = next(r['heading_error_deg'] for r in rows if r['time'] >= decel_start_t)
odom_wz_decel = next(r['odom_wz'] for r in rows if r['time'] >= decel_start_t)
print(f"  At decel start: error={err_decel:.1f}° (odom), odom_wz={odom_wz_decel:.3f}")

err_stop = next(r['heading_error_deg'] for r in rows if r['time'] >= nav_zero_t)
odom_wz_stop = next(r['odom_wz'] for r in rows if r['time'] >= nav_zero_t)
print(f"  At nav stop:    error={err_stop:.1f}° (odom), odom_wz={odom_wz_stop:.3f}")
print(f"  ★ Motor still spinning at {odom_wz_stop:.3f} rad/s when Nav2 sends 0!")

# Coast phase
coast_end_t = None
for r in rows:
    if r['time'] > nav_zero_t and abs(r['odom_wz']) < 0.05:
        coast_end_t = r['time']
        break
if coast_end_t:
    err_coast = next(r['heading_error_deg'] for r in rows if r['time'] >= coast_end_t)
    print(f"  Coast end (wz<0.05):  t={coast_end_t:.3f}")
    print(f"  Coast duration: {coast_end_t - nav_zero_t:.2f}s")
    print(f"  Coast extra rotation: {err_stop - err_coast:.1f}° (odom)")

# 5. Velocity smoother
print("\n[5] VELOCITY SMOOTHER")
ramp_start = next(r['time'] for r in rows if r['cmd_vel_wz'] > 0.01)
ramp_full = next(r['time'] for r in rows if r['cmd_vel_wz'] >= 0.99)
print(f"  Ramp 0→1.0: t={ramp_start:.3f}~{ramp_full:.3f} ({ramp_full - ramp_start:.3f}s)")
print(f"  Expected: 1.0/2.5 = {1.0/2.5:.2f}s (max_accel=2.5)")
cmd_zero_t = None
for r in rows:
    if r['time'] > 3.0 and r['cmd_vel_wz'] < 0.01:
        cmd_zero_t = r['time']
        break
print(f"  cmd_vel→0: t={cmd_zero_t:.3f} (vs nav→0 at {nav_zero_t:.3f})")
print(f"  Smoother delay at stop: {cmd_zero_t - nav_zero_t:.3f}s")

# Check smoother deadband: angular deadband = 0.05
print(f"  Angular deadband: 0.05 rad/s")
small_nav = [(r['time'], r['cmd_vel_nav_wz'], r['cmd_vel_wz'])
             for r in rows if 0.01 < r['cmd_vel_nav_wz'] < 0.06]
if small_nav:
    print(f"  Nav values < 0.06 (near deadband):")
    for t, nav, cmd in small_nav[:3]:
        print(f"    t={t:.3f}: nav={nav:.4f} → cmd={cmd:.4f}")

# 6. Motor response
print("\n[6] MOTOR RESPONSE CHARACTERISTICS")
cmd_first = next(r['time'] for r in rows if r['cmd_vel_wz'] > 0.1)
odom_first = next(r['time'] for r in rows if r['odom_wz'] > 0.1)
print(f"  cmd_vel > 0.1: t={cmd_first:.3f}")
print(f"  odom_wz > 0.1: t={odom_first:.3f}")
print(f"  Motor startup delay: {odom_first - cmd_first:.3f}s")

print("\n  Velocity tracking table:")
print(f"  {'time':>6s}  {'cmd_wz':>8s}  {'odom_wz':>8s}  {'lag':>8s}  {'err(odom)':>10s}")
for spot in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.2, 3.5, 3.8, 4.0, 4.2, 4.4, 4.6]:
    c = min(rows, key=lambda r: abs(r['time'] - spot))
    lag = c['odom_wz'] - c['cmd_vel_wz']
    print(f"  {c['time']:6.2f}  {c['cmd_vel_wz']:8.4f}  {c['odom_wz']:8.4f}  {lag:+8.4f}  {c['heading_error_deg']:10.1f}°")

# 7. Non-standard DWB values
print("\n[7] DWB VELOCITY SAMPLE ANALYSIS")
step = 2 * 1.0 / 19
expected = [round(i * step, 4) for i in range(1, 10)] + [1.0]
print(f"  Expected samples: {expected}")
nav_decel = sorted(set(round(r['cmd_vel_nav_wz'], 4) for r in rows
                       if 2.8 < r['time'] < 4.3 and r['cmd_vel_nav_wz'] > 0.01))
print(f"  Observed (decel): {nav_decel}")
print(f"  Matching closest expected:")
for v in nav_decel:
    closest = min(expected, key=lambda e: abs(e - v))
    diff = v - closest
    print(f"    {v:.4f} → nearest sample {closest:.4f} (Δ={diff:+.4f})")

# 8. Summary
print("\n" + "=" * 80)
print("PROBLEM SUMMARY")
print("=" * 80)
print("""
P1. RotationShim 미활성화
    - 동일 위치 목표 → 경로 길이 0 → forward_sampling_distance 기준점 없음
    - DWB가 max_vel_theta=1.0으로 직접 회전 (0.5 대신 2배 속도)

P2. 속도 추종 지연 (velocity tracking lag)
    - 감속 시 odom_wz >> cmd_vel_wz (최대 {max_lag:.3f} rad/s 차이)
    - 누적 추가 회전: {math.degrees(total_lag_integral):.1f}°
    - Motor LPF (α=0.3) + PID 응답 지연이 원인

P3. 관성 코스팅 (inertia coasting)
    - Nav2 정지 명령 시 odom_wz={odom_wz_stop:.3f} rad/s (즉시 정지 불가)
    - 코스트 추가 회전: {err_stop - err_coast:.1f}°

P4. PID 오버슈트
    - 정상속도 cmd=1.0 대비 odom peak={max_odom_wz:.3f} ({overshoot_pct:+.1f}%)

P5. odom→map yaw 드리프트
    - 누적 18.6° 오프셋 (이전 운행에서 축적)
    - 테스트 자체에서는 약 0.5° 오차 (양호)
""")
