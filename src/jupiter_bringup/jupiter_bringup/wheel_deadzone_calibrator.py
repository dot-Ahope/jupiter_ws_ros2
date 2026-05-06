#!/usr/bin/env python3
"""
Wheel Effective Deadzone & PID Response Calibrator (v2)
========================================================

Jupiter 차동구동 (M1=LEFT, M3=RIGHT) 의 wheel-level effective deadzone 과 PID
응답을 측정.

전제 조건:
  1. 차륜을 들어 올려 자유 회전 가능 상태로 (공중)
  2. jupiter_driver 종료 — 시리얼 포트 점유 해제 (`pkill -f jupiter_driver`)
  3. 펌웨어가 FUNC_REPORT_WHEEL_SPEED (0x08) 송출 중

실행:
  ros2 run jupiter_bringup wheel_deadzone_calibrator             # both phases
  ros2 run jupiter_bringup wheel_deadzone_calibrator --phase deadzone
  ros2 run jupiter_bringup wheel_deadzone_calibrator --phase pid

동작:
  Phase 1 v2 — Effective Deadzone (closed-loop, set_car_motion)
    - Linear sweep: vx ∈ [0.005, 0.010, ..., 0.100] forward + reverse (20 step)
    - Rotation sweep: ω ∈ [0.05, 0.10, ..., 0.50] CCW + CW (14 step)
    - 각 step 1.5s 유지, 마지막 200ms 평균
    - per-wheel ratio 측정 (actual / target) → M1 / M3 비대칭 정량화
    - effective deadzone = ratio ≥ 50% 도달하는 최소 target

  Phase 2 — PID step response
    - target vx 0.20, 0.10, 0.05 / ω 0.40, 0.20, 0.10
    - rise time, steady-state, ratio

소요 시간: Phase 1 v2 ~1.5분, Phase 2 ~1분, 합 ~3분.
결과 저장: /tmp/deadzone_calib.yaml

v1 (open-loop set_motor) 은 펌웨어의 motor_data.speed_mm_s[] 가 PID 루프 안
에서만 갱신되는 한계로 측정 불가 → v2 에서 제거.
"""
import argparse
import sys
import time
from collections import deque

try:
    import yaml
except ImportError:
    yaml = None

try:
    from jupiter_bringup.Rosmaster_Lib import Rosmaster
except ImportError as e:
    print(f"FATAL: Rosmaster_Lib import 실패. workspace setup 후 재실행: {e}")
    sys.exit(1)


DEVICE = "/dev/myserial"
OUTPUT_PATH = "/tmp/deadzone_calib.yaml"


# =============================================================================
# 공통 유틸
# =============================================================================
def connect_mcu():
    """Rosmaster 연결 + auto-report 활성"""
    print("[connect] /dev/myserial 연결 중...")
    try:
        bot = Rosmaster(car_type=1, com=DEVICE, delay=0.002)
        bot.create_receive_threading()
        time.sleep(0.3)
        for _ in range(3):
            bot.set_auto_report_state(True, forever=False)
            time.sleep(0.1)
        bot.set_motor(0, 0, 0, 0)
        time.sleep(0.3)
        print("[connect] OK")
        return bot
    except Exception as e:
        print(f"[connect] FAILED: {e}")
        print("  jupiter_driver 가 실행 중일 가능성. 다음 명령으로 종료 후 재시도:")
        print("    pkill -f jupiter_driver")
        sys.exit(1)


def stop_all(bot):
    bot.set_motor(0, 0, 0, 0)
    time.sleep(0.2)


def warn_lift_wheels():
    print()
    print("=" * 60)
    print("WARNING — 차륜이 공중에서 자유 회전 가능한지 확인하세요.")
    print("바닥 접촉 상태에서는 마찰 + 부하로 측정값 부정확.")
    print("=" * 60)
    try:
        input("준비 완료 시 ENTER 입력 (Ctrl-C 로 취소): ")
    except KeyboardInterrupt:
        print("\n취소됨")
        sys.exit(0)


# =============================================================================
# Phase 1 v2 — Closed-loop Deadzone (set_car_motion)
# =============================================================================
# 이전 v1 (open-loop set_motor) 은 제거됨. 이유: 펌웨어 motor_data.speed_mm_s[]
# 가 PID 루프 안에서만 갱신되어 set_motor 직접 PWM 시 wheel_speeds 측정 불가.
# v2 는 set_car_motion (closed-loop) 으로 PID 활성 상태에서 측정.

def phase_deadzone(bot) -> dict:
    """
    [Phase 1 v2 — closed-loop, set_car_motion 사용]

    이전 (open-loop set_motor) 은 펌웨어 한계로 측정 불가:
      - motor_data.speed_mm_s[] 는 PID 루프 안에서만 갱신됨
      - set_motor 는 PID bypass 직접 PWM → speed_mm_s[] 동결 → wheel_speeds 보고가 stale

    v2: set_car_motion 으로 closed-loop sweep. PID 가 동작 중이라 wheel_speeds 정상 갱신.
    측정 결과는 "effective deadzone" — 모터 raw deadzone 이 아니라 PID 통과 후의 실효 임계.
    path-following 의 실제 거동을 가장 잘 예측하는 값.
    """
    print("\n" + "=" * 60)
    print("Phase 1 v2 — Effective Deadzone (closed-loop set_car_motion)")
    print("=" * 60)

    L = 0.245  # wheelbase (m), wheel_diff = ω × L/2 = ω × 0.1225

    # ---------- Linear sweep ----------
    print("\n[Linear sweep — vx forward/reverse]")
    print(f"{'target':>7s} | {'actual vx':>9s} | {'wheel_L':>8s} | {'wheel_R':>8s} | {'M1 ratio':>8s} | {'M3 ratio':>8s}")
    print("-" * 80)

    linear_targets = [0.005, 0.010, 0.015, 0.020, 0.030, 0.040, 0.050, 0.060, 0.080, 0.100]
    linear_results = []

    for sign, label in [(+1, "forward"), (-1, "reverse")]:
        print(f"\n--- Linear {label} ---")
        for tx in linear_targets:
            target_vx = sign * tx
            actual_vx, actual_wz, wL, wR = step_response_short(bot, target_vx, 0.0)
            # ratio: actual wheel speed magnitude / target speed magnitude
            ratio_M1 = abs(wL) / tx if tx > 1e-4 else 0
            ratio_M3 = abs(wR) / tx if tx > 1e-4 else 0
            linear_results.append({
                "target_vx": target_vx,
                "actual_vx": actual_vx,
                "wL": wL, "wR": wR,
                "ratio_M1": ratio_M1, "ratio_M3": ratio_M3,
            })
            print(f"  {target_vx:+.3f} | {actual_vx:+.3f}    | {wL:+.3f}   | {wR:+.3f}   | "
                  f"{ratio_M1*100:6.0f}%  | {ratio_M3*100:6.0f}%")

    # ---------- Rotation sweep ----------
    print("\n[Rotation sweep — ω CCW(left)/CW(right)]")
    print(f"{'target_ω':>8s} | {'actual_ω':>8s} | {'wheel_L':>8s} | {'wheel_R':>8s} | "
          f"{'M1 ratio':>8s} | {'M3 ratio':>8s}")
    print("-" * 80)

    rot_targets = [0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50]
    rotation_results = []

    for sign, label in [(+1, "CCW (left turn)"), (-1, "CW (right turn)")]:
        print(f"\n--- Rotation {label} ---")
        for tw in rot_targets:
            target_wz = sign * tw
            actual_vx, actual_wz, wL, wR = step_response_short(bot, 0.0, target_wz)
            # 회전 시 expected wheel speed magnitude = ω × L/2
            expected_wheel = tw * (L / 2)
            ratio_M1 = abs(wL) / expected_wheel if expected_wheel > 1e-4 else 0
            ratio_M3 = abs(wR) / expected_wheel if expected_wheel > 1e-4 else 0
            rotation_results.append({
                "target_wz": target_wz,
                "actual_wz": actual_wz,
                "wL": wL, "wR": wR,
                "ratio_M1": ratio_M1, "ratio_M3": ratio_M3,
            })
            print(f"  {target_wz:+.3f}  | {actual_wz:+.3f}   | {wL:+.3f}   | {wR:+.3f}   | "
                  f"{ratio_M1*100:6.0f}%  | {ratio_M3*100:6.0f}%")

    # ---------- 분석 ----------
    print("\n\n=== Phase 1 v2 SUMMARY ===")

    def find_dz(samples, ratio_key, threshold=0.5):
        """ratio_key 가 처음으로 threshold 도달하는 |target| 반환."""
        for r in samples:
            if r[ratio_key] >= threshold:
                # vx 또는 wz 중 적용
                t = r.get("target_vx", r.get("target_wz", 0))
                return abs(t)
        return None

    fwd_lin = [r for r in linear_results if r["target_vx"] > 0]
    rev_lin = [r for r in linear_results if r["target_vx"] < 0]
    rev_lin = sorted(rev_lin, key=lambda r: abs(r["target_vx"]))
    ccw_rot = [r for r in rotation_results if r["target_wz"] > 0]
    cw_rot = [r for r in rotation_results if r["target_wz"] < 0]
    cw_rot = sorted(cw_rot, key=lambda r: abs(r["target_wz"]))

    print("\n[Linear effective deadzone — actual ≥ 50% of target]")
    m1_fwd_dz = find_dz(fwd_lin, "ratio_M1")
    m3_fwd_dz = find_dz(fwd_lin, "ratio_M3")
    m1_rev_dz = find_dz(rev_lin, "ratio_M1")
    m3_rev_dz = find_dz(rev_lin, "ratio_M3")
    print(f"  M1 (LEFT)  forward DZ: vx ≥ {m1_fwd_dz}" if m1_fwd_dz else "  M1 forward: NEVER REACHED 50%")
    print(f"  M3 (RIGHT) forward DZ: vx ≥ {m3_fwd_dz}" if m3_fwd_dz else "  M3 forward: NEVER REACHED 50%")
    print(f"  M1 (LEFT)  reverse DZ: vx ≥ {m1_rev_dz}" if m1_rev_dz else "  M1 reverse: NEVER REACHED 50%")
    print(f"  M3 (RIGHT) reverse DZ: vx ≥ {m3_rev_dz}" if m3_rev_dz else "  M3 reverse: NEVER REACHED 50%")

    print("\n[Rotation effective deadzone]")
    m1_ccw_dz = find_dz(ccw_rot, "ratio_M1")
    m3_ccw_dz = find_dz(ccw_rot, "ratio_M3")
    m1_cw_dz = find_dz(cw_rot, "ratio_M1")
    m3_cw_dz = find_dz(cw_rot, "ratio_M3")
    print(f"  M1 in CCW (left turn): ω ≥ {m1_ccw_dz}" if m1_ccw_dz else "  M1 CCW: NEVER")
    print(f"  M3 in CCW (left turn): ω ≥ {m3_ccw_dz}" if m3_ccw_dz else "  M3 CCW: NEVER")
    print(f"  M1 in CW (right turn): ω ≥ {m1_cw_dz}" if m1_cw_dz else "  M1 CW: NEVER")
    print(f"  M3 in CW (right turn): ω ≥ {m3_cw_dz}" if m3_cw_dz else "  M3 CW: NEVER")

    # 비대칭 진단
    print("\n[좌-우 비대칭 진단]")
    if m1_fwd_dz is not None and m3_fwd_dz is not None:
        ratio = m3_fwd_dz / m1_fwd_dz if m1_fwd_dz > 1e-4 else 0
        flag = "⚠️ 큰 비대칭" if ratio > 1.5 else "✓ 대칭"
        print(f"  M3/M1 forward DZ 비율: {ratio:.2f}× {flag}")
    if m1_rev_dz is not None and m3_rev_dz is not None:
        ratio = m3_rev_dz / m1_rev_dz if m1_rev_dz > 1e-4 else 0
        flag = "⚠️ 큰 비대칭" if ratio > 1.5 else "✓ 대칭"
        print(f"  M3/M1 reverse DZ 비율: {ratio:.2f}× {flag}")

    # path-following 영향 추정
    print("\n[Path-following 영향 추정]")
    # MPPI 가 일반적으로 발행하는 작은 명령 (vx≈0.10, ω≈0.20) 에서 응답 비교
    sample_lin = next((r for r in fwd_lin if abs(abs(r["target_vx"]) - 0.10) < 1e-3), None)
    if sample_lin:
        print(f"  vx=0.10 명령 시: M1={sample_lin['ratio_M1']*100:.0f}%, M3={sample_lin['ratio_M3']*100:.0f}%")
    sample_rot = next((r for r in ccw_rot if abs(abs(r["target_wz"]) - 0.20) < 1e-3), None)
    if sample_rot:
        print(f"  ω=+0.20 (좌회전): M1={sample_rot['ratio_M1']*100:.0f}%, M3={sample_rot['ratio_M3']*100:.0f}%")
    sample_rot_cw = next((r for r in cw_rot if abs(abs(r["target_wz"]) - 0.20) < 1e-3), None)
    if sample_rot_cw:
        print(f"  ω=-0.20 (우회전): M1={sample_rot_cw['ratio_M1']*100:.0f}%, M3={sample_rot_cw['ratio_M3']*100:.0f}%")

    return {
        "linear": linear_results,
        "rotation": rotation_results,
        "deadzone_summary": {
            "M1_fwd": m1_fwd_dz, "M1_rev": m1_rev_dz,
            "M3_fwd": m3_fwd_dz, "M3_rev": m3_rev_dz,
            "M1_ccw": m1_ccw_dz, "M3_ccw": m3_ccw_dz,
            "M1_cw": m1_cw_dz, "M3_cw": m3_cw_dz,
        },
    }


def step_response_short(bot, target_vx: float, target_wz: float, hold: float = 1.5):
    """
    set_car_motion 발행 → hold 초 대기 → 마지막 200ms 평균 측정.
    Returns: (actual_vx, actual_wz, wheel_L, wheel_R) — bot 의 raw frame.
    """
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.4)

    bot.set_car_motion(float(target_vx), 0.0, float(target_wz))
    # PID settling time 대기 (hold - 0.2)
    time.sleep(max(0.0, hold - 0.2))

    # 마지막 200ms sampling
    samples = []
    t_end = time.monotonic() + 0.2
    while time.monotonic() < t_end:
        vx, _, vz = bot.get_motion_data()
        wL, wR = bot.get_wheel_speeds()
        samples.append((vx, vz, wL, wR))
        time.sleep(0.02)

    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.3)

    if not samples:
        return 0.0, 0.0, 0.0, 0.0
    n = len(samples)
    return (
        sum(s[0] for s in samples) / n,
        sum(s[1] for s in samples) / n,
        sum(s[2] for s in samples) / n,
        sum(s[3] for s in samples) / n,
    )


# =============================================================================
# Phase 2 — PID step response
# =============================================================================
def measure_step_response(bot, target_vx: float, target_wz: float, hold_sec: float = 5.0):
    """
    set_car_motion(target_vx, 0, target_wz) 를 명령하고 actual 추적.
    Returns dict with rise_time, steady_state_actual, etc.
    """
    label = f"vx={target_vx:+.2f}, ω={target_wz:+.2f}"
    print(f"\n--- step response ({label}) ---")

    # 사전 정지
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(1.0)

    samples_t = []
    samples_vx = []
    samples_wz = []
    samples_wL = []
    samples_wR = []

    t_start = time.monotonic()
    t_cmd = t_start

    # cmd 발행
    bot.set_car_motion(float(target_vx), 0.0, float(target_wz))

    while time.monotonic() - t_start < hold_sec:
        t_now = time.monotonic() - t_start
        vx, vy, vz = bot.get_motion_data()
        wL, wR = bot.get_wheel_speeds()
        samples_t.append(t_now)
        samples_vx.append(vx)
        samples_wz.append(vz)
        samples_wL.append(wL)
        samples_wR.append(wR)
        time.sleep(0.02)  # 50Hz sampling

    # 정지
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(1.0)

    # 분석: rise_time (target 의 90% 도달 시간)
    target_active = abs(target_vx) > 1e-6
    target_metric = target_vx if abs(target_vx) >= abs(target_wz) else target_wz
    sample_metric = samples_vx if abs(target_vx) >= abs(target_wz) else samples_wz

    rise_time = None
    if abs(target_metric) > 1e-3:
        threshold = 0.9 * target_metric
        for t_s, val in zip(samples_t, sample_metric):
            if (target_metric > 0 and val >= threshold) or (target_metric < 0 and val <= threshold):
                rise_time = t_s
                break

    # 정상상태 actual: 마지막 1초 평균
    n_ss = max(1, int(len(samples_t) * 0.2))
    ss_vx = sum(samples_vx[-n_ss:]) / n_ss
    ss_wz = sum(samples_wz[-n_ss:]) / n_ss
    ss_wL = sum(samples_wL[-n_ss:]) / n_ss
    ss_wR = sum(samples_wR[-n_ss:]) / n_ss

    # 결과
    if abs(target_vx) >= abs(target_wz):
        ratio_vx = ss_vx / target_vx if abs(target_vx) > 1e-3 else 0
        print(f"  target vx = {target_vx:+.3f} → steady-state actual = {ss_vx:+.3f} (ratio {ratio_vx*100:.0f}%)")
    if abs(target_wz) > 1e-3:
        ratio_wz = ss_wz / target_wz if abs(target_wz) > 1e-3 else 0
        print(f"  target ω  = {target_wz:+.3f} → steady-state actual = {ss_wz:+.3f} (ratio {ratio_wz*100:.0f}%)")
    print(f"  steady-state wheel L={ss_wL:+.3f}, R={ss_wR:+.3f}")
    if rise_time is not None:
        print(f"  rise time (90% 도달): {rise_time*1000:.0f} ms")
    else:
        print(f"  rise time: NOT REACHED (90% 미도달, 응답 부족)")

    return {
        "target_vx": target_vx,
        "target_wz": target_wz,
        "rise_time": rise_time,
        "ss_vx": ss_vx,
        "ss_wz": ss_wz,
        "ss_wL": ss_wL,
        "ss_wR": ss_wR,
    }


def phase_pid(bot) -> dict:
    print("\n" + "=" * 60)
    print("Phase 2 — PID Step Response (closed-loop velocity)")
    print("=" * 60)

    # 직진 step
    print("\n[직진 step response]")
    results = {}
    for tx in [0.20, 0.10, 0.05]:
        results[f"vx_{tx}"] = measure_step_response(bot, target_vx=tx, target_wz=0.0)
        time.sleep(1.0)

    # 회전 step
    print("\n[회전 step response]")
    for tw in [0.40, 0.20, 0.10]:
        results[f"wz_{tw}"] = measure_step_response(bot, target_vx=0.0, target_wz=tw)
        time.sleep(1.0)

    # PID gain 적정성 평가
    print("\n=== Phase 2 SUMMARY — PID 응답 평가 ===")
    print("기준: ratio > 90%, rise_time < 500ms 이면 정상")
    issue_count = 0
    for k, r in results.items():
        rt = r["rise_time"]
        rt_str = f"{rt*1000:.0f} ms" if rt is not None else "N/A"
        if k.startswith("vx_") and abs(r["target_vx"]) > 1e-3:
            ratio = r["ss_vx"] / r["target_vx"]
            mark = "✓" if ratio > 0.9 else "⚠️"
            if ratio <= 0.9: issue_count += 1
            print(f"  {mark} {k}: ratio {ratio*100:.0f}%, rise {rt_str}")
        elif k.startswith("wz_") and abs(r["target_wz"]) > 1e-3:
            ratio = r["ss_wz"] / r["target_wz"]
            mark = "✓" if ratio > 0.9 else "⚠️"
            if ratio <= 0.9: issue_count += 1
            print(f"  {mark} {k}: ratio {ratio*100:.0f}%, rise {rt_str}")

    if issue_count > 0:
        print(f"\n⚠️ {issue_count} 항목이 ratio < 90%. PID Kp 부족 또는 모터 deadzone 보상 부족 가능.")
        print("  권장:")
        print("    1. Phase 1 결과로 펌웨어 Motor_Set_Dead_Zone 적용 → 다시 측정")
        print("    2. 여전히 부족하면 Kp 상향 (예: 0.8 → 1.5) — jupiter_driver_params.yaml")
    else:
        print("\n✓ 모든 항목 정상.")

    return results


# =============================================================================
# 메인
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Jupiter 휠 deadzone + PID 캘리브레이션",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--phase", choices=["deadzone", "pid", "both"],
                        default="both",
                        help="실행할 phase (기본: both)")
    parser.add_argument("--no-confirm", action="store_true",
                        help="차륜 들어올림 확인 prompt 스킵")
    args = parser.parse_args()

    if not args.no_confirm:
        warn_lift_wheels()

    bot = connect_mcu()

    all_results = {}
    try:
        if args.phase in ("deadzone", "both"):
            all_results["phase_deadzone"] = phase_deadzone(bot)
        if args.phase in ("pid", "both"):
            all_results["phase_pid"] = phase_pid(bot)
    finally:
        stop_all(bot)
        del bot

    # yaml 저장
    if yaml is not None:
        # numpy/special types 방지
        def to_basic(obj):
            if isinstance(obj, dict):
                return {k: to_basic(v) for k, v in obj.items()}
            if isinstance(obj, (list, tuple)):
                return [to_basic(v) for v in obj]
            if isinstance(obj, (int, float, str, bool, type(None))):
                return obj
            return str(obj)
        try:
            with open(OUTPUT_PATH, "w") as f:
                yaml.safe_dump(to_basic(all_results), f, allow_unicode=True, sort_keys=False)
            print(f"\n[output] saved to {OUTPUT_PATH}")
        except Exception as e:
            print(f"[output] yaml save 실패: {e}")
    else:
        print("\n[output] yaml 모듈 없음 — 콘솔 결과 만 사용")


if __name__ == "__main__":
    main()
