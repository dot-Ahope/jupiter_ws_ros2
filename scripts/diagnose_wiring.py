#!/usr/bin/env python3
# encoding: utf-8
"""
diagnose_wiring.py — Jupiter 모터/엔코더 배선 자동 진단

목적
    물리 바퀴 ↔ MCU 채널(M1~M4) ↔ 엔코더 채널의 매핑이
    펌웨어/드라이버 가정과 일치하는지 확인하고, 어긋남이 있다면 시나리오를 식별한다.

가정된 정상 매핑 (펌웨어 + 드라이버 코드 기반)
    M1 → 좌측 (앞 또는 메인 좌측)
    M3 → 우측 (앞 또는 메인 우측)
    encoder index [0] (m1) → M1 의 회전 카운트
    encoder index [2] (m3) → M3 의 회전 카운트
    cmd_vel angular.z > 0 → 좌측 휠 후진, 우측 휠 전진 (CCW = ROS REP-103)

진단 흐름
    Phase 1: 정지 상태 baseline 엔코더 카운트 측정
    Phase 2: 각 MCU 채널(M1, M2, M3, M4) 에 ±PWM 인가 → 사용자에게 어느 바퀴가 도는지 묻고
             자동으로 엔코더 변화량 기록 → MCU 채널 ↔ 물리 휠 ↔ 엔코더 인덱스 매핑 추출
    Phase 3: (선택) 사용자가 손으로 각 바퀴 회전 → 엔코더 카운트 변화 관측 → encoder 단독 매핑 보강
    Phase 4: 시나리오 자동 분류
                - NORMAL : 모든 매핑 정상
                - CABLE_FULL_SWAP : 모터 + 엔코더 동시 좌↔우 교환 (사용자 보고와 일치)
                - ENCODER_ONLY_SWAP : 엔코더만 교환
                - MOTOR_ONLY_SWAP : 모터 전원선만 교환
                - AB_PHASE_SWAP : 엔코더 A/B 위상 반대 (회전 방향 부호 반전)
                - UNKNOWN
    Phase 5: 권장 수정안 (하드웨어 재배선 / 펌웨어 채널 swap / 부호 반전 등) 제시

실행
    !! 반드시 jupiter_driver_compensated 노드를 먼저 정지한 뒤 실행 !!
    (USB serial /dev/myserial 을 점유 충돌하므로)

    cd /home/jetson/jupiter_ws_ros2
    sudo systemctl stop jupiter_bringup  # 또는 launch 종료
    python3 scripts/diagnose_wiring.py
"""

import argparse
import sys
import time
from pathlib import Path

# Rosmaster_Lib 경로 추가
SRC = Path("/home/jetson/jupiter_ws_ros2/src/jupiter_bringup/jupiter_bringup")
sys.path.insert(0, str(SRC))

try:
    from Rosmaster_Lib import Rosmaster
except ImportError as e:
    print(f"[FATAL] Rosmaster_Lib 임포트 실패: {e}")
    sys.exit(1)


# ============================================================
# 설정
# ============================================================
PWM_MAGNITUDE = 60          # PWM 절대값 (-100~100). 펌웨어 deadzone(M1=720/M3=690)
                            # + 정지마찰 통과 위해 25→60 으로 상향. 차체 잠시 움직임 주의.
PWM_DURATION_S = 0.8        # 한 방향 인가 시간. PWM 키운 만큼 시간 단축.
ENCODER_DELTA_THRESH = 50   # 엔코더가 "변했다" 고 판단할 최소 변화량
NULL_RUN_DELAY_S = 0.4      # 모터 정지 후 안정화 대기 (관성 흡수)


# ============================================================
# 콘솔 헬퍼
# ============================================================
def hline(ch='─', n=72):
    print(ch * n)


def banner(title):
    hline('═')
    print(f" {title}")
    hline('═')


def ask(prompt, choices=None):
    while True:
        ans = input(prompt).strip().lower()
        if not choices or ans in choices:
            return ans
        print(f"  → 가능한 입력: {', '.join(choices)}")


def confirm(prompt):
    return ask(prompt + " [y/n] ", choices=['y', 'n', 'yes', 'no']) in ('y', 'yes')


# ============================================================
# 진단 클래스
# ============================================================
class WiringDiagnosis:
    def __init__(self):
        print("[*] Rosmaster 초기화 중…")
        try:
            self.bot = Rosmaster(car_type=1, com="/dev/myserial", delay=0.002)
            self.bot.create_receive_threading()
        except Exception as e:
            print(f"[FATAL] Rosmaster 초기화 실패: {e}")
            print("       jupiter_driver_compensated 노드가 실행 중이면 먼저 정지하라.")
            sys.exit(1)

        time.sleep(0.5)
        self.bot.set_motor(0, 0, 0, 0)
        time.sleep(NULL_RUN_DELAY_S)

        # 결과 저장:
        #   self.motor_to_wheel[ch] = (physical_label, direction_sign)
        #   self.motor_to_encoder_idx[ch] = (encoder_idx, encoder_sign)
        self.motor_to_wheel = {}       # ch ∈ {1,2,3,4}
        self.motor_to_encoder_idx = {} # ch ∈ {1,2,3,4} → (idx ∈ {0,1,2,3}, sign ∈ {+1,-1})
        self.manual_wheel_to_encoder = {}  # 'L'/'R'/'LF'/'RF' → (idx, sign)

    # ---------------- core motor + encoder probe ----------------
    def _zero_motors(self):
        self.bot.set_motor(0, 0, 0, 0)
        time.sleep(NULL_RUN_DELAY_S)

    def _read_encoder(self):
        return list(self.bot.get_motor_encoder())  # [m1, m2, m3, m4]

    def _drive_one_motor(self, channel, pwm):
        cmd = [0, 0, 0, 0]
        cmd[channel - 1] = pwm
        self.bot.set_motor(*cmd)

    def probe_motor(self, channel):
        """채널 하나에 +PWM, -PWM 인가하고 엔코더 변화량 + 사용자 관측을 수집"""
        print()
        hline()
        print(f"  채널 M{channel} 테스트")
        hline()

        results = {}  # 'pos' / 'neg' → (encoder_deltas[4], user_obs)
        for sign_label, pwm in [('pos', +PWM_MAGNITUDE), ('neg', -PWM_MAGNITUDE)]:
            self._zero_motors()
            before = self._read_encoder()
            print(f"  [M{channel}={pwm:+d} PWM 인가 — {PWM_DURATION_S}s] "
                  "어느 바퀴가 도는지, 그리고 회전 방향을 관찰하라…")
            self._drive_one_motor(channel, pwm)
            time.sleep(PWM_DURATION_S)
            self.bot.set_motor(0, 0, 0, 0)
            time.sleep(NULL_RUN_DELAY_S)
            after = self._read_encoder()
            delta = [after[i] - before[i] for i in range(4)]
            print(f"    엔코더 Δ : m1={delta[0]:+6d}  m2={delta[1]:+6d}  "
                  f"m3={delta[2]:+6d}  m4={delta[3]:+6d}")

            obs = ask(
                "    어느 휠이 돌았나? (L=좌측, R=우측, LF=좌앞, RF=우앞, LB=좌뒤, RB=우뒤, "
                "x=움직임 없음): ",
                choices=['l', 'r', 'lf', 'rf', 'lb', 'rb', 'x']
            )
            results[sign_label] = (delta, obs)

        return results

    def run_phase2(self):
        banner("Phase 2: 채널별 자동 모터 + 엔코더 매핑")
        print("각 모터 채널에 PWM 을 짧게 인가한다. 휠 옆에서 관찰하라.")
        if not confirm("준비됐는가?"):
            print("[!] 중단됨")
            sys.exit(0)

        phase2 = {}
        for ch in [1, 2, 3, 4]:
            phase2[ch] = self.probe_motor(ch)

        # 정리 — 채널 ↔ 물리 휠 매핑 추출
        print()
        banner("Phase 2 결과 정리")
        print(f"{'채널':<6}{'+PWM 휠':<10}{'-PWM 휠':<10}"
              f"{'엔코더 idx (Δ 최대)':<24}{'엔코더 부호 (+PWM)'}")
        hline()
        for ch in [1, 2, 3, 4]:
            pos_delta, pos_obs = phase2[ch]['pos']
            neg_delta, neg_obs = phase2[ch]['neg']
            # 엔코더 변화 가장 큰 인덱스
            mag = [abs(pos_delta[i]) for i in range(4)]
            best_idx = mag.index(max(mag))
            best_mag = mag[best_idx]
            if best_mag < ENCODER_DELTA_THRESH:
                enc_str = "(변화 없음)"
                enc_sign = 0
            else:
                enc_sign = 1 if pos_delta[best_idx] > 0 else -1
                enc_str = f"m{best_idx + 1} (Δ={pos_delta[best_idx]:+d})"
            print(f"M{ch:<5}{pos_obs.upper():<10}{neg_obs.upper():<10}"
                  f"{enc_str:<24}{enc_sign:+d}")

            self.motor_to_wheel[ch] = (pos_obs.upper(), neg_obs.upper())
            if enc_sign != 0:
                self.motor_to_encoder_idx[ch] = (best_idx, enc_sign)

        return phase2

    # ---------------- manual rotation phase ----------------
    def run_phase3(self):
        banner("Phase 3 (선택): 손 회전으로 엔코더 단독 매핑 보강")
        if not confirm("손으로 각 바퀴를 돌려 엔코더 매핑을 확인하겠는가? (생략 가능)"):
            print("[*] Phase 3 건너뜀")
            return None

        wheels = [
            ('L (좌측)', 'L'),
            ('R (우측)', 'R'),
        ]
        for label, key in wheels:
            self._zero_motors()
            before = self._read_encoder()
            input(f"  {label} 바퀴를 **전진 방향**으로 손으로 1~2바퀴 돌리고 ENTER…")
            after = self._read_encoder()
            delta = [after[i] - before[i] for i in range(4)]
            print(f"    엔코더 Δ : m1={delta[0]:+6d}  m2={delta[1]:+6d}  "
                  f"m3={delta[2]:+6d}  m4={delta[3]:+6d}")
            mag = [abs(delta[i]) for i in range(4)]
            best = mag.index(max(mag))
            if mag[best] >= ENCODER_DELTA_THRESH:
                sign = 1 if delta[best] > 0 else -1
                self.manual_wheel_to_encoder[key] = (best, sign)
                print(f"    → 휠 {key} = 엔코더 m{best + 1} (부호 {sign:+d})")
            else:
                print("    → 엔코더 변화 미감지")

    # ---------------- diagnosis ----------------
    def diagnose(self):
        banner("Phase 4: 시나리오 자동 분류")

        # Phase 2 의 매핑을 가정 매핑(M1=좌, M3=우) 과 비교
        ch_to_wheel_pos = {ch: vals[0] for ch, vals in self.motor_to_wheel.items()}

        scenarios = []
        notes = []

        # 1. M1 + M3 의 +PWM 인가 시 실제 회전 휠
        m1_wheel = ch_to_wheel_pos.get(1, 'X')
        m3_wheel = ch_to_wheel_pos.get(3, 'X')

        m1_is_left = m1_wheel.startswith('L')
        m3_is_right = m3_wheel.startswith('R')
        m1_is_right = m1_wheel.startswith('R')
        m3_is_left = m3_wheel.startswith('L')

        # 2. 엔코더 매핑
        enc_m1 = self.motor_to_encoder_idx.get(1)
        enc_m3 = self.motor_to_encoder_idx.get(3)

        print()
        print(f"  M1 +PWM → 휠 {m1_wheel}    (정상: L*)")
        print(f"  M3 +PWM → 휠 {m3_wheel}    (정상: R*)")
        if enc_m1:
            print(f"  M1 회전 시 변하는 엔코더 : m{enc_m1[0]+1} (부호 {enc_m1[1]:+d})   "
                  "(정상: m1 부호 +)")
        if enc_m3:
            print(f"  M3 회전 시 변하는 엔코더 : m{enc_m3[0]+1} (부호 {enc_m3[1]:+d})   "
                  "(정상: m3 부호 +)")
        print()

        # 시나리오 판별
        motor_swapped = (m1_is_right and m3_is_left)
        encoder_swapped = (
            enc_m1 is not None and enc_m3 is not None
            and enc_m1[0] == 2 and enc_m3[0] == 0
        )
        encoder_normal = (
            enc_m1 is not None and enc_m3 is not None
            and enc_m1[0] == 0 and enc_m3[0] == 2
        )
        motor_normal = (m1_is_left and m3_is_right)

        # 부호 반전 (A/B 위상 swap) 검출
        ab_swap_m1 = enc_m1 is not None and enc_m1[1] < 0
        ab_swap_m3 = enc_m3 is not None and enc_m3[1] < 0

        if motor_normal and encoder_normal and not ab_swap_m1 and not ab_swap_m3:
            scenarios.append("NORMAL : 모든 매핑이 펌웨어 가정과 일치한다.")
            recommendations = ["수정 불필요. 다른 원인(기계 마찰/오프셋)을 의심."]
        elif motor_swapped and encoder_swapped:
            scenarios.append(
                "CABLE_FULL_SWAP : 좌/우 6핀 케이블이 통째로 교체됨. "
                "M1↔M3 모터 전원 + 엔코더 모두 좌우 반전."
            )
            recommendations = [
                "권장: 하드웨어 측에서 M1, M3 의 6핀 커넥터를 원래대로 교체.",
                "대안(소프트웨어): 펌웨어에서 M1↔M3 출력 + 엔코더 인덱스 일괄 swap.",
                "주의: 현재 cmd angular>0 → 좌측 전진(=실제는 우측 회전)이라면 "
                "이미 두 swap 이 서로 상쇄되어 동작이 부분적으로만 어긋났을 가능성. "
                "사용자 보고(angular>0 시 좌측 회전)는 정상 ROS 규약과 일치하므로 "
                "결과적으로 'cmd→실제 회전 방향' 은 맞고, '엔코더 odom' 만 어긋남.",
            ]
        elif motor_normal and encoder_swapped:
            scenarios.append(
                "ENCODER_ONLY_SWAP : 모터 전원선은 정상이나 엔코더 A/B 신호선만 좌우 교체."
            )
            recommendations = [
                "권장: 엔코더 시그널 라인만 좌우 교환 (커넥터 6핀 중 A/B 핀).",
                "대안(소프트웨어): 펌웨어에서 encoder m1↔m3 swap 또는 "
                "Rosmaster_Lib 의 wheel_left/right 산출 시 idx 교체.",
                "증상: cmd 와 실제 휠 회전 방향은 일치하지만 odom 이 반대로 적분됨 → "
                "EKF/Nav2 가 'path 따라가지 못함, 오른쪽 편향' 으로 나타남.",
            ]
        elif motor_swapped and encoder_normal:
            scenarios.append(
                "MOTOR_ONLY_SWAP : 모터 전원선 좌우 교체, 엔코더는 정상."
            )
            recommendations = [
                "권장: 모터 M+/M- 전원선만 좌우 교환.",
                "증상: cmd 회전 방향이 ROS 규약과 반대로 나타나야 함 — "
                "사용자 관측(angular>0 → 좌측 전진)이 정상이라면 이 시나리오는 아님.",
            ]
        elif ab_swap_m1 or ab_swap_m3:
            scenarios.append(
                f"AB_PHASE_SWAP : 엔코더 A/B 위상이 반대 "
                f"(M1 부호 반전={ab_swap_m1}, M3 부호 반전={ab_swap_m3})"
            )
            recommendations = [
                "권장: bsp_encoder.c 에서 해당 채널의 ICPolarity 반전.",
                "대안(소프트웨어): Rosmaster_Lib 에서 m1/m3 엔코더 값에 -1 곱.",
            ]
        else:
            scenarios.append(
                "UNKNOWN : 자동 분류 실패. Phase 2/3 raw 데이터를 직접 검토 필요."
            )
            recommendations = [
                "Phase 2 결과 표를 사람이 검토.",
                "특히 M2, M4 의 휠 매핑이 가정(좌뒤/우뒤)과 일치하는지 확인.",
            ]

        # Phase 3 보강 검증
        if self.manual_wheel_to_encoder:
            man_l = self.manual_wheel_to_encoder.get('L')
            man_r = self.manual_wheel_to_encoder.get('R')
            if man_l and man_r:
                if man_l[0] == 0 and man_r[0] == 2:
                    notes.append("Phase 3 보강: 좌↔m1, 우↔m3 매핑 일치 (정상).")
                elif man_l[0] == 2 and man_r[0] == 0:
                    notes.append(
                        "Phase 3 보강: 좌↔m3, 우↔m1 매핑 — 엔코더 좌우 swap 확정."
                    )
                else:
                    notes.append(
                        f"Phase 3 보강: 좌→m{man_l[0]+1}, 우→m{man_r[0]+1} (예상 외)."
                    )

        # 출력
        print()
        for s in scenarios:
            print(f"  >>> {s}")
        if notes:
            print()
            print("  [추가 노트]")
            for n in notes:
                print(f"   - {n}")
        print()
        print("  [권장 수정안]")
        for r in recommendations:
            print(f"   - {r}")
        print()

    # ---------------- cleanup ----------------
    def cleanup(self):
        try:
            self.bot.set_motor(0, 0, 0, 0)
        except Exception:
            pass


# ============================================================
# main
# ============================================================
def main():
    global PWM_MAGNITUDE, PWM_DURATION_S
    parser = argparse.ArgumentParser(description="Jupiter wiring 자동 진단")
    parser.add_argument('--pwm', type=int, default=PWM_MAGNITUDE,
                        help=f"PWM 절대값 (-100~100, 기본 {PWM_MAGNITUDE}). "
                             "모터가 안 돌면 70~80 까지 올려라.")
    parser.add_argument('--duration', type=float, default=PWM_DURATION_S,
                        help=f"한 방향 인가 시간 초 (기본 {PWM_DURATION_S}). "
                             "PWM 크면 줄여라.")
    args = parser.parse_args()
    PWM_MAGNITUDE = max(0, min(100, args.pwm))
    PWM_DURATION_S = max(0.2, args.duration)

    banner("Jupiter Wiring 자동 진단")
    print("주의:")
    print("  - 이 스크립트는 모터를 직접 구동한다. 차체가 살짝 전/후진/회전할 수 있다.")
    print("  - 바퀴를 띄우거나(잭업) 빈 공간에서 실행하라.")
    print(f"  - 사용 PWM={PWM_MAGNITUDE}, 인가시간={PWM_DURATION_S:.2f}s "
          "(모터 안 돌면 --pwm 70 으로 재시도)")
    print("  - jupiter_driver_compensated 노드는 먼저 정지하라 (USB serial 충돌).")
    print()
    if not confirm("진단을 시작할 준비가 됐는가?"):
        print("[*] 중단됨")
        sys.exit(0)

    diag = WiringDiagnosis()
    try:
        # baseline
        banner("Phase 1: Baseline 엔코더 카운트 (정지 상태)")
        diag._zero_motors()
        baseline = diag._read_encoder()
        print(f"  m1={baseline[0]}  m2={baseline[1]}  m3={baseline[2]}  m4={baseline[3]}")
        time.sleep(0.5)
        baseline2 = diag._read_encoder()
        drift = [baseline2[i] - baseline[i] for i in range(4)]
        print(f"  0.5s drift Δ : {drift}  (정지 상태에서 0 에 가까워야 함)")

        # phase 2
        diag.run_phase2()

        # phase 3
        diag.run_phase3()

        # diagnose
        diag.diagnose()

        banner("진단 종료")
        print("  결과를 진행과정 문서에 기록하고, 권장 수정을 적용한 뒤 재시연 권장.")
    except KeyboardInterrupt:
        print("\n[!] 사용자 중단")
    finally:
        diag.cleanup()


if __name__ == '__main__':
    main()
