import matplotlib.pyplot as plt
import numpy as np
import os

def calculate_compensation(angular_input, vx_input):
    """
    jupiter_driver_compensated.py의 핵심 로직을 그대로 옮겨온 함수
    """
    abs_angular = abs(angular_input)
    abs_vx = abs(vx_input)
    
    # Deadzone check (from code)
    if abs_angular < 0.001:
        return 0.0
    
    # --- Logic from lines 146-161 ---
    # 상태별 파라미터 결정
    if abs_vx > 0.01:
        # 동적 마찰 (주행 중)
        BIAS = 0.05     # 주행 중에는 적은 힘으로도 회전 가능
        SLOPE = 0.191   # (0.241 - 0.05)
        mode = "Dynamic"
    else:
        # 정적 마찰 (제자리)
        BIAS = 0.13     # 정지 상태에서는 큰 힘 필요
        SLOPE = 0.111   # (0.241 - 0.13)
        mode = "Static"
    
    angular_comp = (abs_angular * SLOPE) + BIAS
    return angular_comp

# ---------------------------------------------------------
# 시각화 설정
# ---------------------------------------------------------

# 1. 입력 데이터 생성 (Angular Command: 0 ~ 1.2 rad/s)
x_cmds = np.linspace(0, 1.2, 200)

# 2. 두 가지 상황에 대해 로직 적용
# 상황 A: 제자리 회전 (vx = 0.0)
y_static = [calculate_compensation(x, 0.0) for x in x_cmds]

# 상황 B: 주행 중 회전 (vx = 0.2) -> 0.01보다 크므로 Dynamic 적용됨
y_dynamic = [calculate_compensation(x, 0.2) for x in x_cmds]

# 3. 그래프 그리기
plt.figure(figsize=(10, 6))

# Static Line (Blue)
plt.plot(x_cmds, y_static, label='Static Mode (vx=0)\nBias=0.13 (Strong Start)', 
         color='blue', linewidth=2)

# Dynamic Line (Red Dashed)
plt.plot(x_cmds, y_dynamic, label='Dynamic Mode (vx>0.01)\nBias=0.05 (Soft Start)', 
         color='red', linewidth=2, linestyle='--')

# 4. Target Point (Convergence) 확인
# 코드 주석에 있는 "타겟 기준점 (1.0 rad/s = Output 0.241)"이 맞는지 시각화
target_x = 1.0
target_y = calculate_compensation(1.0, 0.0) # 0.241 예상

plt.scatter([target_x], [target_y], color='green', s=100, zorder=5, edgecolors='black')
plt.annotate(f'Sync Point\n({target_x}, {target_y:.3f})', 
             xy=(target_x, target_y), 
             xytext=(target_x+0.05, target_y-0.05),
             arrowprops=dict(facecolor='black', shrink=0.05))

# 5. 그래프 스타일링
plt.title('Logic Visualization: Jupiter Driver Friction Compensation', fontsize=14)
plt.xlabel('Input Angular Velocity Command (rad/s)', fontsize=12)
plt.ylabel('Compensated Output to Motor', fontsize=12)
plt.grid(True, which='both', linestyle=':', alpha=0.6)
plt.legend(fontsize=10)
plt.xlim(0, 1.2)
plt.ylim(0, 0.35)

# 6. Deadzone(0.001) 구간 확대 표시 (Optional)
plt.fill_between([0, 0.02], 0, 0.35, color='gray', alpha=0.1, label='Near Zero Zone')

plt.tight_layout()

current_path = os.path.abspath(__file__)
current_dir = os.path.dirname(current_path)
save_path = os.path.join(current_dir, 'motor_friction_compensation_visualization.png')
plt.savefig(save_path, dpi=300)