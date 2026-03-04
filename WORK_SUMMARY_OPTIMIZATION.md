# Jupiter Robot Navigation Optimization Summary
**Date:** 2026-02-04

## 1. Initial Problem
The Jupiter robot experienced navigation failures, including:
*   **TF Transform Errors:** "[laser] to [map]" transform failures due to missing or misconfigured SLAM/localization components.
*   **Performance Issues:** High CPU load causing queue overflows and delayed processing, mainly due to DenseBoost LiDAR mode and inefficient SLAM parameters.
*   **Control Instability (Oscillation):** The robot oscillated left/right while following paths, failed to rotate in place when obstacles were detected, and exhibited overshooting at high speeds.
*   **Stiction (Static Friction):** The robot failed to start rotating at low angular velocity commands.

## 2. Root Cause Analysis
*   **Hardware/Firmware Mismatch:** The MCU firmware was updated to remove exponential `turn_ratio` squaring (making it linear), but the driver logic and parameters were still tuned for the old non-linear behavior.
*   **Friction & Deadzone:** The robot has significant static friction (stiction). Low velocity commands from Nav2 resulted in insufficient motor torque to move the robot.
*   **Control Loop Mismatch:** Nav2 assumed a linear response, but the hardware had a "deadzone" where inputs yielded no motion, followed by a sudden jump in motion once friction was overcome.
*   **Nav2 Parameters:** Default parameters (min speeds, acceleration limits) were not tuned for the robot's physical constraints.

## 3. Implemented Solutions

### A. System & Performance Optimization
*   **LiDAR Optimization:** Changed `scan_mode` from `DenseBoost` to `Standard` to reduce CPU load.
*   **SLAM Tuning:** Created optimized `slam_params.yaml` with scan throttling (1/4 scans) and aggressive timeouts.
*   **Visualization:** Configured Foxglove Bridge as a systemd service with bandwidth optimization (compression enabled).
*   **URDF Fixes:** Corrected LiDAR rotation (yaw vs pitch) and TF hierarchy (`base_link` vs `base_footprint`) for Visual SLAM compatibility.

### B. Driver Angular Velocity Calibration
*   **Calibration Goal:** Linear mapping where 1.0 rad/s input results in 1.0 rad/s physical rotation.
*   **Iterative Testing:**
    *   Found `SCALE = 0.241` to be the correct linear coefficient for 1.0 rad/s.
    *   Input 1.0 rad/s → Output 0.241 → Result: 1.59 laps in 10s (~1.0 rad/s).

### C. Friction Compensation (The "Affine" Approach)
To solve the stiction/deadzone issue without causing oscillation:
*   **Method:** Applied Affine Transformation (Bias + Slope) instead of simple linear scaling.
*   **Dynamic Friction Context:**
    *   **Static (In-Place Rotation):** Applied high `Bias (0.13)` to overcome static friction.
    *   **Dynamic (Moving Rotation):** Applied low `Bias (0.05)` because kinetic friction is lower when the robot is already moving linearly. This eliminated the oscillation ("fishtailing") caused by applying static friction compensation during motion.
*   **Formula:** `Output = (Slope * Input) + Bias`
    *   Static Mode: `Output = (0.111 * Input) + 0.13`
    *   Dynamic Mode: `Output = (0.191 * Input) + 0.05`

### D. Nav2 Parameter Tuning (`nav2_params_vslam.yaml`)
*   **Minimum Speeds:** Set `min_speed_theta` to **0.15 rad/s** and `min_theta_velocity_threshold` to **0.15 rad/s** to ensure Nav2 doesn't request speeds the hardware cannot physically perform reliably.
*   **Ranges:** Reduced `raytrace_max_range` (15.0 → 3.0m) and `obstacle_max_range` (10.0 → 2.5m) for indoor optimization.
*   **Safety:** Reduced `max_vel_theta` to 1.0 rad/s and `acc_lim_theta` to 0.8 rad/s^2 for smoother control.

## 4. Current Status
The robot now has a robust open-loop feedforward control system that:
1.  Reliably starts rotating from a standstill (overcoming stiction).
2.  Maintains smooth steering while moving (avoiding oscillation).
3.  Operates within the physical limits defined in Nav2.
