# Isaac ROS Visual SLAM Odometry Instability Analysis

## 1. Issue Description
- **Symptom**: The odometry position flickers or jumps unexpectedly.
- **Scenario 1**: Persistent flickering during operation.
- **Scenario 2**: Large jumps when a person walks in front of the robot while the robot is stationary (in Map-less VIO mode).

## 2. Analysis

### Case A: Flickering with Loop Closure
- **Observation**: Odometry jumps to a far location (e.g., -1.15m) and returns immediately.
- **Cause**: **False Positive Loop Closure**. The system incorrectly matches the current view with a wrong previous location.
- **Evidence**: `loop_closure_cloud` topics published around the jump timestamp.

### Case B: Jumps caused by Dynamic Obstacles (VIO Mode)
- **Settings**: `enable_localization_n_mapping: False` (VIO mode).
- **Observation**: Robot stops, person walks by -> Odometry indicates high velocity (up to ~1.15 m/s or even 23 m/s with bad tuning).
- **Cause**: **Violation of Static World Assumption**. The VSLAM algorithm interprets the moving person (dominant visual feature) as the static world, thus concluding the robot is moving in the opposite direction.
- **Failure of IMU Tuning**: Attempting to force the filter to trust IMU by reducing `accel_noise_density` by 10x resulted in **Filter Divergence** (Velocity spiked to 23 m/s), rather than fixing the issue.

## 3. Conclusion & Solutions
- **Parameter Tuning**: Simply tuning noise density is dangerous and ineffective for semantic issues.
- **Structural Fixes**:
    1.  **Enable Mapping**: Use a static map so the robot can distinguish moving objects from the background.
    2.  **Semantic Masking**: Use AI (e.g., Segformer, YOLO) to mask out dynamic objects (people) from the visual features passed to SLAM.
    3.  **Motion Rejection**: Tune internal VSLAM parameters for better rejection of dynamic outliers.

## 4. Navigation Integration
- To use this odometry for navigation, it must be robust.
- Plan: Integrate Isaac VSLAM output (`/visual_slam/tracking/odometry`) with Nav2 stack, ensuring coordinate transforms (`odom` -> `base_link`) are correctly provided by VSLAM.
