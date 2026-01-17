# Jupiter Robot ROS2 Workspace

## Overview
This repository contains the ROS2 software stack for the **Jupiter Robot**, focusing on outdoor navigation, GPS integration, and robust odometry fusion using Lidar and IMU.

## Key Features
- **GPS Integration**: Dual EKF setup using `robot_localization` (Odom+IMU & GPS+Odom+IMU).
- **Mapless Navigation**: Nav2 configuration using `SmacPlanner2D` for navigation without a static map.
- **Sensor Fusion**: `kiss-icp` for Lidar Odometry and sensor fusion with IMU.
- **Hardware Integration**: Support for VLP16 Lidar, Astra Camera, and RTK GPS.

## Setup & Installation
```bash
# Clone the repository
git clone https://github.com/dot-Ahope/jupiter_ws_ros2.git

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
```

## Running the Robot
### Outdoor GPS Navigation
To launch the complete stack for outdoor navigation:
```bash
ros2 launch jupiter_outdoor_gps.launch.py
```

### Calibration
Scripts for calibrating IMU and Odom are located in the root directory.

## Recent Updates (2026-01-17)
- Fixed Nav2 "Costmap needs a mapless planner" issue.
- Integrated `navsat_transform_node` for correct GPS-to-Odom framing.
- Solved Topic mismatch (`/ublox_gps/fix`).
