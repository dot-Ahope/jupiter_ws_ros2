cmake_minimum_required(VERSION 3.8)
project(jupiter_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/Adjust.msg"
  "msg/Arm.msg"
  "msg/Battery.msg"
  "msg/Edition.msg"
  "msg/General.msg"
  "msg/ImageMsg.msg"
  "msg/Joint.msg"
  "msg/JoyState.msg"
  "msg/LaserAvoid.msg"
  "msg/PatrolWarning.msg"
  "msg/PointArray.msg"
  "msg/Position.msg"
  "msg/PWMServo.msg"
  "msg/SensorState.msg"
  # Services
  "srv/Buzzer.srv"
  "srv/CamDevice.srv"
  "srv/Headlight.srv"
  "srv/Patrol.srv"
  "srv/RGBLight.srv"
  "srv/RobotArm.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
