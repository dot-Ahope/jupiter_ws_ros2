cmake_minimum_required(VERSION 3.5)
project(jupiter_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config param rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/base
  scripts/device_srv
  scripts/calibrate_angular
  scripts/calibrate_linear
  scripts/jupiter_driver
  scripts/jupiter_patrol
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
