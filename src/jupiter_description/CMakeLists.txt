cmake_minimum_required(VERSION 3.8)
project(jupiter_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
