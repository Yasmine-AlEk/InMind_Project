cmake_minimum_required(VERSION 3.5)
project(academy_robot_sensors)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
