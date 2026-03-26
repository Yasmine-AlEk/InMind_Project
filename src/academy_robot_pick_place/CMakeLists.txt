cmake_minimum_required(VERSION 3.8)
project(academy_robot_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(academy_robot_interfaces REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

add_executable(object_detector_node src/object_detector_node.cpp)
ament_target_dependencies(object_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  academy_robot_interfaces
)

target_include_directories(object_detector_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(object_detector_node
  ${PCL_LIBRARIES}
)

target_compile_definitions(object_detector_node PRIVATE
  ${PCL_DEFINITIONS}
)

add_executable(pick_place_server src/pick_place_server.cpp)
ament_target_dependencies(pick_place_server
  rclcpp
  rclcpp_action
  geometry_msgs
  academy_robot_interfaces
  moveit_task_constructor_core
)

add_executable(pick_place_client_node src/pick_place_client_node.cpp)
ament_target_dependencies(pick_place_client_node
  rclcpp
  rclcpp_action
  geometry_msgs
  academy_robot_interfaces
)

install(TARGETS
  object_detector_node
  pick_place_server
  pick_place_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
