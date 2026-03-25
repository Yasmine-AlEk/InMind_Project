cmake_minimum_required(VERSION 3.5)
project(academy_robot_safety_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
)

set(srv_files
)

set(action_files
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
#   ${action_files}
#   DEPENDENCIES
#     builtin_interfaces
#   ADD_LINTER_TESTS
# )

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
