cmake_minimum_required(VERSION 3.5)

project(code_map_localization_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(code_map_localization
  "msg/Correlation.msg"
  "msg/Odometry.msg"
  "msg/LocationMatch.msg"
  "msg/Localization.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
