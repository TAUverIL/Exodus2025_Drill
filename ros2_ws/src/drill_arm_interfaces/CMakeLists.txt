cmake_minimum_required(VERSION 3.8)
project(drill_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Home.action"
  "action/MoveY.action"
  "action/DrillToDepth.action"
)

ament_package()
