cmake_minimum_required(VERSION 3.8)
project(go2_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joy_teleop_node src/joy_teleop_node.cpp)

ament_target_dependencies(joy_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS joy_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
