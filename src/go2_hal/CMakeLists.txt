cmake_minimum_required(VERSION 3.8)
project(go2_hal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(go2_hal_node src/go2_hal_node.cpp)

ament_target_dependencies(go2_hal_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS go2_hal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
