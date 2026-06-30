cmake_minimum_required(VERSION 3.8)
project(go2_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(go2_interfaces REQUIRED)

include_directories(include)

add_executable(mission_bt_node src/mission_bt_node.cpp)

ament_target_dependencies(mission_bt_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  behaviortree_cpp
  go2_interfaces
)

install(TARGETS mission_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
