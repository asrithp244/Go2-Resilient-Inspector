cmake_minimum_required(VERSION 3.8)
project(go2_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  go2_perception/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_node
)

ament_package()
