cmake_minimum_required(VERSION 3.8)
project(go2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds urdf models
  DESTINATION share/${PROJECT_NAME}
)

ament_pac