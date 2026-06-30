cmake_minimum_required(VERSION 3.8)
project(go2_health_monitor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(go2_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  go2_health_monitor/topic_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME topic_health_monitor
)

ament_package()
