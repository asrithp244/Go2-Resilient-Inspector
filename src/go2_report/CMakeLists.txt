cmake_minimum_required(VERSION 3.8)
project(go2_report)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(go2_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  go2_report/inspection_report_gen.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inspection_report_gen
)

ament_package()
