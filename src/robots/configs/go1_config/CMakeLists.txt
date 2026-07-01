cmake_minimum_required(VERSION 3.5)
project(go1_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config include launch maps worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES config.json setup.bash
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
