cmake_minimum_required(VERSION 3.16.3)
project(atomos_xlink_hw_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
