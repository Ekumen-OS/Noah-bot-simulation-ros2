cmake_minimum_required(VERSION 3.5)
project(noah_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
