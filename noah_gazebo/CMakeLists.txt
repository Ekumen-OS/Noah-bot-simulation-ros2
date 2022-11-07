cmake_minimum_required(VERSION 3.5)

project(noah_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    photos
    rviz
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
