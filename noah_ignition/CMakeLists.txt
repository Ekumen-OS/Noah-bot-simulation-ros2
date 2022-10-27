cmake_minimum_required(VERSION 3.5)

project(noah_ignition)

# Skip if ignition not present
find_package(ros_ign_gazebo QUIET)
if(NOT ros_ign_gazebo_FOUND)
  message(WARNING "ignition not found, proceeding without that simulator.")
  return()
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    rviz
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
