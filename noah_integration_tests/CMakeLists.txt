cmake_minimum_required(VERSION 3.5)

project(noah_integration_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  ament_add_gtest(
    ${PROJECT_NAME}_odometry_drift_test
    test/odometry_drift_test.cpp
    )
  ament_target_dependencies(${PROJECT_NAME}_odometry_drift_test rclcpp nav_msgs)

  install(TARGETS
    ${PROJECT_NAME}_odometry_drift_test
    DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()
