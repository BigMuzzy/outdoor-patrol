cmake_minimum_required(VERSION 3.8)
project(outdoor_patrol_loc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# heading_to_imu: UM982 dual-antenna QuaternionStamped -> yaw-only Imu.
add_executable(heading_to_imu src/heading_to_imu.cpp)
ament_target_dependencies(heading_to_imu rclcpp geometry_msgs sensor_msgs)

# confidence_gate: inflate NavSatFix covariance on a degraded fix.
add_executable(confidence_gate src/confidence_gate.cpp)
ament_target_dependencies(confidence_gate rclcpp sensor_msgs)

install(TARGETS
  heading_to_imu
  confidence_gate
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
