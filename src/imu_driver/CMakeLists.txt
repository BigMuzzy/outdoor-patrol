cmake_minimum_required(VERSION 3.16)
project(imu_driver CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

set(DRIVER_DEPS
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
)

# Pure protocol library (no ROS deps; reused by tests).
add_library(imu_protocol
  src/frame_parser.cpp
  src/messages.cpp
)
target_include_directories(imu_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Driver node executable
add_executable(imu_driver_node
  src/imu_driver_node.cpp
  src/serial_port.cpp
)
target_include_directories(imu_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(imu_driver_node imu_protocol)
ament_target_dependencies(imu_driver_node ${DRIVER_DEPS})

install(TARGETS imu_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS imu_protocol
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the copyright linter; SPDX headers are present but the corporate
  # copyright-holder registry the linter expects is not configured here.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_parser test/test_frame_parser.cpp)
  target_link_libraries(test_frame_parser imu_protocol)
  target_include_directories(test_frame_parser PRIVATE include)

  ament_add_gtest(test_messages test/test_messages.cpp)
  target_link_libraries(test_messages imu_protocol)
  target_include_directories(test_messages PRIVATE include)
endif()

ament_package()
