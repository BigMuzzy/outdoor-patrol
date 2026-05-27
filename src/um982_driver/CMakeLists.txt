cmake_minimum_required(VERSION 3.16)
project(um982_driver CXX)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DRIVER_DEPS
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  nmea_msgs
  rtcm_msgs
  diagnostic_msgs
  diagnostic_updater
  tf2
  tf2_geometry_msgs
)

# Protocol parser library (pure, no ROS deps; reused by tests).
add_library(um982_protocol
  src/nmea_parser.cpp
  src/unicore_parser.cpp
  src/frame_splitter.cpp
  src/command_builder.cpp
)
target_include_directories(um982_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Driver node executable
add_executable(um982_driver_node
  src/um982_driver_node.cpp
  src/serial_port.cpp
)
target_include_directories(um982_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(um982_driver_node um982_protocol)
ament_target_dependencies(um982_driver_node ${DRIVER_DEPS})

install(TARGETS um982_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS um982_protocol
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
  # Skip copyright/uncrustify until full headers are added in Phase C.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nmea_parser test/test_nmea_parser.cpp)
  target_link_libraries(test_nmea_parser um982_protocol)
  target_include_directories(test_nmea_parser PRIVATE include)

  ament_add_gtest(test_unicore_parser test/test_unicore_parser.cpp)
  target_link_libraries(test_unicore_parser um982_protocol)
  target_include_directories(test_unicore_parser PRIVATE include)

  ament_add_gtest(test_frame_splitter test/test_frame_splitter.cpp)
  target_link_libraries(test_frame_splitter um982_protocol)
  target_include_directories(test_frame_splitter PRIVATE include)

  ament_add_gtest(test_command_builder test/test_command_builder.cpp)
  target_link_libraries(test_command_builder um982_protocol)
  target_include_directories(test_command_builder PRIVATE include)
endif()

ament_package()
