cmake_minimum_required(VERSION 3.16)
project(ntrip_client CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

set(COMMON_DEPS
  rclcpp
  std_msgs
  nmea_msgs
  rtcm_msgs
  diagnostic_msgs
  diagnostic_updater
)

# Shared RTCM framing library
add_library(rtcm_framer SHARED
  src/rtcm_framer.cpp
)
target_include_directories(rtcm_framer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# NTRIP caster client
add_executable(ntrip_client_node src/ntrip_client_node.cpp)
target_include_directories(ntrip_client_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ntrip_client_node rtcm_framer)
ament_target_dependencies(ntrip_client_node ${COMMON_DEPS})

# TCP relay (client/server)
add_executable(tcp_rtcm_relay_node src/tcp_rtcm_relay_node.cpp)
target_include_directories(tcp_rtcm_relay_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(tcp_rtcm_relay_node rtcm_framer)
ament_target_dependencies(tcp_rtcm_relay_node ${COMMON_DEPS})

# Serial radio relay
add_executable(serial_rtcm_relay_node src/serial_rtcm_relay_node.cpp)
target_include_directories(serial_rtcm_relay_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(serial_rtcm_relay_node rtcm_framer)
ament_target_dependencies(serial_rtcm_relay_node ${COMMON_DEPS})

install(TARGETS rtcm_framer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ntrip_client_node
  tcp_rtcm_relay_node
  serial_rtcm_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rtcm_framer test/test_rtcm_framer.cpp)
  if(TARGET test_rtcm_framer)
    target_include_directories(test_rtcm_framer PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_rtcm_framer rtcm_framer)
  endif()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(rtcm_framer)
ament_package()
