cmake_minimum_required(VERSION 3.8)
project(outdoor_patrol_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_localization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# The route reader and the station subsampler, split out of the node so the
# tests below can exercise them without a ROS graph.
add_library(route_goals SHARED src/route_goals.cpp)
target_include_directories(route_goals PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(route_goals yaml-cpp::yaml-cpp)

add_executable(patrol_mission src/patrol_mission.cpp)
target_include_directories(patrol_mission PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(patrol_mission route_goals)
ament_target_dependencies(patrol_mission
  geographic_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  robot_localization
  sensor_msgs
  std_msgs
  diagnostic_msgs
  diagnostic_updater)

install(TARGETS patrol_mission
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS route_goals
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY bt config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_route_goals test/test_route_goals.cpp)
  target_include_directories(test_route_goals PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_route_goals route_goals)
  # The fixture is found relative to the source tree, not the install tree,
  # because colcon test runs from the build directory.
  target_compile_definitions(test_route_goals PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
endif()

ament_export_include_directories(include)
ament_export_libraries(route_goals)
ament_package()
