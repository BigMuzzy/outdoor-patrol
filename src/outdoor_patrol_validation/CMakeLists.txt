cmake_minimum_required(VERSION 3.8)
project(outdoor_patrol_validation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# The RViz panel is optional. The dashboard NODE runs on the robot -- that is
# how a teach pass gets recorded to the robot's own disk instead of the dev
# box's -- and a Pi image has no business carrying rviz_common, Ogre and Qt
# for a plugin it will never load. If the GUI dependencies are absent, build
# and install the Python node alone.
find_package(rviz_common QUIET)
find_package(pluginlib QUIET)
find_package(Qt5 QUIET COMPONENTS Core Widgets)
find_package(yaml_cpp_vendor QUIET)
find_package(yaml-cpp QUIET)

set(BUILD_PANEL FALSE)
if(rviz_common_FOUND AND pluginlib_FOUND AND Qt5_FOUND AND yaml-cpp_FOUND)
  set(BUILD_PANEL TRUE)
  message(STATUS "outdoor_patrol_validation: building the RViz panel")
else()
  message(STATUS
    "outdoor_patrol_validation: rviz_common/Qt5 not found -- building the "
    "dashboard node only (this is expected on the robot image)")
endif()

if(BUILD_PANEL)

# rviz_common's headers are Qt; moc has to run over the panel header.
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/outdoor_patrol_validation/field_dashboard_panel.hpp)

add_library(field_dashboard_panel SHARED
  src/field_dashboard_panel.cpp
  ${MOC_FILES})

target_include_directories(field_dashboard_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(field_dashboard_panel PUBLIC cxx_std_17)
target_link_libraries(field_dashboard_panel
  Qt5::Widgets
  ${YAML_CPP_LIBRARIES})
ament_target_dependencies(field_dashboard_panel
  pluginlib
  rclcpp
  rviz_common
  std_msgs)

# Tells pluginlib the library exports an rviz_common::Panel.
target_compile_definitions(field_dashboard_panel PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS field_dashboard_panel
  EXPORT export_field_dashboard_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

endif()  # BUILD_PANEL

# -- dashboard node ----------------------------------------------------------
# Always built. This is the half that runs on the robot.
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/field_dashboard
  scripts/field_rehearsal
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gate_logic test
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(BUILD_PANEL)
  ament_export_include_directories(include)
  ament_export_targets(export_field_dashboard_panel HAS_LIBRARY_TARGET)
endif()
ament_package()
