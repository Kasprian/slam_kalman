cmake_minimum_required(VERSION 3.8)
project(slam_kalman)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Set include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Kalman Filter library
add_library(kalman_filter
  src/estimation/kalman_filter.cpp
)
target_include_directories(kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kalman_filter
  Eigen3::Eigen
)

# Executable for ROS2 node
add_executable(slam_node
  src/nodes/slam_node.cpp
)
target_link_libraries(slam_node
  kalman_filter
  rclcpp
  sensor_msgs::sensor_msgs
  nav_msgs::nav_msgs
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
  kalman_filter
  slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  Eigen3
)

ament_package()