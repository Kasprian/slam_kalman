cmake_minimum_required(VERSION 3.16)
project(slam_kalman)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Kalman Filter library
add_library(kalman_filter
  src/estimation/kalman_filter.cpp
)
target_include_directories(kalman_filter PUBLIC
  ${Eigen3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(kalman_filter
  Eigen3::Eigen
)

# SLAM Node executable
add_executable(slam_node
  src/nodes/slam_node.cpp
)
target_link_libraries(slam_node
  kalman_filter
  ${rclcpp_LIBRARIES}
)
ament_target_dependencies(slam_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# Install targets
install(TARGETS kalman_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for ROS2
ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
  Eigen3
)
ament_package()