cmake_minimum_required(VERSION 3.5)
project(state_estimator_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# Create executable
add_executable(state_estimator_node src/state_estimator_node.cpp)
ament_target_dependencies(state_estimator_node rclcpp sensor_msgs custom_msgs Eigen3)

# Install targets
install(TARGETS
  state_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()