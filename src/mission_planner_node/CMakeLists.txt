cmake_minimum_required(VERSION 3.5)
project(mission_planner_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Create executable
add_executable(mission_planner_node src/mission_planner_node.cpp)
ament_target_dependencies(mission_planner_node rclcpp geometry_msgs custom_msgs)

# Install targets
install(TARGETS
  mission_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()