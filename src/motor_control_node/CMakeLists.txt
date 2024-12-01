cmake_minimum_required(VERSION 3.5)
project(motor_control_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/path/to/Navio2/C++/Navio  # Path to Navio library headers
)

# Create executable
add_executable(motor_control_node src/motor_control_node.cpp)
ament_target_dependencies(motor_control_node rclcpp custom_msgs Eigen3)

# Link Navio library
target_link_libraries(motor_control_node ${CMAKE_CURRENT_SOURCE_DIR}/path/to/Navio2/C++/build/libnavio.a)

# Install targets
install(TARGETS
  motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()