cmake_minimum_required(VERSION 3.5)
project(gps_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/path/to/Navio2/C++/Navio  # Path to Navio library headers
)

# Create executable
add_executable(gps_node src/gps_node.cpp)
ament_target_dependencies(gps_node rclcpp sensor_msgs)

# Link Navio library
target_link_libraries(gps_node ${CMAKE_CURRENT_SOURCE_DIR}/path/to/Navio2/C++/build/libnavio.a)

# Install targets
install(TARGETS
  gps_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()