cmake_minimum_required(VERSION 3.8)
project(sensor_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include the folder where you copied your Day 1 and Day 6 headers
include_directories(include/sensor_pipeline)

# Build Node 1: The Publisher
add_executable(sensor_publisher_node src/sensor_publisher_node.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp sensor_interfaces)

# Build Node 2: The Monitor
add_executable(safety_monitor_node src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_node rclcpp sensor_interfaces std_msgs std_srvs)

# Install BOTH nodes
install(TARGETS
  sensor_publisher_node
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---> NEW: Install the launch files so ros2 launch can find them <---
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()