cmake_minimum_required(VERSION 3.8)
project(sensor_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_interfaces REQUIRED)

# 2. Include the folder where you copied your Day 1 headers!
include_directories(include/sensor_pipeline)

# 3. Build Node 1: The Publisher
add_executable(sensor_publisher_node src/sensor_publisher_node.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp sensor_interfaces)

# 4. Build Node 2: The Monitor (using the stub we made)
add_executable(safety_monitor_node src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_node rclcpp sensor_interfaces)

# 5. Install BOTH nodes
install(TARGETS
  sensor_publisher_node
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()