cmake_minimum_required(VERSION 3.8)
project(sensor_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Broadly include all possible header locations
include_directories(
  include
  include/${PROJECT_NAME}
  src
)

# Sensor Publisher Node
add_executable(sensor_publisher_node src/sensor_publisher_node.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp std_msgs sensor_interfaces)

# Safety Monitor Node
add_executable(safety_monitor_node src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_node rclcpp std_msgs sensor_interfaces std_srvs)

install(TARGETS
  sensor_publisher_node
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()