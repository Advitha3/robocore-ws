cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# EKF Node
add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp std_msgs)

# PID Controller Node
add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node rclcpp std_msgs)

# Drone Simulator Node
add_executable(drone_simulator_node src/drone_simulator_node.cpp)
ament_target_dependencies(drone_simulator_node rclcpp std_msgs)

# Bag Analyzer Node (NEW)
add_executable(bag_analyzer_node src/bag_analyzer_node.cpp)
ament_target_dependencies(bag_analyzer_node rclcpp std_msgs)

install(TARGETS
  ekf_node
  pid_node
  drone_simulator_node
  bag_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()