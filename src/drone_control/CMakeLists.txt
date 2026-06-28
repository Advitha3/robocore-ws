cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- Node 1: The Brain ---
add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node rclcpp std_msgs)

# --- Node 2: The Simulator ---
add_executable(simulator_node src/drone_simulator_node.cpp)
ament_target_dependencies(simulator_node rclcpp std_msgs)

# Install BOTH nodes
install(TARGETS
  pid_node
  simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()