cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the message generation tools
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare the message files to compile
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
)

ament_package()