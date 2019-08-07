cmake_minimum_required(VERSION 3.5)
project(rcl_executor_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_executor REQUIRED)


add_executable(example_publisher example_publisher.cpp)
ament_target_dependencies(example_publisher rclcpp std_msgs geometry_msgs)

add_executable(example_executor example_executor.c)
ament_target_dependencies(example_executor rcl rcl_executor std_msgs geometry_msgs)

install(TARGETS
  example_publisher
  example_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
