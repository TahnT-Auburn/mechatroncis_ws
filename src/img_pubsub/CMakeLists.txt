cmake_minimum_required(VERSION 3.8)
project(img_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(img_pubsub_node src/img_pubsub_node.cpp)
ament_target_dependencies(
  img_pubsub_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs)

install(TARGETS 
  img_pubsub_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
