cmake_minimum_required(VERSION 3.8)
project(planner_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(planner_server 
  src/planner_server.cpp
  include/planner_server/costmap.hpp)
ament_target_dependencies(
  planner_server 
  rclcpp 
  pluginlib
  nav_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  planner_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_package()
