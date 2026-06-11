cmake_minimum_required(VERSION 3.8)
project(as2_platform_blueos)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(as2_platform_blueos_node
  src/as2_platform_blueos.cpp
  src/as2_platform_blueos_node.cpp
)

target_include_directories(as2_platform_blueos_node PRIVATE include)

ament_target_dependencies(as2_platform_blueos_node
  as2_core
  as2_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS as2_platform_blueos_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch resource web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
