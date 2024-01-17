cmake_minimum_required(VERSION 3.8)
project(tc_laser)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

set(DEPENDENCIES
  "rclcpp"
  "rcutils"
  "sensor_msgs"
)

add_executable(tc_laser_node src/tc_laser_node.cpp src/laser_comm.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})

install(TARGETS tc_laser_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
