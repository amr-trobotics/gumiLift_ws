cmake_minimum_required(VERSION 3.8)
project(tc_ezio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tc_msgs REQUIRED)
find_package(tc_component REQUIRED)

include_directories(include)

set(DEPENDENCIES
  "message_filters"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "geometry_msgs"
  "tc_msgs"
  "tc_component"
  )

add_executable(${PROJECT_NAME}_node src/tc_ezio_node.cpp src/ezio_manager.cpp 
                              src/ezio_comm01.cpp  src/ezio_comm02.cpp
                              src/ezio_comm03.cpp  src/ezio_comm04.cpp 
                              src/ezio_comm05.cpp   
                              src/log.cpp                           
                              )
ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})

install(TARGETS
	${PROJECT_NAME}_node 
	DESTINATION lib/${PROJECT_NAME})

ament_package()
