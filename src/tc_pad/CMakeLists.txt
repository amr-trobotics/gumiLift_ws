cmake_minimum_required(VERSION 3.8)
project(tc_pad)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tc_acs_interface REQUIRED)
find_package(tc_component REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost )
find_package(nlohmann_json REQUIRED)
find_package(tc_msgs REQUIRED)


include_directories(include)

set(DEPENDENCIES
	"rclcpp"
  "rcutils"
	"std_msgs"	
  "tc_acs_interface"
  "tc_component"
  "nlohmann_json"
  "tc_msgs"
)


add_executable(${PROJECT_NAME}_node src/tc_pad_node.cpp src/pad_comm.cpp
                              )
ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})

install(TARGETS
	${PROJECT_NAME}_node 
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
