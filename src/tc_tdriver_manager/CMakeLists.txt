cmake_minimum_required(VERSION 3.8)
project(tc_tdriver_manager)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tc_acs_interface REQUIRED)
find_package(tc_component REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tc_msgs REQUIRED)
find_package(Boost )
find_package(nlohmann_json REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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



add_executable(tdriver_manager_node src/main.cpp src/tdriver_navigation.cpp src/tdriver_status.cpp src/tdriver_base.cpp src/tdriver_control.cpp src/tdriver_config.cpp src/tdriver_other.cpp) 
ament_target_dependencies(tdriver_manager_node ${DEPENDENCIES})

install(
  TARGETS tdriver_manager_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
