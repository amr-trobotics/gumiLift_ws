cmake_minimum_required(VERSION 3.8)
project(tc_communication)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tc_component REQUIRED)
find_package(tc_acs_interface REQUIRED)
find_package(tc_msgs REQUIRED)
find_package(Boost )

include_directories(include)

set(DEPENDENCIES
	"rclcpp"
  "rcutils"
	"std_msgs"	
  "nlohmann_json"
  "tc_component"
  "tc_acs_interface"
  "tc_msgs"
)


add_executable(acs_communication_node src/acs_communication/main.cpp src/acs_communication/acs_communication.cpp src/acs_communication/pio_client.cpp)
ament_target_dependencies(acs_communication_node ${DEPENDENCIES})

add_executable(gui_communication_node src/gui_communication/main.cpp src/gui_communication/gui_communication.cpp)
ament_target_dependencies(gui_communication_node ${DEPENDENCIES})


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

install(TARGETS
  acs_communication_node
  gui_communication_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
