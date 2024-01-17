cmake_minimum_required(VERSION 3.8)
project(tc_pio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tc_msgs REQUIRED)
find_package(tc_component REQUIRED)
find_package(examples_rclcpp_minimal_action_server REQUIRED)

include_directories(include)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "tc_msgs"
  "examples_rclcpp_minimal_action_server"
  "tc_component"
  )

add_executable(${PROJECT_NAME}_node src/main.cpp src/tc_pio_node src/io.cpp src/msg.cpp src/reset.cpp
                                    src/docking.cpp src/charging.cpp src/log.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})


install(TARGETS
${PROJECT_NAME}_node 
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
