cmake_minimum_required(VERSION 3.8)
project(tc_scenario)

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

add_executable(scenario_node src/scenario/main.cpp src/scenario/scenario.cpp)
ament_target_dependencies(scenario_node ${DEPENDENCIES})

add_executable(data_aggregator_node src/data_aggregator/data_aggregator.cpp)
ament_target_dependencies(data_aggregator_node ${DEPENDENCIES})

add_executable(report_node src/report/report_amr_status.cpp)
ament_target_dependencies(report_node ${DEPENDENCIES})

add_executable(sound_node src/io/sound.cpp)
ament_target_dependencies(sound_node ${DEPENDENCIES})

add_executable(light_node src/io/light.cpp)
ament_target_dependencies(light_node ${DEPENDENCIES})

add_executable(ezio_io_node src/io/ezio_io.cpp)
ament_target_dependencies(ezio_io_node ${DEPENDENCIES})

add_executable(manual_node src/manual/manual.cpp)
ament_target_dependencies(manual_node ${DEPENDENCIES})

add_executable(alarm_node src/alarm/alarm.cpp)
ament_target_dependencies(alarm_node ${DEPENDENCIES})

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
  scenario_node
  data_aggregator_node
  report_node
  sound_node
  light_node
  manual_node
  ezio_io_node
  alarm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
