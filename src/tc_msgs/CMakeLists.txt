cmake_minimum_required(VERSION 3.5)

project(tc_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(msg_files

  "msg/AmrTask.msg"
  "msg/IoModMsg.msg"
  "msg/LaserDistance.msg"
  "msg/AmrOpenLoopMotion.msg"
  "msg/AmrAlarm.msg"
  "msg/AmrInitStatus.msg"
  "msg/Piohybrid.msg"
  "msg/Lift.msg"
  "msg/AmrLiftMotion.msg"

  "srv/IoHeader.srv"
  "srv/Io.srv"
  "srv/AcsToPio.srv"
  "srv/IoModToMan.srv"
  "srv/LaserEnable.srv"
  "srv/ArucoMarkers.srv"
  "srv/Motor.srv"
  "srv/AmrTask.srv"
  "srv/AmrInit.srv"
  "srv/CylinderMode.srv"
  "srv/LidarArea.srv"
  "srv/AmbientLight.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
