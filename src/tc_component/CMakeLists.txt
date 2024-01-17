cmake_minimum_required(VERSION 3.8)
project(tc_component)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost )
find_package(nlohmann_json REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_library(tc_component src/async_tcp_socket/tcp_connection.cpp 
  src/async_tcp_socket/async_tcp_server.cpp 
  src/async_tcp_socket/async_tcp_client.cpp)

ament_export_targets(tc_component HAS_LIBRARY_TARGET)

set(DEPENDENCIES
  "nlohmann_json"
)

set_property(TARGET tc_component PROPERTY CXX_STANDARD 17)
set_property(TARGET tc_component PROPERTY CXX_STANDARD_REQUIRED ON)

ament_target_dependencies(tc_component ${DEPENDENCIES})

install(
  DIRECTORY include/async_tcp_socket 
  DESTINATION include
)
install(
  DIRECTORY include/helper 
  DESTINATION include
)

install(
  DIRECTORY include/data_type
  DESTINATION include
)


install(
  TARGETS tc_component
  EXPORT tc_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

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
