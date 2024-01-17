#include <cstdio>
#include "laser_comm.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world adam_modbus package\n");
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto adam_node = std::make_shared<Laser>();
  executor.add_node(adam_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
