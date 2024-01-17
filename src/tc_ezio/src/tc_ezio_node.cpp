#include "ezio_manager.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto Io_event = std::make_shared<EzioManager>();
  //executor.add_node(Io_node);
  executor.add_node(Io_event);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
