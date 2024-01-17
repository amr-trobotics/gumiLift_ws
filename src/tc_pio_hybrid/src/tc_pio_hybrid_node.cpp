#include "hypio_serial_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto hybrid_pio_node = std::make_shared<HyPioSerialPublisher>();
  executor.add_node(hybrid_pio_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
