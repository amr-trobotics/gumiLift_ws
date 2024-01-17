#include "pio.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init (argc,argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto tc_pio = std::make_shared<PIO>();
    executor.add_node(tc_pio);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


