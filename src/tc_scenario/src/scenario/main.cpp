#include "scenario/scenario.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto acs = std::make_shared<Scenario>();

    rclcpp::spin(acs);

    rclcpp::shutdown();

    return 0;
}