#include "gui_communication/gui_communication.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto gui = std::make_shared<GuiCommunication>();

    rclcpp::spin(gui);

    rclcpp::shutdown();

    return 0;
}