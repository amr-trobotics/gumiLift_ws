#include "acs_communication/acs_communication.hpp"
#include "acs_communication/pio_client.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc,argv);
	rclcpp::executors::SingleThreadedExecutor executor;

	auto acs = std::make_shared<AcsCommunication>();
	//auto pio = std::make_shared<PioClient>();

	executor.add_node(acs);	
	//executor.add_node(pio);

	executor.spin(); 
	
	rclcpp::shutdown();

	return 0;

    return 0;
}