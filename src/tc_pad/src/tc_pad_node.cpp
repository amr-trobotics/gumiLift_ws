#include "pad_comm.hpp"

int main(int argc,char** argv)
{
	rclcpp::init(argc,argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto pad_comm = std::make_shared<PadComm>();
	
	executor.add_node(pad_comm);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
