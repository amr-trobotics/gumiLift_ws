#include "tdriver_navigation.hpp"
#include "tdriver_status.hpp"
#include "tdriver_control.hpp"
#include "tdriver_config.hpp"
#include "tdriver_other.hpp"

int main(int argc,char** argv)
{    
	rclcpp::init(argc,argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto tdriver_navi = std::make_shared<TDriverNavigation>();
	auto tdriver_status = std::make_shared<TDriverStatus>();
	auto tdriver_control = std::make_shared<TDriverControl>();
	auto tdriver_config = std::make_shared<TDriverConfig>();
	auto tdriver_other = std::make_shared<TDriverOther>();

	executor.add_node(tdriver_navi);
	executor.add_node(tdriver_status);
	executor.add_node(tdriver_control);
	executor.add_node(tdriver_config);
	executor.add_node(tdriver_other);

	executor.spin();
	 
	rclcpp::shutdown();

	return 0;
}
