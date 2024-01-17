#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tc_msgs/srv/amr_init.hpp"
#include "tc_msgs/msg/amr_init_status.hpp"


#include "async_tcp_socket/async_tcp_client.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"

#include "data_type/t_driver_data_type.hpp"
#include "data_type/gui_data_type.hpp"

#include "tdriver_interface/tdriver_data.hpp"
#include "tdriver_base.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <random>
#include <boost/format.hpp>
#include <vector>
#include <list>

using namespace std::chrono_literals;
using namespace tdriver_api;
using namespace GuiData;

class TDriverStatus : public rclcpp::Node
{
public:
	TDriverStatus();
	~TDriverStatus();

private:
	//Functions
	void initRobot();
	void initClients();
	void getClientMessage();
	void parsingClientMessage(std::string msg);
	void getAmrInitStatus(const tc_msgs::msg::AmrInitStatus::SharedPtr msg);

	void loadMap();
	void localizationRobot();
	void confirmRelocate();

	//Variables
	int map_loading_status_ = -1;
	int robot_localization_status_ = -1;

	std::thread connection_thread;
	std::shared_ptr<ThreadSafeQueue<std::string>> receive_queue_;

	std::shared_ptr<rclcpp::Node> init_node_ ;
	rclcpp::Client<tc_msgs::srv::AmrInit>::SharedPtr init_clinet_;
	rclcpp::Subscription<tc_msgs::msg::AmrInitStatus>::SharedPtr amr_init_status_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
	
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr init_status_publisher_;
	rclcpp::TimerBase::SharedPtr get_client_data_timer_; 


	TDriverBaseClient* client1_;
	TDriverBaseClient* client2_;
	TDriverBaseClient* client3_;
	TDriverBaseClient* client4_;
	TDriverBaseClient* client5_;
	TDriverBaseClient* client6_;
	TDriverBaseClient* client7_;
	TDriverBaseClient* client8_;
	TDriverBaseClient* client9_;
	TDriverBaseClient* client10_;

	std::vector<uint16_t> api_lists;
};