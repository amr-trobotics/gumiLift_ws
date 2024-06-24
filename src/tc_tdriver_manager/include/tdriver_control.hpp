#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "async_tcp_socket/async_tcp_client.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"

#include "tdriver_interface/tdriver_data.hpp"

#include "tc_msgs/msg/amr_open_loop_motion.hpp"
#include "tc_msgs/srv/amr_init.hpp"

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

class TDriverControl : public rclcpp::Node
{
public:
	TDriverControl();
	~TDriverControl();

private:
	//Functions
	void init();
	void connect();
	void getClientMessage();
	void parsingClientMessage(char* msg);
	
	void receiveInitMessage(const std::shared_ptr<tc_msgs::srv::AmrInit::Request> request,
		std::shared_ptr<tc_msgs::srv::AmrInit::Response>	response);

	void getMapChangeMessage(const std_msgs::msg::String::SharedPtr msg);
	void openLoopMotionCallBack(const tc_msgs::msg::AmrOpenLoopMotion::SharedPtr msg);
	void relocation();
	void confirmRelocation();
	void logWrite(std::string msg);
	//Variables

	AsyncTcpClient* client_;
	std::thread connection_thread;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;
	TDriverProtocolHeader makeHeader(const uint16_t api_type, int data_length);
	rclcpp::Subscription<tc_msgs::msg::AmrOpenLoopMotion>::SharedPtr jog_sub_;	
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_change_sub_;
    
	rclcpp::Service<tc_msgs::srv::AmrInit>::SharedPtr init_server_;
    
	std::shared_ptr<TDriverControlApi> control_api_;

	bool quit_ = false;
};