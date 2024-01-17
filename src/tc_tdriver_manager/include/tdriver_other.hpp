#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "async_tcp_socket/async_tcp_client.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"

#include "tdriver_interface/tdriver_data.hpp"

#include "tc_msgs/msg/amr_lift_motion.hpp"

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

class TDriverOther : public rclcpp::Node
{
public:
	TDriverOther();
	~TDriverOther();

private:
	//Functions
	void init();
	void connect();
	void getClientMessage();
	void parsingClientMessage(char* msg);
	
	void jackingHeightCallBack(const tc_msgs::msg::AmrLiftMotion::SharedPtr msg);

	void logWrite(std::string msg);
	//Variables

	AsyncTcpClient* client_;
	std::thread connection_thread;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;
	TDriverProtocolHeader makeHeader(const uint16_t api_type, int data_length);
	rclcpp::Subscription<tc_msgs::msg::AmrLiftMotion>::SharedPtr jack_sub_;
	
    
	std::shared_ptr<TDriverOtherApi> control_api_;

	bool quit_ = false;
};