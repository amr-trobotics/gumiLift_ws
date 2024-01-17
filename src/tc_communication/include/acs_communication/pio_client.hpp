#include "rclcpp/rclcpp.hpp"
#include "async_tcp_socket/async_tcp_client.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "helper/thread_safety_queue.hpp"
#include "data_type/acs_data_type.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>
 
using namespace std::chrono_literals;
using nlohmann::json;

class PioClient : public rclcpp::Node
{

public:
	PioClient();
	~PioClient();

private:
    bool quit_ = false;
    json amr_status_;

    void init();
	void getServerMessage();
	void setServerMessage();
    void connect();
	void stringTobyteArray(std::string str, std::vector<char> &bytes);


	std::thread connection_thread_;
	AsyncTcpClient* client_;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;

    rclcpp::TimerBase::SharedPtr get_server_data_timer_; 
	rclcpp::TimerBase::SharedPtr set_server_data_timer_;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr acs_report_sub_;
	void getAmrStatus(const std_msgs::msg::String::SharedPtr msg);
};