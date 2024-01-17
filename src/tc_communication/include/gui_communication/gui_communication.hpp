#include "rclcpp/rclcpp.hpp"
#include "async_tcp_socket/async_tcp_server.hpp"
#include "std_msgs/msg/string.hpp"

#include "data_type/gui_data_type.hpp"

#include "helper/thread_safety_queue.hpp"
#include "helper/converter.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tc_msgs/msg/amr_init_status.hpp"
#include "data_type/t_driver_data_type.hpp"

#include "nlohmann/json.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using nlohmann::json;

class GuiCommunication : public rclcpp::Node
{
public:
	GuiCommunication();
	~GuiCommunication();

private:
	//Variables
	std::thread thread_;
	AsyncTcpServer* server_;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;
	ThreadSafeQueue<std::string> send_queue_;
	bool quit_ = false;
	bool is_gui_conn = false;
	bool is_init = false;
	int init_status_ = false;
	bool is_send_relocate = false;
	int robot_localization_state;
	json node_info_;
	int seqnm = 0;
	int count = 0;
	std::string relocate_map = "";
	bool start_relocate = false;

	double agv_x_;
	double agv_y_;
	int agv_angle_;
	std::string current_node_;


private:
	void connect(); 
	void getClientMessage();
	void setClientMessage();
	void parsingClientMessage(char* data);
	void getAmrStatus(const std_msgs::msg::String::SharedPtr msg);
	void getSystemStatus(const std_msgs::msg::String::SharedPtr msg);
	void getIOStatus(const std_msgs::msg::String::SharedPtr msg);
	void getGuiStatus();
	void tDriverInitCallBack(const std_msgs::msg::Int32::SharedPtr msg);
	std::string getNodeName(int node);
	void changeMapAndRelocate();
	void sendToPad(int state);

	void logWrite(LogLevel level, std::string s);

	rclcpp::TimerBase::SharedPtr get_client_data_timer; 
	rclcpp::TimerBase::SharedPtr set_client_data_timer;
	rclcpp::TimerBase::SharedPtr get_gui_status_timer_;
	rclcpp::TimerBase::SharedPtr relocation_timer;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gui_msg_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_change_pub_;
	rclcpp::Publisher<tc_msgs::msg::AmrInitStatus>::SharedPtr amr_init_status_pub_;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr t_driver_status_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr io_status_sub;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr t_driver_init_sub_;	
       
};