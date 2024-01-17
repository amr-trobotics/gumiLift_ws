#include "rclcpp/rclcpp.hpp"
#include "async_tcp_socket/async_tcp_server.hpp"
#include "std_msgs/msg/string.hpp"
#include "tc_msgs/msg/amr_init_status.hpp"

#include "data_type/gui_data_type.hpp"

#include "helper/thread_safety_queue.hpp"
#include "helper/converter.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"

#include "nlohmann/json.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include "tc_msgs/srv/io_header.hpp"
#include "tc_msgs/msg/io_mod_msg.hpp"
#include "tc_msgs/srv/io.hpp"
#include "tc_msgs/msg/amr_open_loop_motion.hpp"
#include "tc_msgs/msg/amr_lift_motion.hpp"

#include "data_type/acs_data_type.hpp"
#include "data_type/constants.h"
#include "data_type/datatype.h"


using nlohmann::json;
using namespace std::chrono_literals;
using namespace GuiData;

class PadComm : public rclcpp::Node
{
public:
	PadComm();
	~PadComm();

	

private:
	//Functions
	
	void getClientMessage();
	void setClientMessage();
	void parsingClientMessage(char* data);
    void connect();
	void parsingToSendingMsg(std::string json_data);
	void getEzioInputData(const tc_msgs::msg::IoModMsg::SharedPtr msg);
	void ezioSendToPad();
	void setPadStatus();
	int getKeyState();
	void setEzioDigitalOutput(int id, int state);
	
	void padMsgsReceive(std::string msg_recv);

	void logWrite(LogLevel level, std::string s);

	std::string convertToString(uint8_t *str);

	//Variables
	std::thread thread_;
	AsyncTcpServer* server_;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;
	ThreadSafeQueue<std::string> send_queue_;
	bool quit_ = false;
	bool is_gui_conn = false;
	int init_status_;
	bool is_init_done = false;
	json node_info_;
	int key_mode_ = -1;

	//iomod_info ezio_input_; 
	std::shared_ptr<std::vector<int>> ezio_input_;
	std::shared_ptr<std::vector<int>> ezio_output_;

	rclcpp::TimerBase::SharedPtr get_client_data_timer; 
	rclcpp::TimerBase::SharedPtr set_client_data_timer;
	rclcpp::TimerBase::SharedPtr set_pad_status_timer_;

	//pub
    rclcpp::Publisher<tc_msgs::msg::AmrOpenLoopMotion>::SharedPtr amr_jog_pub_;
    rclcpp::Publisher<tc_msgs::msg::AmrLiftMotion>::SharedPtr amr_jack_pub_;

	rclcpp::Client<tc_msgs::srv::IoHeader>::SharedPtr ezio_client_;   
	rclcpp::Subscription<tc_msgs::msg::IoModMsg>::SharedPtr ezio_sub_;


    //Test용 함수
	int get_system_count = 0;
	int get_tdriver_count = 0;
};
