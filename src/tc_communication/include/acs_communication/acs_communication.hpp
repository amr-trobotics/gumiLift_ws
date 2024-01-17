#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "tc_acs_interface/srv/acs_command.hpp"

#include "nlohmann/json.hpp"

#include "async_tcp_socket/async_tcp_server.hpp"
 
#include "helper/thread_safety_queue.hpp"
#include "helper/amr_logger.hpp"
#include "helper/converter.hpp"
#include "helper/file_io.hpp"

#include "data_type/acs_data_type.hpp"
#include "data_type/t_driver_data_type.hpp"

#include "acs_communication/acs_struct.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>

using namespace std::chrono_literals;
using nlohmann::json;

class AcsCommunication : public rclcpp::Node
{

public:
	AcsCommunication();
	~AcsCommunication();

private:
	char stx_ = 0x02;
    bool quit_ = false;
	bool check_init_ = false;
	bool send_init_ = false;
	bool is_send_fail_report_ = false;
	std::shared_ptr<File_IO> file_io_;
	int seq_ = 0;
	json amr_status_;
	json carrier_info_;
	
	//Acs Server
	std::thread connection_thread_;
	AsyncTcpServer* server_;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;
	ThreadSafeQueue<std::string> send_queue_;


	std::thread connection_thread2_;
	AsyncTcpServer* monitor_server_;
	std::shared_ptr<ThreadSafeQueue<char*>> monitor_receive_queue_;

	//Timer
    rclcpp::TimerBase::SharedPtr get_server_data_timer_; 
	rclcpp::TimerBase::SharedPtr set_server_data_timer_;
	rclcpp::TimerBase::SharedPtr get_agvc_status_timer_;
	rclcpp::TimerBase::SharedPtr keep_alive_timer_;

	std::shared_ptr<rclcpp::Node> service_node_ ;
	
	rclcpp::Client<tc_acs_interface::srv::AcsCommand>::SharedPtr acs_command_client_;
  	
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr acs_conn_pub_;
	
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr acs_report_sub_;
	void getAmrStatus(const std_msgs::msg::String::SharedPtr msg);
	
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr acs_task_sub_;
	void getAmrTaskStatus(const std_msgs::msg::String::SharedPtr msg);

	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr t_driver_init_sub_;
	void tDriverInitCallBack(const std_msgs::msg::Int32::SharedPtr msg);
		

	void initVariables();
    void initPubSub();
    void initTimer();

	void startServer();
	void startMonitorServer();
	void getServerMessage();
	void setServerMessage();
	void getAcsStatus();

	int getTdriverNodeNumber(std::string node);

	//Receive from acs
	void parsingServerMessage(char* data);
	void pingResponse(char* data);
	void commandResponse(char returncode, char* data, int receive_ok = ACCEPT);
	void setTime(char* data);
	void moveWork(char* data);
	void moveWorkCancel(char* data);
	void pauseAmr(char* data);
	void checkMap(char* data);
	void reqeustMapSync(char* data);
	void updateMap(char* data);
	void fetchMap(char* data);
	void stopCharge(char* data);
	void alarmAmr(char* data);
	void scan(char* data);

	void initReportResponse(char* data);
	
	//Send to acs
	void initReport();
	void statusReport();
	void moveCompleted(std::string cur_job_id, std::string canceled_job_id, int type = 0);
	void failReport(std::string cur_job_id, int fail_level, int stop_reason);
	void continueReport(std::string cur_job_id);
	void stateChanged();
	void taskReport(std::string job_id, char number);
	void keepAlive();

	//Send to another node
	void sendToAcsCommand(json j);


	json getPathList(char* data, int count, int* data_index);
	json getTaskList(char* data, int count, int* data_index);
	double getDouble(char* data, int index);

    void logWrite(LogLevel level, std::string msg);
	void logDebug(std::string msg, std::string header);

	std::string byteToString(char* data, int start_index, int length);
	void stringTobyteArray(std::string str, std::vector<char> &bytes);
	void doubleTo8bytes(double value, std::vector<char> &bytes);
};