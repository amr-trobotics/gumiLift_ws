#include "rclcpp/rclcpp.hpp"

#include "tc_acs_interface/srv/acs_docking.hpp"
#include "tc_acs_interface/srv/acs_command.hpp"

#include "async_tcp_socket/async_tcp_client.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/amr_logger.hpp"
#include "data_type/t_driver_data_type.hpp"
#include "data_type/acs_data_type.hpp"

#include "tdriver_interface/tdriver_data.hpp"
#include "tdriver_base.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <random>
#include <boost/format.hpp>

 
class TDriverNavigation : public rclcpp::Node
{
public:
	TDriverNavigation();
	~TDriverNavigation();

private:
	//Functions
	
	void getClientMessage();
	void setClientMessage();
	void parsingClientMessage(char* data);
    void connect();

	json move(std::string source, std::string dest);
	TDriverProtocolHeader makeHeader(uint16_t api_type, int data_length);
	void addMoveTask(json command);
	void turnAmr(double rad, double vw);
	void pauseAmr();
	void stopAmr();
	void resumeAmr();
	void translationAmr(float dist, float vx, float vy);
	
	void logWrite(std::string s);

	//Variables
	std::thread thread_;
	std::thread test_thread_;
	AsyncTcpClient* client_;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;

	ThreadSafeQueue<char*> send_queue_;
	bool quit_ = false;

	rclcpp::Service<tc_acs_interface::srv::AcsCommand>::SharedPtr acs_command_server_;
	rclcpp::Service<tc_acs_interface::srv::AcsDocking>::SharedPtr docking_server_;
    
	void initServiceServers();
	void receiveTransferCommand(const std::shared_ptr<tc_acs_interface::srv::AcsCommand::Request> request,
		std::shared_ptr<tc_acs_interface::srv::AcsCommand::Response>	response);

	void receiveDockingCommand(const std::shared_ptr<tc_acs_interface::srv::AcsDocking::Request> request,
		std::shared_ptr<tc_acs_interface::srv::AcsDocking::Response>	response);

	rclcpp::TimerBase::SharedPtr get_client_data_timer; 
	rclcpp::TimerBase::SharedPtr set_client_data_timer;

	std::shared_ptr<TDriverNavigationApi> navi_api_;
	
};