#ifndef TDRIVER_BASE_HPP
#define TDRIVER_BASE_HPP

#include "rclcpp/rclcpp.hpp"

#include "async_tcp_socket/async_tcp_client.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/amr_logger.hpp"

#include "tdriver_interface/tdriver_data.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <random>
#include <vector>
#include <boost/format.hpp>


using namespace std::chrono_literals;
using namespace tdriver_api;

class TDriverBaseClient
{
public:
	TDriverBaseClient();
	~TDriverBaseClient();
	
	//Functions
    void init(std::shared_ptr<ThreadSafeQueue<std::string>> queue, uint16_t api_list);
    void init(std::shared_ptr<ThreadSafeQueue<std::string>> queue, std::vector<uint16_t> api_list);
	void stop();
    void connect();
	void getClientMessage();
	void setClientMessage();
	void parsingClientMessage(char* data);

	TDriverProtocolHeader makeHeader(uint16_t api_type, int data_length);

	void logWrite(LogLevel level, std::string msg);

	//Variables
	AsyncTcpClient* client_;
	std::thread data_send_thread_;
	std::thread data_receive_thread_;
	std::thread conn_thread_;

	std::shared_ptr<ThreadSafeQueue<std::string>> parent_queue_;
	std::shared_ptr<ThreadSafeQueue<char*>> receive_queue_;

    uint16_t api_lists;
	std::vector<uint16_t> api_list_;

    bool quit_ ;
};

#endif