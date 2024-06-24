#ifndef TCP_CONNECTION_HPP
#define TCP_CONNECTION_HPP

#include "helper/thread_safety_queue.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <iostream>


class TcpConnection : public boost::enable_shared_from_this<TcpConnection>
{

public:
	typedef boost::shared_ptr<TcpConnection> pointer;
	static pointer createPointer(boost::asio::io_context &io_context)
	{
		return pointer(new TcpConnection(io_context));
	}
	
	boost::asio::ip::tcp::socket& getSocket();
	void start(std::shared_ptr<ThreadSafeQueue<char*>> queue);
	void deleteConnection();
	void write(char *data);
	void write(char *data, int length);
	void write(unsigned char* data, int length);
	

private:
	TcpConnection(boost::asio::io_context& io_context);

	//Functions
	void read();
	void writeComplete(const boost::system::error_code& error);
	void readComplete(const boost::system::error_code& error);
	void readComplete(const boost::system::error_code& error, std::shared_ptr<std::vector<char>> recv_data, std::size_t bytes_transferred);


	//Variables
	boost::asio::ip::tcp::socket socket_;
	enum { max_length = 4096 };
	char recv_data_[max_length];
	
	std::shared_ptr<ThreadSafeQueue<char*>> queue_;
};

#endif