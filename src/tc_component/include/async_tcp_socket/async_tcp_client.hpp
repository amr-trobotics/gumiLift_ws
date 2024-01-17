#ifndef ASYNC_TCP_CLIENT_HPP
#define ASYNC_TCP_CLIENT_HPP

#include "boost/asio.hpp"
#include "boost/bind.hpp"

#include "async_tcp_socket/tcp_connection.hpp"
#include "helper/thread_safety_queue.hpp"

#include <iostream>


class AsyncTcpClient
{
public:
    AsyncTcpClient(boost::asio::io_context& io_context, const std::string host, const std::string port, std::shared_ptr<ThreadSafeQueue<char*>> queue);
    void connect();
    void disconnect();
    void write(char* data);
    void write(char* data, int length);    
    void write(unsigned char* data, int length);
private:
    //Function
    void handleConnect(const boost::system::error_code& error);    
    void handleWrite(const boost::system::error_code& error);

    void read();
    void handleRead(const boost::system::error_code& error);

    //Variables
    boost::asio::io_context& io_context_;
    boost::asio::ip::tcp::socket socket_;
    std::string host_;
    std::string port_;

    enum { max_length = 4096 };
    char recv_data_[max_length];    

    std::shared_ptr<ThreadSafeQueue<char*>> queue_;
};


#endif