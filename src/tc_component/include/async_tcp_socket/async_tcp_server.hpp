#ifndef ASYNC_TCP_SERVER_HPP
#define ASYNC_TCP_SERVER_HPP

#include "boost/asio.hpp"
#include "boost/bind.hpp"

#include "async_tcp_socket/tcp_connection.hpp"
#include "helper/thread_safety_queue.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class AsyncTcpServer
{
private:

public:
    AsyncTcpServer(boost::asio::io_context& io_context, short port); 
    AsyncTcpServer(boost::asio::io_context& io_context, short port, std::shared_ptr<ThreadSafeQueue<char*>> queue_); 
    ~AsyncTcpServer();
    
    void startAccept();
    void stopServer();
    void sendData(char* data);
    void sendData(char* data, int length);
    void sendData(unsigned char* data, int length);

    bool isConnected();
 
private:
 
    boost::asio::io_context& io_context_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::shared_ptr<ThreadSafeQueue<char*>> queue_;
    std::shared_ptr<TcpConnection::pointer> current_connection;

    void handleAccept(TcpConnection::pointer new_connection, const boost::system::error_code& error);

};

#endif