#include "async_tcp_socket/async_tcp_client.hpp"

AsyncTcpClient::AsyncTcpClient(boost::asio::io_context& io_context, const std::string host, const std::string port, std::shared_ptr<ThreadSafeQueue<char*>> queue) 
    : io_context_(io_context), socket_(io_context), host_(host), port_(port), queue_(queue)
{

    std::cout << "client constructor!" << std::endl;
}

void AsyncTcpClient::connect()
{
    std::cout << "client try to connect!" << std::endl;
    boost::asio::ip::tcp::resolver resolver(io_context_);
    boost::asio::ip::tcp::resolver::query query(host_, port_);
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    boost::asio::async_connect(socket_, endpoint_iterator,
        boost::bind(&AsyncTcpClient::handleConnect, this, boost::asio::placeholders::error));
}
 
void AsyncTcpClient::disconnect()
{
    boost::system::error_code error;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
    
    if (error) 
    {
        std::cerr << "Error shutting down socket: " << error.message() << std::endl;
    }

    try
    {
        socket_.close();
    }
    catch(const boost::system::system_error& error) 
    {
        std::cerr << "Error disconnecting: " << error.what() << std::endl;
    }

}

void AsyncTcpClient::handleConnect(const boost::system::error_code& error)
{
    if(!error)
    {
        std::cout << "Connected to Client" << std::endl;
        
        read();
        
    }
    else
    {
        std::cout << "Clinet connection Error : " << error.message() << std::endl;
    }
}

void AsyncTcpClient::write(char* data)
{

    boost::asio::async_write(socket_, boost::asio::buffer(data, strlen(data)),
        boost::bind(&AsyncTcpClient::handleWrite, this, boost::asio::placeholders::error)); 
}

void AsyncTcpClient::write(char* data, int length)
{

    boost::asio::async_write(socket_, boost::asio::buffer(data, length),
        boost::bind(&AsyncTcpClient::handleWrite, this, boost::asio::placeholders::error)); 
}


void AsyncTcpClient::write(unsigned char* data, int length)
{

    boost::asio::async_write(socket_, boost::asio::buffer(data, length),
        boost::bind(&AsyncTcpClient::handleWrite, this, boost::asio::placeholders::error)); 
}

void AsyncTcpClient::handleWrite(const boost::system::error_code& error)
{
    if(!error)
    {
        
    }
    else
    {
        std::cout << "write error : " << error.message() << std::endl;
    }
}

void AsyncTcpClient::read()
{
    socket_.async_read_some(boost::asio::buffer(recv_data_, max_length),
        boost::bind(&AsyncTcpClient::handleRead, this, boost::asio::placeholders::error));  
                      
}

void AsyncTcpClient::handleRead(const boost::system::error_code& error)
{
    if (!error)
    {
        //std::cout << "read done" << std::endl;
        queue_->push(recv_data_);

        read();
    }
    else
    {
        std::cout << "read error : " << error.message() << std::endl;
    }
}




