#include "async_tcp_socket/tcp_connection.hpp"


TcpConnection::TcpConnection(boost::asio::io_context& io_context) : socket_(io_context)
{

}


boost::asio::ip::tcp::socket& TcpConnection::getSocket()
{
    return socket_;
}

void TcpConnection::start(std::shared_ptr<ThreadSafeQueue<char*>> queue)
{    
    queue_ = queue;

    std::cout << "read Start" << std::endl;
    read();
} 

void TcpConnection::read()
{
    /*
    auto recv_data = std::make_shared<std::vector<char>>(max_length);
    socket_.async_read_some(boost::asio::buffer(*recv_data),
    [this, recv_data](const boost::system::error_code& error, std::size_t bytes_transferred){
        readComplete(error, recv_data, bytes_transferred);
    });
    */

    socket_.async_read_some(boost::asio::buffer(recv_data_, max_length),
        boost::bind(&TcpConnection::readComplete, this->shared_from_this(),
          boost::asio::placeholders::error));
}

void TcpConnection::write(char* data)
{
    boost::asio::async_write(socket_, boost::asio::buffer(data, strlen(data)), 
        boost::bind(&TcpConnection::writeComplete, this->shared_from_this(),
          boost::asio::placeholders::error));

}

void TcpConnection::write(char* data, int length)
{
    boost::asio::async_write(socket_, boost::asio::buffer(data, length), 
        boost::bind(&TcpConnection::writeComplete, this->shared_from_this(),
          boost::asio::placeholders::error));

}

void TcpConnection::write(unsigned char* data, int length)
{
    boost::asio::async_write(socket_, boost::asio::buffer(data, length), 
        boost::bind(&TcpConnection::writeComplete, this->shared_from_this(),
          boost::asio::placeholders::error));

}

void TcpConnection::writeComplete(const boost::system::error_code& error)
{
    if(!error)
    {
        
    }
    else
    {
        deleteConnection();
    }
}

void TcpConnection::readComplete(const boost::system::error_code& error, std::shared_ptr<std::vector<char>> recv_data, std::size_t bytes_transferred)
{
    if (!error)
    {   
        auto message_type = static_cast<unsigned char>((*recv_data)[2]);
        if(message_type == 1)
        {
            std::cout << "bytes_transferred : " << (int)bytes_transferred << std::endl;
            std::cout << "ping seq : " << (int)(*recv_data)[3] << std::endl;
        }

        auto data_copy = new char[bytes_transferred];
        std::copy(recv_data->begin(), recv_data->begin() + bytes_transferred, data_copy);

        queue_->push(data_copy);

        read();
    }
    else
    {
        std::cout << "read Error" << std::endl;
        std::cout<< error.message() <<std::endl;
        deleteConnection();
    }
}

void TcpConnection::readComplete(const boost::system::error_code& error)
{
    if (!error)
    { 
        char* data_copy = new char[max_length]; // +1 for the null terminator
        
        for(int i=0; i<max_length; i++)
        {
            data_copy[i] = recv_data_[i];
            recv_data_[i] = 0;
        }

        queue_->push(data_copy);

        read();      
    }
    else
    {
        std::cout << "read Error" << std::endl;
        std::cout<< error.message() <<std::endl;
        deleteConnection();
    }
}

void TcpConnection::deleteConnection()
{
    std::cout << "Delete Connection" << std::endl;
    
    socket_.close();
}