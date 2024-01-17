#include "async_tcp_socket/async_tcp_server.hpp"


AsyncTcpServer::AsyncTcpServer(boost::asio::io_context& io_context, short port) 
    : io_context_(io_context), acceptor_(io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
    std::cout << "Init variables" << std::endl;
}

AsyncTcpServer::AsyncTcpServer(boost::asio::io_context& io_context, short port, std::shared_ptr<ThreadSafeQueue<char*>> queue)
    : io_context_(io_context), acceptor_(io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
    
    queue_ = queue;
    std::cout << "Init variables" << std::endl;
}

AsyncTcpServer::~AsyncTcpServer()
{
    std::cout << "~AsyncTcpServer()" << std::endl;
}  

void AsyncTcpServer::startAccept()
{
    auto tcp_connection_ = TcpConnection::createPointer(io_context_);
    std::cout << "Create Tcp Connection" << std::endl;

    acceptor_.async_accept(tcp_connection_->getSocket(),
        boost::bind(&AsyncTcpServer::handleAccept, this, tcp_connection_,
          boost::asio::placeholders::error));

    std::cout << "Binding Accept Handler" << std::endl;
}

void AsyncTcpServer::handleAccept(TcpConnection::pointer new_connection, const boost::system::error_code& error)
{
    if(!error)
    {
        if(current_connection != NULL)
        {
            if(current_connection->get() != nullptr)
            {                
                current_connection->reset();
            }
        }

        current_connection = std::make_shared<TcpConnection::pointer>(new_connection);
        std::cout << "Client Connect" << std::endl;
        new_connection->start(queue_);
    }
    else
    {
        std::cout << "DisConnect" << std::endl;
        new_connection->deleteConnection();
    }

    startAccept();
}

void AsyncTcpServer::stopServer()
{
    std::cout << "Stop Server" << std::endl;
    acceptor_.close();
    delete this;
}

void AsyncTcpServer::sendData(char* data)
{
    if(isConnected())
    {
        current_connection->get()->write(data);
    }
    else
    {
      //  std::cout << "cannot send : no client" << std::endl;
    }
}

void AsyncTcpServer::sendData(char* data, int length)
{
    if(isConnected())
    {
        current_connection->get()->write(data, length);
    }
    else
    {
      //  std::cout << "cannot send : no client" << std::endl;
    }
}

void AsyncTcpServer::sendData(unsigned char* data, int length)
{
    if(isConnected())
    {
        current_connection->get()->write(data, length);
    }
    else
    {
    }
}

bool AsyncTcpServer::isConnected()
{
    if(current_connection != NULL && current_connection->use_count() > 1)
    {
        return current_connection->use_count();
    }
    else
    {
        return false;

    }
}