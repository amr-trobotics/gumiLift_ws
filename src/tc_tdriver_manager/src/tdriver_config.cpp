#include "tdriver_config.hpp"

TDriverConfig::TDriverConfig() : Node("seer_config")
{
    config_api_ = std::make_shared<TDriverConfigApi>();
	receive_queue_ = std::make_shared<ThreadSafeQueue<char*>>();
    connection_thread = std::thread(&TDriverConfig::connect, this); 
    init();
}

TDriverConfig::~TDriverConfig()
{
    quit_ = true;
}


void TDriverConfig::init()
{
    clear_error_sub_ = this->create_subscription<std_msgs::msg::String>
    ("tdriver_error_reset", 10, std::bind(&TDriverConfig::clearAllRobotErrors, this, std::placeholders::_1));

}

void TDriverConfig::connect()
{
    while(!quit_)
    {
        boost::asio::io_context io_context;

        //std::string ip = "10.0.5.91";
        std::string ip = "192.168.192.5";
        std::string port = "19207"; // control
        client_ = new AsyncTcpClient(io_context, ip, port, receive_queue_);
        
        client_->connect();   
    
        io_context.run();
    }    
}

void TDriverConfig::getClientMessage()
{
    if(receive_queue_->count() > 0)
    {
        std::cout << receive_queue_->count() << std::endl;

        parsingClientMessage(receive_queue_->front());
        std::cout << "read" << std::endl;
        receive_queue_->pop();
    }
}

void TDriverConfig::parsingClientMessage(char* data)
{
    unsigned char string_length[4];
    for(int i=4; i<8; i++)
    {
        int j = i-4;
        string_length[j] = data[i];
    }

    
    int length = 0;
    length = string_length[3];
    length |= string_length[2] << 8;
    length |= string_length[1] << 16;
    length |= string_length[0] << 24;


    std::string return_data(&data[16], length);

    //int len = length + 1;
    //char return_data[len];

    //memcpy(return_data, &data[16], length);
    //return_data[length]='\0';

    //std::string s(return_data);

    std::cout << "recv : " << return_data << std::endl;
}

TDriverProtocolHeader TDriverConfig::makeHeader(const uint16_t api_type, int data_length)
{
    auto header = TDriverProtocolHeader(); 
    header.sync_ = 0x5A;
    header.version_ = 0x01;
    header.number_ = htons(0x01);
    header.length_ = htonl(data_length);
    header.type_ = htons(api_type);

    return header;
}

void TDriverConfig::clearAllRobotErrors(const std_msgs::msg::String::SharedPtr msg)
{
    std::string data = msg->data;

    std::cout << "data : " << data << std::endl;
    if(data == "clear")
    {
        auto header = makeHeader(config_api_->clear_robot_all_errors, 0);
        
        int data_size = sizeof(header);    
        //std::cout << "data_size : " << data_size << std::endl;
        unsigned char* snd_data = new unsigned char[data_size];

        
        memcpy(snd_data, &header, sizeof(header));

        client_->write(snd_data, data_size);

        delete snd_data;

        std::cout << "alarm clear send done" << std::endl;
    }        
}