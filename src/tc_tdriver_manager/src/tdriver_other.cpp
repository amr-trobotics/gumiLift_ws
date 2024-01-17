#include "tdriver_other.hpp"

#include <sstream>

TDriverOther::TDriverOther() : Node("seer_other", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    std::cout << "TDriverOther()" << std::endl;

    receive_queue_ = std::make_shared<ThreadSafeQueue<char*>>();
    connection_thread = std::thread(&TDriverOther::connect, this); // 나중에 seer랑 연결하고 호출
        
    control_api_ = std::make_shared<TDriverOtherApi>();
    init();
    std::cout << "TDriverOther() Finished" << std::endl;
}

TDriverOther::~TDriverOther()
{
    quit_ = true;
}

void TDriverOther::connect()
{
    while(!quit_)
    {
        boost::asio::io_context io_context;

        //std::string ip = "10.0.5.91";
        std::string ip = "192.168.192.5";
        std::string port = "19210"; // other
        client_ = new AsyncTcpClient(io_context, ip, port, receive_queue_);
        
        client_->connect();   
    
        io_context.run();
    }    
}

//Functions
void TDriverOther::init()
{    
    jack_sub_ =  this->create_subscription<tc_msgs::msg::AmrLiftMotion>
    ("amr_jack", 10, std::bind(&TDriverOther::jackingHeightCallBack, this, std::placeholders::_1));
    
}

void TDriverOther::getClientMessage()
{
    if(receive_queue_->count() > 0)
    {
        std::cout << receive_queue_->count() << std::endl;

        parsingClientMessage(receive_queue_->front());
        std::cout << "read" << std::endl;
        receive_queue_->pop();
    }
}

void TDriverOther::parsingClientMessage(char* data)
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


    std::cout << "recv : " << return_data << std::endl;
}

TDriverProtocolHeader TDriverOther::makeHeader(const uint16_t api_type, int data_length)
{
    auto header = TDriverProtocolHeader(); 
    header.sync_ = 0x5A;
    header.version_ = 0x01;
    header.number_ = htons(0x01);
    header.length_ = htonl(data_length);
    header.type_ = htons(api_type);

    return header;
}


void TDriverOther::jackingHeightCallBack(const tc_msgs::msg::AmrLiftMotion::SharedPtr msg)
{
    std::cout << "rec ms from amr_jack" << std::endl;
    
    json jack_command;
    bool stop = msg->stop;

    if (stop)
    {
        std::cout << "Stop " << std::endl;
        auto send_data ="";

        int data_length = static_cast<int>(strlen(send_data));
        std::cout << "get data length :  " << data_length  << std::endl;
        auto header = makeHeader(control_api_->jacking_stop, 0);

        int data_size = sizeof(header) + strlen(send_data);
        std::cout << "header : " << sizeof(header) << " send_data : " << strlen(send_data) << std::endl;

        //unsigned char* snd_data = new unsigned char[data_size];
        unsigned char* snd_data = new unsigned char[data_size];

        std::cout << "data size : " << data_size << std::endl;
        
        memcpy(snd_data, &header, sizeof(header));
        memcpy(snd_data + sizeof(header), send_data, strlen(send_data));
        
        std::string log_data = "jacking_stop : ";

        std::stringstream tmp_str;
        tmp_str.str("");
        for(int i=0; i<data_size; i++)
        {        
            int tmp = snd_data[i];
            
            // log_data += std::to_string(tmp);
            // log_data +=  " ";
            tmp_str << std::hex << tmp << " ";
        }
        log_data += tmp_str.str();

        logWrite(log_data);

        client_->write(snd_data, data_size);
        std::cout << "write end" << std::endl;

        delete snd_data;
        
    }
    else
    {
        std::cout << "set height " << std::endl;
        jack_command["height"] = msg->height; // default = 0
        auto str_data = jack_command.dump();
        auto send_data = str_data.c_str();

        std::cout << "make send_data " << std::endl;
        int data_length = static_cast<int>(strlen(send_data));
        std::cout << "get data length :  " << data_length  << std::endl;
        auto header = makeHeader(control_api_->jacking_height, data_length);
        std::cout << "get header" << std::endl;

        int data_size = sizeof(header) + strlen(send_data);
        std::cout << "header : " << sizeof(header) << " send_data : " << strlen(send_data) << std::endl;

        //unsigned char* snd_data = new unsigned char[data_size];
        unsigned char* snd_data = new unsigned char[data_size];

        std::cout << "data size : " << data_size << std::endl;
        
        memcpy(snd_data, &header, sizeof(header));
        memcpy(snd_data + sizeof(header), send_data, strlen(send_data));
        
        std::string log_data = "jacking_height : ";

        std::stringstream tmp_str;
        tmp_str.str("");
        for(int i=0; i<data_size; i++)
        {        
            int tmp = snd_data[i];
            
            // log_data += std::to_string(tmp);
            // log_data +=  " ";
            tmp_str << std::hex << tmp << " ";
        }
        log_data += tmp_str.str();

        logWrite(log_data);

        client_->write(snd_data, data_size);
        std::cout << "write end" << std::endl;

        delete snd_data;
    }
    
    
}

void TDriverOther::logWrite(std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_TDRIVER, LOG_INFO, msg);
}
