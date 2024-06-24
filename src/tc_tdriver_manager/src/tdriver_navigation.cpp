#include "tdriver_navigation.hpp"

TDriverNavigation::TDriverNavigation() : Node("seer_manager", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    try
    {
        std::cout << "TDriverNavigation()" << std::endl;
        receive_queue_ = std::make_shared<ThreadSafeQueue<char*>>();
        navi_api_ = std::make_shared<TDriverNavigationApi>();
        thread_ = std::thread(&TDriverNavigation::connect, this); // 나중에 seer랑 연결하고 호출
        get_client_data_timer = this->create_wall_timer(5ms, std::bind(&TDriverNavigation::getClientMessage,this));
        set_client_data_timer = this->create_wall_timer(5ms, std::bind(&TDriverNavigation::setClientMessage,this));

        initServiceServers();
        std::cout << "TDriverNavigation() end" << std::endl;
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
     
}

TDriverNavigation::~TDriverNavigation()
{
    std::cout << "~TDriverNavigation()" << std::endl;
    quit_ = true;
    test_thread_.join();
    delete client_;
}

void TDriverNavigation::initServiceServers()
{
    acs_command_server_ = this->create_service<tc_acs_interface::srv::AcsCommand>("acs_command_to_tdriver", 
            std::bind(&TDriverNavigation::receiveTransferCommand, this, std::placeholders::_1, std::placeholders::_2));

    docking_server_ = this->create_service<tc_acs_interface::srv::AcsDocking>("acs_docking", 
    std::bind(&TDriverNavigation::receiveDockingCommand, this, std::placeholders::_1, std::placeholders::_2));

    std::cout << "create service server" << std::endl;
}

void TDriverNavigation::connect()
{    
    while(!quit_)
    {
        boost::asio::io_context io_context;

        //std::string ip = "10.0.5.91";
        std::string ip = "192.168.192.5";
        std::string port = "19206"; // navigation
        client_ = new AsyncTcpClient(io_context, ip, port, receive_queue_);
        
        client_->connect();   
    
        io_context.run();
    }
}

void TDriverNavigation::getClientMessage()
{
    if(receive_queue_->count() > 0)
    {
        std::cout << receive_queue_->count() << std::endl;

        parsingClientMessage(receive_queue_->front());
        std::cout << "read" << std::endl;
        receive_queue_->pop();
    }
}

void TDriverNavigation::setClientMessage()
{
    if(send_queue_.count() > 0)
    {
        client_->write(send_queue_.front());
        send_queue_.pop();
    }
}

void TDriverNavigation::parsingClientMessage(char* data)
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

    //logWrite("[Recv]" + return_data);

    std::cout << "end recv" << std::endl;
}

void TDriverNavigation::receiveTransferCommand(const std::shared_ptr<tc_acs_interface::srv::AcsCommand::Request> request,
    std::shared_ptr<tc_acs_interface::srv::AcsCommand::Response>	response)
{
    std::string msg = request->msg;
    
    json command = json::parse(msg);

    auto action = command["action"].get<int>();
    
    logWrite("Receive Action : " + std::to_string(action));

    switch(action)
    {
        case MOVE_WORK:
            addMoveTask(command);
        break;
        case MOVE_WORK_CANCEL:
            stopAmr();
        break;
        case STOP_CHARGE:
        break;
        case PAUSE_AMR:
            pauseAmr();
        break;
        case RESUME_AMR:
            resumeAmr();
        break;
    }

    response->received = true;
    
}
void TDriverNavigation::receiveDockingCommand(const std::shared_ptr<tc_acs_interface::srv::AcsDocking::Request> request,
    std::shared_ptr<tc_acs_interface::srv::AcsDocking::Response>	response)
{
    std::cout << "Receive Docking Command!!!" << std::endl;
    std::cout << "Order : " << request->order << std::endl;
    if(request->order == ROTATION)
    {
        // -vw : cw , vw : ccw
        double rad = request->theta; // 
        double vw = request->vw;

        turnAmr(rad,vw);
    }   
    else if(request->order == TRANSLATION)
    {
        float dist = request->dist;
        float vx = request->vx;
        float vy = request->vy;
        translationAmr(dist, vx, vy);
    } 

    response->received = true;
}

TDriverProtocolHeader TDriverNavigation::makeHeader(const uint16_t api_type, int data_length)
{
    auto header = TDriverProtocolHeader(); 
    header.sync_ = 0x5A;
    header.version_ = 0x01;
    header.number_ = htons(0x01);
    header.length_ = htonl(data_length);
    header.type_ = htons(api_type);

    return header;
}

void TDriverNavigation::addMoveTask(json command)
{
    //add via node
    json move_task;
    int count = command["count"].get<int>();
    
    for(int i=0; i<count; i++)
    {
        std::string source = command["path_list"][i]["from_node"];
        std::string dest = command["path_list"][i]["to_node"];
        //std::string method = command["path_list"][i]["method"];

        move_task["move_task_list"].push_back(move(source, dest));            
    }

    std::cout << move_task.dump() << std::endl;

    auto str_data = move_task.dump();
    auto send_data = str_data.c_str();
    int data_length = static_cast<int>(strlen(send_data));

    auto header = makeHeader(navi_api_->designated_navigation, data_length);
    
    /*
    auto header = TDriverProtocolHeader(); 
    header.sync_ = 0x5A;
    header.version_ = 0x01;
    header.number_ = htons(0x01);
    header.length_ = htonl(strlen(send_data));
    //header.type_ = htons(0x07D2);
    header.type_ = htons(0x0BFA);
    */

    int data_size = sizeof(header) + strlen(send_data);
    //std::cout << "header : " << sizeof(header) << " send_data : " << strlen(send_data) << std::endl;

    //unsigned char* snd_data = new unsigned char[data_size];
    unsigned char* snd_data = new unsigned char[data_size];

    //std::cout << "data size : " << data_size << std::endl;
    
    memcpy(snd_data, &header, sizeof(header));
    memcpy(snd_data + sizeof(header), send_data, strlen(send_data));

    std::string log_data = "designated_navigation : ";

    for(int i=0; i<data_size; i++)
    {
        int tmp = snd_data[i];
        log_data += std::to_string(tmp);
        log_data +=  " ";
    }

    logWrite(log_data);


    
    client_->write(snd_data, data_size);
    std::cout << "write end" << std::endl;
    delete snd_data;
    
}

json TDriverNavigation::move(std::string source, std::string dest)

{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 99999999);    
    
    auto random_string = (boost::format("%08d") % dis(gen) ).str();
    
    std::string dest_node = "LM" + dest;
    std::string source_node = "LM" + source;

    json j = {
                { "id", dest_node },
                { "source_id", source_node },
                { "task_id", random_string },
                //{ "method", method }
             };

    return j;
}

void TDriverNavigation::turnAmr(double rad, double vw)
{
    json turn_command;

    turn_command["angle"] = rad;
    turn_command["vw"] = vw;
    turn_command["mode"] = 1; // default = 0

    auto str_data = turn_command.dump();
    auto send_data = str_data.c_str();

    int data_length = static_cast<int>(strlen(send_data));
    auto header = makeHeader(navi_api_->rotation_navigation,data_length);
/*
    auto header = TDriverProtocolHeader(); 
    header.sync_ = 0x5A;
    header.version_ = 0x01;
    header.number_ = htons(0x01);
    header.length_ = htonl(strlen(send_data));
    header.type_ = htons(0x0BF0);
*/
    int data_size = sizeof(header) + strlen(send_data);
    std::cout << "header : " << sizeof(header) << " send_data : " << strlen(send_data) << std::endl;

    //unsigned char* snd_data = new unsigned char[data_size];
    unsigned char* snd_data = new unsigned char[data_size];

    std::cout << "data size : " << data_size << std::endl;
    
    memcpy(snd_data, &header, sizeof(header));
    memcpy(snd_data + sizeof(header), send_data, strlen(send_data));

    client_->write(snd_data, data_size);
    std::cout << "write end" << std::endl;

    delete snd_data;
}
	
void TDriverNavigation::pauseAmr()
{
    auto header = makeHeader(navi_api_->pause_navigation, 0);

    int data_size = sizeof(header);    

    unsigned char* snd_data = new unsigned char[data_size];

    
    memcpy(snd_data, &header, sizeof(header));

    client_->write(snd_data, data_size);

    delete snd_data;
}

void TDriverNavigation::stopAmr()
{
    auto header = makeHeader(navi_api_->cancel_navigation, 0);

    int data_size = sizeof(header);    
    
    unsigned char* snd_data = new unsigned char[data_size];

    
    memcpy(snd_data, &header, sizeof(header));

    client_->write(snd_data, data_size);

    delete snd_data;
}

void TDriverNavigation::resumeAmr()
{
    auto header = makeHeader(navi_api_->resume_navigation, 0);

    int data_size = sizeof(header);    
    
    unsigned char* snd_data = new unsigned char[data_size];

    
    memcpy(snd_data, &header, sizeof(header));

    client_->write(snd_data, data_size);    

    delete snd_data;
}

void TDriverNavigation::translationAmr(float dist, float vx, float vy)
{
    // args x,y is based on camera coordinate system. and it should convert to amr coordinate system. 
    // camera x -> amr -y
    // camera y -> amr x
    
    json translation;
    translation["vx"] = vx; // positive foward, negative backward
    translation["vy"] = vy; // positive left, negative right
    translation["dist"] = dist;
    translation["mode"] = 1; // default = 0

    std::cout << translation.dump() << std::endl;

    auto str_data = translation.dump();
    auto send_data = str_data.c_str();
    int data_length = static_cast<int>(strlen(send_data));

    auto header = makeHeader(navi_api_->translation, data_length);

    int data_size = sizeof(header) + strlen(send_data);

    unsigned char* snd_data = new unsigned char[data_size];

    memcpy(snd_data, &header, sizeof(header));
    memcpy(snd_data + sizeof(header), send_data, strlen(send_data));

    client_->write(snd_data, data_size);

    delete snd_data;
}

void TDriverNavigation::logWrite(std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_TDRIVER, LOG_INFO, msg);
}



