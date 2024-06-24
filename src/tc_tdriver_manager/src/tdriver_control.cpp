#include "tdriver_control.hpp"

TDriverControl::TDriverControl() : Node("seer_control", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    std::cout << "TDriverControl()" << std::endl;

    receive_queue_ = std::make_shared<ThreadSafeQueue<char*>>();
    connection_thread = std::thread(&TDriverControl::connect, this); // 나중에 seer랑 연결하고 호출
        
    control_api_ = std::make_shared<TDriverControlApi>();
    init();
    std::cout << "TDriverControl() Finished" << std::endl;
}

TDriverControl::~TDriverControl()
{
    quit_ = true;
}

void TDriverControl::connect()
{
    while(!quit_)
    {
        boost::asio::io_context io_context;

        //std::string ip = "10.0.5.91";
        std::string ip = "192.168.192.5";
        std::string port = "19205"; // control
        client_ = new AsyncTcpClient(io_context, ip, port, receive_queue_);
        
        client_->connect();   
    
        io_context.run();
    }    
}

//Functions
void TDriverControl::init()
{    
    jog_sub_ =  this->create_subscription<tc_msgs::msg::AmrOpenLoopMotion>
    ("amr_jog", 10, std::bind(&TDriverControl::openLoopMotionCallBack, this, std::placeholders::_1));

    init_server_ = this->create_service<tc_msgs::srv::AmrInit>("amr_init", 
       std::bind(&TDriverControl::receiveInitMessage, this, std::placeholders::_1, std::placeholders::_2));

    map_change_sub_ = this->create_subscription<std_msgs::msg::String>
    ("map_change", 10, std::bind(&TDriverControl::getMapChangeMessage, this, std::placeholders::_1));

    
}

void TDriverControl::getClientMessage()
{
    if(receive_queue_->count() > 0)
    {
        std::cout << receive_queue_->count() << std::endl;

        parsingClientMessage(receive_queue_->front());
        std::cout << "read" << std::endl;
        receive_queue_->pop();
    }
}

void TDriverControl::parsingClientMessage(char* data)
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

TDriverProtocolHeader TDriverControl::makeHeader(const uint16_t api_type, int data_length)
{
    auto header = TDriverProtocolHeader(); 
    header.sync_ = 0x5A;
    header.version_ = 0x01;
    header.number_ = htons(0x01);
    header.length_ = htonl(data_length);
    header.type_ = htons(api_type);

    return header;
}

void TDriverControl::openLoopMotionCallBack(const tc_msgs::msg::AmrOpenLoopMotion::SharedPtr msg)
{
    //vx 1.0 , vy 1.0 w 1.0 
    json jog_command;

    std::cout << "make json " << std::endl;
    jog_command["vx"] = msg->vx; // default = 0
    jog_command["vy"] = msg->vy; // default = 0
    jog_command["w"] = msg->w; // default = 0

    std::cout << "make json " << std::endl;

    auto str_data = jog_command.dump();
    auto send_data = str_data.c_str();

    std::cout << "make send_data " << std::endl;
    int data_length = static_cast<int>(strlen(send_data));
    std::cout << "get data length :  " << data_length  << std::endl;
    auto header = makeHeader(control_api_->openloop_motion, data_length);
    std::cout << "get header" << std::endl;

    int data_size = sizeof(header) + strlen(send_data);
    std::cout << "header : " << sizeof(header) << " send_data : " << strlen(send_data) << std::endl;

    //unsigned char* snd_data = new unsigned char[data_size];
    unsigned char* snd_data = new unsigned char[data_size];

    std::cout << "data size : " << data_size << std::endl;
    
    memcpy(snd_data, &header, sizeof(header));
    memcpy(snd_data + sizeof(header), send_data, strlen(send_data));
    
    std::string log_data = "openloop_motion : ";
    for(int i=0; i<data_size; i++)
    {        
        int tmp = snd_data[i];
        log_data += std::to_string(tmp);
        log_data +=  " ";
    }

    //logWrite(log_data);

    client_->write(snd_data, data_size);
    std::cout << "write end" << std::endl;

    delete snd_data;
}

void TDriverControl::receiveInitMessage(const std::shared_ptr<tc_msgs::srv::AmrInit::Request> request,
		std::shared_ptr<tc_msgs::srv::AmrInit::Response>	response)
{
    int task = request->data;

    std::cout << "receive init" << std::endl;
    if(task == RELOCATE)
    {
        std::cout << "relocation" << std::endl;
        relocation();
    }
    else if(task == CONFIRM_RELOCATE)
    {
        confirmRelocation();
    }

    response->received = true;
}

void TDriverControl::relocation()
{ 
    auto file_io = File_IO();
    auto pos = file_io.readCurrentPosFile();
    double x = static_cast<double>(pos[1])/1000.0;
    double y = static_cast<double>(pos[2])/1000.0;
    double rad =  pos[3] * (PI / 180.0);

    //vx 1.0 , vy 1.0 w 1.0 
    json relocate;

    relocate["x"] = x; // default = 0
    relocate["y"] = y; // default = 0
    relocate["angle"] = rad; // default = 0

    std::cout << "relocate x : " << x << std::endl;
    std::cout << "relocate y : " << y << std::endl;
    std::cout << "relocate angle : " << pos[3] << std::endl;

    auto str_data = relocate.dump();
    auto send_data = str_data.c_str();

    int data_length = static_cast<int>(strlen(send_data));
    auto header = makeHeader(control_api_->relocation, data_length);

    int data_size = sizeof(header) + strlen(send_data);
 
    //unsigned char* snd_data = new unsigned char[data_size];
    unsigned char* snd_data = new unsigned char[data_size];

   
    memcpy(snd_data, &header, sizeof(header));
    memcpy(snd_data + sizeof(header), send_data, strlen(send_data));

    client_->write(snd_data, data_size);
    
    delete snd_data;
}

void TDriverControl::confirmRelocation()
{
    auto header = makeHeader(control_api_->confirm_relocation, 0);
    
    int data_size = sizeof(header);    
    //std::cout << "data_size : " << data_size << std::endl;
    unsigned char* snd_data = new unsigned char[data_size];

    
    memcpy(snd_data, &header, sizeof(header));

    client_->write(snd_data, data_size);


    delete snd_data;
        
}

void TDriverControl::getMapChangeMessage(const std_msgs::msg::String::SharedPtr msg)
{
    std::string map_name = msg->data;

    json change_map;

    change_map["map_name"] = map_name; // change_map_name

    std::cout << "change_map : " << map_name << std::endl;

    auto str_data = change_map.dump();
    auto send_data = str_data.c_str();

    int data_length = static_cast<int>(strlen(send_data));
    auto header = makeHeader(control_api_->change_map, data_length);

    int data_size = sizeof(header) + strlen(send_data);
 
    //unsigned char* snd_data = new unsigned char[data_size];
    unsigned char* snd_data = new unsigned char[data_size];

   
    memcpy(snd_data, &header, sizeof(header));
    memcpy(snd_data + sizeof(header), send_data, strlen(send_data));

    client_->write(snd_data, data_size);
    
    delete snd_data;
}
	

void TDriverControl::logWrite(std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_TDRIVER, LOG_INFO, msg);
}
