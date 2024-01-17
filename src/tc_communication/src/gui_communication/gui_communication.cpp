#include "gui_communication/gui_communication.hpp"

GuiCommunication::GuiCommunication() : Node("gui_communication")
{
    receive_queue_ = std::make_shared<ThreadSafeQueue<char*>>();
    thread_ = std::thread(&GuiCommunication::connect, this); // 나중에 seer랑 연결하고 호출
    get_client_data_timer = this->create_wall_timer(50ms, std::bind(&GuiCommunication::getClientMessage,this));
    set_client_data_timer = this->create_wall_timer(5ms, std::bind(&GuiCommunication::setClientMessage,this));
    relocation_timer = this->create_wall_timer(1000ms, std::bind(&GuiCommunication::changeMapAndRelocate,this));

    get_gui_status_timer_ = this->create_wall_timer(50ms, std::bind(&GuiCommunication::getGuiStatus, this));
    //pub
    gui_msg_pub_ = this->create_publisher<std_msgs::msg::String>("gui_msgs", 10);
    amr_init_status_pub_ = this->create_publisher<tc_msgs::msg::AmrInitStatus>("amr_init_command", 10);
    map_change_pub_ = this->create_publisher<std_msgs::msg::String>("map_change", 10);
    
    //sub
    t_driver_status_sub = this->create_subscription<std_msgs::msg::String>("t_driver_status", 10, std::bind(&GuiCommunication::getAmrStatus, this, std::placeholders::_1));
    system_status_sub = this->create_subscription<std_msgs::msg::String>("gui_report", 10, std::bind(&GuiCommunication::getSystemStatus, this, std::placeholders::_1));
    t_driver_init_sub_ = this->create_subscription<std_msgs::msg::Int32>
    ("t_driver_init_status", 10, std::bind(&GuiCommunication::tDriverInitCallBack, this, std::placeholders::_1));

    logWrite(LOG_INFO, "Init GuiCommunication Node done");
    
    auto setting = File_IO();
    auto nodes = setting.readNodeFile();
    node_info_ = json::parse(nodes);

    is_init = true;
    RCLCPP_INFO(get_logger(), "GUI node ready ! ");
}

GuiCommunication::~GuiCommunication()
{
    logWrite(LOG_INFO, "~GuiCommunication");

    quit_ = true;

    server_->stopServer();

    thread_.join();

    delete server_;
}

void GuiCommunication::connect()
{    
    while(!quit_)
    {
        boost::asio::io_context io_context;
        short port = 6000;
        server_ = new AsyncTcpServer(io_context,port,receive_queue_);
        logWrite(LOG_INFO, "Make AsyncTcpServer Instance");
        server_->startAccept();
        logWrite(LOG_INFO, "Gui server start listening");
        io_context.run();

        logWrite(LOG_INFO, "Gui server stop listening");
    }
}

void GuiCommunication::getClientMessage()
{
    if(receive_queue_->count() > 0)
    {
        std::cout << receive_queue_->count() << std::endl;

        parsingClientMessage(receive_queue_->front());

        RCLCPP_INFO(get_logger(), "reading! ");

        receive_queue_->pop();
    }
}

void GuiCommunication::setClientMessage()
{    
    if(send_queue_.count() > 0)
    {        
        auto data = send_queue_.front();

        char* amr_data = const_cast<char*>(data.c_str());

        server_->sendData(amr_data);
        send_queue_.pop();
    } 
}

void GuiCommunication::parsingClientMessage(char* data)
{
    std::string str(data);

    str.erase(std::remove(str.begin(), str.end(), '\x02'), str.end());
    str.erase(std::remove(str.begin(), str.end(), '\x03'), str.end());
    
    std::size_t pos = str.find('}');
    
    std::string split_part = str.substr(0, pos+1);
    
    std::cout << split_part << std::endl;

    logWrite(LOG_INFO, "[Receive] " + str);
    logWrite(LOG_INFO, "[Receive] Split str : " + split_part);
    
    json j = json::parse(split_part);

    if(j["DATA_TYPE"] == "63000") // Confirm Relocate
    {
        if(init_status_ == INIT_RUNNING || init_status_ == INIT_NONE)
        {
            return;
        }
        else
        {
            auto confirm = j["confirm"].get<bool>();
        
            if(confirm == false)
            {
                auto node = j["node"].get<int>();
                auto angle = j["angle"].get<int>();
                auto map = j["map"].get<std::string>();


                std::string node_str = std::to_string(node);
                std::cout << "node : " << node << std::endl;
                std::cout << "angle : " << angle << std::endl;
                auto x = node_info_[getNodeName(node)]["x"].get<int>();
                auto y = node_info_[getNodeName(node)]["y"].get<int>();
                std::cout << "x : " << x << std::endl;
                std::cout << "y : " << y << std::endl;

                auto file = File_IO();
                file.writeCurrentPosFile(node, x, y, angle);   

                seqnm = 100;
                relocate_map = map;        
                start_relocate = true;            
            }
        }   
    }
    
    auto gui_msg = std_msgs::msg::String();
    gui_msg.data = split_part;

    gui_msg_pub_->publish(gui_msg);
} 

void GuiCommunication::getAmrStatus(const std_msgs::msg::String::SharedPtr msg)
{
    std::string json_data = msg->data;
    std::string data= "\x02" + json_data + "\x03";

    if(is_gui_conn == true)
    {
        send_queue_.push(data);  

    }
        
    json j = json::parse(json_data);
    
    std::string type_string = j["DATA_TYPE"].get<std::string>();
    auto data_type = std::stoi(type_string);

    switch(data_type)
    {
        case ROBOT_LOCALIZATION_STATUS_INQUIRY:
            robot_localization_state = j["reloc_status"].get<int>();
        break;
        case ROBOT_LOCATION_INQUIRY:
        {
            agv_x_ = j["x"].get<double>();
            agv_y_ = j["y"].get<double>();
            agv_angle_ = j["angle"].get<int>();
            current_node_ = j["current_station"];

            if(current_node_ == "")
            {
                std::string last_station = j["last_station"];
                if(last_station == "")
                {
                    auto file_io = File_IO();
                    auto pos = file_io.readCurrentPosFile();
                    current_node_ = std::to_string(pos[0]);
                }
                else
                {
                    current_node_ = last_station;
                }
            }
        }
        break;
    }
}

void GuiCommunication::getSystemStatus(const std_msgs::msg::String::SharedPtr msg) //amr report
{
    std::string json_data = msg->data;

    std::string data= "\x02" + json_data + "\x03";

    if(is_gui_conn == true)
        send_queue_.push(data);  
    
}

void GuiCommunication::getGuiStatus()
{
    if(server_ != NULL)
    {
        is_gui_conn = server_->isConnected();

        if(is_gui_conn && is_init)
        {
            is_init = false;
            auto file = File_IO();
            std::string config_str = file.readConfigFile();
            
            json parse_json = json::parse(config_str);

            json config;
            config["DATA_TYPE"] = "94000";

            for(const auto& item : parse_json.items())
            {
                std::string key_string = item.key();
                std::string value_string;
                if(key_string == "SOC_L" || key_string == "SOC_H" || key_string == "vx" ||
                    key_string == "vy" || key_string == "w") //double
                {
                    auto config_value = std::stod(item.value().get<std::string>());
                    config[key_string] = config_value;
                }
                else
                {
                    continue;
                }
            }

            std::cout << "send config" << std::endl;

            std::string json_data = config.dump();
            std::string data = "\x02" + json_data + "\x03";

            send_queue_.push(data);

            std::this_thread::sleep_for(100ms); // Do not remove
        }
        else if (!is_gui_conn)
        {
            is_init = true;
        }
    }
}

void GuiCommunication::tDriverInitCallBack(const std_msgs::msg::Int32::SharedPtr msg)
{
    init_status_ = msg->data;
    std::cout << "init status : " << init_status_ << std::endl;
    
    if(init_status_ == INIT_DONE || init_status_ == INIT_FAIL)
    {        
        if(is_send_relocate == false)
        {
            //Send Relocate to GUI
            is_send_relocate = true;
    
            auto file_io = File_IO();
            auto pos = file_io.readCurrentPosFile();
            
            int node = pos[0];
            int angle =  pos[3];
            //vx 1.0 , vy 1.0 w 1.0 
    
            json j;
            j["DATA_TYPE"] = "91000";
            j["node"] = node;
            j["angle"] = angle;

            std::string json_data = j.dump();
            std::string data = "\x02" + json_data + "\x03";
            
            send_queue_.push(data);            
        }
    }
    else if(init_status_ == INIT_RUNNING)
    {
        is_send_relocate = false;
    }
}
void GuiCommunication::changeMapAndRelocate()
{
    switch (seqnm)
    {
        case 100:
            if (start_relocate)
            {
                seqnm = 200;
                logWrite(LOG_INFO, "Start Map changed and Relocate");
                sendToPad(CHANGE_MAP_START);
                count = 0;
            }
            break;
        case 200:
            {
                std::string floor = current_node_.substr(1,1);
                std::string map_name = "";
                if(floor == "1")
                {
                    map_name = "1F";
                }
                else
                {
                    map_name = "2F";
                }
                logWrite(LOG_INFO, "current map -> " + map_name + " / change map -> " + relocate_map);

                if(map_name == relocate_map)
                {                
                    sendToPad(CHANGE_MAP_SKIP);
                    logWrite(LOG_INFO, "Already same map -> skip map change");
                    seqnm = 600;
                }
                else if (relocate_map == "1F" || relocate_map == "2F")
                {
                    sendToPad(CHANGE_MAP_RUNNING);
                    auto msg = std_msgs::msg::String();
                    msg.data = relocate_map;
                    map_change_pub_->publish(msg);

                    logWrite(LOG_INFO, "change map -> " + relocate_map);
                    logWrite(LOG_INFO, "wait change map complete");
                    
                    seqnm = 300;
                }
                else
                {
                    seqnm = 600;
                }
            }
            break;
        case 300:   
            if (count < 40 && robot_localization_state != LOCALIZATION_COMPLETED)
            {
                std::this_thread::sleep_for(1000ms);
                count++;
                seqnm = 300;
            }
            else
            {
                sendToPad(CHANGE_MAP_RUNNING);
                logWrite(LOG_INFO, "change map complete");
                count = 0;
                seqnm = 400;
            }
            break;
        case 400:
            if(robot_localization_state == LOCALIZATION_COMPLETED)
            {
                auto msg2 = tc_msgs::msg::AmrInitStatus();
                msg2.init_status = CHANGE_MAP;
                amr_init_status_pub_->publish(msg2);
                logWrite(LOG_INFO, "change map complete -> send confirm");
                logWrite(LOG_INFO, "wait change map success");

                seqnm = 500;
            }
            else
            {
                count = 0;
                logWrite(LOG_ERR, "change map complete fail");
                seqnm = 600;
            }
            break;
        case 500:
            if (count < 40 && robot_localization_state != LOCALIZATION_SUCCESS)
            {
                std::this_thread::sleep_for(1000ms);
                count++;

                seqnm = 500;
            }
            else
            {
                sendToPad(CHANGE_MAP_FINISH);
                logWrite(LOG_INFO, "change map success");
                seqnm = 600;
                count = 0;
            }
            break;
        case 600:
            {
                if(count > 10)
                {                
                    auto msg3 = tc_msgs::msg::AmrInitStatus();
                    msg3.init_status = DO_INIT;
                    amr_init_status_pub_->publish(msg3);
                    count = 0;   
                    relocate_map = "";
                    logWrite(LOG_INFO, "send relocate" );

                    seqnm = 700;
                }
                else
                {
                    sendToPad(RELOCATE_RUNNING);
                    count++;
                }

            }
            break;
        case 700: // Wait Relocate Success
            {
                if(init_status_ == INIT_RUNNING)
                {
                    sendToPad(RELOCATE_RUNNING);
                    seqnm = 800;
                    logWrite(LOG_INFO, "loading map finished." );
                    count = 0;
                }
            }
            break;
        case 800: // Wait Relocate Success
            {
                if(init_status_ == INIT_DONE && count > 20)
                {
                    seqnm = 900;
                    logWrite(LOG_INFO, "relocate complete");
                    count = 0;
                }
                else
                {
                    sendToPad(RELOCATE_COMPLETE);
                    count++;
                }
            }
            break;
        case 900: // Wait Relocate Success
            {
                if(count < 20)
                {
                    count++;
                    sendToPad(RELOCATE_FINISH);
                    logWrite(LOG_INFO, "relocate success");
                }
                else
                {
                    count = 0;
                    start_relocate = false;
                    seqnm = 100;
                }
            }
            break;
    }
}

void GuiCommunication::sendToPad(int state)
{    
    json j;
    j["DATA_TYPE"] = "99000";
    j["reloc_status"] = state;

    std::string json_data = j.dump();
    std::string data = "\x02" + json_data + "\x03";
    
    send_queue_.push(data);            
}

std::string GuiCommunication::getNodeName(int node)
{
    std::string node_name;
    if(node == 0)
    {
        return "";
    }

    if(node > 4000)
    {
        node -= 4000;
        node_name = "C" + std::to_string(node);
    }
    else if(node > 3000)
    {
        node -= 3000;
        node_name = "E" + std::to_string(node);
    }
    else if(node > 2000)
    {
        node -= 2000;
        node_name = "A" + std::to_string(node);
    }
    else if(node > 1000)
    {
        node -= 1000;
        node_name = "D" + std::to_string(node);
    }
    else
    {
        node_name = "W" + std::to_string(node);
    }
    
    return node_name;
}
void GuiCommunication::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_GUI, level, msg);
}