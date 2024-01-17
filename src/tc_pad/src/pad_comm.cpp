#include "pad_comm.hpp"

PadComm::PadComm() : Node("pad_comm")
{
    receive_queue_ = std::make_shared<ThreadSafeQueue<char *>>();
    thread_ = std::thread(&PadComm::connect, this); // 나중에 seer랑 연결하고 호출
    get_client_data_timer = this->create_wall_timer(50ms, std::bind(&PadComm::getClientMessage, this));
    set_client_data_timer = this->create_wall_timer(15ms, std::bind(&PadComm::setClientMessage, this));

    set_pad_status_timer_ = this->create_wall_timer(100ms, std::bind(&PadComm::setPadStatus, this));
    //srv
    ezio_client_ = this->create_client<tc_msgs::srv::IoHeader>("scenario_io_event");
    // sub
    ezio_sub_ = this->create_subscription<tc_msgs::msg::IoModMsg>("InIOStates", 10, std::bind(&PadComm::getEzioInputData, this, std::placeholders::_1));
    //pub
    amr_jog_pub_ = this->create_publisher<tc_msgs::msg::AmrOpenLoopMotion>("amr_jog", 10);
    amr_jack_pub_ = this->create_publisher<tc_msgs::msg::AmrLiftMotion>("amr_jack", 10);
    //logWrite(LOG_INFO, "Init PadComm Node done");

    auto setting = File_IO();
    auto nodes = setting.readNodeFile();
    node_info_ = json::parse(nodes);

    ezio_input_ = std::make_shared<std::vector<int>>(64, 0);
    ezio_output_ = std::make_shared<std::vector<int>>(64, 0);

    std::cout << "Pad Communication start" << std::endl;
}

PadComm::~PadComm()
{
    //logWrite(LOG_INFO, "~PadComm");

    quit_ = true;

    server_->stopServer();

    thread_.join();

    delete server_;
}

void PadComm::connect()
{
    while (!quit_)
    {
        boost::asio::io_context io_context;
        short port = 6000;
        server_ = new AsyncTcpServer(io_context, port, receive_queue_);
        logWrite(LOG_INFO, "Make AsyncTcpServer Instance");
        server_->startAccept();
        logWrite(LOG_INFO, "Gui server start listening");
        io_context.run();

        logWrite(LOG_INFO, "Gui server stop listening");
    }
}

void PadComm::getClientMessage()
{
    if (receive_queue_->count() > 0)
    {
        std::cout << receive_queue_->count() << std::endl;

        parsingClientMessage(receive_queue_->front());
        std::cout << "read" << std::endl;
        receive_queue_->pop();
    }
}

void PadComm::setClientMessage()
{
    if (send_queue_.count() > 0)
    {
        auto data = send_queue_.front();

        char *amr_data = const_cast<char *>(data.c_str());

        get_system_count++;
        //       logWrite(LOG_INFO, "[Send] " + data);

        if (get_system_count % 50 == 0) // 1 log for 2s
        {
            get_system_count = 0;
            // logWrite(LOG_INFO, "[Send] " + data);
        }

        server_->sendData(amr_data);
        send_queue_.pop();
    }
}

void PadComm::parsingClientMessage(char *data)
{
    std::string str(data);

    str.erase(std::remove(str.begin(), str.end(), '\x02'), str.end());
    str.erase(std::remove(str.begin(), str.end(), '\x03'), str.end());

    std::size_t pos = str.find('}');

    std::string split_part = str.substr(0, pos + 1);

    std::cout << split_part << std::endl;

    logWrite(LOG_INFO, "[Receive] " + str);
    logWrite(LOG_INFO, "[Receive] Split str : " + split_part);

    json j = json::parse(split_part);

    padMsgsReceive(split_part);
}

void PadComm::parsingToSendingMsg(std::string json_data)
{

    std::string data= "\x02" + json_data + "\x03";

    send_queue_.push(data);  
}
void PadComm::ezioSendToPad()
{   
    //std::cout << "ezioSendToPad" << std::endl;
    std::string ezio_status_string;
    json ezio;
    ezio["DATA_TYPE"] = "60000";

    for(int i=0; i<64; i++)
    {
        ezio["DI"].emplace_back(ezio_input_->at(i));
        ezio["DO"].emplace_back(ezio_output_->at(i));
    }

    ezio_status_string = ezio.dump();       
    parsingToSendingMsg(ezio_status_string);
    
}
void PadComm::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_GUI, level, msg);
}

void PadComm::setPadStatus()
{
    if (server_ != NULL)
    {
        is_gui_conn = server_->isConnected();
        if(is_gui_conn == true)
        {
            ezioSendToPad();
        }
        
    }
}
void PadComm::padMsgsReceive(std::string msg_recv)
{
    std::cout << "padMsgsCallBack" << std::endl;
    if(getKeyState() == AUTO_MODE)
    {
        std::string log_message = "Recevice msgs from gui when amr auto : ";
        log_message += msg_recv;
        logWrite(LOG_WARNING, log_message);
        std::cout << log_message << std::endl;
        return;
    }
    else
    {
        try
        {
            std::cout << "receive from gui : " <<msg_recv << std::endl;
            std::string log_message = "Recevice msgs from gui : ";
            log_message += msg_recv;
            logWrite(LOG_INFO, log_message);
            json j = json::parse(msg_recv); // json error {d`~~~~json }0x03

            if(j["DATA_TYPE"] == "61000")
            {
                std::cout << msg_recv << std::endl;
                setEzioDigitalOutput(j["index"], j["state"]);
            }
            else if(j["DATA_TYPE"] == "70000")
            {
                auto vx = j["vx"].get<double>();
                auto vy = j["vy"].get<double>();
                auto w = j["w"].get<double>();
              
                auto msg = tc_msgs::msg::AmrOpenLoopMotion();
                msg.vx = vx;
                msg.vy = vy;
                msg.w = w;

                amr_jog_pub_->publish(msg);
            }
            else if(j["DATA_TYPE"] == "71000")
            {
                std::cout << "jacking pub....." << std::endl;
                
                auto height = j["height"].get<double>();
                auto stop = j["stop"].get<bool>();

                auto msg = tc_msgs::msg::AmrLiftMotion();
                msg.height = height;
                msg.stop = stop;

                amr_jack_pub_->publish(msg);
            }

        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}

int PadComm::getKeyState()
{    
    //Key State Check
    if(ezio_input_->at(IN_AUTO_KEY) == 1 && ezio_input_->at(IN_MANUAL_KEY) == 0)
    {
        key_mode_ = AUTO_MODE;
    }
    else if(ezio_input_->at(IN_MANUAL_KEY) == 1 && ezio_input_->at(IN_AUTO_KEY) == 0)
    {
        key_mode_ = MANUAL_MODE;
    }

    return key_mode_;
}
void PadComm::setEzioDigitalOutput(int id, int state)
{    
    try
    {
        while (!ezio_client_->wait_for_service(1s))
        {
            std::cout << "wait for service" << std::endl;

            if (!rclcpp::ok())
            {
                std::cout << "Service Client cannot found server" << std::endl;
                // Something Error
                return;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }

    auto request = std::make_shared<tc_msgs::srv::IoHeader::Request>();
    request->id = id;
    request->state = state;
    
    auto result = ezio_client_->async_send_request(request);
    std::cout << "Wait for server response" << std::endl;
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "server result : " << result.get()->ret << std::endl;

        std::this_thread::sleep_for(10ms); // Do not remove
    }
}

void PadComm::getEzioInputData(const tc_msgs::msg::IoModMsg::SharedPtr msg)
{
    try
    {
        int m_nCreateIoNo = 0;

        // 1 block of 1 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword01 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword09 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }
        // 2 block of 1 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword02 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword10 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }

        // 3 block of 1 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword03 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword11 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }
        // 4 block of 1 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword04 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword12 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }
        // 1 block of 3 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword05 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword13 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }
        // 1 block of 4 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword06 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword14 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }
        // 1 block of 5 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword07 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword15 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }
        //////////////////////////////////////////
        // 1 block of 6 Module
        for (int nI = 0; nI < 8; nI++)
        {
            if ((msg->wdword08 >> nI & 1) == 1)
                ezio_input_->at(m_nCreateIoNo) = 1;
            else
                ezio_input_->at(m_nCreateIoNo) = 0;

            if ((msg->wdword16 >> nI & 1) == 1)
                ezio_output_->at(m_nCreateIoNo) = 1;
            else
                ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }

        // 다른 다이얼로에 전달 데이터 생성.
        //  DigIO_Send(kIo);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}