#include "manual/manual.hpp"

Manual::Manual() : Node("manual")
{
    std::cout << "manual node start ! "<<std::endl;

    //publisher
    amr_jog_pub_ = this->create_publisher<tc_msgs::msg::AmrOpenLoopMotion>("amr_jog", 10);
    amr_jack_pub_ = this->create_publisher<tc_msgs::msg::AmrLiftMotion>("amr_jack", 10);
    pio_node_pub_ = this->create_publisher<std_msgs::msg::Bool>("pio_changed", 10);
    task_pub_ = this->create_publisher<tc_msgs::msg::AmrTask>("docking_order", 10);
    //subcription
    ezio_sub_ = this->create_subscription<tc_msgs::msg::IoModMsg>
                    ("InIOStates", 10, std::bind(&Manual::getEzioInputData, this, std::placeholders::_1)); 
    gui_msgs_sub_ = this->create_subscription<std_msgs::msg::String>
    ("gui_msgs", 10, std::bind(&Manual::guiMsgsCallBack, this, std::placeholders::_1));

    // ServiceClients
    ezio_client_ = this->create_client<tc_msgs::srv::IoHeader>("scenario_io_event");

    ezio_input_ = std::make_shared<std::vector<int>>(64, 0);
    ezio_output_ = std::make_shared<std::vector<int>>(64, 0);
    
    RCLCPP_INFO(get_logger(), "manual node ready ! ");
}

Manual::~Manual()
{
    
}

void Manual::guiMsgsCallBack(const std_msgs::msg::String::SharedPtr msg)
{
    std::cout << "guiMsgsCallBack" << std::endl;
    if(getKeyState() == MANUAL_MODE)
    {
        try
        {
            std::cout << "receive from gui : " << msg->data << std::endl;
            std::string log_message = "Recevice msgs from gui : ";
            log_message += msg->data;
            logWrite(LOG_INFO, log_message);
            json j = json::parse(msg->data); // json error {d`~~~~json }0x03

            if(j["DATA_TYPE"] == "61000")
            {
                std::cout << msg->data << std::endl;
                setEzioDigitalOutput(j["index"], j["state"]);
            }
            else if(j["DATA_TYPE"] == "70000")
            {
                auto vx = j["vx"].get<double>();
                auto vy = j["vy"].get<double>();
                auto w = j["w"].get<double>();
                if(vx == 0.0 && vy == 0.0 && w == 0.0)
                {
                    //setAmbientLight(LIGHT_NONE); 
                    use_jog = false;
                }
                else if(use_jog == false)
                {
                    //setAmbientLight(LIGHT_WHOLE);
                    use_jog = true;
                }
                
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
            else if(j["DATA_TYPE"] == "72000")
            {
                auto reset = j["reset"].get<bool>();
                if(reset == true)
                {
                    auto reset_order = tc_msgs::msg::AmrTask();
                    reset_order.order = 13;

                    task_pub_->publish(reset_order);

                    RCLCPP_INFO(get_logger(), "PIO reset ");
                }
            }
            else if(j["DATA_TYPE"] == "73000")
            {
                std::cout << "write AirShower Node config" << std::endl;
                auto file_io = File_IO();
                file_io.writeConifgFile(j);
                std::cout << "start_node_1F : " << j["start_node_1F"].get<int>() << std::endl;
                std::cout << "wait_node_1F : " << j["wait_node_1F"].get<int>() << std::endl;
                std::cout << "start_node_2F : " << j["start_node_2F"].get<int>() << std::endl;
                std::cout << "wait_node_1F : " << j["wait_node_1F"].get<int>() << std::endl;


                auto pio_changed = std_msgs::msg::Bool();
                pio_changed.data = true;

                pio_node_pub_->publish(pio_changed);
            }
            else if(j["DATA_TYPE"] == "64000")
            {
                // std::cout << "write config" << std::endl;
                // auto file_io = File_IO();
                // file_io.writeConifgFile(j);
                
                // std::cout << "write config finish" << std::endl;
                // //pio_retry_max = j["docking_retry"].get<int>();

                // std::cout << "SOC_L : " << j["SOC_L"].get<double>() << std::endl;
                // std::cout << "SOC_H : " << j["SOC_H"].get<double>() << std::endl;
                // std::cout << "vx : " << j["vx"].get<double>() << std::endl;
                // std::cout << "vy : " << j["vy"].get<double>() << std::endl;
                // std::cout << "w : " << j["w"].get<double>() << std::endl;
                //std::cout << "docking_retry : " << j["docking_retry"].get<int>() << std::endl;               
            }

        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}
int Manual::getKeyState()
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
void Manual::setEzioDigitalOutput(int id, int state)
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

void Manual::getEzioInputData(const tc_msgs::msg::IoModMsg::SharedPtr msg)
{
    //std::cout << "ezio topic" << std::endl;
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

void Manual::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_SCENARIO, level, msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto man = std::make_shared<Manual>();

    rclcpp::spin(man);

    rclcpp::shutdown();

    return 0;
}