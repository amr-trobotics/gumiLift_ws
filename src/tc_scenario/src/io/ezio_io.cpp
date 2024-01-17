#include "io/ezio_io.hpp"

Ezio_IO::Ezio_IO() : Node("ezio_scenario")
{
    input_sub_ = this->create_subscription<tc_msgs::msg::IoModMsg>
                    ("InIOStates", 10, std::bind(&Ezio_IO::getEzioInputData, this, std::placeholders::_1)); 
    
    ezio_input_ =  std::make_shared<std::vector<int>>(64, 0);
    ezio_output_ =  std::make_shared<std::vector<int>>(64, 0);

    system_status_pub_ = this->create_publisher<std_msgs::msg::String>("ezio_data", 10);
    
    ezio_data_pub_timer_ = this->create_wall_timer(100ms, std::bind(&Ezio_IO::pubEzioInputOutput,this));
    
}

Ezio_IO::~Ezio_IO()
{

}

void Ezio_IO::getEzioInputData(const tc_msgs::msg::IoModMsg::SharedPtr msg)
{
    try
    {          
        int m_nCreateIoNo = 0; 

        //1 block of 1 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword01 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0; 

            if((msg->wdword09 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1; 
        }
        //2 block of 1 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword02 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0; 

            if((msg->wdword10 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;
			
            m_nCreateIoNo += 1;
        }

        //3 block of 1 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword03 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0;         

            if((msg->wdword11 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;
        
            m_nCreateIoNo += 1;
        }
        //4 block of 1 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword04 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0; 
            
            if((msg->wdword12 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;
			
            m_nCreateIoNo += 1;
        }
        //1 block of 3 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword05 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0; 

            if((msg->wdword13 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1;
        }
        //1 block of 4 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword06 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0; 

            if((msg->wdword14 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;
			
            m_nCreateIoNo += 1;
        }
        //1 block of 5 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword07 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0; 

            if((msg->wdword15 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1; 
        } 
        //////////////////////////////////////////
        //1 block of 6 Module
        for(int nI = 0; nI < 8 ; nI++)
        {
            if((msg->wdword08 >> nI & 1) == 1) ezio_input_->at(m_nCreateIoNo) = 1;
            else  ezio_input_->at(m_nCreateIoNo) = 0; 

            if((msg->wdword16 >> nI & 1) == 1) ezio_output_->at(m_nCreateIoNo) = 1;
            else  ezio_output_->at(m_nCreateIoNo) = 0;

            m_nCreateIoNo += 1; 
        }

        //다른 다이얼로에 전달 데이터 생성.
        // DigIO_Send(kIo);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }     
}

void Ezio_IO::pubEzioInputOutput()
{
    auto system_status_msg = std_msgs::msg::String(); 
    
    json ezio;
    ezio["DATA_TYPE"] = "60000";

    for(int i=0; i<64; i++)
    {
        ezio["DI"].emplace_back(ezio_input_->at(i));
        ezio["DO"].emplace_back(ezio_output_->at(i));
    }

    system_status_msg.data = ezio.dump();            
    system_status_pub_->publish(system_status_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto ezio = std::make_shared<Ezio_IO>();

    rclcpp::spin(ezio);

    rclcpp::shutdown();

    return 0;
}