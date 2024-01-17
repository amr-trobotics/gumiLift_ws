#include "hypio_serial_publisher.hpp"

#define PI 3.14159265359

HyPioSerialPublisher::HyPioSerialPublisher()
    : Node("hybrid_pio")
{
    
    RCLCPP_INFO(get_logger(), "Hybrid PIO init starts");
    device_ = -1;
    write_trials_ = 1;
    max_wirte_trials_ = 4;
    error_code_ = 0;

    returnHexData();
    initSerial();

    statusCommandSet();

    timer_main_scan_ = this->create_wall_timer(
        50ms, std::bind(&HyPioSerialPublisher::mainTimerCallback, this));

    sub_pio_id_ = this->create_subscription<tc_msgs::msg::Piohybrid>("pio_id_cmd", 10, std::bind(&HyPioSerialPublisher::subReturnPioIDCallback, this, _1));

    pub_pio_id_ = this->create_publisher<tc_msgs::msg::Piohybrid>("/pio_id_ret", rclcpp::QoS(rclcpp::KeepLast(10)));

    RCLCPP_INFO(get_logger(), "Hybrid PIO init finished");
}

HyPioSerialPublisher::~HyPioSerialPublisher()
{
    close_serial(device_);
}

void HyPioSerialPublisher::mainTimerCallback()
{
    if (pio_id_ != 0 && write_count_ < write_trials_)
    {
        wirtePioID(pio_id_);
        write_count_ += 1;
        dataRead();
    }
}

void HyPioSerialPublisher::subReturnPioIDCallback(tc_msgs::msg::Piohybrid::SharedPtr msg)
{
    
    RCLCPP_INFO(get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    RCLCPP_INFO(get_logger(), "subscription PIO ID: %d ", msg->id);
    logWrite(LOG_INFO, "subscription PIO ID: " + std::to_string(msg->id));
    // rclcpp::WallRate loop_rate(100ms);
    write_count_ = 0;
    write_trials_ = 1;
    pio_id_ = msg->id;
}

void HyPioSerialPublisher::publishingPioID(int nID, int nReturn)
{
    auto outMessage = std::make_unique<tc_msgs::msg::Piohybrid>();
    outMessage->id = nID;
    outMessage->ret = nReturn;
    pub_pio_id_->publish(std::move(outMessage));

    RCLCPP_INFO(get_logger(), "publishing PIO ID: %d, return: %d", nID, nReturn);
    logWrite(LOG_INFO, "publishing PIO ID: " + std::to_string(nID) + "return: " + std::to_string(nReturn));
}

void HyPioSerialPublisher::returnHexData()
{
    // 10 (DC)
    m_stHexData[10].c_strID[0] = 0x31;
    m_stHexData[10].c_strID[1] = 0x30;
    m_stHexData[10].c_strSum[0] = 0x44; // D
    m_stHexData[10].c_strSum[1] = 0x43; // C
    // 11 (DD)
    m_stHexData[11].c_strID[0] = 0x31;
    m_stHexData[11].c_strID[1] = 0x31;
    m_stHexData[11].c_strSum[0] = 0x44; // D
    m_stHexData[11].c_strSum[1] = 0x44; // D
    // 12 (DE)
    m_stHexData[12].c_strID[0] = 0x31;
    m_stHexData[12].c_strID[1] = 0x32;
    m_stHexData[12].c_strSum[0] = 0x44; // D
    m_stHexData[12].c_strSum[1] = 0x45; // E
    // 13 (DF)
    m_stHexData[13].c_strID[0] = 0x31;
    m_stHexData[13].c_strID[1] = 0x33;
    m_stHexData[13].c_strSum[0] = 0x44; // D
    m_stHexData[13].c_strSum[1] = 0x46; // F
    // 14 (DF)
    m_stHexData[14].c_strID[0] = 0x31;
    m_stHexData[14].c_strID[1] = 0x34;
    m_stHexData[14].c_strSum[0] = 0x45; // E
    m_stHexData[14].c_strSum[1] = 0x30; // 0
    // 15
    m_stHexData[15].c_strID[0] = 0x31;
    m_stHexData[15].c_strID[1] = 0x35;
    m_stHexData[15].c_strSum[0] = 0x45; // E
    m_stHexData[15].c_strSum[1] = 0x31; // 1
    // 40
    m_stHexData[40].c_strID[0] = 0x34;
    m_stHexData[40].c_strID[1] = 0x30;
    m_stHexData[40].c_strSum[0] = 0x44; // D
    m_stHexData[40].c_strSum[1] = 0x46; // F
    // 41
    m_stHexData[41].c_strID[0] = 0x34;
    m_stHexData[41].c_strID[1] = 0x31;
    m_stHexData[41].c_strSum[0] = 0x45; // E
    m_stHexData[41].c_strSum[1] = 0x30; // 0
    // 42
    m_stHexData[42].c_strID[0] = 0x34;
    m_stHexData[42].c_strID[1] = 0x32;
    m_stHexData[42].c_strSum[0] = 0x45; // E
    m_stHexData[42].c_strSum[1] = 0x31; // 1
    // 43
    m_stHexData[43].c_strID[0] = 0x34;
    m_stHexData[43].c_strID[1] = 0x33;
    m_stHexData[43].c_strSum[0] = 0x45; // E
    m_stHexData[43].c_strSum[1] = 0x32; // 2
    // 60
    m_stHexData[60].c_strID[0] = 0x36;
    m_stHexData[60].c_strID[1] = 0x30;
    m_stHexData[60].c_strSum[0] = 0x45; // E
    m_stHexData[60].c_strSum[1] = 0x31; // 1
    // 80
    m_stHexData[80].c_strID[0] = 0x38;
    m_stHexData[80].c_strID[1] = 0x30;
    m_stHexData[80].c_strSum[0] = 0x45; // E
    m_stHexData[80].c_strSum[1] = 0x33; // 3
    // 81
    m_stHexData[81].c_strID[0] = 0x38;
    m_stHexData[81].c_strID[1] = 0x31;
    m_stHexData[81].c_strSum[0] = 0x45; // E
    m_stHexData[81].c_strSum[1] = 0x34; // 4
}


// Serial communication
void HyPioSerialPublisher::initSerial()
{
    rclcpp::WallRate loop_rate(100ms);
    rclcpp::WallRate loop_check_rate(20ms);
    int loop_count;

    while (loop_count != 10)
    {
        loop_count++;
        loop_rate.sleep();
    }

    while (device_ == -1 && rclcpp::ok())
    {
        // device_ = open_serial((char*)"/dev/ttyS5", 38400, 0, 0);
        device_ = open_serial((char *)"/dev/ttyUSB0", 38400, 0, 0);

        RCLCPP_INFO(this->get_logger(), "Hybrid_Pio serial communication: '%d'", device_);
        logWrite(LOG_INFO, "Hybrid_Pio serial communicatin:" + std::to_string(device_));

        if (device_ != -1)
        {
            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Hybrid_Pio serial communication succeeded");
            logWrite(LOG_INFO, "Hybrid_Pio serial communicatin succeeded");
        }
        else
        {
            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Hybrid_Pio serial communication failed");
            logWrite(LOG_INFO, "Hybrid_Pio serial communication failed");
        }
    }
}

/*Check Sum*/
unsigned char HyPioSerialPublisher::checksum(unsigned char *data, int length)
{
    unsigned char check_send = 0;
    int i = 0;
    for (i = 1; i < length; i++)
    {
        check_send += data[i];
    }

    return check_send;
}
/**/
void HyPioSerialPublisher::sendDataReading()
{
    if (device_ != -1)
    {
        memset(string_data_, 0, sizeof(string_data_));

        string_data_[0] = 0x3C; //<
        string_data_[1] = 0x42; // B
        string_data_[2] = 0x43; // C
        string_data_[3] = 0x38; // 8
        string_data_[4] = 0x35; // 5
        string_data_[5] = 0x3E; //>
        string_data_[6] = checksum(string_data_, 6);

        write(device_, string_data_, 7);
    }

    RCLCPP_INFO(this->get_logger(), "Hybrid_Pio: '%d'", device_);
    logWrite(LOG_INFO, "Hybrid_Pio:" + std::to_string(device_));
}

// ID sets
// 10 ==> Docking 1
// 11 ==> Docking 2
// 12 ==> Docking 3
// 13 ==> Docking 4
// 40 ==> Airshower 1-1
// 41 ==> Airshower 1-2
// 42 ==> Airshower 2-1
// 43 ==> Airshower 2-2
// 60 ==> charge
// 80 ==> Elv 1
// 81 ==> Elv 2
void HyPioSerialPublisher::wirtePioID(int nID)
{
    unsigned char c_strsumData;
    // if(device_ != -1)
    {
        // memset(string_data_,0, sizeof(string_data_));
        string_data_[0] = 0x3C; //<
        string_data_[1] = 0x42; // B
        string_data_[2] = 0x43; // C
        string_data_[3] = 0x3D; // =
        // M
        string_data_[4] = 0x31; // 1, [2G]
        // AAAAAA
        string_data_[5] = 0x3A; // :
        string_data_[6] = 0x30;
        string_data_[7] = 0x30;
        string_data_[8] = 0x30;
        string_data_[9] = 0x30;
        string_data_[10] = m_stHexData[nID].c_strID[0];
        string_data_[11] = m_stHexData[nID].c_strID[1];
        // CCC
        string_data_[12] = 0x3A; //:
        string_data_[13] = 0x30; // C
        string_data_[14] = 0x30;
        string_data_[15] = 0x30;
        // N
        string_data_[16] = 0x3A; // :
        string_data_[17] = 0x30;
        // 000000
        string_data_[18] = 0x3A;
        string_data_[19] = 0x30;
        string_data_[20] = 0x30;
        string_data_[21] = 0x30;
        string_data_[22] = 0x30;
        string_data_[23] = 0x30;
        string_data_[24] = 0x30;

        c_strsumData = checksum(string_data_, 25);
        string_data_[25] = m_stHexData[nID].c_strSum[0];
        string_data_[26] = m_stHexData[nID].c_strSum[1];
        unsigned int chbyte1 = static_cast<int>(c_strsumData); // checksum check
        // RCLCPP_INFO(get_logger(), "hybrid ID = %d", nID);
        logWrite(LOG_INFO, "hybrid ID: " + std::to_string(nID));

        // RCLCPP_INFO(get_logger(), "check Sum = :: %d", chbyte1);

        string_data_[27] = 0x3E;
        write(device_, string_data_, 28);
    }
}

void HyPioSerialPublisher::dataRead()
{
    // check connection
    if (device_ == -1)
    {
        return;
    }

    // serial data read
    int nRead_Data = 0;

    uint8_t data_read[255];
    memset(data_read, 0, sizeof(data_read));

    nRead_Data = read(device_, data_read, sizeof(uint8_t) * 255);
    // RCLCPP_INFO(get_logger(), "device_ = %d nRead_Data= %d ", device_, nRead_Data);
    if (data_read == NULL)
    {
        publishingPioID(pio_id_, RETURN_ERR);
        RCLCPP_INFO(get_logger(), "publishingPioID NG");
        logWrite(LOG_INFO, "publishingPioID NG");
        return;
    }

    // for (int nI = 0; nI < 40; nI++)
    // {
    //     // unsigned int chbyte2 = static_cast<uint8_t>(data_read[nI]); // Data feedback from PIO device
    //     RCLCPP_INFO(get_logger(), "data_feedback[%d]= %d", nI, chbyte2);
    // }

    status_[1] = data_read[1];
    status_[2] = data_read[2];

    unsigned int chbyte1 = static_cast<char>(string_data_[10]);
    unsigned int chbyte2 = static_cast<char>(string_data_[11]);
    unsigned int chbyte11 = static_cast<uint8_t>(data_read[15]);
    unsigned int chbyte22 = static_cast<uint8_t>(data_read[16]);
    // RCLCPP_INFO(get_logger(), "chbyte1= %d", chbyte1);
    // RCLCPP_INFO(get_logger(), "chbyte2= %d", chbyte2);
    // RCLCPP_INFO(get_logger(), "chbyte11= %d", chbyte11);
    // RCLCPP_INFO(get_logger(), "chbyte22= %d", chbyte22);

    logWrite(LOG_INFO, "chbyte1 == chbyte11, chbyte2 == chbyte22 (true), chbyte1:" + std::to_string(chbyte1) + "chbyte11:" + std::to_string(chbyte11) + 
                        "chbyte2:" + std::to_string(chbyte2) + "chbyte22:" + std::to_string(chbyte22));


    if (chbyte1 == chbyte11 && chbyte2 == chbyte22)
    {
        publishingPioID(pio_id_, RETURN_SUCCESS);
        RCLCPP_INFO(get_logger(), "publishingPioID: SUCCESS");
        logWrite(LOG_INFO,"publishingPioID: SUCCESS");
        write_trials_ = 1; // if succeeded reset the trials
    }
    else
    {
        write_trials_ += 1;
    }
    
    if (write_trials_ > max_wirte_trials_)
    {
        publishingPioID(pio_id_, RETURN_ERR);
        RCLCPP_INFO(get_logger(), "publishingPioID: ERROR");
        error_code_ = ERR_PIO_SET_ID_FAILED;
        
        logWrite(LOG_INFO,"publishingPioID: ERROR");
        write_trials_ = 0;
    }

}

void HyPioSerialPublisher::statusCommandSet()
{
    sendDataReading();
    rclcpp::WallRate loop_rate(100ms);
    loop_rate.sleep();
    dataRead();
    if (status_[1] == 0x42 && status_[2] == 0x43) // BC status
    {
        RCLCPP_INFO(get_logger(), "Status command set succeeded");
        logWrite(LOG_INFO,"Status command set succeeded");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Status command set failed");
        logWrite(LOG_INFO,"Status command set failed");
    }

}

void HyPioSerialPublisher::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_HPIO, level, msg);
}