#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "serial.h"
#include "tc_msgs/msg/piohybrid.hpp"
#include "helper/amr_logger.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

//Return data for result;
#define RETURN_NONE 0
#define RETURN_SUCCESS 1
#define RETURN_ERR 2
#define RETURN_ABORT 3
#define RETURN_CANCEL 4

#define ERR_PIO_SET_ID_FAILED 620412

struct stHexData
{
   char c_strID[2];
   char c_strSum[2];
};


class HyPioSerialPublisher : public rclcpp::Node
{
public:
   HyPioSerialPublisher();
   ~HyPioSerialPublisher();

   struct stHexData m_stHexData[100]; 

   int write_count_;
   uint8_t status_[255];
   int write_trials_;
   int max_wirte_trials_;

public:
   int device_;
   int pio_id_;
   int error_code_;
   unsigned char string_data_[255];

public:
   void initSerial();
   unsigned char checksum(unsigned char *data, int length);   

   void sendDataReading();
   void mainTimerCallback();
   void dataRead();
   void wirtePioID(int nID); //Master set ID
   void returnHexData();
   void subReturnPioIDCallback(const tc_msgs::msg::Piohybrid::SharedPtr msg);
   void publishingPioID(int nID, int nReturn);
   void statusCommandSet(); // to check "BC" status at initialization
   
   void logWrite(LogLevel level, std::string msg);
public:
   rclcpp::TimerBase::SharedPtr timer_main_scan_; 
   rclcpp::Subscription<tc_msgs::msg::Piohybrid>::SharedPtr sub_pio_id_; 
   rclcpp::Publisher<tc_msgs::msg::Piohybrid>::SharedPtr pub_pio_id_;
};
