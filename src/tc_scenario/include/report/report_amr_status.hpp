#include "rclcpp/rclcpp.hpp"

#include "data_type/t_driver_data_type.hpp"
#include "data_type/acs_data_type.hpp"

#include "data_type/t_driver_data_type.hpp"
#include "data_type/acs_data_type.hpp"
#include "data_type/pio_data_type.hpp"
#include "data_type/constants2.h"
#include "data_type/gui_data_type.hpp"

#include "helper/amr_logger.hpp"
#include "helper/thread_safety_queue.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tc_acs_interface/msg/amr_status.hpp"
#include "tc_msgs/msg/amr_alarm.hpp"

#include "nlohmann/json.hpp"
#include <boost/asio.hpp>

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <sstream>
#include <typeinfo>
#include <memory>
#include <thread>
#include <chrono>
#include <algorithm>
#include <map>
#include <cmath>

using namespace std::chrono_literals;
using nlohmann::json;

class ReportAmrStatus : public rclcpp::Node
{	
    public:
		ReportAmrStatus();
		~ReportAmrStatus();

    private:
      int send_count =0; 
      void init();

      void setAmrStatus(json amr_status);

      void setGuiStatus(json amr_status);
      void getGuiStatus(json amr_status);

      void getEzioStatus(const std_msgs::msg::String::SharedPtr msg);
      void getAmrStatus(const std_msgs::msg::String::SharedPtr msg);
      void getAcsConnection(const std_msgs::msg::Bool::SharedPtr msg);
      void getAlarmMsg(const tc_msgs::msg::AmrAlarm::SharedPtr msg);

      void pubAcsStatus();
      void pubGuiStatus();

      std::string getNodeName(int node);

      rclcpp::TimerBase::SharedPtr acs_pub_timer_;
      rclcpp::TimerBase::SharedPtr gui_pub_timer;

      rclcpp::Subscription<tc_msgs::msg::AmrAlarm>::SharedPtr alarm_sub_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr acs_conn_sub_;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ezio_sub_;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr amr_status_sub_;

      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acs_report_pub_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gui_report_pub_;

      json gui_report_;
      json acs_report_;

      bool isLoaded = false;

      bool is_reset_done = true;

      ThreadSafeQueue<std::string> gui_send_queue_;
      ThreadSafeQueue<std::string> acs_send_queue_;
};