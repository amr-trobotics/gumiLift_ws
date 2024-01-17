#include "rclcpp/rclcpp.hpp"

#include "helper/thread_safety_queue.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"

#include "data_type/constants2.h"
#include "data_type/t_driver_data_type.hpp"
#include "data_type/acs_data_type.hpp"

#include "nlohmann/json.hpp"

#include "tc_msgs/msg/io_mod_msg.hpp"
#include "tc_msgs/msg/amr_alarm.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include <iostream>
#include <map>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using nlohmann::json;

class AlarmReport : public rclcpp::Node
{
	public:
		AlarmReport();
		~AlarmReport();

    private:
        void init();    
        void pubAmrAlarms();
		
		void checkReset();
		void checkEMS();
		void checkBumper();
		void logWrite(LogLevel level, std::string msg);

		void resetAmr();

		void getEzioData(const std_msgs::msg::String::SharedPtr msg);
		void getTconAlarmData(const std_msgs::msg::Int32::SharedPtr msg);
		void getTDriverAlarmData(const std_msgs::msg::String::SharedPtr msg);

    	rclcpp::Publisher<tc_msgs::msg::AmrAlarm>::SharedPtr amr_alarm_pub_;
    	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_reset_pub_;
		
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr t_driver_sub_;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tcon_alarm_sub_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ezio_input_sub_;
    	rclcpp::TimerBase::SharedPtr alarm_pub_timer;

		
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr scenario_init_sub_;	
        void scenarioInit(const std_msgs::msg::Int32::SharedPtr msg);

		std::shared_ptr<std::vector<int>> ezio_input_;
		std::shared_ptr<std::vector<int>> ezio_output_;

		std::shared_ptr<std::map<int,int>> current_alarm;

 		bool motor_alarm = false;
		bool is_calibration = false;
		bool is_reset_start = false;
		bool is_reset_done = true;
		bool is_all_alarm_clear = true;
		bool is_bumper_detected_ = false;
		bool is_emo_detected_ = false;

		bool quit_ = false;
};