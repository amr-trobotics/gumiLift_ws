#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "helper/thread_safety_queue.hpp"
#include "data_type/t_driver_data_type.hpp"
#include "nlohmann/json.hpp"

#include <chrono>


using nlohmann::json;
using namespace std::chrono_literals;

class DataAggregator : public rclcpp::Node // Just Subscribe T-Driver Data, Ezio, Docking
{
	public:
		DataAggregator();
		~DataAggregator();

	private:
        void init();
        void tDriverCallBack(const std_msgs::msg::String::SharedPtr msg);

		void getRobotInformationInquiry(json data);
		void getRobotRunningInformationInquiry(json data);
		void getRobotLocationInformationInquiry(json data);
		void getRobotSpeedInquiry(json data);
		void getRobotBlockedStatusInquiry(json data);
		void getRobotBatteryStatusInquiry(json data);
		void getRobotEstopStatusInquiry(json data);
		void getRobotNavigationStatusInquiry(json data);
		void getRobotMapLoadingStatusInquiry(json data);
		void getRobotAlarmStatusInquiry(json data);
		void getRobotMotorStatusInquiry(json data);
		void getRobotLocalizationStatusInquiry(json data);
		void getRobotJackingStatusInquiry(json data);

		void pubTDriverStatus();

    
		ThreadSafeQueue<std::string> t_driver_send_queue_;

		rclcpp::TimerBase::SharedPtr t_driver_timer_;

    	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr t_driver_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr t_driver_sub_;
};