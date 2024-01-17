#include "rclcpp/rclcpp.hpp"

#include "tc_acs_interface/srv/acs_command.hpp"
#include "tc_acs_interface/msg/amr_status.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tc_msgs/srv/aruco_markers.hpp"
#include "tc_msgs/msg/amr_task.hpp"
#include "tc_msgs/msg/amr_init_status.hpp"
#include "tc_msgs/msg/amr_lift_motion.hpp"
#include "tc_msgs/msg/amr_alarm.hpp"

#include "data_type/t_driver_data_type.hpp"
#include "data_type/acs_data_type.hpp"
#include "data_type/pio_data_type.hpp"
#include "data_type/datatype2.h"
#include "data_type/constants2.h"
#include "data_type/gui_data_type.hpp"

#include "scenario/validation.hpp"


#include "helper/amr_logger.hpp"
#include "helper/converter.hpp"
#include "helper/file_io.hpp"
#include "helper/thread_safety_queue.hpp"

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
#include <stack>
#include <cmath>

using namespace std::chrono_literals;
using nlohmann::json;

class Scenario : public rclcpp::Node
{
	public:
		Scenario();
		~Scenario();
    
    private:
        std::shared_ptr<std::map<std::string, int>> task_map_;

        bool quit_ = false;
        bool init_done_ = false;
        bool localization_done_ = false;
        bool have_command_ = false;
        int init_state_ = false;
        bool check_error_ = false;
        bool is_obstacle_sensing_ = false;
        bool send_move_completed = false;
        bool send_continue_report_ = false;
        bool is_pio_alarm_ = false;
        int mode_;
        bool is_lift_up_ = false;
        bool motor_cali_done_ = false;
        int alarm_state_ = 0;
        bool tray_rear_detection_ = false;
        bool check_reset_ = false;

        std::thread motor_init_thread_;

        std::string start_node_1f;
        std::string waiting_node_1f;

        std::string start_node_2f;
        std::string waiting_node_2f;

        int task_result_ = 0;
        int task_feedback_ = 0;
        int task_error_code_ = 0;

        double lift_upper_position_ = -0.075;
        double lift_lower_position_ = -0.14;
        double lift_as_position_ = -0.095;

        std::string cur_job_id_ = "";
        
        json current_command_;
		json amr_status_json_;
		json node_info_;
        json ezio_input_;
        json ezio_output_;

        std::shared_ptr<Validation> validation_;	
	    std::shared_ptr<File_IO> file_io_;
	    std::shared_ptr<ThreadSafeQueue<json>> task_queue_;
        
		rclcpp::Service<tc_acs_interface::srv::AcsCommand>::SharedPtr acs_command_server_;
        void receiveAcsCommand(const std::shared_ptr<tc_acs_interface::srv::AcsCommand::Request> request,
			std::shared_ptr<tc_acs_interface::srv::AcsCommand::Response> response);

        std::shared_ptr<rclcpp::Node> acs_service_node_ ;
		rclcpp::Client<tc_acs_interface::srv::AcsCommand>::SharedPtr cmd_client_;
		rclcpp::Client<tc_msgs::srv::ArucoMarkers>::SharedPtr docking_client_;

		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr t_driver_sub_;	
        void tDriverCallBack(const std_msgs::msg::String::SharedPtr msg);

		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr t_driver_init_sub_;	
        void tDriverInitCallBack(const std_msgs::msg::Int32::SharedPtr msg);
        void setMotorInit();
		
        rclcpp::Subscription<tc_msgs::msg::AmrAlarm>::SharedPtr alarm_sub_;		
		void getAlarmMsg(const tc_msgs::msg::AmrAlarm::SharedPtr msg);
        
        rclcpp::Subscription<tc_msgs::msg::AmrTask>::SharedPtr task_sub_;
		void receivePIOMessage(const tc_msgs::msg::AmrTask::SharedPtr msg);

		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ezio_sub_;
        void getEzioStatus(const std_msgs::msg::String::SharedPtr msg);

		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pio_sub_;
        void getPioNodeStatus(const std_msgs::msg::Bool::SharedPtr msg);

        rclcpp::TimerBase::SharedPtr agv_mode_timer_;
        void checkAgvMode();
		

		rclcpp::TimerBase::SharedPtr amr_pub_timer_;
		void pubAmrStatus();

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acs_report_pub_; 
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acs_task_pub_;        
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sound_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light_pub_;
    	rclcpp::Publisher<tc_msgs::msg::AmrTask>::SharedPtr task_pub_;
    	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_change_pub_;
        rclcpp::Publisher<tc_msgs::msg::AmrInitStatus>::SharedPtr amr_init_status_pub_;
        rclcpp::Publisher<tc_msgs::msg::AmrLiftMotion>::SharedPtr amr_jack_pub_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tcon_alarm_pub_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tcon_init_pub_;


        void initVariables();
        void initTimers();
        void initPubSub();
        void readMapFile();

        void setLight(int light);
        void setSound(int sound);
        void setFailReport(int level, int stop_reason);
        void setContinueReport();
        void pubAlarm(int alarm_code);
        void amrAction(int action);

        json getPathList(json path_list, int count, std::vector<std::string> &via_node);
        void getTaskList(json task_list, int count);
		void doTask(int type, int carrier_type, int result = 0);
        void pubAcsReport(std::string data);

        std::string getTdriverNodeNumber(std::string node);
        std::string getNodeName(int node);

        
        void moveWork(json command);
        void moveWorkCancel(json command);
        void pauseAmr(json command);
        bool amrGetAruco();
        void amrDoTask(json command);
        void amrDoTaskFirst(json command);

        void airshowerOpen(std::string node_name, int task_number);
        void airshowerPass(std::string from_node, std::string to_node, int task_number);
        void airshowerExit(std::string from_node, std::string to_node, int task_number);

        void elevatorOpen(std::string node_name, int task_number);
        void elevatorPass(std::string from_node, std::string to_node, int task_number, std::string target_map);
        void elevatorExit(std::string from_node, std::string to_node, int task_number);

        void acquireOrDeposit(std::string node_name, int task_number, int task_type);
        void charge(int task_number);
        void chargeStop();

        void liftControl(double height, bool stop = false);
        void waitLoading();
        void waitUnLoading();

        void changeMap(std::string from_node, std::string target_map);
        void parseNavigationInquiry(json inquiry);

        void checkKeyState();
        void checkBuzzerStop();
        void checkLiftButton();

        void readConfigFile();

        void test();
        
		void logWrite(LogLevel level, std::string msg);
};