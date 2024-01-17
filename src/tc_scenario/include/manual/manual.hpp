#include "rclcpp/rclcpp.hpp"


#include "tc_msgs/msg/amr_open_loop_motion.hpp"
#include "tc_msgs/msg/amr_lift_motion.hpp"


#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tc_msgs/srv/io_header.hpp"
#include "tc_msgs/msg/io_mod_msg.hpp"
#include "tc_msgs/srv/io.hpp"

#include "data_type/acs_data_type.hpp"
#include "data_type/t_driver_data_type.hpp"
#include "data_type/pio_data_type.hpp"
#include "data_type/constants2.h"
#include "data_type/gui_data_type.hpp"

#include "tc_msgs/msg/amr_task.hpp"
#include "helper/amr_logger.hpp"
#include "helper/converter.hpp"
#include "helper/file_io.hpp"

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
using namespace GuiData;


class Manual : public rclcpp::Node
{
	public:
		Manual();
		~Manual();

    private:
    int key_mode_ = -1;
    bool use_jog = false;
    std::shared_ptr<std::vector<int>> ezio_input_;
	std::shared_ptr<std::vector<int>> ezio_output_;

    rclcpp::Node::SharedPtr node_handle_;
    //sub
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gui_msgs_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr t_driver_sub_;	
    rclcpp::Subscription<tc_msgs::msg::IoModMsg>::SharedPtr ezio_sub_;
    
    //pub
    rclcpp::Publisher<tc_msgs::msg::AmrOpenLoopMotion>::SharedPtr amr_jog_pub_;
    rclcpp::Publisher<tc_msgs::msg::AmrLiftMotion>::SharedPtr amr_jack_pub_;
    rclcpp::Publisher<tc_msgs::msg::AmrTask>::SharedPtr task_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pio_node_pub_;
    //Client
    rclcpp::Client<tc_msgs::srv::IoHeader>::SharedPtr ezio_client_;  

    void guiMsgsCallBack(const std_msgs::msg::String::SharedPtr msg);
    int getKeyState();
    void setEzioDigitalOutput(int id, int state);
    void getEzioInputData(const tc_msgs::msg::IoModMsg::SharedPtr msg);
    void logWrite(LogLevel level, std::string msg);
};