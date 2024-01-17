#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <vector>
#include <utility>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "tc_msgs/srv/io_header.hpp"
#include "tc_msgs/msg/io_mod_msg.hpp"
#include "tc_msgs/srv/io.hpp"

#include "std_msgs/msg/int32.hpp"

#include "data_type/acs_data_type.hpp"
#include "data_type/constants2.h"
#include "data_type/datatype.h"

#include "helper/recursive_mutex.hpp"

using namespace std::chrono_literals;


class AmrLight : public rclcpp::Node
{
    public:
        AmrLight();
        ~AmrLight();

    private:        
  		RecursiveMutex mutex_;

        int current_light = LIGHT_NONE;

        rclcpp::Client<tc_msgs::srv::Io>::SharedPtr io_array_client_;   
		rclcpp::Node::SharedPtr service_node_;

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light_req_sub_;	
        void lightRequestCallBack(const std_msgs::msg::Int32::SharedPtr msg);
		
        void turnOnLight(int light);
        void turnOffLight();
        
		std::vector<int> getLightChannel(int light);
};