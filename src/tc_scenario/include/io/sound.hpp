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


class AmrSound : public rclcpp::Node
{
    public:
        AmrSound();
        ~AmrSound();

    private:        
  		RecursiveMutex mutex_;
        int current_sound = SOUND_NONE;

        rclcpp::Client<tc_msgs::srv::Io>::SharedPtr io_array_client_;   
		rclcpp::Node::SharedPtr service_node_;

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sound_req_sub_;	
        void soundRequestCallBack(const std_msgs::msg::Int32::SharedPtr msg);
		
        void turnOnSound(int sound);
        void turnOffSound();
        std::vector<int> getSoundChannel(int sound);
};