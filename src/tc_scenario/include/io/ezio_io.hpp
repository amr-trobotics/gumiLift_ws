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

#include "std_msgs/msg/string.hpp"

#include "data_type/acs_data_type.hpp"
#include "data_type/constants2.h"
#include "data_type/datatype.h"

#include "helper/recursive_mutex.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using nlohmann::json;


class Ezio_IO : public rclcpp::Node
{
    public:
        Ezio_IO();
        ~Ezio_IO();
        
    private:        
  		RecursiveMutex mutex_;

        rclcpp::TimerBase::SharedPtr ezio_data_pub_timer_;

		rclcpp::Subscription<tc_msgs::msg::IoModMsg>::SharedPtr input_sub_;
		void getEzioInputData(const tc_msgs::msg::IoModMsg::SharedPtr msg);

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;

        std::shared_ptr<std::vector<int>> ezio_input_;
		std::shared_ptr<std::vector<int>> ezio_output_;

        void pubEzioInputOutput();

};