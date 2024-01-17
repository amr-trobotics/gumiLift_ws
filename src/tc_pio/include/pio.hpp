/// @brief tc_pio package is used to send and receive the IO from IO packages

#include <chrono>
#include <memory>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <mutex>
#include <cmath>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/time.h>
#include <functional>
#include <thread>
#include <vector>
#include <sys/utsname.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp> 

#include "tc_msgs/srv/motor.hpp"
#include "tc_msgs/srv/cylinder_mode.hpp"
#include "tc_msgs/srv/amr_task.hpp"
#include "tc_msgs/srv/io_header.hpp"
#include "tc_msgs/srv/io.hpp"
#include "tc_msgs/srv/io_mod_to_man.hpp"
#include "tc_msgs/srv/acs_to_pio.hpp"
#include "tc_msgs/msg/io_mod_msg.hpp"
#include "tc_msgs/msg/amr_task.hpp"
#include "tc_msgs/msg/piohybrid.hpp"
#include "tc_msgs/msg/lift.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "data_type/datatype.h"
#include "data_type/tr_share_enum.hpp"
// #include "data_type/constants.h"
#include "data_type/constants2.h"
// #include "tc_utils/utils.h"
#include "data_type/datatype2.h"
#include "data_type/host_data.h"
#include <stdexcept>


using namespace std::chrono_literals;
using namespace std::placeholders;


#define PI 3.14159265359

class PIO : public rclcpp::Node
{
public:
    PIO();
    ~PIO();

private:
    
    // struct variables
    iomod_info io_state_;                     // IO input data save
    ErrorInfo err_info_;                        // Log error
    DockingProcess docking_process_;           // Docking order, result, feedback from ACS <=> PIO
    
 
    // Sequence variables, need to be reset after called

    int pio_docking_comm_process_seq_ = 0;
    int pio_charging_comm_process_seq_ = 0;
    int pio_charging_off_process_seq_ = 0;

    // timer variables
    int main_timer_seq_ = 0;

    // global variables
    int timeout_ = 0;                         // time out
    int error_code_;
    bool cyl_fwd_limit_state = false;

    int pio_id_;    // PIO ID cmd
    int pio_id_ret; // PIO ID ret
    int lift_request_;
    int lift_ret_;

    //log add
    std::string array_pin_in = "";
    std::string array_pin_out = "";

private:
    //Read Ezio Output
    std::shared_ptr<std::vector<int>> ezio_output_;

    // ros variables
    rclcpp::Client<tc_msgs::srv::IoHeader>::SharedPtr client_io_update_;              // IO service client 1-by-1        
    rclcpp::Client<tc_msgs::srv::Io>::SharedPtr client_array_io_;                     // IO service client array     
    
    rclcpp::Subscription<tc_msgs::msg::IoModMsg>::SharedPtr sub_io_state_;            // IO Subscription
    rclcpp::Subscription<tc_msgs::msg::AmrTask>::SharedPtr docking_order_received_;   // docking order from MainScenario/ACS
    rclcpp::Publisher<tc_msgs::msg::AmrTask>::SharedPtr docking_return_;              // return docking status

    // timer
    rclcpp::TimerBase::SharedPtr timer_; //Io Scan Callback 

    // pub - sub
    rclcpp::Subscription<tc_msgs::msg::Lift>::SharedPtr lift_sub_;  // lift motor sub
    rclcpp::Publisher<tc_msgs::msg::Lift>::SharedPtr lift_pub_; // lift motor pub
    rclcpp::Subscription<tc_msgs::msg::Piohybrid>::SharedPtr sub_pio_id_;  // PIO hybrid ID sub
    rclcpp::Publisher<tc_msgs::msg::Piohybrid>::SharedPtr pub_pio_id_; // PIO hybrid ID pub
private:

    void subReturnPioIDCallback(tc_msgs::msg::Piohybrid::SharedPtr msg);
    void publishingPioID(int nID);

    // Reset
    void pioReset();
    void ioReset();
    void globalVariablesReset();

    
    // timer function
    void mainPioTimerCallback();

    // Callback function
    void ioSubscriptionCallback(const tc_msgs::msg::IoModMsg::SharedPtr msg);
    void dockingOrderReceivedCallback(tc_msgs::msg::AmrTask::SharedPtr msg);
    void updateIoModule(int _id, int _state);
    void updateIoArray(const std::vector<int> &id, const std::vector<int> &state, int size);
    void publishDockingReturn(int _order, int _feedback, int _carrier, int _result, int _error_code);
    void liftCallback(tc_msgs::msg::Lift::SharedPtr msg);
    void liftPublishing(int req);


    // PIO communication and docking
    int pioDockingCommProcess(int task, int carrier_type, int speed);
    void csValue(int lot);
    int taskToPioId(int task); // convert task to PIO id

    // PIO communication and charging
    int pioChargingCommProcess(int task);
    int pioChargingOffProcess(int task);

    // Lidar function
    void lidarSetting(int id, int C1, int C2, int D1, int D2, int E1, int E2);
    void lidarRestart(int id);

    // Log Function
    void loggingCallback();
    void coordLogging(char* strname);
    void tacktimeLogging(char* strname, char* strdata);
    void logErr(int nErr);
    void taskLogging(Tacklist& _tacklist);
    void logSys(int _Type, const char *frm,...);
    std::string logCreateDirectory(string foldername);
    int rmdirs(const char *path, int force);
    void logRemoveFolder(const char *path);

    // Time Function
    unsigned int getTickCount();
    unsigned long elaspseTime(unsigned int dw_start_tick_count);
    bool getTimeOut(unsigned int  dw_start_tick_count, int umillisecound);

};

