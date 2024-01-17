/// @brief IO service is prgrammed here.
#include "pio.hpp"

// Write to IO as a service
void PIO::updateIoModule(int _id, int _state)
{
    auto report_request = std::make_shared<tc_msgs::srv::IoHeader::Request>();

    report_request->id = _id;
    report_request->state = _state;    
    
    //logSys(TrShareEnum::log_Io,"updateIoModule, ID = %d , State =  %d",id, state );

    static int count_report_server = 0;    
    while(!client_io_update_->wait_for_service(0.1s)) 
    {
        count_report_server++;
        if(count_report_server > 2)
        {
            count_report_server = 0;
            logSys(TrShareEnum::log_Io,"updateIoModule::report server no exist");
            // RCLCPP_INFO(this ->get_logger(), "updateIoModule::report server no exist");
            return;
        }

        // logSys(TrShareEnum::log_Io,"updateIoModule::no report server"); 
        // RCLCPP_INFO(this ->get_logger(), "updateIoModule::no report server");             
        if(!rclcpp::ok())
        return;
    }


    using ReportResponseFuture = rclcpp::Client<tc_msgs::srv::IoHeader>::SharedFuture;
    auto report_response_callback = [this](ReportResponseFuture future)
    {
        auto result = future.get();
        // logSys(TrShareEnum::log_seq, "updateIoModule response: %d",result->ret);
        // RCLCPP_INFO(this ->get_logger(), "updateIoModule answer %ld",result->ret); 
    };

    //server
    auto report_result = client_io_update_->async_send_request(report_request, report_response_callback);  
}

// Write to IO as a service
void PIO::updateIoArray(const std::vector<int> &id, const std::vector<int> &state, int size)
{
    auto request = std::make_shared<tc_msgs::srv::Io::Request>();
    request->size = size;
    request->id.resize(size);
    request->state.resize(size);

    for (int i = 0; i<size; i++)
    {
        request->id[i] = id[i];
        request->state[i] = state[i];
        // logSys(TrShareEnum::log_seq, "id[%d] = %d, state[%d] = %d", i, id[i], i, state[i]);
    }

    static int _count = 0; 
    while(!client_array_io_->wait_for_service(0.1s)) 
    {
        _count++;
        if(_count > 2)
        {
            _count = 0;
            logSys(TrShareEnum::log_Io,"updateIoArray::report server no exist");
            return;
        }         
        if(!rclcpp::ok())
        return;
    }

    using ReportResponseFuture = rclcpp::Client<tc_msgs::srv::Io>::SharedFuture;
    auto response = [this](ReportResponseFuture future)
    {
        auto result = future.get();
        // logSys(TrShareEnum::log_seq, "updateIoArray response: %d",result->received);
    };

    //server
    auto report_result = client_array_io_->async_send_request(request, response);  
}



//************************** LIDAR **************************
// Output
// CDE combination, area
// C(C1,C2) = (Out 01, Out 02)  = (1,0) => 0
//                              = (0,1) => 1
// D(D1,D2) = (Out 03, Out 04)  = (1,0) => 0
//                              = (0,1) => 1
// E(E1,E2) = (Out 05, Out 06)  = (1,0) => 0
//                              = (0,1) => 1
//
// Normal case: CDE = (0,0,0)
// Docking case CDE = (.,.,.)

// Out 07 => Restart device
// Out 08 => Not use


// Input
// In01 : Warning area
// In02 : Not use
// In03 : Lidar error
// In04 : Contamination

/* 230228 Jo 
void PIO::lidarSetting(int id, int C1, int C2, int D1, int D2, int E1, int E2)
{  
    if (id == LIDAR_FRONT)
    {
        updateIoModule(OUT_LIDAR1_OUT01, C1);
        updateIoModule(OUT_LIDAR1_OUT02, C2);
        updateIoModule(OUT_LIDAR1_OUT03, D1);
        updateIoModule(OUT_LIDAR1_OUT04, D2);
        updateIoModule(OUT_LIDAR1_OUT05, E1);
        updateIoModule(OUT_LIDAR1_OUT06, E2);
        logSys(TrShareEnum::log_seq, "[lidarSetting] Front Lidar setting, ID: %d, C1: %d, C2: %d, D1: %d, D2: %d, E1: %d, E2: %d", id, C1, C2, D1, D2, E1, E2);
        return;
    }
    else if (id == LIDAR_REAR)
    {
        updateIoModule(OUT_LIDAR2_OUT01, C1);
        updateIoModule(OUT_LIDAR2_OUT02, C2);
        updateIoModule(OUT_LIDAR2_OUT03, D1);
        updateIoModule(OUT_LIDAR2_OUT04, D2);
        updateIoModule(OUT_LIDAR2_OUT05, E1);
        updateIoModule(OUT_LIDAR2_OUT06, E2);
        logSys(TrShareEnum::log_seq, "[lidarSetting] Rear Lidar setting, ID: %d, C1: %d, C2: %d, D1: %d, D2: %d, E1: %d, E2: %d", id, C1, C2, D1, D2, E1, E2);
        return;
    }
    else
    {
        logSys(TrShareEnum::log_seq, "[lidarSetting] Lidar setting, ID: %d is incorrect (1: front lidar, 2: rear lidar)", id);
    }
}

void PIO::lidarRestart(int id)
{
    if (id == LIDAR_FRONT) 
    {
        updateIoModule(OUT_LIDAR1_OUT07, 1);
        logSys(TrShareEnum::log_seq, "[lidarRestart] Front lidar restarted succesfully");
    }
    else if (id == LIDAR_REAR) 
    {
        updateIoModule(OUT_LIDAR2_OUT07, 1);
        logSys(TrShareEnum::log_seq, "[lidarRestart] Rear lidar restarted succesfully");
    }
    else
    {
        logSys(TrShareEnum::log_seq, "[lidarRestart] Lidar restart, ID: %d is incorrect (1: front lidar, 2: rear lidar)", id);
    }
}
*/