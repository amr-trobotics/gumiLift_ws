#include "pio.hpp"

PIO::PIO()
    : Node("tc_pio_node")
{
    // Node init
    RCLCPP_INFO(this->get_logger(), "tc_pio_node init start");

    
    ezio_output_ =  std::make_shared<std::vector<int>>(64, 0);
    
    sub_pio_id_ = this->create_subscription<tc_msgs::msg::Piohybrid>("pio_id_ret", 10, std::bind(&PIO::subReturnPioIDCallback, this, _1));
    pub_pio_id_ = this->create_publisher<tc_msgs::msg::Piohybrid>("pio_id_cmd", rclcpp::QoS(rclcpp::KeepLast(10)));
    lift_sub_ = this->create_subscription<tc_msgs::msg::Lift>("lift_ret", 10, std::bind(&PIO::liftCallback, this, _1));
    lift_pub_ = this->create_publisher<tc_msgs::msg::Lift>("lift_cmd", rclcpp::QoS(rclcpp::KeepLast(10)));

    sub_io_state_ = this->create_subscription<tc_msgs::msg::IoModMsg>("InIOStates", 10, std::bind(&PIO::ioSubscriptionCallback, this, _1));

    docking_order_received_ = this->create_subscription<tc_msgs::msg::AmrTask>("docking_order", 10, std::bind(&PIO::dockingOrderReceivedCallback, this, _1));
    docking_return_ = this->create_publisher<tc_msgs::msg::AmrTask>("docking_return", rclcpp::QoS(rclcpp::KeepLast(10)));
    client_io_update_ = this->create_client<tc_msgs::srv::IoHeader>("scenario_io_event");
    client_array_io_ = this->create_client<tc_msgs::srv::Io>("io_array_event");

    // Timer
    timer_ = this->create_wall_timer(150ms, std::bind(&PIO::mainPioTimerCallback, this));

    // rclcpp::WallRate loop_rate(1000ms);
    // loop_rate.sleep();
    error_code_ = NO_ERROR;
    
    pioReset();

    // reset PIO
    updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
    rclcpp::WallRate loop_rate(500ms);
    loop_rate.sleep();
    publishingPioID(PIO_RESET); // set PIO ID
    loop_rate.sleep();
    updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
       

    RCLCPP_INFO(this->get_logger(), "tc_pio_node init finihsed");
}
PIO::~PIO()
{
    // destructor
}

void PIO::mainPioTimerCallback()
{
    // Error case
    if (error_code_ != NO_ERROR)
    {
        publishDockingReturn(docking_process_.order, docking_process_.order, docking_process_.carrier, RETURN_ERR, error_code_);
        logSys(TrShareEnum::log_seq, "[mainPioTimerCallback] Task::%d = RETURN_ERR", docking_process_.order);
        logSys(TrShareEnum::log_err, "[mainPioTimerCallback] Task::%d = RETURN_ERR", docking_process_.order);
        error_code_ = NO_ERROR;
        globalVariablesReset();
        pioReset();
        // // reset PIO ID
        updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
        rclcpp::WallRate loop_rate(500ms);
        loop_rate.sleep();
        publishingPioID(PIO_RESET); // set PIO ID
        loop_rate.sleep();
        updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
        logSys(TrShareEnum::log_seq, "Reset PIO ID to Reset ID = %d", PIO_RESET);

        logSys(TrShareEnum::log_seq, "End task...");
    }

    // Task Docking, Airshower, Elevator
    else if (docking_process_.order == Task::Docking1 || docking_process_.order == Task::Docking2 ||
             docking_process_.order == Task::Docking3 || docking_process_.order == Task::Docking4 ||
             docking_process_.order == Task::Airshower11 || docking_process_.order == Task::Airshower12 ||
             docking_process_.order == Task::Airshower21 || docking_process_.order == Task::Airshower22 ||
             docking_process_.order == Task::Elevator1F || docking_process_.order == Task::Elevator2F
             )
    {
        if (pioDockingCommProcess(docking_process_.order, docking_process_.carrier, LOW_SPEED) == RETURN_SUCCESS)
            {
                pioReset();
                logSys(TrShareEnum::log_seq, "[mainPioTimerCallback] Task::%d = RETURN_SUCCESS", docking_process_.order);
                //publishDockingReturn(docking_process_.order, docking_process_.order, docking_process_.carrier, RETURN_SUCCESS, NO_ERROR);
                publishDockingReturn(docking_process_.order, 0, docking_process_.carrier, RETURN_SUCCESS, NO_ERROR);
                globalVariablesReset();
                // // reset PIO ID
                updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
                rclcpp::WallRate loop_rate(500ms);
                loop_rate.sleep();
                publishingPioID(PIO_RESET); // set PIO ID
                loop_rate.sleep();
                updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
                logSys(TrShareEnum::log_seq, "Reset PIO ID to Reset ID = %d", PIO_RESET);

                logSys(TrShareEnum::log_seq, "End task...");
            }
    }

    // Task Charging PIO communication is running
    else if (docking_process_.order == Task::Charging)
    {
        if (pioChargingCommProcess(docking_process_.order) == RETURN_SUCCESS)
        {
            publishDockingReturn(docking_process_.order, docking_process_.order, docking_process_.carrier, RETURN_SUCCESS, NO_ERROR);
            logSys(TrShareEnum::log_seq, "[mainPioTimerCallback] Task::Charging::%d= RETURN_SUCCESS", docking_process_.order);
            globalVariablesReset();
            pioReset();
            // // reset PIO ID
            updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
            rclcpp::WallRate loop_rate(500ms);
            loop_rate.sleep();
            publishingPioID(PIO_RESET); // set PIO ID
            loop_rate.sleep();
            updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
            logSys(TrShareEnum::log_seq, "Reset PIO ID to Reset ID = %d", PIO_RESET);

            logSys(TrShareEnum::log_seq, "End task...");
        }
    }

    // Task Charging OFF (or Charging done) PIO communication is running
    else if (docking_process_.order == Task::ChargingOff)
    {
        if (pioChargingOffProcess(docking_process_.order) == RETURN_SUCCESS)
        {
            publishDockingReturn(docking_process_.order, docking_process_.feedback, docking_process_.carrier, RETURN_SUCCESS, NO_ERROR);
            logSys(TrShareEnum::log_seq, "[mainPioTimerCallback] Task::ChargingOff::%d = RETURN_SUCCESS", docking_process_.order);
            globalVariablesReset();
            pioReset();
            // // reset PIO ID
            updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
            rclcpp::WallRate loop_rate(500ms);
            loop_rate.sleep();
            publishingPioID(PIO_RESET); // set PIO ID
            loop_rate.sleep();
            updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
            logSys(TrShareEnum::log_seq, "Reset PIO ID to Reset ID = %d", PIO_RESET);

            logSys(TrShareEnum::log_seq, "End task...");
        }
    }
    else if(docking_process_.order == RESET_HPIO) // reset HPIO
    {
        logSys(TrShareEnum::log_seq, "[mainPioTimerCallback] Task::Reset HPIO");
        updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
        rclcpp::WallRate loop_rate(500ms);
        loop_rate.sleep();
        publishingPioID(PIO_RESET); // set PIO ID
        loop_rate.sleep();
        updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT

        globalVariablesReset();
        pioReset();
        logSys(TrShareEnum::log_seq, "[RESET_HPIO] Reset PIO succedded");
        logSys(TrShareEnum::log_seq, "End task...");
    }
    else if(docking_process_.order == RESET_PIO) // reset PIO
    {
        globalVariablesReset();
        pioReset();
        logSys(TrShareEnum::log_seq, "[mainPioTimerCallback] Task::Reset PIO");
        logSys(TrShareEnum::log_seq, "[RESET_PIO] Reset PIO succedded");
        logSys(TrShareEnum::log_seq, "End task...");
    }

}


