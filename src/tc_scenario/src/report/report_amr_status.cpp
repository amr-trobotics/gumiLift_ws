#include "report/report_amr_status.hpp"

ReportAmrStatus::ReportAmrStatus() : Node("report_amr_status")
{
    std::cout << "init report" << std::endl;
    init();
    acs_pub_timer_ = this->create_wall_timer(10ms, std::bind(&ReportAmrStatus::pubAcsStatus,this));
    gui_pub_timer = this->create_wall_timer(10ms, std::bind(&ReportAmrStatus::pubGuiStatus,this));
}

ReportAmrStatus::~ReportAmrStatus()
{

}

void ReportAmrStatus::init()
{
    gui_report_["alarm_code"] = 0;
    
    ezio_sub_ = this->create_subscription<std_msgs::msg::String>
    ("ezio_data", 10, std::bind(&ReportAmrStatus::getEzioStatus, this, std::placeholders::_1));

    amr_status_sub_ = this->create_subscription<std_msgs::msg::String>
    ("acs_report", 10, std::bind(&ReportAmrStatus::getAmrStatus, this, std::placeholders::_1));

    acs_conn_sub_ = this->create_subscription<std_msgs::msg::Bool>
    ("acs_connection", 10, std::bind(&ReportAmrStatus::getAcsConnection, this, std::placeholders::_1));
    
    alarm_sub_ = this->create_subscription<tc_msgs::msg::AmrAlarm>
    ("tcon_alarms", 10, std::bind(&ReportAmrStatus::getAlarmMsg, this, std::placeholders::_1));

    acs_report_pub_ = this->create_publisher<std_msgs::msg::String>("amr_report", 10);
    gui_report_pub_ = this->create_publisher<std_msgs::msg::String>("gui_report", 10);
}

void ReportAmrStatus::getAlarmMsg(const tc_msgs::msg::AmrAlarm::SharedPtr msg)
{    

    if(msg->state == ERROR_NONE) // Not Error
    {
        gui_report_["alarm_code"] = 0;

        acs_report_["amr_alrams1"] = 0;
        acs_report_["amr_alrams2"] = 0;
        acs_report_["amr_alrams3"] = 0;
        is_reset_done = true;
    }
    else if(msg->state == IN_RESET_SEQ)
    {
        is_reset_done = false;
    }
    else if(msg->state == RESET_DONE)
    {
        gui_report_["alarm_code"] = 0;

        acs_report_["amr_alrams1"] = 0;
        acs_report_["amr_alrams2"] = 0;
        acs_report_["amr_alrams3"] = 0;
        is_reset_done = true;
    }
    else //ERROR
    {
        if(msg->code[0] != 3301)
        {
            gui_report_["alarm_code"] = msg->code[0];

            acs_report_["amr_alrams1"] = msg->code[0];
            acs_report_["amr_alrams2"] = msg->code[1];
            acs_report_["amr_alrams3"] = msg->code[2];
        }
    }
}

void ReportAmrStatus::getAcsConnection(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool acs_conn = msg->data; 
    gui_report_["acs_comm"] = acs_conn;
}

void ReportAmrStatus::getEzioStatus(const std_msgs::msg::String::SharedPtr msg)
{
    std::string ezio_msg = msg->data;

    json j = json::parse(msg->data);
    auto di = j["DI"];

    auto ezdo = j["DO"];

    gui_send_queue_.push(ezio_msg);
}

void ReportAmrStatus::getAmrStatus(const std_msgs::msg::String::SharedPtr msg)
{
    json amr_status = json::parse(msg->data);
    
    setAmrStatus(amr_status);
    setGuiStatus(amr_status);
    
}

void ReportAmrStatus::setGuiStatus(json amr_status)
{
    gui_report_["DATA_TYPE"] = "90000";    

    double jack_height = amr_status["jack_height"];
    
    if(jack_height > 0.03)
    {
        gui_report_["loaded"] = "LOADED";
    }
    else
    {
        gui_report_["loaded"] = "EMPTY";
    }
    getGuiStatus(amr_status); //string ?

    float bat_level = amr_status["battery_soc"].get<float>();
    gui_report_["battery_level"] = bat_level/100.0;

    int mode = amr_status["mode"].get<int>();
    if(mode == MANUAL_MODE)
    {
        amr_status["state"] = MANUAL;
    }

    int state = amr_status["state"].get<int>();
    std::string vehicle_state = "";
    switch(state)
    {
        case SHUTDOWN:
            vehicle_state = "SHUTDOWN";
        break;
        case INIT:
            vehicle_state = "INIT";
        break;
        case DISCONNECTED:
            vehicle_state = "DISCONNECTED";
        break;
        case MANUAL:
            vehicle_state = "MANUAL";
        break;
        case ABORTING:
            vehicle_state = "ABORTING";
        break;
        case NOT_ASSIGNED:
            vehicle_state = "NOT_ASSIGNED";
        break;
        case MOVE:
            vehicle_state = "MOVE";
        break;
        case PAUSED:
            vehicle_state = "PAUSED";
        break;
        case ARRIVAL:
            vehicle_state = "ARRIVAL";
        break;
        case DOCKING:
            vehicle_state = "DOCKING";
        break;
        case UNDOCKING:
            vehicle_state = "UNDOCKING";
        break;
        case ACQUIRING:
            vehicle_state = "ACQUIRING";
        break;
        case DEPOSITING:
            vehicle_state = "DEPOSITING";
        break;
        case MOVEPORT:
            vehicle_state = "MOVEPORT";
        break;
        case CHARGING:
            vehicle_state = "CHARGING";
        break;
        case OPENDOOR:
            vehicle_state = "OPENDOOR";
        break;
        case SCANNING:
            vehicle_state = "SCANNING";
        break;
        case STOP_CHARGING:
            vehicle_state = "STOP_CHARGING";
        break;
        case AMR_ALARM:
            vehicle_state = "AMR_ALARM";
        break;
    }

    if(is_reset_done == false)
    {
        vehicle_state = "RESET";        
    }
    gui_report_["status"] = vehicle_state;
    gui_send_queue_.push(gui_report_.dump());
    // if(send_count % 10 == 0)
    // {
    //     gui_send_queue_.push(gui_report_.dump());
    //     send_count = 1;
    // }
    // else
    // {
    //     send_count++;
    // }
 
}

void ReportAmrStatus::getGuiStatus(json amr_status)
{

}

void ReportAmrStatus::pubGuiStatus()
{
    if(gui_send_queue_.count() > 0)
    {
        auto msg = std_msgs::msg::String();

        auto data = gui_send_queue_.front();
        msg.data = data; 
/*
        json cmd = json::parse(data);
        std::string datatype = cmd["DATA_TYPE"];

        if(datatype == "90000")
            std::cout << data << std::endl;
*/
        gui_report_pub_->publish(msg);
        gui_send_queue_.pop();
    }
}

void ReportAmrStatus::setAmrStatus(json amr_status)
{
    /*
        amr_status_json_["x"] = 0.0;
        amr_status_json_["y"] = 0.0;
        amr_status_json_["angle"] = 0;
        amr_status_json_["node"] = 0;
        amr_status_json_["next_node"] = 0;
        amr_status_json_["vx"] = 0.0;
        amr_status_json_["vy"] = 0.0;
        amr_status_json_["battery_soc"] = 0.0;
        amr_status_json_["battery_volt"] = 0;
        amr_status_json_["unfinished_stations"] = 0;
        amr_status_json_["state"] = INIT;
    */


    acs_report_["node"] = getNodeName(amr_status["node"].get<int>());
    acs_report_["next_node"] = getNodeName(amr_status["next_node"].get<int>());

    std::string cur_node = acs_report_["node"];

    if(cur_node != "")
    {
        std::string floor = cur_node.substr(1,1);
        if(floor == "1")
        {
            acs_report_["map_name"] = "1F";
        }
        else
        {
            acs_report_["map_name"] = "2F";
        }
    }

    acs_report_["bat_volt"] = amr_status["battery_volt"].get<int>();
    acs_report_["bat_soc"] = amr_status["battery_soc"].get<float>();

    acs_report_["x"] = amr_status["x"];
    acs_report_["y"] = amr_status["y"];

    double vx = amr_status["vx"].get<double>();
    double vy = amr_status["vy"].get<double>();

    int state = amr_status["state"].get<int>();

    if(state == INIT)
    {
        amr_status["state"] = MANUAL;
    }

    double velocity = std::sqrt(vx*vx + vy*vy);    
    acs_report_["velocity"] = velocity * 1000;

    
    int mode = amr_status["mode"].get<int>();
    if(mode == MANUAL_MODE)
    {
        amr_status["state"] = MANUAL;
    }
    
    acs_report_["state"] = amr_status["state"];
    acs_report_["lift_height"] = amr_status["jack_height"];

    int angle = amr_status["angle"].get<int>();
    if( angle < 0)
    {
        angle += 360;
    }

    acs_report_["angle"] = angle;

    acs_send_queue_.push(acs_report_.dump());   
}

void ReportAmrStatus::pubAcsStatus()
{
    if(acs_send_queue_.count() > 0)
    {
        auto msg = std_msgs::msg::String();

        auto data = acs_send_queue_.front();
        msg.data = data;

        acs_report_pub_->publish(msg);
        acs_send_queue_.pop(); 
    }
}

std::string ReportAmrStatus::getNodeName(int node)
{
    std::string node_name;
    if(node == 0)
    {
        return "";
    }

    if(node > 5000)
    {
        node -= 5000;
        node_name = "D" + std::to_string(node);
    }
    else if(node > 4000)
    {
        node -= 4000;
        node_name = "C" + std::to_string(node);
    }
    else if(node > 3000)
    {
        node -= 3000;
        node_name = "E" + std::to_string(node);
    }
    else if(node > 2000)
    {
        node -= 2000;
        node_name = "A" + std::to_string(node);
    }
    else if(node > 1000)
    {
        node -= 1000;
        node_name = "D" + std::to_string(node);
    }
    else
    {
        node_name = "W" + std::to_string(node);
    }
    
    return node_name;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto report = std::make_shared<ReportAmrStatus>();

    rclcpp::spin(report);

    rclcpp::shutdown();

    return 0;
}