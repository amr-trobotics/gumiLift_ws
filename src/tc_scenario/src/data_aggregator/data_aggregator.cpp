#include "data_aggregator/data_aggregator.hpp"

DataAggregator::DataAggregator(): Node("data_aggregator")
{
    init();
    t_driver_timer_ = this->create_wall_timer(20ms, std::bind(&DataAggregator::pubTDriverStatus,this));
    std::cout << "aggregator init done" << std::endl;
}

DataAggregator::~DataAggregator()
{

}

void DataAggregator::init()
{
    //sub
    t_driver_sub_ = this->create_subscription<std_msgs::msg::String>
    ("whole_t_driver_status", 10, std::bind(&DataAggregator::tDriverCallBack, this, std::placeholders::_1));

    //pub
    t_driver_pub = this->create_publisher<std_msgs::msg::String>("t_driver_status", 10);
}


void DataAggregator::tDriverCallBack(const std_msgs::msg::String::SharedPtr msg)
{
    json j = json::parse(msg->data);
    
    std::string str_type = j["DATA_TYPE"];
    int data_type = std::stoi(str_type);

    switch(data_type)
    {
        case ROBOT_INFO_INQUIRY:
            getRobotInformationInquiry(j);
        break;
        case ROBOT_RUNNING_INFO_INQUIRY:
            getRobotRunningInformationInquiry(j);
        break;
        case ROBOT_LOCATION_INQUIRY:
            getRobotLocationInformationInquiry(j);
        break;
        case ROBOT_SPEED_INQUIRY:
            getRobotSpeedInquiry(j);
        break;
        case ROBOT_BLOCKED_INQUIRY:
            getRobotBlockedStatusInquiry(j);
        break;
        case ROBOT_BATTERY_INQUIRY:
            getRobotBatteryStatusInquiry(j);
        break;
        case ROBOT_ESTOP_INQUIRY:
            getRobotEstopStatusInquiry(j);
        break;
        case ROBOT_NAVIGATION_INQUIRY:
            getRobotNavigationStatusInquiry(j);
        break;
        case ROBOT_MAP_LOADING_INQUIRY:
            getRobotMapLoadingStatusInquiry(j);
        break;
        case ROBOT_ALARM_INQUIRY:
            getRobotAlarmStatusInquiry(j);
        break;
        case ROBOT_MOTOR_STATUS_INQUIRY:
            getRobotMotorStatusInquiry(j);
        break;
        case ROBOT_LOCALIZATION_STATUS_INQUIRY:
            getRobotLocalizationStatusInquiry(j);
        break;
        case ROBOT_JACKING_STATUS_INQUIRY:
            getRobotJackingStatusInquiry(j);
        break;
    }
}

void DataAggregator::getRobotLocalizationStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["reloc_status"] = data["reloc_status"];

    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotInformationInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["controller_humi"] = data["controller_humi"];
    j["controller_temp"] = data["controller_temp"];
    j["controller_voltage"] = data["controller_voltage"];
    j["odo"] = data["odo"];
    j["time"] = data["time"];
    j["total_time"] = data["total_time"];

    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotRunningInformationInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["id"] = data["id"];
    j["version"] = data["version"];
    j["map_version"] = data["map_version"];
    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotLocationInformationInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["angle"] = (int)(data["angle"].get<float>() * 180 / PI);
    j["confidence"] = data["confidence"];
    j["x"] = data["x"];
    j["y"] = data["y"];


    std::string cur_station = data["current_station"];
    j["current_station"] = cur_station.erase(0,2);

    std::string last_station = data["last_station"];
    j["last_station"] = last_station.erase(0,2);
    
    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotSpeedInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["is_stop"] = data["is_stop"];
    j["motor_steer_angles"].push_back(1); // Change After
    j["r_spin"] = data["r_spin"];
    j["r_steer"] = data["r_steer"];
    j["r_steer_angles"] = data["r_steer_angles"];
    j["r_vx"] = data["r_vx"];
    j["r_vy"] = data["r_vy"];
    j["r_w"] = data["r_w"];
    j["spin"] = data["spin"];
    j["steer"] = data["steer"];
    j["steer_angles"] = data["steer_angles"];
    j["vx"] = data["vx"];
    j["vy"] = data["vy"];
    j["w"] = data["w"];
    j["acs_comm"] = true;

    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotBlockedStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["block_x"] = data["block_x"];
    j["block_y"] = data["block_y"];
    j["blocked"] = data["blocked"];

    t_driver_send_queue_.push(j.dump());
}
void DataAggregator::getRobotBatteryStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["auto_charge"] = data["auto_charge"];
    j["battery_level"] = data["battery_level"]; // maximum = 1
    j["charging"] = data["charging"];
    j["current"] = data["current"];
    j["manual_charge"] = data["manual_charge"];
    j["voltage"] = data["voltage"];

    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotEstopStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["driver_emc"] = data["driver_emc"];
    j["electric"] = data["electric"];
    j["emergency"] = data["emergency"];
    j["soft_emc"] = data["soft_emc"];

    t_driver_send_queue_.push(j.dump());
}
void DataAggregator::getRobotNavigationStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["task_status"] = data["task_status"];
    j["task_type"] = data["task_type"];
    j["finished_path"] = data["finished_path"];
    j["unfinished_path"] = data["unfinished_path"];
    
    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotMapLoadingStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["loadmap_status"] = data["loadmap_status"];
    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotAlarmStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["fatals"] = data["fatals"];
    j["errors"] = data["errors"];
    j["warnings"] = data["warnings"];
    j["notices"] =  data["notices"];

    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotMotorStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];

    for (const auto& motor : data["motor_info"]) 
    {
        json item;
        item["motor_name"] = motor["motor_name"];
        item["current"] = motor["current"];
        item["voltage"] = motor["voltage"];
        j["motor_info"].push_back(item);
    }

    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::getRobotJackingStatusInquiry(json data)
{
    json j;
    j["DATA_TYPE"] = data["DATA_TYPE"];
    j["jack_state"] = data["jack_state"];
    j["jack_height"] = data["jack_height"];
    j["jack_error_code"] = data["jack_error_code"];
    j["jack_enable"] =  data["jack_enable"];

    t_driver_send_queue_.push(j.dump());
}

void DataAggregator::pubTDriverStatus()
{
 
    if(t_driver_send_queue_.count() > 0)
    { 
        auto data = t_driver_send_queue_.front();

        auto t_driver_msg = std_msgs::msg::String();
        t_driver_msg.data = data;
        
        t_driver_pub->publish(t_driver_msg);

        t_driver_send_queue_.pop();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto data_aggregator = std::make_shared<DataAggregator>();

    rclcpp::spin(data_aggregator);

    rclcpp::shutdown();

    return 0;
}