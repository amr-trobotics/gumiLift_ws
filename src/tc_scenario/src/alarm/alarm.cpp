#include "alarm/alarm.hpp"

AlarmReport::AlarmReport() : Node("amr_alarm_node")
{
    scenario_init_sub_ = this->create_subscription<std_msgs::msg::Int32>
    ("amr_scenario_init", 10, std::bind(&AlarmReport::scenarioInit, this, std::placeholders::_1));
}

AlarmReport::~AlarmReport()
{
    quit_ = true;
} 

void AlarmReport::scenarioInit(const std_msgs::msg::Int32::SharedPtr msg)
{
    int scenario_init_done = msg->data;
    logWrite(LOG_INFO, "Scenario init done -> " + std::to_string(scenario_init_done));
    
    if(scenario_init_done == SCENARIO_INIT_DONE)
    {
        init();
    }
}


void AlarmReport::init()   
{   
    ezio_input_ = std::make_shared<std::vector<int>>(64, 0);
    ezio_output_ = std::make_shared<std::vector<int>>(64, 0);
    auto file_io = File_IO();
    current_alarm = std::make_shared<std::map<int, int>>(file_io.getAlarmCode());
    
    std::cout << "read alarm file" << std::endl;
    t_driver_sub_ = this->create_subscription<std_msgs::msg::String>
    ("t_driver_status", 10, std::bind(&AlarmReport::getTDriverAlarmData, this, std::placeholders::_1));

    tcon_alarm_sub_ = this->create_subscription<std_msgs::msg::Int32>
    ("amr_alarms", 10, std::bind(&AlarmReport::getTconAlarmData, this, std::placeholders::_1));

    ezio_input_sub_ = this->create_subscription<std_msgs::msg::String>
                    ("ezio_data", 10, std::bind(&AlarmReport::getEzioData, this, std::placeholders::_1)); 
    
    amr_alarm_pub_ = this->create_publisher<tc_msgs::msg::AmrAlarm>("tcon_alarms", 10);

    alarm_reset_pub_ = this->create_publisher<std_msgs::msg::String>("tdriver_error_reset", 10);

    alarm_pub_timer = this->create_wall_timer(100ms, std::bind(&AlarmReport::pubAmrAlarms,this));
}

void AlarmReport::pubAmrAlarms()
{
    std::vector<int> alarms;

    auto msg = tc_msgs::msg::AmrAlarm();
    msg.state = ERROR_NONE;
    msg.code.resize(3);

    for(auto const& [key,value] : *current_alarm)
    {
        if(value == 1)
        {
            msg.state = ERROR_OCCUR;
            if(key < 3200) //fatals
            {
                alarms.push_back(key);
            }
            else if(key < 3300) //errors
            {
                alarms.push_back(key);                
            }
            else if(10000 < key)
            {
                alarms.push_back(key);                 
            }
            else //warning
            {
                if(key == 3301)
                {
                    if(alarms.size() < 3)
                    {
                        alarms.push_back(key);
                    }
                }
            }

            logWrite(LOG_INFO, "Alarm Occur -> " + std::to_string(key));
        }
    }
    
    std::sort(alarms.begin(), alarms.end());
    
    while(alarms.size() < 3)
    {
        alarms.push_back(0);
    }


    for(int i=0; i<alarms.size(); i++)
    {
        msg.code[i] = alarms[i];
    }

    if(is_reset_start == true && is_reset_done == false) // in reset seq
    {
        msg.state = IN_RESET_SEQ;
    }
    else if(is_reset_start == true && is_reset_done == true)
    {
        if(is_all_alarm_clear == true)
        {
            msg.state = RESET_DONE;               
            is_emo_detected_ = false;          
            is_bumper_detected_ = false;
        }     
        else
            msg.state == RESET_FAIL;
    }

    amr_alarm_pub_->publish(msg);
    
}

void AlarmReport::checkReset()
{
    if(ezio_input_->at(IN_SAFETY_PLC_RESET) == 1 && is_reset_start == false)
    {
        is_reset_start = true;
        is_reset_done = false;
        std::thread t(&AlarmReport::resetAmr, this);
        std::this_thread::sleep_for(10ms); // Do not remove
        t.detach();
    }
}

void AlarmReport::checkEMS()
{
    if(ezio_input_->at(IN_EMS_BTN_1) == 1)
    {
        logWrite(LOG_INFO, "EMS 1 On");
        (*current_alarm)[3201] = 1;
        is_emo_detected_ = true;
    }
    else if(ezio_input_->at(IN_EMS_BTN_2) == 1)
    {
        logWrite(LOG_INFO, "EMS 2 On");
        (*current_alarm)[3201] = 1;
        is_emo_detected_ = true;
    }
    else if(ezio_input_->at(IN_EMS_BTN_3) == 1)
    {
        logWrite(LOG_INFO, "EMS 3 On");
        (*current_alarm)[3201] = 1;
        is_emo_detected_ = true;
    }
    else if(ezio_input_->at(IN_EMS_BTN_4) == 1)
    {
        logWrite(LOG_INFO, "EMS 4 On");
        (*current_alarm)[3201] = 1;
        is_emo_detected_ = true;
    }
}

void AlarmReport::checkBumper()
{
    if(ezio_input_->at(IN_BUMPER_DETECTION_1) == 0)
    {
        logWrite(LOG_INFO, "Bumper 1 Detected On");
        is_bumper_detected_ = true;
        //(*current_alarm)[3202] = 1;
    }
    if(ezio_input_->at(IN_BUMPER_DETECTION_2) == 1)
    {
        logWrite(LOG_INFO, "Bumper 2 Detected On");
        is_bumper_detected_ = true;
        //(*current_alarm)[3202] = 1;
    }
}

void AlarmReport::resetAmr()
{
    logWrite(LOG_INFO, "Reset Start");
    std::cout << "Reset start" << std::endl;

    //Wait For All Alarm Clear
    bool all_alarm_clear = false;
    is_all_alarm_clear = all_alarm_clear;
    
    int count = 0;
    while(all_alarm_clear == false && !quit_ && count < 75) // maximum 150s 
    {
        count++;
        all_alarm_clear = true;

        for(auto [key,value] : *current_alarm)
        {
            if(value == 1)
            {
                std::cout << "error : " << key << std::endl;
                all_alarm_clear = false;
            }
        }


        if(all_alarm_clear == false)
        {            
            for(auto& keyvalue : *current_alarm)
            {
                keyvalue.second = 0;
            }
            std::cout << "all alarm clear" << std::endl;

            std::this_thread::sleep_for(1.5s);

            auto msg = std_msgs::msg::String();
            msg.data = "clear";
            alarm_reset_pub_->publish(msg);
        }
        std::this_thread::sleep_for(0.5s);
    }

    if(all_alarm_clear == true)
    {
        logWrite(LOG_INFO, "All alarm clear");
        count = 0;
        while(is_calibration == true && count < 300)
        {
            std::cout << "wait calibration" << std::endl;
            std::this_thread::sleep_for(1s);
            count++;
        }
    }

    if(is_calibration == true)
    {
        all_alarm_clear = false;
    }
    logWrite(LOG_INFO, "Reset Seq Done");
    
    is_all_alarm_clear = all_alarm_clear;
    is_reset_done = true;
    std::this_thread::sleep_for(1s);
    is_reset_start = false;
}

void AlarmReport::getEzioData(const std_msgs::msg::String::SharedPtr msg)
{
    json ezio_data = json::parse(msg->data);
    
    for(int i=0; i<64; i++)
    {
        ezio_input_->at(i) = ezio_data["DI"][i];
    }
    checkReset();
    checkBumper();
    checkEMS();
    
}

void AlarmReport::getTconAlarmData(const std_msgs::msg::Int32::SharedPtr msg)
{    
    int alarm_code = msg->data;
    std::cout << "tcon alarm code : " << alarm_code << std::endl;

    if(alarm_code > 3300 && alarm_code < 10000)
    {
        std::cout << "warning : " << alarm_code << std::endl;
    }
    else
    {
        (*current_alarm)[alarm_code] = 1;
    }
}

void AlarmReport::getTDriverAlarmData(const std_msgs::msg::String::SharedPtr msg)
{    
    try
    {
        json alarms = json::parse(msg->data);

        std::string type_string = alarms["DATA_TYPE"].get<std::string>();
        auto data_type = std::stoi(type_string);

        if(data_type == ROBOT_ALARM_INQUIRY)
        {
            auto fatals = alarms["fatals"];
            auto errors = alarms["errors"];
            auto warnings = alarms["warnings"];


            if(fatals.size() > 0)
            {
                for(int i=0; i<fatals.size(); i++)
                {
                    for(auto& [key, value] : fatals[i].items())
                    {
                        int error_code = std::atoi(key.c_str());
                        if(error_code == ROBOT_MAP_FILE_RESOLUTION)
                        {
                            (*current_alarm)[3105] = 1;
                        }
                        else if(error_code == ROBOT_MAP_FORMAT_ERROR)
                        {
                            (*current_alarm)[3106] = 1;
                        }
                        else if(error_code == ROBOT_BATTERY_TOO_LOW)
                        {
                            (*current_alarm)[3107] = 1;                            
                        }
                        else if(error_code == ROBOT_BATTERY_TEMPER_TOO_HIGH)
                        {
                            (*current_alarm)[3108] = 1;                            
                        }
                        else
                        {

                        }
                        std::cout << "get tdriver fatals" << std::endl;
                    }
                }
            }
            else
            {
                (*current_alarm)[3105] = 0; 
                (*current_alarm)[3106] = 0;      
                (*current_alarm)[3107] = 0;
                (*current_alarm)[3108] = 0;
            }

            if(errors.size() > 0)
            {
                for(int i=0; i<errors.size(); i++)
                {
                    for(auto& [key, value] : errors[i].items())
                    {
                        int error_code = std::atoi(key.c_str());
                        if(error_code == ROBOT_BLOCKED)
                        {
                            (*current_alarm)[3301] = 1;
                        }
                        else if(error_code == ROBOT_ODO_LOST)
                        {
                            if(is_bumper_detected_ == true)
                            {
                                (*current_alarm)[3202] = 1;
                            }
                            else if(is_emo_detected_ == true)
                            {

                            }
                            else
                            {
                                (*current_alarm)[3210] = 1;
                            }
                            motor_alarm = true;    
                        }
                        else if(error_code == ROBOT_MOTOR_TIMEOUT)
                        {                            
                            if(is_bumper_detected_ == true)
                            {
                                (*current_alarm)[3202] = 1;
                            }
                            else if(is_emo_detected_ == true)
                            {

                            }
                            else
                            {
                                (*current_alarm)[3215] = 1; 
                            } 
                            motor_alarm = true;                          
                        }
                        else if(error_code == ROBOT_CONFIDENCE_LOW)
                        {
                            (*current_alarm)[3211] = 1;                            
                        }
                        else if(error_code == ROBOT_NAVI_ERROR)
                        {
                            (*current_alarm)[3212] = 1;                            
                        }
                        else if(error_code == ROBOT_BATTERY_LOW)
                        {
                            (*current_alarm)[3213] = 1;                            
                        }
                        else if(error_code == ROBOT_PATH_PLANNING_FAIL)
                        {
                            std::cout << "path_planning_error" << std::endl;
                            (*current_alarm)[3214] = 1;                            
                        }
                        else
                        {

                        }
                    }
                }
            }
            else
            {  
                (*current_alarm)[3301] = 0;
                (*current_alarm)[3210] = 0;
                (*current_alarm)[3215] = 0;
                (*current_alarm)[3211] = 0;
                (*current_alarm)[3212] = 0;
                (*current_alarm)[3213] = 0;
                (*current_alarm)[3214] = 0;
            }

            if(warnings.size() > 0)
            {
                for(int i=0; i<warnings.size(); i++)
                {
                    bool motor_cali = false;
                    for(auto& [key, value] : warnings[i].items())
                    {
                        int error_code = std::atoi(key.c_str());
                        if(error_code == ROBOT_CALIBRATION)
                        {
                            motor_cali = true;
                        }
                    }
                    is_calibration = motor_cali;
                }
            }
            else
            {
                is_calibration = false;                
            }
        }
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void AlarmReport::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_ALARM, level, msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto alarm = std::make_shared<AlarmReport>();

    rclcpp::spin(alarm);

    rclcpp::shutdown();

    return 0;
}
