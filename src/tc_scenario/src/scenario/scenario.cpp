#include "scenario/scenario.hpp"

Scenario::Scenario() : Node("scenario")
{
    initVariables();
    readConfigFile();
    initPubSub();
    initTimers();

    //std::cout << "init scenraio node done" << std::endl;
}

Scenario::~Scenario()
{
    have_command_ = true;
}
	
void Scenario::tDriverInitCallBack(const std_msgs::msg::Int32::SharedPtr msg)
{
    int init_state = msg->data;
    int current_job = current_command_["current_job"].get<int>();
    init_state_ = init_state;

    if(current_job != CMD_NONE) // change map
    {
 
    }
    else if(init_state != INIT_DONE)
    {
        setLight(LIGHT_NONE);
        amr_status_json_["state"] = INIT;
    }
    else if(init_state == INIT_DONE && localization_done_ == false)
    {
        tray_rear_detection_ = ezio_input_[IN_TRAY_REAR_DETECTION].get<int>();
        motor_init_thread_ = std::thread(&Scenario::setMotorInit, this);
        localization_done_ = true;
    }
}

void Scenario::setMotorInit()
{

    logWrite(LOG_INFO, "motor_cali_done_ : " + std::to_string(motor_cali_done_));

    int count = 0;
    while(motor_cali_done_ == false)
    {
        if(motor_cali_done_ == true || count > 300)
        {
            std::this_thread::sleep_for(1000ms);
        }
        count++;
        std::this_thread::sleep_for(1000ms);
    }
    logWrite(LOG_INFO, "Wait Motor Cali done -> " + std::to_string(motor_cali_done_));

    if(motor_cali_done_ == true)
    {
        std::this_thread::sleep_for(1000ms);
        logWrite(LOG_INFO, "Init done");

        if(tray_rear_detection_ == true)
        {        
            logWrite(LOG_INFO, "lift up");
            liftControl(lift_upper_position_); 

            if(ezio_input_[IN_TRAY_REAR_DETECTION].get<int>() == 0)
            {
                std::this_thread::sleep_for(1000ms);
                logWrite(LOG_INFO, "lift down");
                liftControl(lift_lower_position_);
            }
        }
        else
        {
            logWrite(LOG_INFO, "lift down");
            liftControl(lift_lower_position_);
        }
        
        init_done_ = true;
        std::this_thread::sleep_for(500ms);

        amr_status_json_["state"] = NOT_ASSIGNED;        
        int mode = amr_status_json_["mode"].get<int>();
        if(mode == MANUAL_MODE)
        {
            setLight(LIGHT_WHITE);
        }
        else
        {
            setLight(LIGHT_BLUE);
        }
        setSound(SOUND_NONE);
    }
    else
    {
        logWrite(LOG_ERR, "TDriver Init Error");
    }

    auto msg = std_msgs::msg::Int32();
    msg.data = SCENARIO_INIT_DONE;
    tcon_init_pub_->publish(msg);

    std::this_thread::sleep_for(500ms);
}

void Scenario::readConfigFile()
{
    auto config = file_io_->readConfigFile();
    json nodes = json::parse(config);

    start_node_1f = nodes["start_node_1F"];
    waiting_node_1f = nodes["wait_node_1F"];

    start_node_2f = nodes["start_node_2F"];
    waiting_node_2f = nodes["wait_node_2F"];
}

void Scenario::getPioNodeStatus(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool read = msg->data;
    if(read)
        readConfigFile();
}

void Scenario::initVariables()
{
    mode_ = -1;
    init_done_ = false;
    localization_done_ = false;
    have_command_ = false;
    check_error_ = false;
    is_obstacle_sensing_ = false;
    motor_cali_done_ = false;

    current_command_["current_job_id"] = "";
    current_command_["final_goal"] = "";
    current_command_["current_job"] = CMD_NONE;

    file_io_ = std::make_shared<File_IO>();
    validation_ = std::make_shared<Validation>();
    task_queue_ = std::make_shared<ThreadSafeQueue<json>>();

    task_map_ = std::make_shared<std::map<std::string, int>>();
    task_map_->insert(std::make_pair("AQ", AQ));
    task_map_->insert(std::make_pair("DP", DP));
    task_map_->insert(std::make_pair("CHG", CHG));
    task_map_->insert(std::make_pair("OD", OD));
    task_map_->insert(std::make_pair("MP", MP));
    task_map_->insert(std::make_pair("ASE_OPEN", ASE_OPEN));
    task_map_->insert(std::make_pair("ASE_PASS", ASE_PASS));
    task_map_->insert(std::make_pair("ASE_EXIT", ASE_EXIT));
    task_map_->insert(std::make_pair("ASX_OPEN", ASX_OPEN));
    task_map_->insert(std::make_pair("ASX_PASS", ASX_PASS));
    task_map_->insert(std::make_pair("ASX_EXIT", ASX_EXIT));
    task_map_->insert(std::make_pair("EL_OPEN", EL_OPEN));
    task_map_->insert(std::make_pair("EL_PASS", EL_PASS));
    task_map_->insert(std::make_pair("EL_EXIT", EL_EXIT));

    acs_service_node_ = rclcpp::Node::make_shared("acs_service_node");
    
    std::cout << "readCurrentPosFile" << std::endl;
    auto pos = file_io_->readCurrentPosFile();
    std::cout << "readCurrentPosFile 22" << std::endl;
    amr_status_json_["node"] = pos[0];
    amr_status_json_["x"] = pos[1];
    amr_status_json_["y"] = pos[2];
    amr_status_json_["angle"] = pos[3];
    amr_status_json_["next_node"] = 0;
    amr_status_json_["vx"] = 0.0;
    amr_status_json_["vy"] = 0.0;
    amr_status_json_["battery_soc"] = 0.0;
    amr_status_json_["battery_volt"] = 0;
    amr_status_json_["unfinished_stations"] = 0;
    amr_status_json_["state"] = INIT;
    amr_status_json_["jack_height"] = 0.0;
    amr_status_json_["mode"] = -1;

    amr_status_json_["current_pio_id"] = 0;

    auto ezio_input = std::make_shared<std::vector<int>>(64, 0);
    auto ezio_output = std::make_shared<std::vector<int>>(64, 0);

    for(int i=0; i<64; i++)
    {
        ezio_input_.push_back(ezio_input->at(i));
        ezio_output_.push_back(ezio_output->at(i));
    }
    std::cout << "ezio_output_.push_backe" << std::endl;


    readMapFile();
    logWrite(LOG_INFO, "Init variables");
    
}

void Scenario::initTimers()
{
    amr_pub_timer_ = this->create_wall_timer(50ms, std::bind(&Scenario::pubAmrStatus,this));
    agv_mode_timer_ = this->create_wall_timer(50ms, std::bind(&Scenario::checkAgvMode,this));
    logWrite(LOG_INFO, "Init timer");
}

void Scenario::initPubSub()
{
    acs_command_server_ = this->create_service<tc_acs_interface::srv::AcsCommand>("acs_command", 
        std::bind(&Scenario::receiveAcsCommand, this, std::placeholders::_1, std::placeholders::_2));

    cmd_client_ = acs_service_node_->create_client<tc_acs_interface::srv::AcsCommand>("acs_command_to_tdriver");
    
    docking_client_ = acs_service_node_->create_client<tc_msgs::srv::ArucoMarkers>("aruco_pose");
    
    t_driver_sub_ = this->create_subscription<std_msgs::msg::String>
    ("t_driver_status", 10, std::bind(&Scenario::tDriverCallBack, this, std::placeholders::_1));

    t_driver_init_sub_ = this->create_subscription<std_msgs::msg::Int32>
    ("t_driver_init_status", 10, std::bind(&Scenario::tDriverInitCallBack, this, std::placeholders::_1));
    
    task_sub_ = this->create_subscription<tc_msgs::msg::AmrTask>
                ("docking_return", 10, std::bind(&Scenario::receivePIOMessage, this, std::placeholders::_1));
    
    alarm_sub_ = this->create_subscription<tc_msgs::msg::AmrAlarm>
    ("tcon_alarms", 10, std::bind(&Scenario::getAlarmMsg, this, std::placeholders::_1));
    
    ezio_sub_ = this->create_subscription<std_msgs::msg::String>
    ("ezio_data", 10, std::bind(&Scenario::getEzioStatus, this, std::placeholders::_1));

	pio_sub_ = this->create_subscription<std_msgs::msg::Bool>
    ("pio_changed", 10, std::bind(&Scenario::getPioNodeStatus, this, std::placeholders::_1));


    acs_report_pub_ = this->create_publisher<std_msgs::msg::String>("acs_report", 10);
    
    sound_pub_ = this->create_publisher<std_msgs::msg::Int32>("sound_request", 10);
    light_pub_ = this->create_publisher<std_msgs::msg::Int32>("light_request", 10);
    task_pub_ = this->create_publisher<tc_msgs::msg::AmrTask>("docking_order", 10);
    acs_task_pub_ = this->create_publisher<std_msgs::msg::String>("acs_task_report", 10);
    map_change_pub_ = this->create_publisher<std_msgs::msg::String>("map_change", 10);
    amr_init_status_pub_ = this->create_publisher<tc_msgs::msg::AmrInitStatus>("amr_init_command", 10);
    amr_jack_pub_ = this->create_publisher<tc_msgs::msg::AmrLiftMotion>("amr_jack", 10);
    tcon_alarm_pub_ = this->create_publisher<std_msgs::msg::Int32>("amr_alarms", 10);
    tcon_init_pub_ = this->create_publisher<std_msgs::msg::Int32>("amr_scenario_init", 10);

    logWrite(LOG_INFO, "Init publisher and subscriber");
}

void Scenario::readMapFile()
{
    auto nodes = file_io_->readNodeFile();
    
    node_info_ = json::parse(nodes);

    //std::cout << "read node info done" << std::endl;
    logWrite(LOG_INFO, "Read node info done");
}

void Scenario::receiveAcsCommand(const std::shared_ptr<tc_acs_interface::srv::AcsCommand::Request> request,
    std::shared_ptr<tc_acs_interface::srv::AcsCommand::Response>	response)
{
    auto msg = request->msg;
    json command = json::parse(msg);
    
    int amr_mode = amr_status_json_["mode"].get<int>();
    
    if(amr_mode == AUTO_MODE)
    {
        std::thread t([this, &command]
        {
            int command_type = command["type"].get<int>();

            switch(command_type)
            {
                case MOVE_WORK:
                    moveWork(command);
                break;
                case MOVE_WORK_CANCEL:
                    moveWorkCancel(command);
                break;
                case STOP_CHARGE:
                    pauseAmr(command);
                break;
                case PAUSE_AMR:
                    //test();
                break;
                default:
                break;
            }
        });

        t.detach();
    }

    std::this_thread::sleep_for(100ms); // Do not remove. This time for copy acs data to char[] in lamda.
    response->received = true;
}

void Scenario::test()
{
    //std::cout << "waiting change map " << std::endl;
    
    std::string from_node = "W204";
    std::string target_map = "1F";
    
    
    changeMap(from_node, target_map);
    //std::cout << "change map finished -> " << std::endl;
}


void Scenario::moveWork(json command)
{    
    logWrite(LOG_INFO, "Receive Move Work");

    logWrite(LOG_INFO, command.dump());

    json move_work = command;
    std::string job_id = move_work["job_id"];
    auto change_route = move_work["change_route"].get<int>();
    std::string final_goal = move_work["final_goal"];
    auto path_count = move_work["path_count"].get<int>();
    auto path_list = move_work["path_list"];
    auto operation_target = move_work["operation_target"].get<int>();
    std::string target_name = move_work["target_name"];
    auto task_count = move_work["task_count"].get<int>();
    auto task_list = move_work["task_list"];
    auto current_job = current_command_["current_job"].get<int>();

    int cur_state = amr_status_json_["state"].get<int>();

    std::string cur_job_id = current_command_["current_job_id"];

    if(current_job == CMD_TASK_READY || current_job == CMD_TASK_RUNNING || current_job == CMD_TASK_DONE)
    {
        if(cur_state == CHARGING)
        {
            logWrite(LOG_INFO, "AGV is in task seq but task is charging. continue movework");
        }
        else
        {
            logWrite(LOG_INFO, "AGV is in task seq. Ignore ACS Command");
            return;
        }
    }

    if(change_route == ROUTE_CLEAR_PRE_CMD) //parse move command
    {
        
        while(task_queue_->count() > 0)
        {
            task_queue_->pop();
            logWrite(LOG_INFO, "Remove Task");
        }

        //std::cout << "receive ROUTE_CLEAR_PRE_CMD" << std::endl;

        logWrite(LOG_INFO, "receive ROUTE_CLEAR_PRE_CMD");

        int amr_status = amr_status_json_["task_status"].get<int>();
        if(amr_status == TASK_STAUTS_RUNNING)
        {
            amrAction(MOVE_WORK_CANCEL);
            //std::cout << "send amr stop in moveWork" << std::endl;
            logWrite(LOG_INFO, "send amr stop");

            while(amr_status != TASK_STAUTS_CANCELED)
            {
                //std::cout << "wait cancel success in moveWork" << std::endl;
                std::this_thread::sleep_for(100ms); // Do not remove
                amr_status = amr_status_json_["task_status"].get<int>();
                logWrite(LOG_INFO, "wait amr navigation cancled");
            } 
        }
    }
    else if(cur_job_id != "" && job_id != cur_job_id)
    {
        //std::cout << "cur_job_id != received_job_id" << std::endl;
        /*
            Wrong Command - job id mismatch Error
        */
        return;
    }

    have_command_ = true;

    cur_state = amr_status_json_["state"].get<int>();

    bool is_charging = false;
    while(cur_state == CHARGING)
    {
        is_charging = true;
        std::this_thread::sleep_for(100ms); // Do not remove
        cur_state = amr_status_json_["state"].get<int>();
        //std::cout << "wait charging finished" << std::endl;
    }

    if(is_charging == true)
    {
        std::this_thread::sleep_for(2000ms); // Do not remove        
    }

    while(cur_state != NOT_ASSIGNED)
    {
        std::this_thread::sleep_for(100ms); // Do not remove
        cur_state = amr_status_json_["state"].get<int>();
        //std::cout << "wait agv state NOT_ASSIGNED changed" << std::endl;
    }


    std::vector<std::string> via_node;

    logWrite(LOG_INFO, "getPathList");
    auto paths = getPathList(path_list, path_count, via_node);

    if(cur_job_id != job_id || send_continue_report_ == true)
    { 
        send_continue_report_ = false;
        getTaskList(task_list, task_count);
        logWrite(LOG_INFO, "getTaskList");
    }   
    else
    {
        //std::cout << "ignore job cause of same job id" << std::endl;
    }
     

    auto validation_result = validation_->validateMoveCommand(job_id, via_node);

    if(validation_result == false)
    {
        logWrite(LOG_INFO, "Validation Fail");
        //std::cout << "validation fail" << std::endl;
        return;
    }


    if(path_count > 0 )
    {
        std::string path_last_node = path_list[path_count-1]["to_node"];
        if(final_goal != path_last_node)
        {        
            while(task_queue_->count() > 0)
            {
                task_queue_->pop();
                logWrite(LOG_INFO, "Remove Task -> final goal : " + final_goal + " / last_node : " + path_last_node);
            }
        }
    }

    logWrite(LOG_INFO, "task_queue_->count() : " + std::to_string(task_queue_->count()));
    
    //Send to t-driver
    
    paths["action"] = MOVE_WORK;
    auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();
    request->msg = paths.dump();


    try
    {
        while(!cmd_client_->wait_for_service(1s))
        {            
            if(!rclcpp::ok())
            {
                //std::cout << "Service Client cannot found server" << std::endl;
                //Something Error
                return;
            }
        }
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }


    auto result = cmd_client_->async_send_request(request);
    if(rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        //std::cout << "server result : " << result.get()->received << std::endl;

        std::this_thread::sleep_for(10ms); // Do not remove
    }

    if(path_count < 1)
    {
        current_command_["current_job"] = CMD_MOVE;
    }

    cur_job_id_ = job_id;
    current_command_["final_goal"] = final_goal;

    logWrite(LOG_INFO, "Send to T-Driver");
}

json Scenario::getPathList(json path_list, int count, std::vector<std::string> &via_node)
{
    json paths;

    for(int i=0; i<count; i++)
    {
        json path;
        std::string acs_from_node = path_list[i]["from_node"];
        std::string acs_to_node = path_list[i]["to_node"];
 
        if(acs_from_node == "D101")
        {
            json path2;
            path2["from_node"] = "5101";
            path2["to_node"] = "1101";
            path2["method"] = "backward";
            paths["path_list"].push_back(path2);
        }
        else if(acs_from_node == "D102")
        {
            json path2;
            path2["from_node"] = "5102";
            path2["to_node"] = "1102";
            path2["method"] = "backward";
            paths["path_list"].push_back(path2);
        }
        else if(acs_from_node == "D201")
        {
            json path2;
            path2["from_node"] = "5201";
            path2["to_node"] = "1201";
            path2["method"] = "backward";
            paths["path_list"].push_back(path2);

        }
        else if(acs_from_node == "D201")
        {
            json path2;
            path2["from_node"] = "5202";
            path2["to_node"] = "1202";
            path2["method"] = "backward";
            paths["path_list"].push_back(path2);
        }
        
        std::string tdriver_from_node = getTdriverNodeNumber(acs_from_node);
        std::string tdriver_to_node = getTdriverNodeNumber(acs_to_node);

        path["from_node"] = tdriver_from_node;
        path["to_node"] = tdriver_to_node;

        via_node.push_back(acs_to_node);

        paths["path_list"].push_back(path);
    }
    
    paths["count"] = paths["path_list"].size();
    

    return paths;
}

void Scenario::getTaskList(json task_list, int count)
{
    std::vector<json> tasks;
    for(int i=0; i<count; i++)
    {     
        json task;
        task["from_node"] = task_list[i]["from_node"];
        task["to_node"] = task_list[i]["to_node"];
        task["task_number"] = task_list[i]["task_number"];
        task["task_type"] = task_list[i]["task_type"];
        task["target_map"] = task_list[i]["target_map"];
        
        tasks.push_back(task);
    }
    
    std::sort(tasks.begin(), tasks.end(), [](const auto& lhs, const auto& rhs) 
    {
        return lhs["task_number"] < rhs["task_number"];
    });    

    for(int i=0; i<count; i++)
    {
        task_queue_->push(tasks.at(i));
    }
}

std::string Scenario::getTdriverNodeNumber(std::string node)
{
    int node_number = std::stoi(node.substr(1,3));
    std::string node_header = node.substr(0, 1); 
 
    if(node_header == "D")
    {
        node_number += 1000;
    }
    else if(node_header == "A")
    {
        node_number += 2000;        
    }
    else if(node_header == "E")
    {
        node_number += 3000;        
    }
    else if(node_header == "C")
    {
        node_number += 4000;        
    }

    return std::to_string(node_number);
} 

void Scenario::moveWorkCancel(json command)
{
    json move_work_cancel = command;
    std::string job_id = move_work_cancel["job_id"];
    std::string job_id_to_cancel = move_work_cancel["job_id_to_cancel"];
    int cancel_option = (int)move_work_cancel["cancel_option"].get<char>();
    std::string current_job_id = current_command_["current_job_id"];

    std::string log_string = "move work canceled job id : " + job_id_to_cancel;
    logWrite(LOG_INFO, log_string);


    while(task_queue_->count() > 0)
    {
        task_queue_->pop();
        //std::cout << "remove task" << std::endl;
        logWrite(LOG_INFO, "remove task");
    }

    //std::cout << log_string << std::endl;
    
    doTask(PIO_RESET_1 , 0);
    logWrite(LOG_INFO, "PIO Reset");

    amrAction(MOVE_WORK_CANCEL);
    logWrite(LOG_INFO, "send amr stop");
    
    int amr_status = amr_status_json_["task_status"].get<int>();
    if(amr_status == TASK_STAUTS_RUNNING || amr_status != TASK_STAUTS_COMPLETED)
    {
        while(amr_status != TASK_STAUTS_CANCELED)
        {
            //std::cout << "wait cancel success" << std::endl;
            std::this_thread::sleep_for(100ms); // Do not remove
            amr_status = amr_status_json_["task_status"].get<int>();
        }
    }

    if(current_job_id != "")
    {           
        json report;
        report["type"] = "move_complete";
        report["current_job_id"] = job_id;
        report["canceled_job_id"] = job_id_to_cancel;      
        report["complete_type"] = 2;                          

        pubAcsReport(report.dump());      
    }
    

    logWrite(LOG_INFO, command.dump());
}

void Scenario::pauseAmr(json command)
{
    logWrite(LOG_INFO, command.dump());
}

void Scenario::tDriverCallBack(const std_msgs::msg::String::SharedPtr msg)
{
    //TODO: Make Timer pub to acs_comm node 
    
    json j = json::parse(msg->data);
    
    std::string type_string = j["DATA_TYPE"].get<std::string>();
    auto data_type = std::stoi(type_string);

    switch(data_type)
    {
        case ROBOT_LOCATION_INQUIRY:
        {
            amr_status_json_["x"] = j["x"].get<double>();
            amr_status_json_["y"] = j["y"].get<double>();
            json path2;
            path2["from_node"] = "5102";
            amr_status_json_["angle"] = j["angle"].get<int>();
            std::string cur_station = j["current_station"];

            if(cur_station == "")
            {
                std::string last_station = j["last_station"];
                if(last_station == "")
                {
                    auto pos = file_io_->readCurrentPosFile();
                    amr_status_json_["node"] = pos[0];
                }
                else
                {
                    amr_status_json_["node"] = std::stoi(last_station);
                }
            }
            else
            {
                int unfinished_path_size = amr_status_json_["unfinished_stations"].get<int>();
                if(unfinished_path_size > 0)
                {
                    auto unfinished_stations = amr_status_json_["unfinished_path"];
                    
                    std::string next_station = unfinished_stations[0];
                    int next_node = std::stoi(next_station.erase(0,2));
                    int cur_node = std::stoi(cur_station);

                    if(cur_node != next_node)
                    {
                        amr_status_json_["node"] = cur_node;
                        amr_status_json_["next_node"] = next_node;
                    }
                }
                else
                {
                    amr_status_json_["node"] = std::stoi(cur_station);
                    amr_status_json_["next_node"] = 0;
                }
            }
        }
        break;

        case ROBOT_SPEED_INQUIRY:
            amr_status_json_["vx"] = j["vx"].get<double>();
            amr_status_json_["vy"] = j["vy"].get<double>();
        break;
        case ROBOT_BATTERY_INQUIRY:
            amr_status_json_["battery_soc"] = j["battery_level"].get<float>()*100;
            amr_status_json_["battery_volt"] = (int)(j["voltage"].get<float>());
        break;

        case ROBOT_ESTOP_INQUIRY:
        break;

        case ROBOT_NAVIGATION_INQUIRY:
            parseNavigationInquiry(j);
        break;
        
        case ROBOT_LOCALIZATION_STATUS_INQUIRY:
            amr_status_json_["reloc_status"] = j["reloc_status"];
        break;
        case ROBOT_JACKING_STATUS_INQUIRY:        
            amr_status_json_["jack_state"] = j["jack_state"];
            amr_status_json_["jack_height"] = j["jack_height"];
            amr_status_json_["jack_error_code"] = j["jack_error_code"];
            amr_status_json_["jack_enable"] =  j["jack_enable"];
        break;
        case ROBOT_ALARM_INQUIRY:            
            auto warnings = j["warnings"];

            if(warnings.size() > 0)
            {
                for(int i=0; i<warnings.size(); i++)
                {
                    bool motor_cali = true;
                    for(auto& [key, value] : warnings[i].items())
                    {
                        int error_code = std::atoi(key.c_str());
                        if(error_code == ROBOT_CALIBRATION)
                        {
                            motor_cali = false;
                        }
                    }
                    motor_cali_done_ = motor_cali;
                }
            }
            else
            {
                motor_cali_done_ = true;
            }

        break;
    }
}

void Scenario::parseNavigationInquiry(json j)
{    
    int amr_mode = amr_status_json_["mode"].get<int>();
    int task_status = j["task_status"].get<int>();
    int task_type = j["task_type"].get<int>(); 

    amr_status_json_["task_status"] = task_status;
    amr_status_json_["task_type"] = task_type;

    auto unfinished_stations = j["unfinished_path"];
    amr_status_json_["unfinished_path"] =j["unfinished_path"];
    amr_status_json_["unfinished_stations"] = unfinished_stations.size();
    

    if(task_type == TRANSLATIONAL_ROTATION && task_status == TASK_STAUTS_COMPLETED)
    {

    }
    else if(task_type == TRANSLATIONAL_ROTATION && task_status == TASK_STAUTS_RUNNING)
    {

    }
    else if(task_type == OTHER && task_status == TASK_STAUTS_RUNNING)
    {
        
    }
    else if(task_status == TASK_STAUTS_RUNNING)
    {
        current_command_["current_job_id"] = cur_job_id_;
        int amr_state = current_command_["current_job"].get<int>();

        if(is_obstacle_sensing_ == false)
        {
            if(amr_state != CMD_TASK_READY && amr_state != CMD_TASK_RUNNING && amr_state != CMD_TASK_DONE)
            {
                current_command_["current_job"] = CMD_MOVE;
            }

            amr_state = current_command_["current_job"].get<int>();

            if(amr_state == CMD_MOVE)
            {
                amr_status_json_["state"] = MOVE;
                setLight(LIGHT_GREEN);
                setSound(SOUND_MOVING_PORT);

                if(task_queue_->count() > 0)
                {                    
                    auto command = task_queue_->front();
                    amrDoTaskFirst(command);
                } 
            }
            else
            {
                //Move when task ( a/s pass, e/v pass)
            }        
        }
        else
        {
            setLight(LIGHT_RED);
            setSound(SOUND_OBSTACLE);      
            setFailReport(INFO, OBSTACLE_DETECT_ALARM);
        }
    }
    else if(task_status == TASK_STAUTS_SUSPENDED)
    {
        if(task_result_ == 3)
        {
            logWrite(LOG_INFO, "A/S Open Busy Done : ResumeAmr()");
            std::string tmp = "task_result_ : " + std::to_string(task_result_) + " / task_status: " + std::to_string(task_status);
            logWrite(LOG_INFO, tmp);

            amrAction(RESUME_AMR);              
        }
    }
    else if(task_status == TASK_STAUTS_FAILED)
    {
        std::string cur_job_id = current_command_["current_job_id"];
        if(cur_job_id == "")
        {
            
        }
    }
    else if(task_status == TASK_STAUTS_CANCELED)
    {
        current_command_["current_job_id"] = "";   
        have_command_ = false;
        if(amr_mode == AUTO_MODE && alarm_state_ != ERROR_OCCUR)
        {
            amr_status_json_["state"] = NOT_ASSIGNED;
            setLight(LIGHT_BLUE);
            setSound(SOUND_NONE);
        }
        validation_->clearValidation();
    }
    else if(task_status == TASK_STAUTS_COMPLETED)
    {        
        int amr_state = current_command_["current_job"].get<int>();

        if(amr_state == CMD_MOVE)
        {
            setLight(LIGHT_GREEN);
            setSound(SOUND_NONE);
            //std::cout << "set state arrival" << std::endl;

            //std::cout << "task_queue_->count()" << task_queue_->count() << std::endl;

            if(task_queue_->count() > 0)
            {
                current_command_["current_job"] = CMD_TASK_READY;
            }
            else
            {
                current_command_["current_job"] = CMD_NONE;                
            }
            amr_status_json_["state"] = ARRIVAL;
        }
        else if(amr_state == CMD_TASK_READY)
        {
            logWrite(LOG_INFO, "task count : " + std::to_string(task_queue_->count()));

            auto command = task_queue_->front();
            std::string task_type = command["task_type"];            
            int task_priority = (*task_map_)[task_type];            
            logWrite(LOG_INFO, "current task_priority : " + std::to_string(task_priority));
            
            auto th = std::thread(&Scenario::amrDoTask, this, command);
            std::this_thread::sleep_for(100ms);
            th.detach();

            current_command_["current_job"] = CMD_TASK_RUNNING;

            task_queue_->pop();
        }
        else if(amr_state == CMD_TASK_RUNNING)
        {
            //wait task running done
        }
        else if(amr_state == CMD_TASK_DONE)
        {
            if(task_queue_->count() > 0)
            {        
                current_command_["current_job"] = CMD_TASK_READY;
            }
            else
            {
                int cur_state = amr_status_json_["state"].get<int>();
                if(cur_state != CHARGING)
                {
                    setLight(LIGHT_NONE);
                    current_command_["current_job"] = CMD_NONE;     
                }
                else
                {
                    //stay in charging 
                }
            }
        }
        else if(amr_state == CMD_TASK_FAIL) // means PIO_TIME_OUT
        {
            //Do PIO RESET
        }
        else // CMD_NONE
        {
            std::string current_job_id = current_command_["current_job_id"];

            if(current_job_id != "")
            {
                json report;
                report["type"] = "move_complete";
                report["current_job_id"] = current_command_["current_job_id"];
                report["canceled_job_id"] = "";      
                report["complete_type"] = 0;                          

                pubAcsReport(report.dump());      
                
                current_command_["current_job_id"] = "";
                validation_->clearValidation();          
            }

            have_command_ = false;

            if(alarm_state_ != ERROR_OCCUR)
            {
                int mode = amr_status_json_["mode"].get<int>();
                if(mode == MANUAL_MODE)
                {
                    setLight(LIGHT_WHITE);
                }
                else
                {
                    amr_status_json_["state"] = NOT_ASSIGNED;
                    setLight(LIGHT_BLUE);
                }
                setSound(SOUND_NONE);
            }
        }
    }
    else
    {
        int mode = amr_status_json_["mode"].get<int>();
        if(init_done_ == true && mode == AUTO_MODE)
        {
            amr_status_json_["state"] = NOT_ASSIGNED;
        }  
    }
}


bool Scenario::amrGetAruco()
{
    auto request = std::make_shared<tc_msgs::srv::ArucoMarkers::Request>();
    request->task = 1;

    std::vector<float> data;

    //Check Service server is alive
    try
    {
        while(!docking_client_->wait_for_service(1s))
        {
            std::cout << "wait for service" << std::endl;
            
            if(!rclcpp::ok())
            {
                std::cout << "Service Client cannot found server" << std::endl;
                //Something Error
                return false;
            }
        }
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    
    //Send Request
    int count = 0;    
    bool detected = false;

    float dist = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    float theta = 0.0;
    float vw = 0.0;

    while(count < 5)
    {    
        auto result = docking_client_->async_send_request(request);
        std::cout << "Wait for server response" << std::endl;

        if(rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            detected = result.get()->detected;
            dist = result.get()->dist;
            vx = result.get()->vx;
            vy = result.get()->vy;
            theta = result.get()->theta;
            vw = result.get()->vw;

            std::cout << "server result : " << detected << std::endl;

            if(detected == true)
            {
                break;
            }
            else
            {
                std::cout << "server result : " << detected << std::endl;
            }
        }
        count++;
        std::this_thread::sleep_for(1s);
    }

    if(detected == true)
    {
        auto abs_theta = std::abs(theta);

        logWrite(LOG_INFO, "theta : " + std::to_string(abs_theta));
        if(abs_theta < 15)
        {
            return true;
        }
    }
    else
    {
        logWrite(LOG_ERR, "Aruco marker detect fail");
    }
    return false;
}

void Scenario::amrDoTaskFirst(json command)
{ 
    std::string task_type = command["task_type"];    
    auto tdriver_node = amr_status_json_["node"].get<int>();

    int task_status = amr_status_json_["task_status"].get<int>();
    int task_priority = (*task_map_)[task_type];

    if(std::stoi(start_node_2f) == tdriver_node || tdriver_node == std::stoi(waiting_node_2f))
    {
        if(task_priority == ASE_OPEN || task_priority == ASX_OPEN)
        {
            int order = AIRSHOWER_2_2;
            if(task_feedback_ != order) //Send Message
            {
                logWrite(LOG_INFO, "amrDoTaskFirst Goal 2F -> " + task_type);
                doTask(order , 0);
            }
        }
    }
    else if(tdriver_node == std::stoi(start_node_1f) || tdriver_node == std::stoi(waiting_node_1f))
    {
        if(task_priority == ASE_OPEN || task_priority == ASX_OPEN)
        {
            int order = AIRSHOWER_1_2;
            if(task_feedback_ != order) //Send Message
            {
                logWrite(LOG_INFO, "amrDoTaskFirst Goal 1F -> " + task_type);
                doTask(order , 0);
            }
            else if(task_result_ != 3 && task_status == TASK_STAUTS_RUNNING) //Not Busy
            {
                logWrite(LOG_INFO, "Wait A/S Open Busy : PauseAmr()");
                std::string tmp = "task_result_ : " + std::to_string(task_result_) + " / task_status: " + std::to_string(task_status);
                logWrite(LOG_INFO, tmp);

                amrAction(PAUSE_AMR);
            }
            else if(task_result_ == 3 && task_status == TASK_STAUTS_SUSPENDED)
            {
                logWrite(LOG_INFO, "A/S Open Busy Done : ResumeAmr()");
                std::string tmp = "task_result_ : " + std::to_string(task_result_) + " / task_status: " + std::to_string(task_status);
                logWrite(LOG_INFO, tmp);

                amrAction(RESUME_AMR);              
            }
        }        
    }
    else if(tdriver_node == 210 || tdriver_node == 211)
    {
        if(task_priority == ASE_OPEN || task_priority == ASX_OPEN)
        {
            //std::cout << "task_status : " << task_status << std::endl;
            //std::cout << "task_result_ : " << task_result_ << std::endl;
            
            if(task_result_ != 3 && task_status == TASK_STAUTS_RUNNING) //Not Busy
            {
                logWrite(LOG_INFO, "Wait A/S Open Busy : PauseAmr()");
                std::string tmp = "task_result_ : " + std::to_string(task_result_) + " / task_status: " + std::to_string(task_status);
                logWrite(LOG_INFO, tmp);
            
                amrAction(PAUSE_AMR);
            }
            else if(task_result_ == 3 && task_status == TASK_STAUTS_SUSPENDED)
            {
                logWrite(LOG_INFO, "A/S Open Busy Done : ResumeAmr()");
                std::string tmp = "task_result_ : " + std::to_string(task_result_) + " / task_status: " + std::to_string(task_status);
                logWrite(LOG_INFO, tmp);
                
                amrAction(RESUME_AMR);            
            }
        }        
    }    
}

void Scenario::amrDoTask(json command)
{    
    std::string from_node = command["from_node"];
    std::string to_node = command["to_node"];    
    std::string task_type = command["task_type"];
    std::string target_map = command["target_map"];
    int task_number = command["task_number"].get<int>();

    auto tdriver_node = amr_status_json_["node"].get<int>();
    auto node_name = getNodeName(tdriver_node);

    int task_priority = (*task_map_)[task_type];
    logWrite(LOG_INFO, "amrDoTask Goal -> " + task_type);

    switch(task_priority)
    {
        case AQ:
            acquireOrDeposit(node_name, task_number, task_priority);
        break;
        case DP:
            acquireOrDeposit(node_name, task_number, task_priority);
        break;
        case CHG:
            charge(task_number);
        break;
        case OD:
        break;
        case MP:
        break;
        case ASE_OPEN:
            airshowerOpen(node_name, task_number);
        break;
        case ASE_PASS:
            airshowerPass(from_node, to_node, task_number);
        break;
        case ASE_EXIT:
            airshowerExit(from_node, to_node, task_number);
        break;
        case ASX_OPEN:
            airshowerOpen(node_name, task_number);
        break;
        case ASX_PASS:
            airshowerPass(from_node, to_node, task_number);
        break;
        case ASX_EXIT:
            airshowerExit(from_node, to_node, task_number);
        break;
        case EL_OPEN:
            elevatorOpen(node_name, task_number);
        break;
        case EL_PASS:
            elevatorPass(from_node, to_node, task_number, target_map);
        break;
        case EL_EXIT:
            elevatorExit(from_node, to_node, task_number);
        break;
    }
}

void Scenario::airshowerOpen(std::string node_name, int task_number)
{
    setLight(LIGHT_GREEN);
    setSound(SOUND_MOVING_AIRSHOWER);
    int order = 0;
    int count = 0;
    while(order == 0 && count < 60)
    {
        //std::cout << "node_name : " << node_name << std::endl;
        if(node_name == "A201")
        {
            order = AIRSHOWER_2_2;
        }
        else if(node_name == "A203")
        {
            order = AIRSHOWER_2_1;
        }
        else if(node_name == "A101")
        {
            order = AIRSHOWER_1_2;
        }
        else if(node_name == "A103")
        {
            order = AIRSHOWER_1_1;
        }
        std::this_thread::sleep_for(1000ms);
        count++;
        node_name = getNodeName(amr_status_json_["node"].get<int>());
    }

    if(order == 0)
    {
        pubAlarm(AIRSHOWER_ALARM);
        //std::cout << "error when open door" << std::endl;
        return;
    }
    logWrite(LOG_INFO, "airshowerOpen Start : Current node -> " + node_name);


    amr_status_json_["state"] = OPENDOOR;
    count = 0;
    //std::cout << "start air shower open " <<  std::endl;

    int current_pio_id = amr_status_json_["current_pio_id"].get<int>();

    if(current_pio_id == order)
    {
        doTask(PIO_RESET_1 , 0);     
        std::this_thread::sleep_for(7000ms);   
    }

    amr_status_json_["current_pio_id"] = order;

    logWrite(LOG_INFO, "airshowerOpen Check feedback");

    std::string log_string = "task_feedback_ : " + std::to_string(task_feedback_);
    logWrite(LOG_INFO, log_string);

    log_string = "order : " + std::to_string(order);
    logWrite(LOG_INFO, log_string);

    
    while(count < 60 && task_feedback_ != order && check_error_ == false) //Send Message
    {
        //std::cout << "wait feedback == order" <<  std::endl;
        doTask(order , 0);
        std::this_thread::sleep_for(1000ms);
        count++;
    }


    double current_lift_height = amr_status_json_["jack_height"].get<double>();
    if(current_lift_height < -0.085 && check_error_ == false)
    {
        logWrite(LOG_INFO, "Lift Up Before AirshowerOpen When No carrier -> height : lift_as_position_");
        liftControl(lift_as_position_);
    }

    logWrite(LOG_INFO, "airshowerOpen Check result");

    log_string = "task_result_ : " + std::to_string(task_result_);
    logWrite(LOG_INFO, log_string);    

    count = 0;
    while(task_result_ != 3 && task_result_ != 2 && check_error_ == false) // Maximum 5mins
    { 
        //std::cout << "wait open door busy : " << task_result_ << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
        log_string = "task_result_ : " + std::to_string(task_result_);
        logWrite(LOG_INFO, log_string);
    }

    if(task_result_ == 3)
    {
        json report;
        report["type"] = "task";
        report["current_job_id"] = current_command_["current_job_id"];
        report["task_number"] = task_number;
        pubAcsReport(report.dump());

        current_command_["current_job"] = CMD_TASK_DONE;

        logWrite(LOG_ERR, "airshowerOpen Finished");
        
        //std::cout << "open door done" <<  std::endl;
    }
    else
    {
        log_string = "task_result_ : " + std::to_string(task_result_);
        logWrite(LOG_INFO, log_string); 

        logWrite(LOG_ERR, "Task Error when AirShower Open");

        if(task_error_code_ == Error_Code::CANNOT_RECEIVE_READY)
        {
            pubAlarm(Error_Code::AS_EQUIPMENT_FAIL);            
        }
        else
        {
            pubAlarm(task_error_code_);
        }
        //std::cout << "something error when open door" << std::endl;
        //SomthingError
    }
}

void Scenario::airshowerPass(std::string from_node, std::string to_node, int task_number)
{
    std::string log_string;
    //std::cout << "a/s pass start" << std::endl;
    logWrite(LOG_INFO, "airshowerPass Start");

    int count = 0;
    while(count < 60 && task_result_ != 3 && check_error_ == false)
    {
        //std::cout << "wait a/s pass busy : " << task_result_ <<std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    if(task_result_ == 3)
    {
        logWrite(LOG_INFO, "from node : " + from_node);
        logWrite(LOG_INFO, "to node: " + to_node);
        //move
        json paths;
        paths["action"] = MOVE_WORK;
        paths["count"] = 1;

        json path;

        std::string tdriver_from_node = getTdriverNodeNumber(from_node);
        std::string tdriver_to_node = getTdriverNodeNumber(to_node);

        path["from_node"] = tdriver_from_node;
        path["to_node"] = tdriver_to_node;
        path["method"] = "forward";

        paths["path_list"].push_back(path);

        try
        {
            while(!cmd_client_->wait_for_service(1s))
            {            
                if(!rclcpp::ok())
                {
                    //std::cout << "Service Client cannot found server" << std::endl;
                    //Something Error
                    return;
                }
            }
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }

        auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();
        request->msg = paths.dump();

        auto result = cmd_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            //std::cout << "server result : " << result.get()->received << std::endl;

            std::this_thread::sleep_for(10ms); // Do not remove            
        }
        
        count = 0;
        int task_status =  amr_status_json_["task_status"].get<int>();
        while(count < 120 && task_status != TASK_STAUTS_RUNNING && check_error_ == false)
        {
            //std::cout << "wait agv moving start" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;         
        }

        logWrite(LOG_INFO, "Agv moving Start");

        int current_pio_id = amr_status_json_["current_pio_id"].get<int>();
        //std::cout << "current_pio_id : " << current_pio_id << std::endl;

        doTask(current_pio_id , 0, 3);
        std::this_thread::sleep_for(1000ms);
        
        while(count < 120 && task_status != TASK_STAUTS_COMPLETED && check_error_ == false)
        {
            //std::cout << "wait agv moving finish" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }

        logWrite(LOG_INFO, "Agv moving Finish");

        //std::cout << "task_result in a/s pass after moving : " << task_result_ << std::endl;

        if(task_status == TASK_STAUTS_COMPLETED)
        {
            //std::cout << "task_result in a/s pass after moving2 : " << task_result_ << std::endl;
            json report;
            report["type"] = "task";
            report["current_job_id"] = current_command_["current_job_id"];
            report["task_number"] = task_number;

            pubAcsReport(report.dump());
            doTask(current_pio_id , 0, 1);

            //std::cout << "task_result in a/s pass after send success : " << task_result_ << std::endl;
            count = 0;
            while(task_result_ != 1 && count < 1200 && check_error_ == false)
            {
                //std::cout << "wait pio result success" << std::endl;
                std::this_thread::sleep_for(50ms);
                count++;
            }
            if(task_result_ !=1 )
            {
                logWrite(LOG_ERR, "Task Error when AirShower Pass Finish");

                pubAlarm(AIRSHOWER_ALARM);   

                return;             
            }

            logWrite(LOG_INFO, "task_result in a/s pass : " + std::to_string(task_result_));

            //std::cout << "task_result in a/s pass after return success : " << task_result_ << std::endl;
            std::this_thread::sleep_for(20ms);
            //std::cout << "a/s pass done" << std::endl;
            logWrite(LOG_INFO, "airshowerPass Finished");
            current_command_["current_job"] = CMD_TASK_DONE;
        }
        else
        {
            log_string = "task_result_ : " + std::to_string(task_result_);
            logWrite(LOG_INFO, log_string); 
            logWrite(LOG_ERR, "Task Error when AirShower Pass");

            pubAlarm(AIRSHOWER_ALARM);
        }
    }
    else
    {
        log_string = "task_result_ : " + std::to_string(task_result_);
        logWrite(LOG_INFO, log_string); 
        logWrite(LOG_ERR, "Task Error when AirShower Pass");


        if(task_error_code_ == Error_Code::CANNOT_RECEIVE_READY)
        {
            pubAlarm(Error_Code::AS_EQUIPMENT_FAIL);            
        }
        else
        {
            pubAlarm(task_error_code_);
        }

        //std::cout << "Error when a/s pass" << std::endl;
    }
}

void Scenario::airshowerExit(std::string from_node, std::string to_node, int task_number)
{
    //std::cout << "a/s exit start" << std::endl;
    logWrite(LOG_INFO, "airshowerExit Start");

    int order = 0;
    //std::cout << "to_node : " << to_node << std::endl;
    if(to_node == "A201")
    {
        order = AIRSHOWER_2_2;
    }
    else if(to_node == "A203")
    {
        order = AIRSHOWER_2_1;
    }
    else if(to_node == "A103")
    {
        order = AIRSHOWER_1_1;
    }
    else if(to_node == "A101")
    {
        order = AIRSHOWER_1_2;
    }

    amr_status_json_["current_pio_id"] = order;
    
    int count = 0;
    while(count < 60 && task_feedback_ != order && check_error_ == false) //Send Message
    {
        doTask(order , 0);
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    std::string log_string = "task_result_ : " + std::to_string(task_result_);
    logWrite(LOG_INFO, log_string);
    
    logWrite(LOG_INFO, "Wait Door Open");
    count = 0;
    while(task_result_ != 3 && task_result_ != 2 && check_error_ == false)
    {
        //std::cout << "wait open door busy" << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    log_string = "task_result_ : " + std::to_string(task_result_);
    logWrite(LOG_INFO, log_string);
    
    logWrite(LOG_INFO, "Finish Door Open");

    if(task_result_ == 3)
    {
        logWrite(LOG_INFO, "from node : " + from_node);
        logWrite(LOG_INFO, "to node: " + to_node);

        //move
        json paths;
        paths["action"] = MOVE_WORK;
        paths["count"] = 1;

        json path;

        std::string tdriver_from_node = getTdriverNodeNumber(from_node);
        std::string tdriver_to_node = getTdriverNodeNumber(to_node);

        path["from_node"] = tdriver_from_node;
        path["to_node"] = tdriver_to_node;
        path["method"] = "forward";

        paths["path_list"].push_back(path);

        try
        {
            while(!cmd_client_->wait_for_service(1s))
            {            
                if(!rclcpp::ok())
                {
                    //std::cout << "Service Client cannot found server" << std::endl;
                    //Something Error
                    return;
                }
            }
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }

        auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();
        request->msg = paths.dump();

        auto result = cmd_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            //std::cout << "server result : " << result.get()->received << std::endl;

            std::this_thread::sleep_for(10ms); // Do not remove            
        }
        
        count = 0;
        int task_status =  amr_status_json_["task_status"].get<int>();
        while(count < 120 && task_status != TASK_STAUTS_RUNNING && check_error_ == false)
        {
            //std::cout << "wait agv moving start" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }

        logWrite(LOG_INFO, "Agv moving Start");
        int current_pio_id = amr_status_json_["current_pio_id"].get<int>();

        doTask(current_pio_id , 0, 3);
        std::this_thread::sleep_for(1000ms);
        
        while(count < 120 && task_status != TASK_STAUTS_COMPLETED && check_error_ == false)
        {
            //std::cout << "wait airshower exit success" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }
        logWrite(LOG_INFO, "Agv moving Finished");

        if(task_status == TASK_STAUTS_COMPLETED)
        {
            double current_lift_height = amr_status_json_["jack_height"].get<double>();
            if(current_lift_height < -0.085)
            {
                logWrite(LOG_INFO, "Lift Down Before AirshowerExit When No carrier -> height : lift_lower_position_");
                liftControl(lift_lower_position_);
            }

            doTask(current_pio_id , 0, 1);

            count = 0;
            while(count < 60 && task_result_ != 1 && check_error_ == false)
            {
                //std::cout << "wait open door busy" << std::endl;
                std::this_thread::sleep_for(1000ms);
                count++;
            }

            log_string = "task_result_ : " + std::to_string(task_result_);
            logWrite(LOG_INFO, log_string);
            
            if(task_result_ != 1)
            {
                logWrite(LOG_ERR, "Task Error when AirShower Pass");

                pubAlarm(AIRSHOWER_ALARM);

                return;
            }

            logWrite(LOG_INFO, "Finish Door Close");


            json report;
            report["type"] = "task";
            report["current_job_id"] = current_command_["current_job_id"];
            report["task_number"] = task_number;

            pubAcsReport(report.dump());
            //doTask(current_pio_id , 0, 1);

            current_command_["current_job"] = CMD_TASK_DONE;
            //std::cout << "a/s exit done" << std::endl; 
            setLight(LIGHT_GREEN);
            setSound(SOUND_NONE);

            logWrite(LOG_INFO, "airshowerExit Finished");
        }
        else
        {
            log_string = "task_result_ : " + std::to_string(task_result_);
            logWrite(LOG_INFO, log_string); 
            logWrite(LOG_ERR, "Task Error when AirShower Exit");

            pubAlarm(AIRSHOWER_ALARM);
        }
    }
    else
    {
        std::string log_string = "task_result_ : " + std::to_string(task_result_);
        logWrite(LOG_INFO, log_string); 
        logWrite(LOG_ERR, "Task Error when AirShower Exit");


        if(task_error_code_ == Error_Code::CANNOT_RECEIVE_READY)
        {
            pubAlarm(Error_Code::AS_EQUIPMENT_FAIL);            
        }
        else
        {
            pubAlarm(task_error_code_);
        }

        //std::cout << "Error when a/s exit" << std::endl;
    }
}

void Scenario::elevatorOpen(std::string node_name, int task_number)
{
    for(int i=1; i<=5; i++)
    {
        logWrite(LOG_INFO, "elevatorOpen Start : Current node -> " + node_name);
        
        setLight(LIGHT_GREEN);
        setSound(SOUND_WAIT_ELEVATOR);
        int order = 0;
        int count = 0;
        while(order ==0 && count < 60)
        {
            //std::cout << "node_name : " << node_name << std::endl;
            if(node_name == "E201")
            {
                order = ELEVATOR_2;
            }
            else if(node_name == "E101")
            {
                order = ELEVATOR_1;
            }
            node_name = getNodeName(amr_status_json_["node"].get<int>());
            std::this_thread::sleep_for(1000ms);
            count++;
        }
        amr_status_json_["state"] = OPENDOOR;

        int current_pio_id = amr_status_json_["current_pio_id"].get<int>();

        if(current_pio_id == order)
        {
            doTask(PIO_RESET_1 , 0);     
            std::this_thread::sleep_for(7000ms);   
        }

        amr_status_json_["current_pio_id"] = order;

        //std::cout << "start elevator open " <<  std::endl;
        logWrite(LOG_INFO, "elevatorOpen Check feedback");

        std::this_thread::sleep_for(1000ms);
        count = 0;
        while(count < 60 && task_feedback_ != order && check_error_ == false) //Send Message
        {
            //std::cout << "wait feedback == order" <<  std::endl;
            doTask(order , 0);
            std::this_thread::sleep_for(1000ms);
            count++;
        }

        logWrite(LOG_INFO, "elevatorOpen Check task_result_");
        count = 0;
        while(task_result_ != 3 && task_result_ != 2 && count <= 60 && check_error_ == false)
        {
            //std::cout << "wait elevator open busy : " << task_result_ << std::endl;
            std::this_thread::sleep_for(1000ms);
            count++;
        }
        
        if(task_result_ == 3) // ok
        {
            break;
        }
        else
        {
            doTask(PIO_RESET_1 , 0);
            std::this_thread::sleep_for(15000ms);
            amr_status_json_["current_pio_id"] = PIO_RESET_1;
            logWrite(LOG_INFO, "elevatorOpen retry count -> " + std::to_string(i));
        }
    }

    if(task_result_ == 3)
    {
        json report;
        report["type"] = "task";
        report["current_job_id"] = current_command_["current_job_id"];
        report["task_number"] = task_number;
        pubAcsReport(report.dump());

        current_command_["current_job"] = CMD_TASK_DONE;
        
        //std::cout << "open door done" <<  std::endl;
        logWrite(LOG_INFO, "elevatorOpen Finished");
    }
    else
    {
        logWrite(LOG_ERR, "elevatorOpen Error");
        

        if(task_error_code_ == Error_Code::CANNOT_RECEIVE_READY)
        {
            pubAlarm(Error_Code::EV_EQUIPMENT_FAIL);            
        }
        else
        {
            pubAlarm(task_error_code_);
        }
        
        //std::cout << "something error when open door" << std::endl;
        //SomthingError
    }
}

void Scenario::elevatorPass(std::string from_node, std::string to_node, int task_number, std::string target_map)
{
    //std::cout << "e/v pass start" << std::endl;
    //std::cout << "target_map : " << target_map << std::endl;

    logWrite(LOG_INFO, "elevatorPass Start");
    int count = 0;
    while(count < 60 && task_result_ != 3 && check_error_ == false)
    {
        //std::cout << "wait e/v pass busy : " << task_result_ <<std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    if(task_result_ == 3)
    {
        setLight(LIGHT_GREEN);
        setSound(SOUND_START_ENTER);
        //move
        json paths;
        paths["action"] = MOVE_WORK;
        paths["count"] = 1;

        logWrite(LOG_INFO, "from node : " + from_node);
        logWrite(LOG_INFO, "to node: " + to_node);

        json path;

        std::string tdriver_from_node = getTdriverNodeNumber(from_node);
        std::string tdriver_to_node = getTdriverNodeNumber(to_node);

        path["from_node"] = tdriver_from_node;
        path["to_node"] = tdriver_to_node;
        path["method"] = "forward";

        paths["path_list"].push_back(path);

        try
        {
            while(!cmd_client_->wait_for_service(1s))
            {            
                if(!rclcpp::ok())
                {
                    //std::cout << "Service Client cannot found server" << std::endl;
                    //Something Error
                    return;
                }
            }
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }

        auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();
        request->msg = paths.dump();

        auto result = cmd_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            //std::cout << "server result : " << result.get()->received << std::endl;

            std::this_thread::sleep_for(10ms); // Do not remove            
        }
        
        count = 0;
        int task_status =  amr_status_json_["task_status"].get<int>();
        while(count < 120 && task_status != TASK_STAUTS_RUNNING && check_error_ == false)
        {
            //std::cout << "wait agv moving start" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }
        logWrite(LOG_INFO, "Agv Moving Start");

        int current_pio_id = amr_status_json_["current_pio_id"].get<int>();
        //std::cout << "current_pio_id : " << current_pio_id << std::endl;

        doTask(current_pio_id , 0, 3);
        std::this_thread::sleep_for(1000ms);
        
        while(count < 120 && task_status != TASK_STAUTS_COMPLETED && check_error_ == false)
        {
            //std::cout << "wait agv moving finish" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }
        logWrite(LOG_INFO, "Agv Moving Finished");

        //std::cout << "task_result in e/v pass after moving : " << task_result_ << std::endl;

        if(task_status == TASK_STAUTS_COMPLETED)
        {
            //std::cout << "task_result in e/v pass after moving2 : " << task_result_ << std::endl;
            json report;
            report["type"] = "task";
            report["current_job_id"] = current_command_["current_job_id"];
            report["task_number"] = task_number;

            pubAcsReport(report.dump());
            doTask(current_pio_id , 0, 1);


            //std::cout << "task_result in e/v pass after send success : " << task_result_ << std::endl;
            count = 0;
            while(task_result_ != 1 && count < 1200 && check_error_ == false)
            {
                //std::cout << "wait pio result success" << std::endl;
                std::this_thread::sleep_for(50ms);
                count++;
            } 
            //std::cout << "task_result in e/v pass after return success : " << task_result_ << std::endl;
            
            if(task_result_ != 1)
            {
                logWrite(LOG_ERR, "elevatorPass Error");
                pubAlarm(ELEVATOR_ALARM);
                return;
                
            }
            setLight(LIGHT_GREEN);
            setSound(SOUND_NONE);

            logWrite(LOG_INFO, "Start Change_Map");
            //std::cout << "waiting change map -> " << target_map << std::endl;
            changeMap(from_node, target_map);
            //std::cout << "change map finished -> " << target_map << std::endl;
            logWrite(LOG_INFO, "Finish Change_Map");

            std::this_thread::sleep_for(20ms);
            //std::cout << "a/s pass done" << std::endl;
            current_command_["current_job"] = CMD_TASK_DONE;
            logWrite(LOG_INFO, "elevatorPass Finish");
        }
        else
        {
            logWrite(LOG_ERR, "elevatorPass Error");
            pubAlarm(ELEVATOR_ALARM);
        }
    }
    else
    {
        logWrite(LOG_ERR, "elevatorPass Error");
        
        if(task_error_code_ == Error_Code::CANNOT_RECEIVE_READY)
        {
            pubAlarm(Error_Code::EV_EQUIPMENT_FAIL);            
        }
        else
        {
            pubAlarm(task_error_code_);
        }

        //std::cout << "Error when a/s pass" << std::endl;
    }
}

void Scenario::elevatorExit(std::string from_node, std::string to_node, int task_number)
{
    //std::cout << "e/v exit start" << std::endl;
    logWrite(LOG_INFO, "elevatorExit Start");

    int order = 0;
    //std::cout << "to_node : " << to_node << std::endl;
    if(to_node == "E201")
    {
        order = ELEVATOR_2;
    }
    else if(to_node == "E101")
    {
        order = ELEVATOR_1;
    }

    amr_status_json_["current_pio_id"] = order;
    
    int count = 0;
    while(count < 60 && task_feedback_ != order && check_error_ == false) //Send Message
    {
        doTask(order , 0);
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    setLight(LIGHT_GREEN);

    if(order == ELEVATOR_1)
    {
        setSound(SOUND_FIRST_FLOOR_EXIT);
    }
    else
    {
        setSound(SOUND_SECOND_FLOOR_EXIT);
    }

    count = 0;
    while(task_result_ != 3 && task_result_ != 2 && check_error_ == false)
    {
        //std::cout << "wait e/v exit busy" << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    if(task_result_ == 3)
    {
        setSound(SOUND_START_EXIT);

        //move
        json paths;
        paths["action"] = MOVE_WORK;
        paths["count"] = 1;

        logWrite(LOG_INFO, "from node : " + from_node);
        logWrite(LOG_INFO, "to node: " + to_node);

        json path;

        std::string tdriver_from_node = getTdriverNodeNumber(from_node);
        std::string tdriver_to_node = getTdriverNodeNumber(to_node);

        path["from_node"] = tdriver_from_node;
        path["to_node"] = tdriver_to_node;
        path["method"] = "backward";

        paths["path_list"].push_back(path);

        try
        {
            while(!cmd_client_->wait_for_service(1s))
            {            
                if(!rclcpp::ok())
                {
                    //std::cout << "Service Client cannot found server" << std::endl;
                    //Something Error
                    return;
                }
            }
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }

        auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();
        request->msg = paths.dump();

        auto result = cmd_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            //std::cout << "server result : " << result.get()->received << std::endl;

            std::this_thread::sleep_for(10ms); // Do not remove            
        }
        
        count = 0;
        int task_status =  amr_status_json_["task_status"].get<int>();
        while(count < 120 && task_status != TASK_STAUTS_RUNNING && check_error_ == false)
        {
            //std::cout << "wait e/v exit success" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }

        int current_pio_id = amr_status_json_["current_pio_id"].get<int>();

        doTask(current_pio_id , 0, 3);
        std::this_thread::sleep_for(1000ms);
        
        while(count < 120 && task_status != TASK_STAUTS_COMPLETED && check_error_ == false)
        {
            //std::cout << "wait e/v exit success" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }

        if(task_status == TASK_STAUTS_COMPLETED)
        {
            doTask(current_pio_id , 0, 1);

            count = 0;
            while(count < 60 && task_result_ != 1 && check_error_ == false)
            {
                //std::cout << "wait open door busy" << std::endl;
                std::this_thread::sleep_for(1000ms);
                count++;
            }

            std::string log_string = "task_result_ : " + std::to_string(task_result_);
            logWrite(LOG_INFO, log_string);

            if(task_result_ != 1)
            {
                logWrite(LOG_ERR, "elevatorExit Error");
                pubAlarm(ELEVATOR_ALARM);
                return;                
            }
            
            logWrite(LOG_INFO, "Finish Door Close");

            json report;
            report["type"] = "task";
            report["current_job_id"] = current_command_["current_job_id"];
            report["task_number"] = task_number;

            pubAcsReport(report.dump());

            current_command_["current_job"] = CMD_TASK_DONE;
            //std::cout << "e/v exit done" << std::endl;
            logWrite(LOG_INFO, "elevatorExit Finish");
        }
        else
        {
            logWrite(LOG_ERR, "elevatorExit Error");
            pubAlarm(ELEVATOR_ALARM);
        }
    }
    else
    {
        logWrite(LOG_ERR, "elevatorExit Error");

        if(task_error_code_ == Error_Code::CANNOT_RECEIVE_READY)
        {
            pubAlarm(Error_Code::EV_EQUIPMENT_FAIL);            
        }
        else
        {
            pubAlarm(task_error_code_);
        }
        //std::cout << "Error when a/s exit" << std::endl;
    }
}

void Scenario::changeMap(std::string from_node, std::string target_map)
{
    auto msg = std_msgs::msg::String();
    if(from_node.substr(1,1) == "2")
    {
        msg.data = "1F";        
    }
    else if(from_node.substr(1,1) == "1")
    {
        msg.data = "2F";        
    }
    
    logWrite(LOG_INFO, "Change_Map Send to T-Driver -> Map Name : " + msg.data);

    map_change_pub_->publish(msg);

    //std::cout << "send change map to tdriver control : " << from_node.substr(1,1) << std::endl;


    int count = 0;

    int robot_localization_state =  amr_status_json_["reloc_status"].get<int>();
    while(count < 60 && robot_localization_state != LOCALIZATION_COMPLETED)
    {
        //std::cout << "wait change map start"<< std::endl;
        robot_localization_state = amr_status_json_["reloc_status"].get<int>();
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    count = 0;
    if(robot_localization_state == LOCALIZATION_COMPLETED)
    {
        auto msg2 = tc_msgs::msg::AmrInitStatus();
        msg2.init_status = CHANGE_MAP;
        amr_init_status_pub_->publish(msg2);

        while(count < 60 && robot_localization_state != LOCALIZATION_SUCCESS)
        {
            //std::cout << "wait change map success"<< std::endl;
            robot_localization_state = amr_status_json_["reloc_status"].get<int>();
            std::this_thread::sleep_for(1000ms);
            count++;
        }
    }
    else
    {
        //std::cout << "error when map change" << std::endl;
    }

    //std::cout << "change map finish" << std::endl;
    //wait loading map done
}

void Scenario::doTask(int order, int carrier_type, int result)
{
    auto amr_pio_task = tc_msgs::msg::AmrTask();
    amr_pio_task.order = order;
    amr_pio_task.carrier = carrier_type;
    amr_pio_task.result = result;

    task_pub_->publish(amr_pio_task);

    //std::cout << "pub do Task : " << order << std::endl;
}

void Scenario:: acquireOrDeposit(std::string node_name, int task_number, int task_type)
{
    logWrite(LOG_ERR, "acquireOrDeposit Start");
    std::this_thread::sleep_for(1000ms);
    int order = 0;
    int count = 0;

    while(order == 0 && count < 60)
    {
        if(node_name == "D101") // AQ
        {
            order = DOCKING_1;
        }
        else if(node_name == "D102") // DP
        {
            order = DOCKING_2;
        }
        else if(node_name == "D201")
        {
            order = DOCKING_3;
        }
        else if(node_name == "D202")
        {
            order = DOCKING_4;
        }
        node_name = getNodeName(amr_status_json_["node"].get<int>());
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    logWrite(LOG_ERR, "acquireOrDeposit Start");

    auto aruco_result = amrGetAruco();
    if(aruco_result == false)
    {
        pubAlarm(CART_ALARM);  
        return;
    }

    double current_lift_height = amr_status_json_["jack_height"].get<double>();
    if(current_lift_height > -0.1 && task_type == AQ)
    {
        logWrite(LOG_INFO, "Lift Down Before AQ When no carrier -> height : lift_lower_position_");
        liftControl(lift_lower_position_);
    }

    //std::cout << "start acquire : " << node_name << std::endl;
    count =0;
    while(count < 60 && task_feedback_ != order) //Send Message
    {
        //std::cout << "wait feedback == order" <<  std::endl;
        doTask(order , 0);
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    amr_status_json_["current_pio_id"] = order;
    amr_status_json_["state"] = DOCKING;

    count = 0;
    while(count < 60 && task_result_ != 3)
    {
        //std::cout << "wait acquire busy : " << task_result_ << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    if(task_result_ == 3)
    {
        std::string to_node_str = node_name.substr(1,3);
        int to_node = std::stoi(to_node_str);
        to_node+=5000;

        //move
        json paths;
        paths["action"] = MOVE_WORK;
        paths["count"] = 1;

        json path;

        std::string tdriver_from_node = getTdriverNodeNumber(node_name);
        std::string tdriver_to_node = std::to_string(to_node);

        path["from_node"] = tdriver_from_node;
        path["to_node"] = tdriver_to_node;
        path["method"] = "forward";

        paths["path_list"].push_back(path);

        try
        {
            while(!cmd_client_->wait_for_service(1s))
            {            
                if(!rclcpp::ok())
                {
                    //std::cout << "Service Client cannot found server" << std::endl;
                    //Something Error
                    return;
                }
            }
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }

        auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();
        request->msg = paths.dump();

        auto result = cmd_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            //std::cout << "server result : " << result.get()->received << std::endl;

            std::this_thread::sleep_for(10ms); // Do not remove            
        }
        
        setSound(SOUND_DOCKING);

        count = 0;
        int task_status =  amr_status_json_["task_status"].get<int>();
        while(count < 120 && task_status != TASK_STAUTS_RUNNING)
        {
            //std::cout << "wait agv moving start" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }

        int current_pio_id = amr_status_json_["current_pio_id"].get<int>();
        //std::cout << "current_pio_id : " << current_pio_id << std::endl;

        doTask(current_pio_id , 0, 3);
        std::this_thread::sleep_for(1000ms);
        
        while(count < 120 && task_status != TASK_STAUTS_COMPLETED)
        {
            //std::cout << "wait agv moving finish" << std::endl;
            std::this_thread::sleep_for(1000ms);
            task_status =  amr_status_json_["task_status"].get<int>();
            count++;                
        }

        if(task_status == TASK_STAUTS_COMPLETED)
        {
            setLight(LIGHT_YELLOW);
            setSound(SOUND_LOAD_UNLOAD);

            if(task_type == AQ)
            {
                amr_status_json_["state"] = ACQUIRING;
                liftControl(lift_upper_position_);   
                is_lift_up_ = true;             
            }
            else
            {
                amr_status_json_["state"] = DEPOSITING;
                liftControl(lift_lower_position_);   
                is_lift_up_ = false;              
            }
            
            amr_status_json_["state"] = UNDOCKING;

            std::this_thread::sleep_for(1500ms);

            json report;
            report["type"] = "task";
            report["current_job_id"] = current_command_["current_job_id"];
            report["task_number"] = task_number;

            pubAcsReport(report.dump());
            doTask(current_pio_id , 0, 1);

            count = 0;
            while(task_result_ != 1 && count < 1200)
            {
                //std::cout << "wait pio result success" << std::endl;
                std::this_thread::sleep_for(50ms);
                count++;
            }

            std::this_thread::sleep_for(20ms);
            current_command_["current_job"] = CMD_TASK_DONE;
            logWrite(LOG_ERR, "acquireOrDeposit Finish");
        }
        else
        {
            pubAlarm(CART_ALARM);   
        }
    }
    else
    {
        if(task_error_code_ == Error_Code::CANNOT_RECEIVE_READY)
        {
            pubAlarm(Error_Code::STATION_EQUIPMENT_FAIL);            
        }
        else
        {
            pubAlarm(task_error_code_);
        } 

        //std::cout << "Something Error in acquire" << std::endl;
    } 
}

void Scenario::liftControl(double height, bool stop)
{
    auto msg = tc_msgs::msg::AmrLiftMotion();
    msg.height = height;
    msg.stop = stop;

    amr_jack_pub_->publish(msg);

    int jack_state =  amr_status_json_["jack_state"].get<int>();

    std::this_thread::sleep_for(3000ms);

    int count = 0;
    while(jack_state == 4 && count < 10)
    {
        jack_state =  amr_status_json_["jack_state"].get<int>();
        //std::cout << "jack_state : " << jack_state << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    logWrite(LOG_INFO,"jack state in liftControl : " + std::to_string(jack_state));
    if(height > -0.1)
    {
        waitLoading();
    }
    else
    {
        waitUnLoading();
    }
}

void Scenario::waitLoading()
{
    int count = 0; 

    int jack_state =  amr_status_json_["jack_state"].get<int>();


    while(count < 30)
    {
        logWrite(LOG_INFO,"jack state in waitLoading : " + std::to_string(jack_state));
        if(jack_state == 4 || jack_state == 1)
        {
            break;
        }

        jack_state =  amr_status_json_["jack_state"].get<int>();
        //std::cout << "jack_state : " << jack_state << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }
}

void Scenario::waitUnLoading()
{
    int count = 0; 

    int jack_state =  amr_status_json_["jack_state"].get<int>();

    while(count < 30)
    {
        logWrite(LOG_INFO,"jack state in waitUnLoading : " + std::to_string(jack_state));
        if(jack_state == 3 || jack_state == 4)
        {
            break;
        }

        jack_state =  amr_status_json_["jack_state"].get<int>();
        //std::cout << "jack_state : " << jack_state << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }
}

void Scenario::charge(int task_number)
{
    logWrite(LOG_INFO, "start charge");
    int order = CHARGE_ON;
    
    //std::cout << "start charge " <<  std::endl;

    int count = 0;
    while(count < 60 && task_feedback_ != order) //Send Message
    {
        //std::cout << "wait feedback == order" <<  std::endl;
        doTask(order , 0);
        std::this_thread::sleep_for(1000ms);
        count++;
    }


    count = 0;
    while(count < 60 && task_result_ != 1)
    {
        //std::cout << "wait charge finished : " << task_result_ << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }

    if(task_result_ == 1)
    {
        have_command_ = false;
        amr_status_json_["current_pio_id"] = order;
        amr_status_json_["state"] = CHARGING;
        setLight(LIGHT_PURPLE);
        setSound(SOUND_NONE);
        current_command_["current_job"] = CMD_TASK_DONE;
        
        //TaskReport
        json report;
        report["type"] = "task";
        report["current_job_id"] = current_command_["current_job_id"];
        report["task_number"] = task_number;

        pubAcsReport(report.dump());

        //MoveCompleted
        json report2;
        report2["type"] = "move_complete";
        report2["current_job_id"] = current_command_["current_job_id"];
        report2["canceled_job_id"] = "";              
        report2["complete_type"] = 0;              

        pubAcsReport(report2.dump());      
        
        current_command_["current_job_id"] = "";   
        logWrite(LOG_INFO, "start charge finished"); 

        std::this_thread::sleep_for(3000ms);

        auto th = std::thread(&Scenario::chargeStop, this);
        std::this_thread::sleep_for(20ms);
        th.detach();
    }
    else
    {
        pubAlarm(3220);
        //Alarm Occur
    }
}

void Scenario::chargeStop()
{
    float bat_soc = amr_status_json_["battery_soc"].get<float>();
    std::string current_job_id = current_command_["current_job_id"];

    //std::cout << "start charge stop" << std::endl;

    logWrite(LOG_INFO, "wait bat_soc > 90.0 and have_command = true"); 
    while(bat_soc <= 90.0 && have_command_ == false)
    {
        std::this_thread::sleep_for(1000ms);
        bat_soc = amr_status_json_["battery_soc"].get<float>();
    }

    logWrite(LOG_INFO, "start charge stop"); 
    int order = CHARGE_OFF;
   
    int count = 0;
    while(count < 60 && task_feedback_ != order) //Send Message
    {
        //std::cout << "wait feedback == order" <<  std::endl;
        doTask(order , 0);
        std::this_thread::sleep_for(1000ms);
        count++;
    }
    amr_status_json_["current_pio_id"] = order;
    setLight(LIGHT_NONE);

    count = 0;
    while(count < 60 && task_result_ != 1)
    {
        //std::cout << "wait charge stop finished : " << task_result_ << std::endl;
        std::this_thread::sleep_for(1000ms);
        count++;
    }
    logWrite(LOG_INFO, "charge stop result :" + std::to_string(task_result_)); 

    //std::cout << "finish charge stop" << std::endl;

    if(task_result_ == 1)
    {
        logWrite(LOG_INFO, "finish charge stop");
        amr_status_json_["state"] = STOP_CHARGING;
    }
    else
    {
        logWrite(LOG_ERR, "error charge stop try again");
        doTask(PIO_RESET_1 , 0);
        std::this_thread::sleep_for(5000ms);

        count = 0;
        while(count < 60 && task_feedback_ != order) //Send Message
        {
            //std::cout << "wait feedback == order" <<  std::endl;
            doTask(order , 0);
            std::this_thread::sleep_for(1000ms);
            count++;
        }
        amr_status_json_["current_pio_id"] = order;
        setLight(LIGHT_NONE);

        count = 0;
        while(count < 60 && task_result_ != 1)
        {
            //std::cout << "wait charge stop finished : " << task_result_ << std::endl;
            std::this_thread::sleep_for(1000ms);
            count++;
        } 
        if(task_result_ == 1)
        {
            logWrite(LOG_INFO, "finish charge stop");
            amr_status_json_["state"] = STOP_CHARGING;
        }
        else
        {
            logWrite(LOG_ERR, "error charge stop 2 times");
            pubAlarm(3221);
        }
    }
}

void Scenario::receivePIOMessage(const tc_msgs::msg::AmrTask::SharedPtr msg)
{
    task_feedback_ = msg->feedback;
    task_result_ = msg->result;
    task_error_code_ = msg->error_code;

    //std::cout << "task_result_ : " << task_result_ << std::endl;
}

std::string Scenario::getNodeName(int node)
{
    std::string node_name;
    if(node == 0)
    {
        return "";
    }

    if(node > 4000)
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

void Scenario::pubAmrStatus()
{        
    amr_status_json_["current_job_id"] = current_command_["current_job_id"];
    auto acs_report_msg = std_msgs::msg::String();
    acs_report_msg.data = amr_status_json_.dump();
    acs_report_pub_->publish(acs_report_msg);
}

void Scenario::pubAcsReport(std::string data)
{
    auto msg = std_msgs::msg::String();
    msg.data = data;

    acs_task_pub_->publish(msg);
}

void Scenario::getAlarmMsg(const tc_msgs::msg::AmrAlarm::SharedPtr msg)
{    
    alarm_state_ = msg->state;
    if(msg->state == ERROR_OCCUR && check_error_ == false) // Error
    {
        logWrite(LOG_ERR, "Alarm Occur -> " + std::to_string(msg->code[0]));
        
        if(msg->code[0] == 3202)
        {    
            check_error_ = true;
            logWrite(LOG_ERR, "Bumper Alarm Occur" );
            setFailReport(ALARM, msg->code[0]);
            amrAction(MOVE_WORK_CANCEL);
            setSound(SOUND_BUMPER);

            doTask(PIO_STOP , 0);    
        }
        else if(msg->code[0] == 3301)
        {
            check_error_ = false;
            is_obstacle_sensing_ = true;
        }        
        else
        {
            check_error_ = true;
            setFailReport(ALARM, msg->code[0]);
            amrAction(MOVE_WORK_CANCEL);
            setSound(SOUND_ALARM);   
            doTask(PIO_STOP , 0);  
        }
        setLight(LIGHT_RED);
        ////std::cout << "error_code : " << msg->code[0] << std::endl;
    }
    else if(msg->state == ERROR_OCCUR)
    {
        if(msg->code[0] == 3202)
        {
            setSound(SOUND_BUMPER);
        }
        else
        {
            setSound(SOUND_ALARM); 
        }          
    }
    else if(msg->state == IN_RESET_SEQ && check_error_ == true)
    {
        amr_status_json_["pio_status"] = TASK_NONE;
        check_error_ = false;
        //std::cout << "in reset seq" << std::endl;
        setLight(LIGHT_WHITE);    
    }
    else if(msg->state == RESET_FAIL)
    {
        check_error_ = false;
    }
    else if(msg->state == RESET_DONE && check_reset_ == false) // Reset Done
    { 
        check_reset_ = true;
        if(is_pio_alarm_ == true)
        {
            is_pio_alarm_ = false;
            current_command_["current_job"] = CMD_TASK_FAIL;
        }

        while(task_queue_->count() > 0)
        {
            task_queue_->pop();
            logWrite(LOG_INFO, "Remove Task in reset seq");
        }

        logWrite(LOG_INFO, "tray_dectection_front : " + std::to_string(ezio_input_[IN_TRAY_FRONT_DETECTION].get<int>()));
        logWrite(LOG_INFO, "tray_dectection_rear : " + std::to_string(ezio_input_[IN_TRAY_REAR_DETECTION].get<int>()));

        if(ezio_input_[IN_TRAY_FRONT_DETECTION] == 1 && ezio_input_[IN_TRAY_REAR_DETECTION] == 1)
        {        
            logWrite(LOG_INFO, "lift up");
            liftControl(lift_upper_position_);
            std::this_thread::sleep_for(1500ms);
            logWrite(LOG_INFO, "tray_dectection_rear : " + std::to_string(ezio_input_[IN_TRAY_REAR_DETECTION].get<int>()));

            if(ezio_input_[IN_TRAY_REAR_DETECTION].get<int>() == 0)
            {
                std::this_thread::sleep_for(1500ms);
                logWrite(LOG_INFO, "lift down");
                liftControl(lift_lower_position_);
            }
        }
        else
        {
            logWrite(LOG_INFO, "lift down");
            liftControl(lift_lower_position_);
        }

        ////std::cout << "in reset done" << std::endl;
        check_error_ = false;       

        int mode = amr_status_json_["mode"].get<int>();
        if(mode == MANUAL_MODE)
        {
            setLight(LIGHT_WHITE);
        }
        else
        {
            setLight(LIGHT_BLUE);
        }
        setSound(SOUND_NONE); 
        
        setContinueReport();

        amr_status_json_["state"] = NOT_ASSIGNED;
        validation_->clearValidation();

        /*
        if(send_move_completed == false)
        {
            send_move_completed = true;
            json report;
            report["type"] = "move_complete";
            report["current_job_id"] = current_command_["current_job_id"];
            report["canceled_job_id"] = "";  
            report["complete_type"] = 3;       

            pubAcsReport(report.dump());  
        }
        */

        doTask(PIO_RESET_1,0);
        
        current_command_["current_job_id"] = "";

    }
    else if(msg->state == ERROR_NONE)
    {       
        if(is_obstacle_sensing_ == true)
            setContinueReport();

        ////std::cout << "error none" << std::endl;
        check_error_ = false;
        is_obstacle_sensing_ = false;
        check_reset_ = false;
       // send_move_completed = false;
    }
    else // Nothing 
    {
    }
}

void Scenario::amrAction(int action)
{
    auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();

    json command;
    command["action"] = action;        
    
    request->msg = command.dump();

    try
    {
        while (!cmd_client_->wait_for_service(1s))
        {
            //std::cout << "wait for service" << std::endl;

            if (!rclcpp::ok())
            {
                //std::cout << "Service Client cannot found server" << std::endl;
                // Something Error
                return;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    auto result = cmd_client_->async_send_request(request);
    //std::cout << "Wait for server response" << std::endl;
    if (rclcpp::spin_until_future_complete(acs_service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        //std::cout << "server result : " << result.get()->received << std::endl;

        std::this_thread::sleep_for(100ms); // Do not remove
    }
}

void Scenario::setContinueReport()
{
    json report;
    std::string job_id = current_command_["current_job_id"];
    if(job_id == "")
    {
        job_id = cur_job_id_;
    }
    report["type"] = "continue_report";
    report["current_job_id"] = job_id;
    pubAcsReport(report.dump());
    send_continue_report_ = true;  
}

void Scenario::setFailReport(int level, int stop_reason)
{
    logWrite(LOG_ERR, "Send fail report Start -> " + std::to_string(stop_reason));
    json report;
    std::string job_id = current_command_["current_job_id"];
    if(job_id == "")
    {
        job_id = cur_job_id_;
    }
    report["type"] = "fail_report";
    report["current_job_id"] = job_id;
    report["level"] = level;
    report["stop_reason"] = stop_reason;
    pubAcsReport(report.dump());    
    amr_status_json_["state"] = AMR_ALARM; 
}

void Scenario::pubAlarm(int alarm_code)
{
    auto msg = std_msgs::msg::Int32();
    msg.data = alarm_code;
    tcon_alarm_pub_->publish(msg);

    is_pio_alarm_ = true;
}

void Scenario::setLight(int light)
{
    if(init_done_ == false)
    {
        light = LIGHT_NONE;
    }

    auto msg = std_msgs::msg::Int32();
    msg.data = light;
    light_pub_->publish(msg);
}

void Scenario::setSound(int sound)
{
    auto msg = std_msgs::msg::Int32();
    msg.data = sound;
    sound_pub_->publish(msg);
}

void Scenario::getEzioStatus(const std_msgs::msg::String::SharedPtr msg)
{
    std::string ezio_msg = msg->data;

    json j = json::parse(msg->data);

    ezio_input_ = j["DI"];
    ezio_output_ = j["DO"];
    
    checkKeyState();
    checkBuzzerStop();
    checkLiftButton();
}

void Scenario::checkKeyState()
{
    if(init_done_ == true)
    {
        if(ezio_input_[IN_MANUAL_KEY] == 1 && ezio_input_[IN_AUTO_KEY] == 0)
        {        
            amr_status_json_["mode"] = MANUAL_MODE;
            amr_status_json_["state"] = MANUAL;
        }
        else if(ezio_input_[IN_MANUAL_KEY] == 0 && ezio_input_[IN_AUTO_KEY] == 1)
        {
            amr_status_json_["mode"] = AUTO_MODE;
        }
    }
    else
    {
        amr_status_json_["mode"] = -1;    
    }
}

void Scenario::checkAgvMode()
{    
    if(init_done_ == true)
    {
        int amr_mode = amr_status_json_["mode"].get<int>();

        if(amr_mode == mode_)
        {
            //do nothing
        }
        else if(amr_mode == AUTO_MODE)
        {
            mode_ = amr_status_json_["mode"].get<int>();
            setContinueReport();
            
            json report;
            report["type"] = "status_report";
            pubAcsReport(report.dump());    
            setLight(LIGHT_BLUE);
        }
        else if(amr_mode == MANUAL_MODE)
        {
            //doTask(PIO_RESET_1 , 0);

            while(task_queue_->count() > 0)
            {
                task_queue_->pop();
            }

            amrAction(MOVE_WORK_CANCEL);


            setSound(SOUND_NONE); 
            setLight(LIGHT_WHITE);
            mode_ = amr_status_json_["mode"].get<int>();
        }
    }
}

void Scenario::checkBuzzerStop()
{
    if(ezio_input_[IN_BUZZER_STOP] == 1)
    {
        setSound(SOUND_NONE);
    }
}

void Scenario::checkLiftButton()
{
    if(ezio_input_[IN_LIFT_UP] == 1)
    {
        auto msg = tc_msgs::msg::AmrLiftMotion();
        msg.height = lift_upper_position_;
        msg.stop = false;

        amr_jack_pub_->publish(msg);
    }
    else if(ezio_input_[IN_LIFT_DOWN] == 1)
    {
        auto msg = tc_msgs::msg::AmrLiftMotion();
        msg.height = lift_lower_position_;
        msg.stop = false;

        amr_jack_pub_->publish(msg);
    }
}

void Scenario::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_SCENARIO, level, msg);
}