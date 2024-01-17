#include "acs_communication/acs_communication.hpp"

AcsCommunication::AcsCommunication() : Node("acs_communication")
{
    initVariables();
    initPubSub();
    initTimer();

    // std::cout << "init acs communication done" << std::endl;
}

AcsCommunication::~AcsCommunication()
{
    quit_ = true;
}

void AcsCommunication::initVariables()
{
    quit_ = false;
    check_init_ = false;
    send_init_ = false;
    is_send_fail_report_ = false;
    receive_queue_ = std::make_shared<ThreadSafeQueue<char *>>();
    file_io_ = std::make_shared<File_IO>();

    // Init amr status for report
    amr_status_["xpos"] = 0.0;
    amr_status_["ypos"] = 0.0;
    amr_status_["map_name"] = "2F";
    amr_status_["angle"] = 0.0;
    amr_status_["battery"] = 0.0;
    amr_status_["current_node"] = "";
    amr_status_["next_node"] = "";
    amr_status_["lift_state"] = LIFT_DOWN;
    amr_status_["num_of_carrier"] = 0;
    amr_status_["carrier_list"];
    amr_status_["velocity"] = 0.0;
    amr_status_["agv_state"] = INIT;

    // carrier_info_
    carrier_info_["slot_number"] = 0;
    carrier_info_["mat_id"] = "";
    carrier_info_["mat_direction"] = 0.0;
    carrier_info_["mat_weight"] = 0;

    logWrite(LOG_INFO, "Init variables");
}

void AcsCommunication::initPubSub()
{
    service_node_ = rclcpp::Node::make_shared("acs_service_node");

    acs_command_client_ = service_node_->create_client<tc_acs_interface::srv::AcsCommand>("acs_command");

    acs_conn_pub_ = this->create_publisher<std_msgs::msg::Bool>("acs_connection", 10);

    acs_report_sub_ = this->create_subscription<std_msgs::msg::String>("amr_report", 10, std::bind(&AcsCommunication::getAmrStatus, this, std::placeholders::_1));

    acs_task_sub_ = this->create_subscription<std_msgs::msg::String>("acs_task_report", 10, std::bind(&AcsCommunication::getAmrTaskStatus, this, std::placeholders::_1));

    t_driver_init_sub_ = this->create_subscription<std_msgs::msg::Int32>("t_driver_init_status", 10, std::bind(&AcsCommunication::tDriverInitCallBack, this, std::placeholders::_1));

    logWrite(LOG_INFO, "Init publisher and subscriber");
}

void AcsCommunication::initTimer()
{
    get_server_data_timer_ = this->create_wall_timer(50ms, std::bind(&AcsCommunication::getServerMessage, this));
    set_server_data_timer_ = this->create_wall_timer(10ms, std::bind(&AcsCommunication::setServerMessage, this));
    get_agvc_status_timer_ = this->create_wall_timer(100ms, std::bind(&AcsCommunication::getAcsStatus, this));

    logWrite(LOG_INFO, "Init timer");
}

void AcsCommunication::tDriverInitCallBack(const std_msgs::msg::Int32::SharedPtr msg)
{
    int init_state = msg->data;
    if (check_init_ == true)
    {
    }
    else if (init_state == INIT_DONE)
    {
        logWrite(LOG_INFO, "TDriver init Done");
        // std::cout << "init done" << std::endl;
        check_init_ = true;
        connection_thread_ = std::thread(&AcsCommunication::startServer, this);
        connection_thread2_ = std::thread(&AcsCommunication::startMonitorServer, this);
    }
}

void AcsCommunication::getAmrTaskStatus(const std_msgs::msg::String::SharedPtr msg)
{
    json report = json::parse(msg->data);

    std::string type = report["type"];
    if (type == "task")
    {
        std::string job_id = report["current_job_id"];
        char task_number = (char)report["task_number"].get<int>();

        taskReport(job_id, task_number);
    }
    else if (type == "move_complete")
    {
        std::string job_id = report["current_job_id"];
        std::string canceled_job_id = report["canceled_job_id"];
        int complete_type = report["complete_type"].get<int>();

        // std::cout << "job_id" << job_id << std::endl;
        // std::cout << "canceled_job_id" << canceled_job_id << std::endl;
        moveCompleted(job_id, canceled_job_id, complete_type);
    }
    else if (type == "fail_report")
    {
        if (is_send_fail_report_ == false)
        {
            is_send_fail_report_ = true;
            std::string job_id = report["current_job_id"];
            int level = report["level"].get<int>();
            int reason = report["stop_reason"].get<int>();
            failReport(job_id, level, reason);
        }
    }
    else if (type == "continue_report")
    {
        if (is_send_fail_report_ == true)
        {
            is_send_fail_report_ = false;
            std::string job_id = report["current_job_id"];
            continueReport(job_id);
        }
    }
    else if (type == "status_report")
    {
        statusReport();
    }
}

void AcsCommunication::getAmrStatus(const std_msgs::msg::String::SharedPtr msg)
{
    json amr_status = json::parse(msg->data);

    std::string last_node = amr_status_["current_node"];
    std::string current_node = amr_status["node"];
    std::string next_node = amr_status["next_node"];

    int agv_state = amr_status["state"].get<int>();
    int agv_last_state = amr_status_["agv_state"].get<int>();

    bool need_status_report = false;
    bool need_state_report = false;

    double x = amr_status["x"].get<double>() * 1000;
    double y = amr_status["y"].get<double>() * 1000;
    int angle = amr_status["angle"].get<int>();
    auto tdriver_node = getTdriverNodeNumber(current_node);

    if (check_init_ == true)
    {
        if (last_node != current_node)
        {
            file_io_->writeCurrentPosFile(tdriver_node, x, y, angle);
            need_status_report = true;
        }

        if (agv_state != agv_last_state)
        {
            file_io_->writeCurrentPosFile(tdriver_node, x, y, angle);
            need_state_report = true;
        }
    }

    amr_status_["xpos"] = x;
    amr_status_["ypos"] = y;
    amr_status_["map_name"] = amr_status["map_name"];
    amr_status_["angle"] = angle;
    amr_status_["velocity"] = amr_status["velocity"];
    amr_status_["battery"] = amr_status["bat_soc"];
    amr_status_["current_node"] = amr_status["node"];
    amr_status_["next_node"] = amr_status["next_node"];
    amr_status_["agv_state"] = amr_status["state"];

    double lift_height = amr_status["lift_height"].get<double>();

    if (lift_height > -0.085)
    {
        amr_status_["lift_state"] = LIFT_UP;
    }
    else
    {
        amr_status_["lift_state"] = LIFT_DOWN;
    }

    int state = amr_status_["agv_state"].get<int>();

    if (state != NULL && state == MANUAL)
    {
        need_status_report = false;
        // need_state_report = false;
    }

    if (need_status_report)
    {
        // file_io_->writeCurrentPosFile(tdriver_node, x, y, angle);
        statusReport();
    }

    if (need_state_report)
    {
        // file_io_->writeCurrentPosFile(tdriver_node, x, y, angle);
        stateChanged();
    }
}

int AcsCommunication::getTdriverNodeNumber(std::string node)
{
    if (node == "")
    {
        return 0;
    }
    int node_number = std::stoi(node.substr(1, 3));
    std::string node_header = node.substr(0, 1);

    if (node_header == "D")
    {
        node_number += 1000;
    }
    else if (node_header == "A")
    {
        node_number += 2000;
    }
    else if (node_header == "E")
    {
        node_number += 3000;
    }
    else if (node_header == "C")
    {
        node_number += 4000;
    }

    return node_number;
}

void AcsCommunication::startServer()
{
    try
    {
        boost::asio::io_context io_context;
        // short port = 7080;
        short port = 7780;
        server_ = new AsyncTcpServer(io_context, port, receive_queue_);
        logWrite(LOG_INFO, "Make AsyncTcpServer Instance");
        server_->startAccept();
        logWrite(LOG_INFO, "Acs server start listening");

        while (!quit_)
        {
            io_context.poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        logWrite(LOG_INFO, "Acs server stop listening");
    }
    catch (const std::exception &e)
    {
        logWrite(LOG_INFO, "Acs server error");
        logWrite(LOG_ERR, e.what());
        server_->stopServer();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        delete server_;
        server_ = nullptr;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void AcsCommunication::startMonitorServer()
{
    try
    {
        boost::asio::io_context io_context;
        // short port = 7080;
        short port = 7777;
        monitor_server_ = new AsyncTcpServer(io_context, port, monitor_receive_queue_);
        logWrite(LOG_INFO, "Make AsyncTcpServer Instance");
        monitor_server_->startAccept();
        logWrite(LOG_INFO, "Acs monitor server start listening");

        while (!quit_)
        {
            io_context.poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        logWrite(LOG_INFO, "Acs monitor server stop listening");
    }
    catch (const std::exception &e)
    {
        logWrite(LOG_INFO, "Acs monitor server error");
        logWrite(LOG_ERR, e.what());
        monitor_server_->stopServer();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        delete monitor_server_;
        monitor_server_ = nullptr;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void AcsCommunication::getServerMessage()
{
    if (receive_queue_->count() > 0)
    {
        char *data = receive_queue_->front();

        bool is_command = true;
        int index = 0;

        while (is_command)
        {
            char stx = data[index];
            if (stx == 0x02)
            {
                char length[4] = {};
                for (int i = 0; i < 4; i++)
                {
                    length[i] = data[i + index + 4];
                }
                int data_length = Converter::bufferToIntLittleEndian(length);

                int size = data_length + 8;
                std::vector<char> acs_data;
                acs_data.insert(acs_data.end(), data + index, data + index + size);

                char *acs_data_str = acs_data.data();

                parsingServerMessage(acs_data_str);

                auto message_type = static_cast<unsigned char>(data[2 + index]);

                if (message_type != AcsCommand::ping && message_type != AgvReportResponse::keep_alive_res)
                {
                    std::string acs_log = "";
                    for (int i = 0; i < size; i++)
                    {
                        acs_log += (int)data[i];
                        acs_log += " ";
                    }

                    logDebug(acs_log, "acs_send");
                }
                ////std::cout << "origin seq : " <<  seq << std::endl;
                index += size;
                // //std::cout << std::endl;
            }
            else
            {
                is_command = false;
            }
        }
        receive_queue_->pop();
        delete data;
    }
}

void AcsCommunication::parsingServerMessage(char *data)
{
    std::thread th([this, &data]
                   {
        auto message_type = static_cast<unsigned char>(data[2]);
        int seq = static_cast<unsigned char>(data[3]);

        if(message_type != AcsCommand::ping && message_type != AgvReportResponse::keep_alive_res)
        {
            logWrite(LOG_INFO, "Receive Type : " + std::to_string(message_type));
        }

        switch (message_type)
        {
            case AcsCommand::ping:
                ////std::cout <<"[" << (int)seq << "] ping res" << std::endl;
                pingResponse(data);
                ////std::cout <<"[" << (int)seq << "] ping res finished" << std::endl;
                break;
            case AcsCommand::set_time:
                //std::cout << "setTime res" << std::endl;
                setTime(data);
                //std::cout << "setTime res finished" << std::endl;
                break;
            case AcsCommand::move_work:
                //std::cout <<"[" << (int)seq << "] moveWork res" << std::endl;
                moveWork(data);
                //std::cout <<"[" << (int)seq << "] moveWork res finished" << std::endl;
                break;
            case AcsCommand::move_work_cancel:
                //std::cout <<"[" << (int)seq << "] moveWorkCancel res" << std::endl;
                moveWorkCancel(data);
                //std::cout <<"[" << (int)seq << "] moveWorkCancel res finished" << std::endl;
                break;
            case AcsCommand::amr_pause:
                //std::cout << "amr_pause res" << std::endl;
                pauseAmr(data);
                //std::cout << "amr_pause res finished" << std::endl;
                break;
            case AcsCommand::alarm_on_off:
                //std::cout << "alarm_on_off res" << std::endl;
                alarmAmr(data);
                //std::cout << "alarm_on_off res finished" << std::endl;
                break;
            case AcsCommand::scan:
                //std::cout <<"[" << (int)seq << "] scan res" << std::endl;
                scan(data);
                //std::cout <<"[" << (int)seq << "] scan res finished" << std::endl;
                break;
            case AcsCommand::stop_charge:
                //std::cout << "stopCharge res" << std::endl;
                stopCharge(data);
                //std::cout << "stopCharge res finished" << std::endl;
                break;
            case AcsCommand::check_map:
                //std::cout << "checkMap res" << std::endl;
                checkMap(data);
                //std::cout << "checkMap res finished" << std::endl;
                break;
            case AcsCommand::fetch_map:
                //std::cout << "fetchMap res" << std::endl;
                fetchMap(data);
                //std::cout << "fetchMap res finished" << std::endl;
                break;
            case AcsCommand::req_map_sync:
                //std::cout << "req_map_sync res" << std::endl;
                reqeustMapSync(data);
                //std::cout << "req_map_sync res finished" << std::endl;
                break;
            case AcsCommand::update_map:
                //std::cout << "updateMap res" << std::endl;
                updateMap(data);
                //std::cout << "updateMap res finished" << std::endl;
                break;

            case AgvReportResponse::init_report_res:
                //std::cout << "init_report_res" << std::endl;
                initReportResponse(data);
                //std::cout << "init_report_res finished" << std::endl;
                break;
            case AgvReportResponse::keep_alive_res:
                ////std::cout << "keep_alive_res" << std::endl;            
                ////std::cout << "keep_alive_res finished" << std::endl;
                break;
            case AgvReportResponse::status_report_res:
                //std::cout << "status_report_res" << std::endl;            
                //std::cout << "status_report_res finished" << std::endl;
                break;
            case AgvReportResponse::move_completed_res:
                //std::cout << "move_completed_res" << std::endl;            
                //std::cout << "move_completed_res finished" << std::endl;
                break;
            case AgvReportResponse::fail_report_res:
                //std::cout << "fail_report_res" << std::endl;            
                //std::cout << "fail_report_res finished" << std::endl;
                break;
            case AgvReportResponse::continue_report_res:
                //std::cout << "continue_report_res" << std::endl;            
                //std::cout << "continue_report_res finished" << std::endl;
                break;
            case AgvReportResponse::state_changed_res:
                //std::cout << "state_changed_res" << std::endl;            
                //std::cout << "state_changed_res finished" << std::endl;
                break;
            case AgvReportResponse::task_report_res:
                //std::cout << "task_report_res" << std::endl;            
                //std::cout << "task_report_res finished" << std::endl;
                break;
            default:
                break;
        } });

    std::this_thread::sleep_for(30ms);
    th.detach();
}

void AcsCommunication::pingResponse(char *data)
{
    try
    {
        auto logger = AmrLogger::getInstance();

        int seq = static_cast<unsigned char>(data[3]);

        std::string seq_str = std::to_string(seq);
        std::string log_string = "[" + seq_str + "] Receive Ping req";
        logWrite(LOG_INFO, log_string);

        int data_size = 0;

        char length[4] = {};
        for (int i = 0; i < 4; i++)
        {
            length[i] = data[i + 4];
        }

        int data_length = Converter::bufferToIntLittleEndian(length);

        auto report = AmrReport();
        report.stx_ = stx_;
        report.ver_ = data[1];
        report.msg_type_ = AcsCommandResponse::ping_res;
        report.seq_ = data[3];
        report.data_size_ = data_length;

        data_size = data_length + 4;

        std::vector<char> response;

        const char *report_ptr = reinterpret_cast<const char *>(&report);
        response.insert(response.end(), report_ptr, report_ptr + sizeof(AmrReport));

        for (int i = 0; i < data_length; i++)
        {
            response.push_back(data[i + 8]);
            data_size++;
        }

        char *cstr = response.data();
        size_t cstr_length = static_cast<size_t>(data_size);
        std::string res(cstr, cstr_length);
        send_queue_.push(res);
        // server_->sendData(cstr, cstr_length);
        log_string = "[" + seq_str + "] Send Ping req";
        logWrite(LOG_INFO, log_string);
    }
    catch (const std::exception &e)
    {
        logWrite(LOG_ERR, "Error Occur when send ping resp");
        logWrite(LOG_ERR, e.what());
    }
    //
}

void AcsCommunication::setTime(char *data)
{
    logWrite(LOG_INFO, "Receive SetTime req");

    /*
        SetTime 구현 해야함
    */

    commandResponse(AcsCommandResponse::set_time_res, data);
}

void AcsCommunication::moveWork(char *data)
{
    try
    {
        /*
        data_length += 8;

        std::string log_debug = "";
        for(int i=0; i<data_length; i++)
        {
            int a = (int)data[i];
            log_debug += std::to_string(a);
            log_debug += " ";
        }

        logDebug(log_debug,"MoveWork");
        */

        int state = amr_status_["agv_state"].get<int>();

        if (state == ARRIVAL || state == OPENDOOR)
        {
            commandResponse(AcsCommandResponse::move_work_res, data, REJECTED);
            return;
        }

        int data_index = 8;
        logWrite(LOG_INFO, "Receive MoveWork req");

        // std::cout << "move work data len : " << data_length << std::endl;

        char length[4] = {};
        for (int i = 0; i < 4; i++)
        {
            length[i] = data[i + 4];
        }

        int data_length = Converter::bufferToIntLittleEndian(length);

        // job id len
        data_index++;
        auto job_id_len = Converter::twoBytesToIntLittleEndian(data[data_index - 1], data[data_index]);
        // std::cout << "job id len : " << job_id_len << std::endl;
        data_index++;

        // job id
        auto job_id = byteToString(data, data_index, job_id_len);
        // std::cout << "job id : " << job_id << std::endl;
        data_index += job_id_len;

        // change route
        auto change_route = data[data_index];
        // std::cout << "change route : " << change_route << std::endl;
        data_index++;

        // final goal len
        data_index++;
        auto final_goal_len = Converter::twoBytesToIntLittleEndian(data[data_index - 1], data[data_index]);
        // std::cout << "final_goal_len : " << final_goal_len << std::endl;
        data_index++;

        // job final goal
        auto final_goal = byteToString(data, data_index, final_goal_len);
        // std::cout << "final_goal : " << final_goal << std::endl;
        data_index += final_goal_len;

        // path count
        auto path_count = (int)data[data_index];
        // std::cout << "path_count : " << path_count << std::endl;
        data_index++;

        // path list
        auto path_list = getPathList(data, path_count, &data_index);

        // operation target
        auto operation_target = (int)data[data_index];
        // std::cout << "operation_target : " << operation_target << std::endl;
        data_index++;

        // target name len
        data_index++;
        auto target_name_len = Converter::twoBytesToIntLittleEndian(data[data_index - 1], data[data_index]);
        // std::cout << "target_name_len : " << target_name_len << std::endl;
        data_index++;

        // target name
        auto target_name = byteToString(data, data_index, target_name_len);
        // std::cout << "target_name : " << target_name << std::endl;
        data_index += target_name_len;

        // task count
        auto task_count = (int)data[data_index];
        // std::cout << "task_count : " << task_count << std::endl;
        data_index++;

        // task list
        auto task_list = getTaskList(data, task_count, &data_index);

        json move_work;
        move_work["type"] = MOVE_WORK;
        move_work["job_id"] = job_id;
        move_work["change_route"] = change_route;
        move_work["final_goal"] = final_goal;
        move_work["path_count"] = path_count;
        move_work["path_list"] = path_list;
        move_work["operation_target"] = operation_target;
        move_work["target_name"] = target_name;
        move_work["task_count"] = task_count;
        move_work["task_list"] = task_list;

        commandResponse(AcsCommandResponse::move_work_res, data);

        sendToAcsCommand(move_work);
        logWrite(LOG_INFO, move_work.dump());
    }
    catch (const std::exception &e)
    {
        commandResponse(AcsCommandResponse::move_work_res, data, REJECTED);
        logWrite(LOG_ERR, "Error Occur when receive move work");
        logWrite(LOG_ERR, e.what());
    }
}

void AcsCommunication::moveWorkCancel(char *data)
{
    std::string current_node = amr_status_["current_node"];
    if (current_node != "")
    {
        auto tdriver_node = getTdriverNodeNumber(current_node);
        if (tdriver_node >= 1000)
        {
            logWrite(LOG_INFO, "Reject MoveWorkCancel -> Tdriver Node : " + std::to_string(tdriver_node));
            commandResponse(AcsCommandResponse::move_work_cancel_res, data, REJECTED);
            return;
        }
    }

    commandResponse(AcsCommandResponse::move_work_cancel_res, data);

    int data_index = 8;
    logWrite(LOG_INFO, "Receive MoveWorkCancel req");

    char length[4] = {};
    for (int i = 0; i < 4; i++)
    {
        length[i] = data[i + 4];
    }

    int data_length = Converter::bufferToIntLittleEndian(length);
    // std::cout << "move work cancel data len : " << data_length << std::endl;

    // job id len
    data_index++;
    auto job_id_len = Converter::twoBytesToIntLittleEndian(data[data_index - 1], data[data_index]);
    // std::cout << "job id len : " << job_id_len << std::endl;
    data_index++;

    // job id
    auto job_id = byteToString(data, data_index, job_id_len);
    // std::cout << "job id : " << job_id << std::endl;
    data_index += job_id_len;

    // job id to cancel len
    data_index++;
    auto job_id_to_cancel_len = Converter::twoBytesToIntLittleEndian(data[data_index - 1], data[data_index]);
    // std::cout << "job_id_to_cancel_len : " << job_id_to_cancel_len << std::endl;
    data_index++;

    // job id to cancel
    auto job_id_to_cancel = byteToString(data, data_index, job_id_to_cancel_len);
    // std::cout << "job_id_to_cancel : " << job_id_to_cancel << std::endl;
    data_index += job_id_to_cancel_len;

    auto cancel_option = (int)data[data_index];
    // std::cout << "cancel_option : " << cancel_option << std::endl;

    json move_work_cancel;
    move_work_cancel["type"] = MOVE_WORK_CANCEL;
    move_work_cancel["job_id"] = job_id;
    move_work_cancel["job_id_to_cancel"] = job_id_to_cancel;
    move_work_cancel["cancel_option"] = cancel_option;

    sendToAcsCommand(move_work_cancel);

    logWrite(LOG_INFO, move_work_cancel.dump());

    std::string log_debug = "";
    for (int i = 0; i < data_index; i++)
    {
        int a = (int)data[i];
        log_debug += std::to_string(a);
        log_debug += " ";
    }

    logDebug(log_debug, "MoveWorkCancel");
}

void AcsCommunication::pauseAmr(char *data)
{
    int data_index = 8;
    logWrite(LOG_INFO, "Receive PauseAmr req");

    char length[4] = {};
    for (int i = 0; i < 4; i++)
    {
        length[i] = data[i + 4];
    }

    int data_length = Converter::bufferToIntLittleEndian(length);
    // std::cout << "pause amr data len : " << data_length << std::endl;

    // job id len
    data_index++;
    auto at_len = Converter::twoBytesToIntLittleEndian(data[data_index - 1], data[data_index]);
    // std::cout << "at_len : " << at_len << std::endl;
    data_index++;

    // job id
    auto at = byteToString(data, data_index, at_len);
    // std::cout << "at : " << at << std::endl;
    data_index += at_len;

    json pause_amr;
    pause_amr["type"] = PAUSE_AMR;
    pause_amr["at"] = at;

    sendToAcsCommand(pause_amr);

    commandResponse(AcsCommandResponse::pause_res, data);
    logWrite(LOG_INFO, pause_amr.dump());
}

void AcsCommunication::checkMap(char *data)
{
    logWrite(LOG_INFO, "Receive CheckMap req");
    std::vector<char> map;
    std::vector<char> len;
    auto map_info = json::parse(file_io_->getMapInfo());
    int map_count = map_info["map_count"].get<int>();
    Converter::intTo4BytesLittleEndian(map_count, len);

    int map_size = 0;
    for (int i = 0; i < 4; i++)
    {
        map.push_back(len[i]);
        map_size++;
    }

    // std::cout << "map_count : " << map_count << std::endl;
    for (const auto &item : map_info.items())
    {
        std::string key = item.key();

        if (key == "map_count")
        {
            continue;
        }

        std::string map_name = item.value()["map_name"];
        // std::cout << "map_name : " << map_name << std::endl;

        int map_name_len = map_name.length();
        Converter::intTo2BytesLittleEndian(map_name_len, len);
        map.push_back(len[0]);
        map.push_back(len[1]);
        map_size += 2;

        for (int i = 0; i < map_name_len; i++)
        {
            map.push_back(map_name[i]);
            map_size++;
        }

        std::string map_version = item.value()["map_version"];
        // std::cout << "map_version : " << map_version << std::endl;

        int map_version_len = map_version.length();
        Converter::intTo2BytesLittleEndian(map_version_len, len);
        map.push_back(len[0]);
        map.push_back(len[1]);
        map_size += 2;

        for (int i = 0; i < map_version_len; i++)
        {
            map.push_back(map_version[i]);
            map_size++;
        }
    }
    // std::cout << "map_size : " << map_size << std::endl;

    // std::cout << "map parse done" << std::endl;

    // std::cout << "map_info : " << map_info << std::endl;
    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = data[1];
    report.msg_type_ = AcsCommandResponse::check_map_res;
    report.seq_ = data[3];
    report.data_size_ = 5 + map_size;

    // std::cout << "report.data_size : " << report.data_size_ << std::endl;

    std::vector<char> response;
    // std::cout << "response.size 1 : " << response.size() << std::endl;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    response.insert(response.end(), report_ptr, report_ptr + sizeof(AmrReport));
    // std::cout << "response.size 2 : " << response.size() << std::endl;

    response.push_back(ACCEPT);
    response.push_back(0);
    response.push_back(0);
    response.push_back(0);
    response.push_back(0);
    // std::cout << "response.size 3 : " << response.size() << std::endl;

    response.insert(response.end(), map.begin(), map.end());

    char *cstr = response.data();
    size_t cstr_length = static_cast<size_t>(response.size());
    // std::cout << "response.size 4 : " << response.size() << std::endl;
    std::string res(cstr, cstr_length);

    send_queue_.push(res);

    // commandResponse(AcsCommandResponse::check_map_res, data);
}

void AcsCommunication::reqeustMapSync(char *data)
{
    commandResponse(AcsCommandResponse::req_map_sync_res, data);
}

void AcsCommunication::updateMap(char *data)
{
    commandResponse(AcsCommandResponse::update_map_res, data);
}

void AcsCommunication::fetchMap(char *data)
{
    commandResponse(AcsCommandResponse::fetch_map_res, data);
}

void AcsCommunication::stopCharge(char *data)
{
    int data_index = 8;
    logWrite(LOG_INFO, "Receive StopCharge req");

    char length[4] = {};
    for (int i = 0; i < 4; i++)
    {
        length[i] = data[i + 4];
    }

    int data_length = Converter::bufferToIntLittleEndian(length);
    // std::cout << "stop charge data len : " << data_length << std::endl;

    // job id len
    data_index++;
    auto job_id_len = Converter::twoBytesToIntLittleEndian(data[data_index - 1], data[data_index]);
    // std::cout << "job id len : " << job_id_len << std::endl;
    data_index++;

    // job id
    auto job_id = byteToString(data, data_index, job_id_len);
    // std::cout << "job id : " << job_id << std::endl;
    data_index += job_id_len;

    json stop_charge;

    stop_charge["type"] = STOP_CHARGE;
    stop_charge["job_id"] = job_id;

    sendToAcsCommand(stop_charge);

    commandResponse(AcsCommandResponse::stop_charge_res, data);
    logWrite(LOG_INFO, stop_charge.dump());
}

void AcsCommunication::alarmAmr(char *data)
{
    commandResponse(AcsCommandResponse::alarm_res, data);
}

json AcsCommunication::getPathList(char *data, int count, int *data_index)
{

    // std::cout << "*********" << std::endl << std::endl;
    // std::cout << "Path List" << std::endl;
    // std::cout << "from - to" << std::endl;
    json path_list;

    int index = *data_index;

    for (int i = 0; i < count; i++)
    {
        // from node len
        index++;
        auto from_node_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "from_node_len : " << from_node_len << std::endl;
        index++;

        // from node
        auto from_node = byteToString(data, index, from_node_len);
        ////std::cout << "from_node : " << from_node << std::endl;
        index += from_node_len;

        // to node len
        index++;
        auto to_node_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "to_node_len : " << to_node_len << std::endl;
        index++;

        // to node
        auto to_node = byteToString(data, index, from_node_len);
        ////std::cout << "to_node : " << to_node << std::endl;
        index += to_node_len;

        auto from_node_str = std::stoi(from_node.substr(1, 3));
        auto to_node_str = std::stoi(to_node.substr(1, 3));

        // std::cout << from_node_str << " - " << to_node_str << std::endl;

        // path direction
        auto path_direction = data[index];
        ////std::cout << "path_direction : " << path_direction << std::endl;
        index++;

        // start move angle
        auto start_move_angle = getDouble(data, index);
        ////std::cout << "start_move_angle : " << start_move_angle << std::endl;
        index += 8;

        // start head angle
        auto start_head_angle = getDouble(data, index);
        ////std::cout << "start_head_angle : " << start_head_angle << std::endl;
        index += 8;

        // end move angle
        auto end_move_angle = getDouble(data, index);
        ////std::cout << "end_move_angle : " << end_move_angle << std::endl;
        index += 8;

        // end head angle
        auto end_head_angle = getDouble(data, index);
        ////std::cout << "end_head_angle : " << end_head_angle << std::endl;
        index += 8;

        // to node angle
        auto to_node_angle = getDouble(data, index);
        ////std::cout << "to_node_angle : " << to_node_angle << std::endl;
        index += 8;

        // radius
        auto radius = getDouble(data, index);
        ////std::cout << "radius : " << radius << std::endl;
        index += 8;

        // max velocity
        auto max_velocity = getDouble(data, index);
        ////std::cout << "max_velocity : " << max_velocity << std::endl;
        index += 8;

        // time start
        auto time_start = getDouble(data, index);
        ////std::cout << "time_start : " << time_start << std::endl;
        index += 8;

        // time end
        auto time_end = getDouble(data, index);
        ////std::cout << "time_end : " << time_end << std::endl;
        index += 8;

        // start body turn
        auto start_body_turn = getDouble(data, index);
        ////std::cout << "start_body_turn : " << start_body_turn << std::endl;
        index += 8;

        // start wait
        auto start_wait = getDouble(data, index);
        ////std::cout << "start_wait : " << start_wait << std::endl;
        index += 8;

        // start accel
        auto start_accel = getDouble(data, index);
        ////std::cout << "start_accel : " << start_accel << std::endl;
        index += 8;

        // start move1
        auto start_move_1 = getDouble(data, index);
        ////std::cout << "start_move_1 : " << start_move_1 << std::endl;
        index += 8;

        // start move_r
        auto start_move_r = getDouble(data, index);
        ////std::cout << "start_move_r : " << start_move_r << std::endl;
        index += 8;

        // start edge wait
        auto start_edge_wait = getDouble(data, index);
        ////std::cout << "start_edge_wait : " << start_edge_wait << std::endl;
        index += 8;

        // edge wait end
        auto edge_wait_end = getDouble(data, index);
        ////std::cout << "edge_wait_end : " << edge_wait_end << std::endl;
        index += 8;

        // start_move_2
        auto start_move_2 = getDouble(data, index);
        ////std::cout << "start_move_2 : " << start_move_2 << std::endl;
        index += 8;

        // start break
        auto start_break = getDouble(data, index);
        ////std::cout << "start_break : " << start_break << std::endl;
        index += 8;

        // dist accel
        auto dist_accel = getDouble(data, index);
        /////std::cout << "dist_accel : " << dist_accel << std::endl;
        index += 8;

        // dist_move_1
        auto dist_move_1 = getDouble(data, index);
        ////std::cout << "dist_move_1 : " << dist_move_1 << std::endl;
        index += 8;

        // dist_move_r
        auto dist_move_r = getDouble(data, index);
        // //std::cout << "dist_move_r : " << dist_move_r << std::endl;
        index += 8;

        // dist_move_2
        auto dist_move_2 = getDouble(data, index);
        ////std::cout << "dist_move_2 : " << dist_move_2 << std::endl;
        index += 8;

        json path;
        path["from_node"] = from_node;
        path["to_node"] = to_node;
        path["path_direction"] = path_direction;
        path["start_move_angle"] = start_move_angle;
        path["start_head_angle"] = start_head_angle;
        path["end_move_angle"] = end_move_angle;
        path["end_head_angle"] = end_head_angle;
        path["to_node_angle"] = to_node_angle;
        path["radius"] = radius;
        path["max_velocity"] = max_velocity;
        path["time_start"] = time_start;
        path["time_end"] = time_end;
        path["start_body_turn"] = start_body_turn;
        path["start_wait"] = start_wait;
        path["start_accel"] = start_accel;
        path["start_move_1"] = start_move_1;
        path["start_move_r"] = start_move_r;
        path["start_edge_wait"] = start_edge_wait;
        path["edge_wait_end"] = edge_wait_end;
        path["start_move_2"] = start_move_2;
        path["start_break"] = start_break;
        path["dist_accel"] = dist_accel;
        path["dist_move_1"] = dist_move_1;
        path["dist_move_r"] = dist_move_r;
        path["dist_move_2"] = dist_move_2;
        path_list.push_back(path);
    }

    // std::cout << "*********" << std::endl << std::endl;
    *data_index = index;
    return path_list;
}

json AcsCommunication::getTaskList(char *data, int count, int *data_index)
{
    json task_list;

    int index = *data_index;

    // std::cout << "index : " << index << std::endl;

    for (int i = 0; i < count; i++)
    {
        // task number
        auto task_number = data[index];
        ////std::cout << "task_number : " << task_number << std::endl;
        index++;

        // task_type len
        index++;
        auto task_type_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "task_type_len : " << task_type_len << std::endl;
        index++;

        // task_type
        auto task_type = byteToString(data, index, task_type_len);
        ////std::cout << "task_type : " << task_type << std::endl;
        index += task_type_len;

        // slot number
        auto slot_number = data[index];
        ////std::cout << "slot_number : " << slot_number << std::endl;
        index++;

        // mat id len
        index++;
        auto mat_id_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "mat_id_len : " << mat_id_len << std::endl;
        index++;

        // mat_id
        auto mat_id = byteToString(data, index, mat_id_len);
        ////std::cout << "mat_id : " << mat_id << std::endl;
        index += mat_id_len;

        // mat_direction
        auto mat_direction = getDouble(data, index);
        ////std::cout << "mat_direction : " << mat_direction << std::endl;
        index += 8;

        // mat weight
        auto mat_weight = (int)data[index];
        ////std::cout << "mat_weight : " << mat_weight << std::endl;
        index++;

        // target_port_type len
        index++;
        auto target_port_type_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "target_port_type_len : " << target_port_type_len << std::endl;
        index++;

        // target_port_type
        auto target_port_type = byteToString(data, index, target_port_type_len);
        ////std::cout << "target_port_type : " << target_port_type << std::endl;
        index += target_port_type_len;

        // target_stage_number
        auto target_stage_number = (int)data[index];
        ////std::cout << "target_stage_number : " << target_stage_number << std::endl;
        index++;

        // from_port_type_len
        index++;
        auto from_port_type_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "from_port_type_len : " << from_port_type_len << std::endl;
        index++;

        // from_port_type
        auto from_port_type = byteToString(data, index, from_port_type_len);
        // td::cout << "from_port_type : " << from_port_type << std::endl;
        index += from_port_type_len;

        // from_stage_number
        auto from_stage_number = (int)data[index];
        ////std::cout << "from_stage_number : " << from_stage_number << std::endl;
        index++;

        // target_map_len
        index++;
        auto target_map_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "target_map_len : " << target_map_len << std::endl;
        index++;

        // target_map
        auto target_map = byteToString(data, index, target_map_len);
        ////std::cout << "target_map : " << target_map << std::endl;
        index += target_map_len;

        // from node len
        index++;
        auto from_node_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        // //std::cout << "from_node_len : " << from_node_len << std::endl;
        index++;

        // from node
        auto from_node = byteToString(data, index, from_node_len);
        // //std::cout << "from_node : " << from_node << std::endl;
        index += from_node_len;

        // to node len
        index++;
        auto to_node_len = Converter::twoBytesToIntLittleEndian(data[index - 1], data[index]);
        ////std::cout << "to_node_len : " << to_node_len << std::endl;
        index++;

        // to node
        auto to_node = byteToString(data, index, from_node_len);
        // //std::cout << "to_node : " << to_node << std::endl;
        index += to_node_len;

        json task;
        task["task_number"] = task_number;
        task["task_type"] = task_type;
        task["slot_number"] = slot_number;
        task["mat_id"] = mat_id;
        task["mat_direction"] = mat_direction;
        task["mat_weight"] = mat_weight;
        task["target_port_type"] = target_port_type;
        task["target_stage_number"] = target_stage_number;
        task["from_port_type"] = from_port_type;
        task["from_stage_number"] = from_stage_number;
        task["target_map"] = target_map;
        task["from_node"] = from_node;
        task["to_node"] = to_node;

        task_list.push_back(task);

        if (task_type == "AQ")
        {
            carrier_info_["slot_number"] = (int)slot_number;
            carrier_info_["mat_id"] = mat_id;
            carrier_info_["mat_direction"] = mat_direction;
            carrier_info_["mat_weight"] = mat_weight;
        }
    }

    *data_index = index;
    return task_list;
}

double AcsCommunication::getDouble(char *data, int index)
{
    std::vector<char> vec;

    for (int i = 0; i < 8; i++)
    {
        vec.push_back(data[index + i]);
    }

    char *double_cstr = vec.data();

    double double_value;
    std::memcpy(&double_value, &double_cstr, sizeof(double));

    double rounded_value = std::round(double_value * 1000) / 1000;

    return rounded_value;
}

void AcsCommunication::commandResponse(char returncode, char *data, int receive_ok)
{
    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = data[1];
    report.msg_type_ = returncode;
    report.seq_ = data[3];
    report.data_size_ = 5;

    std::vector<char> response;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    response.insert(response.end(), report_ptr, report_ptr + sizeof(AmrReport));

    char received = (char)receive_ok;
    response.push_back(received);
    response.push_back(0);
    response.push_back(0);
    response.push_back(0);
    response.push_back(0);

    char *cstr = response.data();
    size_t cstr_length = static_cast<size_t>(response.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
}

void AcsCommunication::scan(char *data)
{
    std::vector<char> scan_report_vec;

    scan_report_vec.push_back(ACCEPT);
    scan_report_vec.push_back(0);
    scan_report_vec.push_back(0);
    scan_report_vec.push_back(0);
    scan_report_vec.push_back(0);

    int num_of_carrier = amr_status_["lift_state"].get<int>();

    scan_report_vec.push_back((char)num_of_carrier);

    if (num_of_carrier > 0)
    {
        char slot_number = (char)carrier_info_["slot_number"].get<int>();
        scan_report_vec.push_back(slot_number);

        // mat_id
        std::string mat_id = carrier_info_["mat_id"];
        stringTobyteArray(mat_id, scan_report_vec);

        char mat_weight = (char)carrier_info_["mat_weight"].get<int>();
        scan_report_vec.push_back(mat_weight);

        double mat_direction = carrier_info_["mat_direction"].get<double>();
        doubleTo8bytes(mat_direction, scan_report_vec);
    }

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AcsCommandResponse::scan_res;
    report.seq_ = data[3];
    report.data_size_ = scan_report_vec.size();

    std::vector<char> scan_report;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    scan_report.insert(scan_report.end(), report_ptr, report_ptr + sizeof(AmrReport));
    scan_report.insert(scan_report.end(), scan_report_vec.begin(), scan_report_vec.end());

    char *cstr = scan_report.data();
    size_t cstr_length = static_cast<size_t>(scan_report.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
}

void AcsCommunication::initReport()
{
    std::vector<char> init_report_vec;
    // x,y pos
    double xpos = amr_status_["xpos"].get<double>(); // xpos
    double ypos = amr_status_["ypos"].get<double>(); // ypos

    std::string log_string = "x : " + std::to_string(xpos) + " / y : " + std::to_string(ypos);
    doubleTo8bytes(xpos, init_report_vec);
    doubleTo8bytes(ypos, init_report_vec);

    // map_name
    std::string map_name = amr_status_["map_name"];
    stringTobyteArray(map_name, init_report_vec);
    log_string += " / map name : " + map_name;

    // angle, battery
    double angle = amr_status_["angle"].get<double>();     // angle
    double battery = amr_status_["battery"].get<double>(); // battery

    doubleTo8bytes(angle, init_report_vec);
    doubleTo8bytes(battery, init_report_vec);

    log_string += " / angle : " + std::to_string(angle) + " / battery : " + std::to_string(battery);
    // current_node
    std::string current_node = amr_status_["current_node"];
    stringTobyteArray(current_node, init_report_vec);

    log_string += " / current_node : " + current_node;

    char lift_state = (char)amr_status_["lift_state"].get<int>(); // lift_state
    init_report_vec.push_back(lift_state);
    char char_num_of_carrier = (char)amr_status_["num_of_carrier"].get<int>(); // num_of_carrier
    init_report_vec.push_back(char_num_of_carrier);

    int num_of_carrier = amr_status_["num_of_carrier"].get<int>();

    log_string += " / lift_state : " + std::to_string(lift_state);
    log_string += " / num_of_carrier : " + std::to_string(num_of_carrier);

    if (num_of_carrier > 0)
    {
        // amr_init_report.carrier_list_ = amr_status_["carrier_list"]; //num_of_carrier
    }

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::init_report;
    report.seq_ = seq_++;
    report.data_size_ = init_report_vec.size();

    std::vector<char> init_report;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    init_report.insert(init_report.end(), report_ptr, report_ptr + sizeof(AmrReport));
    init_report.insert(init_report.end(), init_report_vec.begin(), init_report_vec.end());

    char *cstr = init_report.data();
    size_t cstr_length = static_cast<size_t>(init_report.size());
    std::string res(cstr, cstr_length);

    logWrite(LOG_INFO, "Send initReport -> " + log_string);
    logDebug(res, "InitReport");

    send_queue_.push(res);
}

void AcsCommunication::initReportResponse(char *data)
{
    int data_index = 8;
    logWrite(LOG_INFO, "Receive initReport res");

    // result code
    char result[4] = {};
    for (int i = 0; i < 4; i++)
    {
        result[i] = data[i + data_index];
    }

    int result_code = Converter::bufferToIntLittleEndian(result);
    data_index += 4;

    char period[4] = {};
    for (int i = 0; i < 4; i++)
    {
        period[i] = data[i + data_index];
    }

    int keep_alive_period = Converter::bufferToIntLittleEndian(period);
    data_index += 4;

    int keep_alive_count = (int)data[data_index];

    // std::cout << "result_code : " << result_code << std::endl;
    // std::cout << "keep_alive_period : " << keep_alive_period << std::endl;
    // std::cout << "keep_alive_count : " << keep_alive_count << std::endl;

    logWrite(LOG_INFO, "keep_alive_period : " + std::to_string(keep_alive_period));

    if (keep_alive_timer_ != NULL && keep_alive_timer_->is_steady())
    {
        keep_alive_timer_->reset();
        keep_alive_timer_ = this->create_wall_timer(std::chrono::milliseconds(keep_alive_period),
                                                    std::bind(&AcsCommunication::keepAlive, this));
    }
    else
    {
        keep_alive_timer_ = this->create_wall_timer(std::chrono::milliseconds(keep_alive_period),
                                                    std::bind(&AcsCommunication::keepAlive, this));
    }
}

void AcsCommunication::statusReport()
{
    std::vector<char> status_report_vec;

    // x,y pos
    double xpos = amr_status_["xpos"].get<double>(); // xpos
    double ypos = amr_status_["ypos"].get<double>(); // ypos

    doubleTo8bytes(xpos, status_report_vec);
    doubleTo8bytes(ypos, status_report_vec);

    // map_name
    std::string map_name = amr_status_["map_name"];
    stringTobyteArray(map_name, status_report_vec);

    // angle, velocity, battery
    double angle = amr_status_["angle"].get<double>();       // angle
    double velocity = amr_status_["velocity"].get<double>(); // velocity
    double battery = amr_status_["battery"].get<double>();   // battery

    doubleTo8bytes(angle, status_report_vec);
    doubleTo8bytes(velocity, status_report_vec);
    doubleTo8bytes(battery, status_report_vec);

    // current_node
    std::string current_node = amr_status_["current_node"];
    stringTobyteArray(current_node, status_report_vec);

    // next_node
    std::string next_node = amr_status_["next_node"];
    stringTobyteArray(next_node, status_report_vec);

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::status_report;
    report.seq_ = seq_++;
    report.data_size_ = status_report_vec.size();

    std::vector<char> status_report;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    status_report.insert(status_report.end(), report_ptr, report_ptr + sizeof(AmrReport));
    status_report.insert(status_report.end(), status_report_vec.begin(), status_report_vec.end());

    char *cstr = status_report.data();
    size_t cstr_length = static_cast<size_t>(status_report.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);

    std::string log_string = "status report : " + current_node + " -> " + next_node + " / battery : " + std::to_string(battery);
    logWrite(LOG_INFO, log_string);
    logDebug(res, "StatusReport");
}

void AcsCommunication::moveCompleted(std::string cur_job_id, std::string canceled_job_id, int type)
{
    // std::cout << "send move Completed start" << std::endl;
    std::vector<char> move_completed_vec;

    // cur_job_id
    stringTobyteArray(cur_job_id, move_completed_vec);

    // canceled_job_id
    stringTobyteArray(canceled_job_id, move_completed_vec);

    char complete_type = (char)type;
    move_completed_vec.push_back(complete_type);

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::move_completed;
    report.seq_ = seq_++;
    report.data_size_ = move_completed_vec.size();

    std::vector<char> move_completed;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    move_completed.insert(move_completed.end(), report_ptr, report_ptr + sizeof(AmrReport));
    move_completed.insert(move_completed.end(), move_completed_vec.begin(), move_completed_vec.end());

    char *cstr = move_completed.data();
    size_t cstr_length = static_cast<size_t>(move_completed.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
    // std::cout << "send move Completed finished" << std::endl;
    logWrite(LOG_INFO, "Send move Completed");
    logDebug(res, "MoveComplete");
}

void AcsCommunication::failReport(std::string cur_job_id, int fail_level, int stop_reason)
{
    logWrite(LOG_INFO, "Send fail report Start -> " + std::to_string(stop_reason));
    // std::cout << "send fail report start" << std::endl;
    std::vector<char> fail_report_vec;

    // cur_job_id
    stringTobyteArray(cur_job_id, fail_report_vec);

    // fail_level
    char level = (char)fail_level;
    fail_report_vec.push_back(level);

    // stop_reason
    std::vector<char> stop_reason_vec;
    Converter::intTo4BytesLittleEndian(stop_reason, stop_reason_vec);
    fail_report_vec.insert(fail_report_vec.end(), stop_reason_vec.begin(), stop_reason_vec.end());

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::fail_report;
    report.seq_ = seq_++;
    report.data_size_ = fail_report_vec.size();

    std::vector<char> fail_report;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    fail_report.insert(fail_report.end(), report_ptr, report_ptr + sizeof(AmrReport));
    fail_report.insert(fail_report.end(), fail_report_vec.begin(), fail_report_vec.end());

    char *cstr = fail_report.data();
    size_t cstr_length = static_cast<size_t>(fail_report.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
    // std::cout << "send fail report finished" << std::endl;
    logWrite(LOG_INFO, "Send fail report Done");
    logDebug(res, "FailReport");
}

void AcsCommunication::continueReport(std::string cur_job_id)
{
    // std::cout << "send continue report start" << std::endl;
    std::vector<char> continue_report_vec;

    // cur_job_id
    stringTobyteArray(cur_job_id, continue_report_vec);

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::continue_report;
    report.seq_ = seq_++;
    report.data_size_ = continue_report_vec.size();

    std::vector<char> continue_report;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    continue_report.insert(continue_report.end(), report_ptr, report_ptr + sizeof(AmrReport));
    continue_report.insert(continue_report.end(), continue_report_vec.begin(), continue_report_vec.end());

    char *cstr = continue_report.data();
    size_t cstr_length = static_cast<size_t>(continue_report.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
    // std::cout << "send continue report finished" << std::endl;
    logWrite(LOG_INFO, "Send continue report");
    logDebug(res, "ContinueReport");
}

void AcsCommunication::stateChanged()
{
    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::state_changed;
    report.seq_ = seq_++;
    report.data_size_ = 1;

    char state = amr_status_["agv_state"].get<char>();

    std::vector<char> state_changed;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    state_changed.insert(state_changed.end(), report_ptr, report_ptr + sizeof(AmrReport));
    state_changed.push_back(state);

    char *cstr = state_changed.data();
    size_t cstr_length = static_cast<size_t>(state_changed.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
    std::string log_string = "state changed : " + std::to_string(state);
    logWrite(LOG_INFO, log_string);
    logDebug(res, "StateChanged");
}

void AcsCommunication::taskReport(std::string job_id, char number)
{
    std::vector<char> task_report_vec;

    // job id
    stringTobyteArray(job_id, task_report_vec);
    task_report_vec.push_back(number);

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::task_report;
    report.seq_ = seq_++;
    report.data_size_ = task_report_vec.size();

    std::vector<char> task_report;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    task_report.insert(task_report.end(), report_ptr, report_ptr + sizeof(AmrReport));
    task_report.insert(task_report.end(), task_report_vec.begin(), task_report_vec.end());

    char *cstr = task_report.data();
    size_t cstr_length = static_cast<size_t>(task_report.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
    logWrite(LOG_INFO, "Send task Report : " + job_id + " / task Number : " + std::to_string(number));
    logDebug(res, "TaskReport");
}

void AcsCommunication::keepAlive()
{
    std::vector<char> keep_alive_vec;

    // x,y pos
    double xpos = amr_status_["xpos"].get<double>(); // xpos
    double ypos = amr_status_["ypos"].get<double>(); // ypos

    doubleTo8bytes(xpos, keep_alive_vec);
    doubleTo8bytes(ypos, keep_alive_vec);

    // map_name
    std::string map_name = amr_status_["map_name"];
    stringTobyteArray(map_name, keep_alive_vec);

    // angle, velocity, battery
    double angle = amr_status_["angle"].get<double>(); // angle
    doubleTo8bytes(angle, keep_alive_vec);

    // current_node
    std::string current_node = amr_status_["current_node"];
    stringTobyteArray(current_node, keep_alive_vec);

    double battery = amr_status_["battery"].get<double>(); // battery
    doubleTo8bytes(battery, keep_alive_vec);

    char lift_state = amr_status_["lift_state"].get<char>();
    char agv_state = amr_status_["agv_state"].get<char>();

    keep_alive_vec.push_back(lift_state);
    keep_alive_vec.push_back(agv_state);

    auto report = AmrReport();
    report.stx_ = stx_;
    report.ver_ = 0x00;
    report.msg_type_ = AgvReport::keep_alive;
    report.seq_ = seq_++;
    report.data_size_ = keep_alive_vec.size();

    std::vector<char> keep_alive;
    const char *report_ptr = reinterpret_cast<const char *>(&report);
    keep_alive.insert(keep_alive.end(), report_ptr, report_ptr + sizeof(AmrReport));
    keep_alive.insert(keep_alive.end(), keep_alive_vec.begin(), keep_alive_vec.end());

    char *cstr = keep_alive.data();
    size_t cstr_length = static_cast<size_t>(keep_alive.size());
    std::string res(cstr, cstr_length);

    send_queue_.push(res);
}

void AcsCommunication::setServerMessage()
{
    if (server_ != NULL)
    {
        if (send_queue_.count() > 0)
        {
            auto data = send_queue_.front();
            char *acknowledge = new char[data.size()];

            std::memcpy(acknowledge, data.c_str(), data.size());

            server_->sendData(acknowledge, data.size());
            monitor_server_->sendData(acknowledge, data.size());

            send_queue_.pop();

            delete[] acknowledge;
        }
    }
    else
    {
        if (send_queue_.count() > 0)
        {
            send_queue_.pop();
        }
    }
}

void AcsCommunication::sendToAcsCommand(json j)
{
    auto request = std::make_shared<tc_acs_interface::srv::AcsCommand::Request>();

    request->msg = j.dump();

    while (!acs_command_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            // std::cout << "Service Client cannot found server" << std::endl;
            // Something Error
            return;
        }
    }

    auto result = acs_command_client_->async_send_request(request);

    logWrite(LOG_INFO, "Wait for response from acs command service server");

    if (rclcpp::spin_until_future_complete(service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // std::cout << "server result : " << result.get()->received << std::endl;
        std::string server_result = result.get()->received ? "true" : "false";
        logWrite(LOG_INFO, "acs command service server result : " + server_result);
    }

    logWrite(LOG_INFO, "Send to scenario : " + j.dump());
}

void AcsCommunication::getAcsStatus()
{
    if (server_ != NULL)
    {
        bool is_acs_conn = server_->isConnected();
        if (is_acs_conn == true && send_init_ == false)
        {
            // stateChanged();
            send_init_ = true;
            initReport();
            stateChanged();
            // std::cout << "send init report" << std::endl;
        }
        else if (is_acs_conn == false)
        {
            send_init_ = false;
        }

        auto msg = std_msgs::msg::Bool();
        msg.data = is_acs_conn;
        acs_conn_pub_->publish(msg);
    }
}

void AcsCommunication::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_ACS, level, msg);
}

void AcsCommunication::logDebug(std::string msg, std::string header)
{
    std::string log_str = header;
    log_str += " : ";
    for (int i = 0; i < msg.length(); i++)
    {
        log_str += std::to_string((int)msg[i]);
        log_str += " ";
    }
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_ACS_DEBUG, LOG_INFO, log_str);
}

std::string AcsCommunication::byteToString(char *data, int start_index, int length)
{
    int end_index = start_index + length;
    std::string str;

    for (int i = start_index; i < end_index; i++)
    {
        str += data[i];
    }

    return str;
}

void AcsCommunication::stringTobyteArray(std::string str, std::vector<char> &bytes)
{
    // str
    std::vector<char> len;
    int str_len = str.length();
    Converter::intTo2BytesLittleEndian(str_len, len);
    bytes.push_back(len[0]);
    bytes.push_back(len[1]);

    for (int i = 0; i < str_len; i++)
    {
        bytes.push_back(str[i]);
    }
}

void AcsCommunication::doubleTo8bytes(double value, std::vector<char> &bytes)
{
    char double_bytes[sizeof(double)];
    std::memcpy(double_bytes, &value, sizeof(double));

    for (int i = 0; i < 8; i++)
    {
        bytes.push_back(double_bytes[i]);
    }
}