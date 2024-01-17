#include "acs_communication/pio_client.hpp"

PioClient::PioClient() : Node("pio_client")
{
    std::cout << "init pio client done" << std::endl;
    init();
}

PioClient::~PioClient()
{
    quit_ = true;
}

void PioClient::init()
{
    quit_ = false;

    receive_queue_ = std::make_shared<ThreadSafeQueue<char *>>();

    amr_status_["xpos"] = 0.0;
    amr_status_["ypos"] = 0.0;
    amr_status_["map_name"] = "2F";
    amr_status_["angle"] = 180;
    amr_status_["current_node"] = "W207";
    amr_status_["agv_state"] = "NOT_ASSIGNED";

    connection_thread_ = std::thread(&PioClient::connect, this);

    get_server_data_timer_ = this->create_wall_timer(50ms, std::bind(&PioClient::getServerMessage, this));
    set_server_data_timer_ = this->create_wall_timer(200ms, std::bind(&PioClient::setServerMessage, this));
    acs_report_sub_ = this->create_subscription<std_msgs::msg::String>
        ("amr_report", 10, std::bind(&PioClient::getAmrStatus, this, std::placeholders::_1));
    
}

void PioClient::connect()
{    
    while(!quit_)
    {
        boost::asio::io_context io_context;
        std::string ip = "10.14.182.5";
        std::string port = "9001"; 
        client_ = new AsyncTcpClient(io_context, ip, port, receive_queue_);
        
        client_->connect();  
        std::cout << "try to connect" << std::endl; 
    
        io_context.run();
        std::cout << "connect end" << std::endl;

        std::this_thread::sleep_for(1000ms);
    }
}

void PioClient::getServerMessage()
{
    if (receive_queue_->count() > 0)
    {
        char* data = receive_queue_->front();
        receive_queue_->pop();
    }
}

void PioClient::setServerMessage()
{
    if (client_ != NULL)
    {
        try
        {
            double xpos = amr_status_["xpos"].get<double>();
            std::string x = std::to_string(xpos);

            double ypos = amr_status_["ypos"].get<double>();
            std::string y = std::to_string(ypos);

            std::string map_name = amr_status_["map_name"];
            int angle = amr_status_["angle"].get<int>();
            std::string angle_str = std::to_string(angle);

            std::string node = amr_status_["current_node"];
            std::string state = amr_status_["agv_state"];

            std::string amr_state = x + "," + y + "," + map_name + "," + angle_str + "," + node + "," + state;

            std::vector<char> report_vec;
            stringTobyteArray(amr_state, report_vec);        
        
            char *cstr = report_vec.data();
            int size = report_vec.size();

            client_->write(cstr,size);
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}

void PioClient::getAmrStatus(const std_msgs::msg::String::SharedPtr msg)
{
    try
    {
        json amr_status = json::parse(msg->data);

        double x = amr_status["x"].get<double>() * 1000;
        double y = amr_status["y"].get<double>() * 1000;
        int angle = amr_status["angle"].get<int>();

        amr_status_["xpos"] = x;
        amr_status_["ypos"] = y;
        amr_status_["map_name"] = amr_status["map_name"];
        amr_status_["angle"] = angle;
        amr_status_["current_node"] = amr_status["node"];
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
        amr_status_["agv_state"] = vehicle_state;
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void PioClient::stringTobyteArray(std::string str, std::vector<char> &bytes)
{
    int str_len = str.length();
    for (int i=0; i<str_len; i++)
    {
        bytes.push_back(str[i]);
    }
}
