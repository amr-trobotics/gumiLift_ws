#include "tdriver_status.hpp"

TDriverStatus::TDriverStatus() : Node("seer_status", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    std::cout << "TDriverStatus()" << std::endl;

    init_node_ = rclcpp::Node::make_shared("tdriver_init_node");
    init_clinet_ = init_node_->create_client<tc_msgs::srv::AmrInit>("amr_init");

    receive_queue_ = std::make_shared<ThreadSafeQueue<std::string>>();
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("whole_t_driver_status", 10);
    init_status_publisher_ = this->create_publisher<std_msgs::msg::Int32>("t_driver_init_status", 10);
    get_client_data_timer_ = this->create_wall_timer(5ms, std::bind(&TDriverStatus::getClientMessage,this));
    
    amr_init_status_sub_ = this->create_subscription<tc_msgs::msg::AmrInitStatus>
    ("amr_init_command", 10, std::bind(&TDriverStatus::getAmrInitStatus, this, std::placeholders::_1));

    initClients();
    connection_thread = std::thread(&TDriverStatus::initRobot, this); 

    std::cout << "TDriverStatus() Finished" << std::endl;
}

TDriverStatus::~TDriverStatus()
{
    delete client1_;
    delete client2_;
    delete client3_;
    delete client4_;
    delete client5_;
    delete client6_;
    delete client7_;
}

void TDriverStatus::initRobot()
{
    loadMap();
    localizationRobot();
}

void TDriverStatus::loadMap()
{
    auto msg = std_msgs::msg::Int32();
    msg.data = INIT_START;
    init_status_publisher_->publish(msg);
    auto seer_api = TDriverStatusApi();

    //Load Map
    auto load_map_client = new TDriverBaseClient();
    load_map_client->init(receive_queue_, seer_api.robot_map_loading_inquiry);

    int count = 0;

    std::cout << "loading map start" << std::endl;
    
    while(map_loading_status_ != LOADING_MAP_SUCCESS && count < 60)
    {
        count++;
        std::cout << "wait for loading map success" << std::endl;
        std::this_thread::sleep_for(1000ms);
        
        msg.data = INIT_RUNNING;
        init_status_publisher_->publish(msg);
        //Fail Sequence 추가 필요    
    }
    std::this_thread::sleep_for(3000ms);
    load_map_client->stop();
    std::cout << "end of loading map success" << std::endl;
    //Relocate ( It is based on Communicate Node ) ***********************

    delete load_map_client;
}

void TDriverStatus::localizationRobot()
{
    try
    {
        while(!init_clinet_->wait_for_service(1s))
        {
            std::cout << "wait for service" << std::endl;
            
            if(!rclcpp::ok())
            {
                std::cout << "Service Client cannot found server" << std::endl;
                //Something Error
                return;
            }
        }
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    std::cout << "start robot_localization_client" << std::endl;
    //Localization robot

    auto request = std::make_shared<tc_msgs::srv::AmrInit::Request>();
    request->data = RELOCATE;

    auto result = init_clinet_->async_send_request(request);
    std::cout << "Wait for server response" << std::endl;

    if(rclcpp::spin_until_future_complete(init_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "server result : " << result.get()->received << std::endl;

        std::this_thread::sleep_for(10ms); // Do not remove
        
    }

    int count = 0;
    while(robot_localization_status_ != LOCALIZATION_COMPLETED && robot_localization_status_ != LOCALIZATION_SUCCESS && count < 600)
    {
        count++;
        //std::cout << "wait for localization : " << robot_localization_status_ << std::endl;
        std::this_thread::sleep_for(100ms); 

        auto msg = std_msgs::msg::Int32();
        msg.data = INIT_RUNNING;
        init_status_publisher_->publish(msg);       
    }

    //Add Check Reloacte Pos logic 
     
    std::cout << "localization status : " << robot_localization_status_ << std::endl;
       

    confirmRelocate();
}

void TDriverStatus::confirmRelocate()
{
    std::cout << "start robot_localization_client" << std::endl;
    //Localization robot

    auto msg = std_msgs::msg::Int32();
    msg.data = INIT_RUNNING;
    init_status_publisher_->publish(msg);    
    std::this_thread::sleep_for(2s);

    int count = 0;
    while(robot_localization_status_ != LOCALIZATION_SUCCESS && count < 60)
    {
        count++;
        auto request2 = std::make_shared<tc_msgs::srv::AmrInit::Request>();
        request2->data = CONFIRM_RELOCATE;

        auto result2 = init_clinet_->async_send_request(request2);
        std::cout << "Wait for server response" << std::endl;

        if(rclcpp::spin_until_future_complete(init_node_, result2) == rclcpp::FutureReturnCode::SUCCESS)
        {
            std::cout << "server result : " << result2.get()->received << std::endl;

            std::this_thread::sleep_for(1s); // Do not remove
            
        }

        //Do Confrim Reloaction
        if(robot_localization_status_ == LOCALIZATION_SUCCESS)
        {
            break;
            //std::cout << "Wait for localization status : " << robot_localization_status_ << std::endl;
        }
    }

    auto msg2 = std_msgs::msg::Int32();
    msg2.data = INIT_DONE;
    init_status_publisher_->publish(msg2);

    std::cout << "localization status : " << robot_localization_status_ << std::endl;
    std::cout << "end of localization" << std::endl;

}

void TDriverStatus::initClients()
{    
	client1_ = new TDriverBaseClient();
    client2_ = new TDriverBaseClient();
	client3_ = new TDriverBaseClient();
	client4_ = new TDriverBaseClient();
	client5_ = new TDriverBaseClient();
	client6_ = new TDriverBaseClient();
	client7_ = new TDriverBaseClient();
	client8_ = new TDriverBaseClient();
    
    auto seer_api = TDriverStatusApi();
    api_lists.push_back(seer_api.info_inquiry);
    api_lists.push_back(seer_api.battery_inquiry); 
    api_lists.push_back(seer_api.blocked_status_inquiry);
    api_lists.push_back(seer_api.estop_inquiry);
    api_lists.push_back(seer_api.motor_status_inquiry);
    
    std::cout << "push back api" << std::endl;
     
    std::cout << "init" << std::endl;
    client1_->init(receive_queue_, seer_api.robot_localizaion_inquiry);
    client2_->init(receive_queue_, seer_api.navigation_status_inquiry);
    client3_->init(receive_queue_, seer_api.location_inquiry);
    client4_->init(receive_queue_, seer_api.alarm_status_inquiry);
    client5_->init(receive_queue_, seer_api.speed_inquiry);
    client6_->init(receive_queue_, seer_api.map_loading_inquiry);
    client7_->init(receive_queue_, seer_api.robot_jacking_status_inquiry);
    client8_->init(receive_queue_, api_lists);
}

void TDriverStatus::getClientMessage()
{
    if(receive_queue_->count() > 0)
    {
        parsingClientMessage(receive_queue_->front());
        receive_queue_->pop();
    }
}

void TDriverStatus::parsingClientMessage(std::string msg)
{
    json j = json::parse(msg);

    std::string str_type = j["DATA_TYPE"];
    int data_type = std::stoi(str_type);

    if(data_type == ROBOT_LOCALIZATION_STATUS_INQUIRY)
    {
        robot_localization_status_ = j["reloc_status"].get<int>();
    }
    else if(data_type == ROBOT_MAPLOADING_STATUS_INQUIRY)
    {
        map_loading_status_ = j["loadmap_status"].get<int>();
    }

    auto t_driver_msg = std_msgs::msg::String();
    t_driver_msg.data = msg;
    status_publisher_->publish(t_driver_msg);
}

void TDriverStatus::getAmrInitStatus(const tc_msgs::msg::AmrInitStatus::SharedPtr msg)
{
    if(msg->init_status == DO_INIT)
    {
        std::cout << "receive relocate again" << std::endl;
        auto th = std::thread(&TDriverStatus::initRobot, this);

        th.detach();
    }
    else if(msg->init_status == CHANGE_MAP)
    {
        std::cout << "receive change map" << std::endl;
        
        auto th = std::thread(&TDriverStatus::confirmRelocate, this);

        th.detach();        
    }

}