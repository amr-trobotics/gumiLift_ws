#include "io/sound.hpp"

AmrSound::AmrSound() : Node("amr_sound")
{
    service_node_ = rclcpp::Node::make_shared("io_sound_node");
    io_array_client_ = service_node_->create_client<tc_msgs::srv::Io>("io_array_event");
    sound_req_sub_ = this->create_subscription<std_msgs::msg::Int32>
    ("sound_request", 10, std::bind(&AmrSound::soundRequestCallBack, this, std::placeholders::_1));

}

AmrSound::~AmrSound()
{
    
}


void AmrSound::soundRequestCallBack(const std_msgs::msg::Int32::SharedPtr msg)
{
    int sound = msg->data;

    if(sound == current_sound)
    {
        return;
    }
    else if(sound == SOUND_NONE)
    {
        turnOffSound();
    }
    else
    {
        turnOnSound(sound);
    }

    current_sound = sound;

}

void AmrSound::turnOnSound(int sound)
{  
    std::lock_guard<RecursiveMutex> lock(mutex_);
    try
    {
        while (!io_array_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                std::cout << "Service Client cannot found server" << std::endl;
                // Something Error
                return;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    auto request = std::make_shared<tc_msgs::srv::Io::Request>();    

    auto channel = getSoundChannel(sound); //4
    request->id.resize(channel.size()+1); //5
    request->state.resize(channel.size()+1); //5
    request->size = channel.size()+1; //5

    int size = channel.size();
    
    for(int i=0; i<size; i++)
    {
        int output_pin = OUT_BUZZER_CH1 + i;
        request->id[i] = output_pin; // 0,1,2,3
        request->state[i] = channel.at(i);
    }

    request->id[channel.size()] = OUT_BUZZER_STOP; // 4
    request->state[channel.size()] = 0;

    auto result = io_array_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {        
        std::this_thread::sleep_for(10ms); // Do not remove
    }
}

void AmrSound::turnOffSound()
{
    std::lock_guard<RecursiveMutex> lock(mutex_);
    try
    {
        while (!io_array_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                std::cout << "Service Client cannot found server" << std::endl;
                // Something Error
                return;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    auto request = std::make_shared<tc_msgs::srv::Io::Request>();    

    auto channel = getSoundChannel(0); 
    request->id.resize(channel.size()+1); 
    request->state.resize(channel.size()+1); 
    request->size = channel.size()+1; 

    int size = channel.size();
    
    for(int i=0; i<size; i++)
    {
        int output_pin = OUT_BUZZER_CH5 - i;
        request->id[i] = output_pin; // 0,1,2,3
        request->state[i] = channel.at(i);
    }

    request->id[channel.size()] = OUT_BUZZER_STOP; // 4
    request->state[channel.size()] = 1;

    auto result = io_array_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {        
        std::this_thread::sleep_for(10ms); // Do not remove
    }        
}

std::vector<int> AmrSound::getSoundChannel(int sound)
{
    std::vector<int> result(5,0);

    switch (sound)
    {
        case SOUND_ALARM: 
            result.at(0) = 1;      
            break;        
        case SOUND_BUMPER:
            result.at(1) = 1;      
            break;
        case SOUND_OBSTACLE:
            result.at(0) = 1;   
            result.at(1) = 1;   
            break;
        case SOUND_DOCKING:
            result.at(2) = 1;   
            break;
        case SOUND_LOAD_UNLOAD:
            result.at(0) = 1;   
            result.at(2) = 1;   
            break;
        case SOUND_LOADING:
            result.at(1) = 1;    
            result.at(2) = 1;   
            break;
        case SOUND_BLOCKING:
            result.at(0) = 1; 
            result.at(1) = 1; 
            result.at(2) = 1; 
            break;
        case SOUND_MOVING_STK:
            result.at(3) = 1; 
            break;
        case SOUND_MOVING_PORT:
            result.at(0) = 1; 
            result.at(3) = 1; 
            break;
        case SOUND_MOVING_EV:
            result.at(1) = 1;
            result.at(3) = 1; 
            break;
        case SOUND_MOVING_CHARGE:
            result.at(0) = 1; 
            result.at(1) = 1; 
            result.at(3) = 1; 
            break;
        case SOUND_PASS_AUTO_DOOR:
            result.at(2) = 1; 
            result.at(3) = 1; 
            break;
        case SOUND_ACS_PAUSE:
            result.at(0) = 1; 
            result.at(2) = 1; 
            result.at(3) = 1; 
            break;
        case SOUND_AGV_PAUSE:
            result.at(1) = 1; 
            result.at(2) = 1; 
            result.at(3) = 1; 
            break;
        case SOUND_MOVING_AIRSHOWER:
            result.at(0) = 1;
            result.at(1) = 1; 
            result.at(2) = 1; 
            result.at(3) = 1; 
            break;
        case SOUND_CALL_ELEVATOR:
            result.at(4) = 1; 
            break;
        case SOUND_WAIT_ELEVATOR:
            result.at(0) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_START_ENTER:
            result.at(1) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_ENTER_NOW:
            result.at(0) = 1;
            result.at(1) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_GET_OUT:
            result.at(2) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_FIRST_FLOOR_EXIT:
            result.at(0) = 1; 
            result.at(2) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_SECOND_FLOOR_EXIT:
            result.at(1) = 1;
            result.at(2) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_SHOCK_DETECTION:
            result.at(0) = 1;
            result.at(1) = 1;
            result.at(2) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_DANGER:
            result.at(3) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_EXIT_THIS_FLOOR:
            result.at(0) = 1;
            result.at(3) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_START_EXIT:
            result.at(1) = 1;
            result.at(3) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_EXIT_NOW:
            result.at(0) = 1;
            result.at(1) = 1;
            result.at(3) = 1; 
            result.at(4) = 1; 
            break;
        case SOUND_ERROR:
            result.at(2) = 1;
            result.at(3) = 1; 
            result.at(4) = 1; 
            break;
        default:
            break;
    }

    return result;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto sound = std::make_shared<AmrSound>();

    rclcpp::spin(sound);

    rclcpp::shutdown();

    return 0;
}