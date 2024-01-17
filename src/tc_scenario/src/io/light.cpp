#include "io/light.hpp"

AmrLight::AmrLight() : Node("amr_light")
{
    service_node_ = rclcpp::Node::make_shared("io_light_node");
    io_array_client_ = service_node_->create_client<tc_msgs::srv::Io>("io_array_event");
    light_req_sub_ = this->create_subscription<std_msgs::msg::Int32>
    ("light_request", 10, std::bind(&AmrLight::lightRequestCallBack, this, std::placeholders::_1));
}

AmrLight::~AmrLight()
{

}

void AmrLight::lightRequestCallBack(const std_msgs::msg::Int32::SharedPtr msg)
{
    int light = msg->data;

    if(light == current_light)
    {
        return;
    }
    else if(light == LIGHT_NONE)
    {
        turnOffLight();
    }
    else
    {
        turnOnLight(light);
    }

    current_light = light;

}

void AmrLight::turnOnLight(int light)
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

    auto channel = getLightChannel(light); // must be 3
    auto request = std::make_shared<tc_msgs::srv::Io::Request>();

    int led_size = channel.size()*4;

    request->id.resize(led_size);
    request->state.resize(led_size);
    request->size = led_size;

    int size = channel.size();
    for(int i=0; i<size; i++)
    {
        int output_pin_1 = OUT_LED1_BLUE + i;
        int output_pin_2 = OUT_LED2_BLUE + i;
        int output_pin_3 = OUT_LED3_BLUE + i;
        int output_pin_4 = OUT_LED4_BLUE + i;

        int index = i*4;
        request->id[index] = output_pin_1;
        request->id[index+1] = output_pin_2;
        request->id[index+2] = output_pin_3;
        request->id[index+3] = output_pin_4;

        request->state[index] = channel.at(i);
        request->state[index+1] = channel.at(i);
        request->state[index+2] = channel.at(i);
        request->state[index+3] = channel.at(i); 
    }    
    
    auto result = io_array_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(service_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::this_thread::sleep_for(10ms); // Do not remove
    }
}

void AmrLight::turnOffLight()
{

}


std::vector<int> AmrLight::getLightChannel(int light)
{
    std::vector<int> result(3,0);
    //0 - b  / 1 - g / 2 - r
    switch (light)
    {
        case LIGHT_WHITE: 
            result.at(0) = 1; 
            result.at(1) = 1; 
            result.at(2) = 1;     
            break;        
        case LIGHT_GREEN:
            result.at(1) = 1;      
            break;
        case LIGHT_YELLOW:
            result.at(1) = 1;   
            result.at(2) = 1;   
            break;
        case LIGHT_RED:
            result.at(2) = 1;   
            break;
        case LIGHT_RED_FLICKER:
            result.at(2) = 1;   
            break;
        case LIGHT_WHITE_FLICKER:
            result.at(0) = 1;   
            result.at(1) = 1;    
            result.at(2) = 1;   
            break;
        case LIGHT_BLUE:
            result.at(0) = 1; 
            break;
        case LIGHT_PURPLE:
            result.at(0) = 1; 
            result.at(2) = 1; 
            break;    
        default:
            break;
    }

    return result;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto light = std::make_shared<AmrLight>();

    rclcpp::spin(light);

    rclcpp::shutdown();

    return 0;
}