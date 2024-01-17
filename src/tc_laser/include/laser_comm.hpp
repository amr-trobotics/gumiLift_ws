
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "modbus.h"


using namespace std::chrono_literals;


class Laser : public rclcpp::Node 
{
public:
    Laser();
    ~Laser();

    modbus adam = modbus("192.168.192.20", 502);
    uint16_t read_input_regs[8];
    float distance_val_1;
    uint16_t alarm_val_1;
    float distance_val_2;
    uint16_t alarm_val_2;
    float distance_val_3;
    uint16_t alarm_val_3;
    float distance_val_4;
    uint16_t alarm_val_4;
    const float maxRange = 0.98;   //Maximum range in centimeters
    const float minRange = 0.1;     //Minimum range

private:
    
    void timer_callback();
    void adam_callback();
    void adam_sensor1_publisher(float distance_val, uint16_t alarm_val);
    void adam_sensor2_publisher(float distance_val, uint16_t alarm_val);
    void adam_sensor3_publisher(float distance_val, uint16_t alarm_val);
    void adam_sensor4_publisher(float distance_val, uint16_t alarm_val);
    
    

private:
    
    rclcpp::Node::SharedPtr node;
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::TimerBase::SharedPtr adam_subscription_; 
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr laser_sensor1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr laser_sensor2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr laser_sensor3_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr laser_sensor4_pub_;


};


