#include "laser_comm.hpp"
#include <unistd.h>


using namespace std::chrono_literals;

Laser::Laser()
: Node("laser_node")
{
    RCLCPP_INFO(get_logger(), "connect adam modbus Main");
     //Node handle 가져 오기.
	node = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    laser_sensor1_pub_ = this->create_publisher<sensor_msgs::msg::Range>("laser_sensor1", rclcpp::QoS(rclcpp::KeepLast(10)));
    laser_sensor2_pub_ = this->create_publisher<sensor_msgs::msg::Range>("laser_sensor2", rclcpp::QoS(rclcpp::KeepLast(10)));
    laser_sensor3_pub_ = this->create_publisher<sensor_msgs::msg::Range>("laser_sensor3", rclcpp::QoS(rclcpp::KeepLast(10)));
    laser_sensor4_pub_ = this->create_publisher<sensor_msgs::msg::Range>("laser_sensor4", rclcpp::QoS(rclcpp::KeepLast(10)));
    // create a modbus object
    // modbus adam = modbus("10.0.0.1", 502);
    // // set slave id
    adam.modbus_set_slave_id(1);

    // connect with the server
    if (adam.modbus_connect())
    {
         RCLCPP_INFO(this->get_logger(), "adam-6017 connected - LASER starting");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "adam-6017 unconnected -LASER stopping");
    }
    //uint16_t read_input_regs[5];
    adam_subscription_= this->create_wall_timer(40ms, std::bind(&Laser::adam_callback, this));
    timer_ = this->create_wall_timer(40ms, std::bind(&Laser::timer_callback, this));
}

Laser::~Laser()
{
     adam.modbus_close();

}
void Laser::adam_callback()
{
    // read chanel 1,2
    adam.modbus_read_input_registers(0x40000, 8, read_input_regs);
    //sensor1
    alarm_val_1 = __uint16_t(read_input_regs[1]/40000);
    if (alarm_val_1 == 0){
        distance_val_1 = (float)(((read_input_regs[0]*1000/65535-133)*1.7 +100)/1000);
    }
    else{
        distance_val_1 = std::numeric_limits<float>::infinity();
    }
    //sensor2
    alarm_val_2 = __uint16_t(read_input_regs[5]/40000);
    if (alarm_val_2 == 0){
        distance_val_2 = (float)(((read_input_regs[4]*1000/65535-133)*1.7 +100)/1000);
    }
    else{
        distance_val_2 = std::numeric_limits<float>::infinity();
    }
    //sensor3
    alarm_val_3 = __uint16_t(read_input_regs[3]/40000);
    if (alarm_val_3 == 0){
        distance_val_3 = (float)(((read_input_regs[2]*1000/65535-133)*1.7 +100)/1000);
    }
    else{
        distance_val_3 = std::numeric_limits<float>::infinity();
    }
    //sensor4
    alarm_val_4 = __uint16_t(read_input_regs[7]/40000);
    if (alarm_val_4 == 0){
        distance_val_4 = (float)(((read_input_regs[6]*1000/65535-133)*1.7 +100)/1000);
    }
    else{
        distance_val_4 = std::numeric_limits<float>::infinity();
    }
    

    RCLCPP_INFO(this->get_logger(), "sensor1: %f %d", distance_val_1, alarm_val_1);
    RCLCPP_INFO(this->get_logger(), "sensor2: %f %d", distance_val_2, alarm_val_2);
    RCLCPP_INFO(this->get_logger(), "sensor3: %f %d", distance_val_3, alarm_val_3);
    RCLCPP_INFO(this->get_logger(), "sensor4: %f %d", distance_val_4, alarm_val_4);

}
void Laser::adam_sensor1_publisher(float distance_val, uint16_t alarm_val)
{
    auto adam_msg = sensor_msgs::msg::Range();
    adam_msg.header.frame_id = "adam_sensor1_link";
    adam_msg.header.stamp = this->now();
    adam_msg.min_range = minRange;
    adam_msg.max_range = maxRange;
    adam_msg.range = distance_val;
    laser_sensor1_pub_->publish(adam_msg);
}
void Laser::adam_sensor2_publisher(float distance_val, uint16_t alarm_val)
{
    auto adam_msg = sensor_msgs::msg::Range();
    adam_msg.header.frame_id = "adam_sensor2_link";
    adam_msg.header.stamp = this->now();
    adam_msg.min_range = minRange;
    adam_msg.max_range = maxRange;
    adam_msg.range = distance_val;
    laser_sensor2_pub_->publish(adam_msg);
}
void Laser::adam_sensor3_publisher(float distance_val, uint16_t alarm_val)
{
    auto adam_msg = sensor_msgs::msg::Range();
    adam_msg.header.frame_id = "adam_sensor3_link";
    adam_msg.header.stamp = this->now();
    adam_msg.min_range = minRange;
    adam_msg.max_range = maxRange;
    adam_msg.range = distance_val;
    laser_sensor3_pub_->publish(adam_msg);
}
void Laser::adam_sensor4_publisher(float distance_val, uint16_t alarm_val)
{
    auto adam_msg = sensor_msgs::msg::Range();
    adam_msg.header.frame_id = "adam_sensor4_link";
    adam_msg.header.stamp = this->now();
    adam_msg.min_range = minRange;
    adam_msg.max_range = maxRange;
    adam_msg.range = distance_val;
    laser_sensor4_pub_->publish(adam_msg);
}
void Laser::timer_callback()
{
    adam_sensor1_publisher(distance_val_1,alarm_val_1);
    adam_sensor2_publisher(distance_val_2,alarm_val_2);
    adam_sensor3_publisher(distance_val_3,alarm_val_3);
    adam_sensor4_publisher(distance_val_4,alarm_val_4);
}
