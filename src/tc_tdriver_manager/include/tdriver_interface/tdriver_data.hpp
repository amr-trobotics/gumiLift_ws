#ifndef TDRIVER_DATA_HPP
#define TDRIVER_DATA_HPP

#include "nlohmann/json.hpp"

using nlohmann::json;



namespace tdriver_api
{

    #define PI 3.14159265359

    #define RELOCATE 1
    #define CONFIRM_RELOCATE 2

    class TDriverProtocolHeader 
    {
        public:
            TDriverProtocolHeader() = default;

            uint8_t sync_; // 0x5A Message synchronization header
            uint8_t version_; // Protocol version
            uint16_t number_; // Serial number
            uint32_t length_; // Data area length
            uint16_t type_; // Message type (API Number)
            uint8_t reserved_[6] = { 0 }; // Reserved area
    };

    struct TDriverStatusApi
    {
        uint16_t info_inquiry = 0x03E8;
        uint16_t running_info_inquiry = 0x03EA;
        uint16_t location_inquiry = 0x03EC;
        uint16_t speed_inquiry = 0x03ED;
        uint16_t blocked_status_inquiry = 0x03EE;
        uint16_t battery_inquiry = 0x03EF;
        uint16_t estop_inquiry = 0x03F4;
        uint16_t navigation_status_inquiry = 0x03FC;
        uint16_t map_loading_inquiry = 0x03FE;
        uint16_t alarm_status_inquiry = 0x041A;
        uint16_t robot_localizaion_inquiry = 0x03FD;
        uint16_t robot_map_loading_inquiry = 0x03FE;
        uint16_t motor_status_inquiry = 0x0410;
        uint16_t robot_jacking_status_inquiry = 0x0403;
    };

    struct TDriverNavigationApi
    {
        uint16_t pause_navigation = 0x0BB9;
        uint16_t resume_navigation = 0x0BBA;
        uint16_t cancel_navigation = 0x0BBB;
        uint16_t rotation_navigation = 0x0BF0;
        uint16_t translation = 0x0BEF;
        uint16_t designated_navigation = 0x0BFA;
    };

    struct TDriverControlApi
    {
        uint16_t openloop_motion = 0x07DA;
        uint16_t relocation = 0x07D2;
        uint16_t confirm_relocation = 0x07D3;
        uint16_t change_map = 0x07E6;
    };

    struct TDriverConfigApi
    {
        uint16_t clear_robot_all_errors = 0x0FA9;
    };

    struct TDriverOtherApi
    {
        uint16_t jacking_height = 0x17B9; // set height ( 0m ~ 0.12m )
        uint16_t jacking_stop = 0x17B8; // stop
    };

    /*
    받아야할 데이터
        //Status API Port 19204
        - API Name : API Number
        - Robot Information Inquiry : 0x03E8 -> no json data
        - Robot Running Information Inquiry : 0x03EA -> no json data
        - Robot Location Inquiry : 0x03EC -> no json data
        - Robot Speed Inquiry : 0x03ED -> no json data
        - Robot Blocked Status Inquiry : 0x03EE -> no json data
        - Robot Battery Status Inquiry : 0x03EF ( simple => default = no)
        - Robot Path Status Inquiry : meaningless during path navigation
        - Robot Estop STatus Inquiry : 0x03F4 
        - Robot Navigation Status Inquiry : 0x03FC ( simple => default = no)
        - Robot Map Loading Status Inquiry : 0x03FE
        - Robot Alarm Status Inquiry : 0x041A

        - Robot Laser Status Inquiry : 0x03F1 ( return beams3D => default = false )
        - Robot I/O Status Inquiry : 0x03F5 
        - Robot Ultrasonic Status Inquiry : 0x03F8
        - Robot Localization Status Inquiry : 0x03FD
        - Batch Data Inquiry => 1,2 사용

        // Control API Port 19205
        - Switch Map : 0x07E6 ( map_name )
        
        //Navigation API Port 19206
        - Pause Navigation : 0x0BB9
        - Resume Navigation : 0x0BBA
        - Cancel Navigation : 0x0BBB
        - Rotation : 0x0BF0 ( angle(rad), vw(angular speed), mode(mileage mode = default))
        - Designated Path Navigation : 0x0BFA
        - Clear Designated Path Navigation : 0x0BFB
        

        1. Start the Robot2. Wait for the robot to boot complete (After the robot is successfully connected using TCP, the boot is completed.)
        3. Perform relocation 2002
        4. Query location status and wait for relocating completion 1021(Localization Status Inquiry)
        5. Confirm correct location 2003
        6. Perform path navigation 3051
        7. Query navigation status and wait for navigation to complete 1020
        8. Repeat steps 6 and 7 to achieve continuous path navigation

        TDriver Init Seq
        StatusNode                true     ControlNode 
        Check 1022(Loading Map)  --------> 2002(Relocation)
                                           wait localizaion
                                 Complete
        Check 1021(Localization) --------> 2003(Confirm)
        Check 1021(Localization) --------> SUCCESS ( then, Initializing is finished )

        
    */

}


#endif