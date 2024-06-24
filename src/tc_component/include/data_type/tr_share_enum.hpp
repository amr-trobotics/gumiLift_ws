#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TrShareEnum : public rclcpp::Node
{

public:

	// enum ButtonRole {
    //     log_sysTEM = 0,
    //     LOG_COORD = 1,
    //     log_err = 2,
    //     LOG_TACKTIME = 3,
    //     log_seq = 4,
    //     LOG_GUI = 5,
    //     LOG_MOTOR = 6,
    //     LOG_HOST = 7,
    //     LOG_BMS = 8,
    //     LOG_HYBRID = 9,
    //     LOG_IO = 10
    // };

    enum LogRole {
        log_sys = 0,
        log_Coord,
        log_err,
        log_TackTime,
        log_seq,
        log_Gui,
        log_Motor,
        log_host,
        log_Bms,
        Log_Hybrid,
        log_Io,
        log_Data
    };

    enum ErrRole
    {
        Warning = 0,
        Heavy = 1,
        ByOp = 2
    }; 

    enum Arm_State
    {
        shutdown = 0,
        init = 1,
        disconnected = 2,
        manual = 3,
        aborting = 4,
        notassigned = 5,
        move = 6,
        paused = 7,
        arrival = 8,
        docking = 9,
        undocking = 10,
        Acquiring = 11, //적재 하는 경우
        depositing = 12, //투입하는 경우
        moveport = 13,
        charging = 14,
        opendoor = 15,
        scanning = 16,
        stop_charging = 17,
        alarm = 18
    }; 

    enum SysState
    {
        none = -1,
        on = 1,
        off = 0,
    };

    enum MainTabName
    {
        tbMain = 0,
        tbJog = 1,
        tbSetTeach = 2,
        tbSetNode = 3,
        tbSetSystem1 = 4,
        tbSetSystem2 = 5,
        tbSetSafety = 6,
        tbSetTimeout = 7,
        tbSetPio = 8,
        tbAlarm = 9,
        tbBackup = 10,
        tbSemi = 11
    };

    enum numMode
    {
        modeNone = 0,
        modeManual = 1,
        modeAuto = 2,
    };
};
