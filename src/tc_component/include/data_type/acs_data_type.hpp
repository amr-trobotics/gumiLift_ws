#ifndef ACS_DATA_TYPE_HPP
#define ACS_DATA_TYPE_HPP

enum AcsCommand
{
    ping = 0x01,
    set_time = 0x02,
    move_work = 0x03,
    move_work_cancel = 0x04,
    amr_pause = 0x05,
    alarm_on_off = 0x07,
    scan = 0x08,
    stop_charge = 0x09,
    check_map = 0x14,
    fetch_map = 0x15,
    req_map_sync = 0x16,
    update_map = 0x17
};

enum AcsCommandResponse
{
    ping_res = 0x65,
    set_time_res = 0x66,
    move_work_res = 0x67,
    move_work_cancel_res = 0x68,
    pause_res = 0x69,
    alarm_res = 0x6B,
    scan_res = 0x6C,
    stop_charge_res = 0x6D,
    check_map_res = 0x78,
    fetch_map_res = 0x79,
    req_map_sync_res = 0x7A,
    update_map_res = 0x7B
};

enum AgvReport
{
    keep_alive = 0x31,
    init_report = 0x32,
    status_report = 0x33,
    move_completed = 0x34,
    fail_report = 0x35,
    continue_report = 0x36,
    state_changed = 0x37,
    task_report = 0x38
};

enum AgvReportResponse
{
    keep_alive_res = 0x95,
    init_report_res = 0x96,
    status_report_res = 0x97,
    move_completed_res = 0x98,
    fail_report_res = 0x99,
    continue_report_res = 0x9A,
    state_changed_res = 0x9B,
    task_report_res = 0x9C    
};

//MoveCommand
//Command
#define MOVE_WORK 1
#define MOVE_WORK_CANCEL 2
#define STOP_CHARGE 3
#define PAUSE_AMR 4
#define RESUME_AMR 5

//
#define ROUTE_NONE 0
#define ROUTE_CLEAR_PRE_CMD 1

//Target
#define NONE 0
#define STK_PORT 1
#define EQ_PORT 2
#define AUTO_DOOR 3
#define CHARGE 4
#define ELEVATOR 5
#define AIR_SHOWER 6

//Cancel Option
#define CANCEL_REQ 0
#define CANCEL_REQ_IMMEDIATELY 1

//Lift State
#define LIFT_DOWN 0
#define LIFT_UP 1

//Task
#define AQ 0
#define DP 1 
#define CHG 2
#define OD 3
#define MP 4
#define ASE_OPEN 5
#define ASE_PASS 6
#define ASE_EXIT 7
#define ASX_OPEN 8
#define ASX_PASS 9
#define ASX_EXIT 10
#define EL_OPEN 11
#define EL_PASS 12
#define EL_EXIT 13

//Agv State
#define SHUTDOWN 0
#define INIT 1
#define DISCONNECTED 2
#define MANUAL 3
#define ABORTING 4
#define NOT_ASSIGNED 5
#define MOVE 6
#define PAUSED 7
#define ARRIVAL 8
#define DOCKING 9
#define UNDOCKING 10
#define ACQUIRING 11
#define DEPOSITING 12
#define MOVEPORT 13
#define CHARGING 14
#define OPENDOOR 15
#define SCANNING 16
#define STOP_CHARGING 17
#define AMR_ALARM 18

//AlarmCode
#define OBSTACLE_DETECT_ALARM 10001
#define BREAK_DOWN_ALARM 10002
#define REPAIRING_ALARM 10003
#define BATTERY_ALARM 10004
#define TEACHING_ALARM 10005
#define AUTODOOR_ALARM 10006
#define AIRSHOWER_ALARM 10007
#define ELEVATOR_ALARM 10008
#define CART_ALARM 10009

//Response
#define REJECTED 0
#define ACCEPT 1

// Sound Type
#define SOUND_NONE 0
#define SOUND_ALARM 1
#define SOUND_BUMPER 2
#define SOUND_OBSTACLE 3
#define SOUND_DOCKING 4
#define SOUND_LOAD_UNLOAD 5
#define SOUND_LOADING 6
#define SOUND_BLOCKING 7
#define SOUND_MOVING_STK 8
#define SOUND_MOVING_PORT 9
#define SOUND_MOVING_EV 10
#define SOUND_MOVING_CHARGE 11
#define SOUND_PASS_AUTO_DOOR 12
#define SOUND_ACS_PAUSE 13
#define SOUND_AGV_PAUSE 14
#define SOUND_MOVING_AIRSHOWER 15
#define SOUND_CALL_ELEVATOR 16
#define SOUND_WAIT_ELEVATOR 17
#define SOUND_START_ENTER 18
#define SOUND_ENTER_NOW 19
#define SOUND_GET_OUT 20
#define SOUND_FIRST_FLOOR_EXIT 21
#define SOUND_SECOND_FLOOR_EXIT 22
#define SOUND_SHOCK_DETECTION 23
#define SOUND_DANGER 24
#define SOUND_EXIT_THIS_FLOOR 25
#define SOUND_START_EXIT 26
#define SOUND_EXIT_NOW 27
#define SOUND_ERROR 28

//CMD type
#define CMD_NONE 0
#define CMD_MOVE 1
#define CMD_TASK_READY 2
#define CMD_TASK_RUNNING 3
#define CMD_TASK_DONE 4
#define CMD_TASK_FAIL 5

// Light Type
#define LIGHT_NONE 0
#define LIGHT_WHITE 1
#define LIGHT_GREEN 2
#define LIGHT_YELLOW 3
#define LIGHT_RED 4
#define LIGHT_RED_FLICKER 5
#define LIGHT_WHITE_FLICKER 6
#define LIGHT_BLUE 7
#define LIGHT_PURPLE 8

// AmbientLight
#define LIGHT_LEFT 1
#define LIGHT_RIGHT 2
#define LIGHT_FRONT 3
#define LIGHT_REAR 4
#define LIGHT_WHOLE 5
#define LIGHT_NONE 6

// INPUT STATUS TYPE
#define MANUAL_MODE 0
#define AUTO_MODE 1

// ERROR RESET
#define ERROR_NONE 0
#define ERROR_OCCUR 1
#define IN_RESET_SEQ 2
#define RESET_FAIL 3
#define RESET_DONE 4

// SAFETY AREA

#define AREA_0 0
#define AREA_1 1
#define AREA_2 2
#define AREA_3 3
#define AREA_4 4
#define AREA_5 5
#define AREA_6 6
#define AREA_7 7


//Task Order Number
#define DOCKING_1 1
#define DOCKING_2 2
#define DOCKING_3 3
#define DOCKING_4 4
#define AIRSHOWER_1_1 5
#define AIRSHOWER_1_2 6
#define AIRSHOWER_2_1 7
#define AIRSHOWER_2_2 8
#define CHARGE_ON 9
#define CHARGE_OFF 10
#define ELEVATOR_2 11
#define ELEVATOR_1 12
#define PIO_RESET_1 13
#define PIO_STOP 14


//Alarm Report
#define INFO 0
#define ALARM 1
#define REJECT 2

//Scenario Init 
#define SCENARIO_INIT_DONE 1



#endif