#ifndef GUI_DATA_TYPE_HPP
#define GUI_DATA_TYPE_HPP

namespace GuiData
{
    #define ERROR_NONE 0
    #define ERROR_ACS 1
    #define ERROR_PIO 2
    #define ERROR_TDRIVER 3

    #define CMD_NONE 0
    #define CMD_TRANSFER_COMMAND 1
    #define CMD_TRANSFER_DATA 2
    #define CMD_TRANSFER_INFO 3
    #define CMD_TIME_DATA 4

    #define INIT_NONE 0
    #define INIT_RUNNING 1
    #define INIT_DONE 2
    #define INIT_FAIL 3
 
    #define DO_INIT 1
    #define CHANGE_MAP 2
}


#define RELOCATE_FAIL 0
#define RELOCATE_FINISH 1
#define RELOCATE_RUNNING 2
#define RELOCATE_COMPLETE 3
#define CHANGE_MAP_START 4
#define CHANGE_MAP_SKIP 5
#define CHANGE_MAP_RUNNING 6
#define CHANGE_MAP_FINISH 7
#define CHANGE_MAP_FAIL 8


#endif