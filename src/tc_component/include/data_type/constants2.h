#ifndef _YH_
#define _YH_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//**********EZIO IN************//
//EZIOCH01 define I32N-T

#define IN_MOTOR_MC_ON 1 // NC - 0 Alarm T-Driver Motor
#define IN_EMS_BTN_1 2 // Stop
#define IN_EMS_BTN_2 3 // Stop
#define IN_EMS_BTN_3 4 // TDRIVER ALARM
#define IN_EMS_BTN_4 5 // TDRIVER TURN ON
#define IN_START 6  //TASK
#define IN_STOP 7 //STOPPER ALARM 
#define IN_SAFETY_OFF_ON 8 // POWER moller Error TASK
#define IN_BUZZER_STOP 9 // POWER moller Error TASK
#define IN_TEACH 10 // MC, ON when charge, OFF when stop charge
#define IN_LIFT_UP 11 // Reset
#define IN_LIFT_DOWN 12 // Sound Off
#define IN_PC_ON 13 //Push lock  
#define IN_FALL_DETECTION_L 14 //Push lock
#define IN_FALL_DETECTION_R 15 // CYLINDER Backward TASK

#define IN_WHEEL_0_POS 19 //Contact Sensor Detect TASK
#define IN_WHEEL_90_POS 20 // STOPPER UP SENSOR TASK
#define IN_LIFT_UP_DETECTION 21 // STOPPER DOWN SENSOR TASK
#define IN_LIFT_DOWN_DETECTION 22 // TASK
#define IN_TRAY_FRONT_DETECTION 23 // TASK
#define IN_TRAY_REAR_DETECTION 24 // TASK
#define IN_RTURN_0_POS 25 // front door 
#define IN_RTURN_90_POS 26 // front door
#define IN_RTURN_HOMEPOS 27 // Deleted , Don't use
#define IN_TDRIVER_ON 28 // don't use
#define IN_TDRIVER_ALARM 29 // don't use 

#define IN_PIO_GO 30 //wrong in amr4

/*
    #define IN_REAR_RWD_DETECTION 28 // TASK
    #define IN_REAR_BOX_DETECTION 29 // TASK 
    #define IN_REAR_BASKET_DETECTION 30 // TASK
    #define IN_REAR_FWD_DETECTION 31 //TASK
*/

//EZIOCH03 define I8O8P-T
#define IN_LIDAR1_OSSD_MON 32 // lidar1 warning zone

#define IN_LIDAR1_CONTAM 34 // BASKET IN
#define IN_LIDAR1_ERROR 35 // Don't use
#define IN_LIDAR1_OSSD_PAIR 36 // lidar1 protective zone
#define IN_SAFETY_PLC_ERROR 37 // Safety PLC ErrorSafety, PLC Device Error Status
#define IN_SAFETY_PLC_RESET 38 // Safety PLC Reset, Reset Button
#define IN_BUMPER_DETECTION_1 39 // 4 Bumpers Only use this one
//EZIOCH04 define I8O8P-T
#define IN_LIDAR2_OSSD_MON 40 // Don't use

#define IN_LIDAR2_CONTAM 42 // Don't use
#define IN_LIDAR2_ERROR 43 // Don't use
#define IN_LIDAR2_OSSD_PAIR 44
#define IN_MANUAL_KEY 45 // Manual
#define IN_AUTO_KEY 46 // Auto
#define IN_BUMPER_DETECTION_2 47

//EZIOCH05 define I8O8P-T
#define IN_PIO_OUT01_LREQ 48
#define IN_PIO_OUT02_UREQ 49
#define IN_PIO_OUT03 50
#define IN_PIO_OUT04_READY 51
#define IN_PIO_OUT05 52
#define IN_PIO_OUT06 53
#define IN_PIO_OUT07 54
#define IN_PIO_OUT08_ESTOP 55

//**********EZIO OUT************//
//EZIOCH02 define O32N-T

#define OUT_BUZZER_STOP 1 


#define OUT_PIO_SELECT 4
#define OUT_LAMP_RESET 5
#define OUT_LAMP_START 6
#define OUT_LAMP_STOP 7
#define OUT_SAFETY_OFF_ON 8
#define OUT_LAMP_BUZZER_STOP 9
#define OUT_LAMP_TEACH 10
#define OUT_LAMP_LIFT_UP 11
#define OUT_LAMP_LIFT_DOWN 12
#define OUT_LAMP_PC_ON 13
#define OUT_BUZZER_DW 14
#define OUT_BUZZER_CH1 15
#define OUT_BUZZER_CH2 16
#define OUT_BUZZER_CH3 17
#define OUT_BUZZER_CH4 18
#define OUT_BUZZER_CH5 19
#define OUT_LED1_BLUE 20
#define OUT_LED1_GREEN 21
#define OUT_LED1_RED 22
#define OUT_LED2_BLUE 23
#define OUT_LED2_GREEN 24
#define OUT_LED2_RED 25
#define OUT_LED3_BLUE 26
#define OUT_LED3_GREEN 27
#define OUT_LED3_RED 28
#define OUT_LED4_BLUE 29
#define OUT_LED4_GREEN 30
#define OUT_LED4_RED 31
//EZIOCH03 define I8O8P-T
#define OUT_LIDAR1_C1 32
#define OUT_LIDAR1_C2 33
#define OUT_LIDAR1_D1 34
#define OUT_LIDAR1_D2 35
#define OUT_LIDAR1_E1 36
#define OUT_LIDAR1_E2 37
#define OUT_LIDAR1_RESTART 38
#define OUT_LIDAR1_OUT08 39
//EZIOCH04 define I8O8P-T
#define OUT_LIDAR2_C1 40
#define OUT_LIDAR2_C2 41
#define OUT_LIDAR2_D1 42
#define OUT_LIDAR2_D2 43
#define OUT_LIDAR2_E1 44
#define OUT_LIDAR2_E2 45
#define OUT_LIDAR2_RESTART 46
#define OUT_LIDAR2_OUT08 47
//EZIOCH05 define I8O8P-T
#define OUT_PIO_IN01_VALID 48
#define OUT_PIO_IN02_CS0 49
#define OUT_PIO_IN03_CS1 50
#define OUT_PIO_IN04_CS2 51
#define OUT_PIO_IN05_TR_REQ 52
#define OUT_PIO_IN06_BUSY 53
#define OUT_PIO_IN07_COMPT 54
#define OUT_PIO_IN08_CONT 55


//****ezioID****//
#define ezioID01 0
#define ezioID02 1
#define ezioID03 2
#define ezioID04 3
#define ezioID05 4
#define ezioID06 5

#endif /*_YH_*/




